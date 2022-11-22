//
// MgmtLink.cpp
//
//  Created on: Mar 30, 2021
//      Author: aw@pionier-manufaktur.de
//

#include "MgmtLink.h"
#include "EVConfig.h"

#include <main.h>

#include <pb_decode.h>
#include <pb_encode.h>

extern CRC_HandleTypeDef hcrc;

static size_t cobsEncode(const void *data, size_t length, uint8_t *buffer) {
    uint8_t *encode = buffer;  // Encoded byte pointer
    uint8_t *codep = encode++; // Output code pointer
    uint8_t code = 1;          // Code value

    for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte) {
        if (*byte) // Byte not zero, write it
            *encode++ = *byte, ++code;

        if (!*byte || code == 0xff) // Input is zero or block completed, restart
        {
            *codep = code, code = 1, codep = encode;
            if (!*byte || length)
                ++encode;
        }
    }
    *codep = code; // Write final code value

    // add final 0
    *encode++ = 0x00;

    return encode - buffer;
}

static inline int32_t calculateCrc(uint8_t *buf, size_t len) {
    HAL_CRC_Init(&hcrc);
    // NOTE: according to the docu of HAL_CRC_Calculate, the buffer
    // pointer should be casted to uint32_t*
    return HAL_CRC_Calculate(&hcrc, (uint32_t *)(void *)buf, len);
}

MgmtLink::MgmtLink(UART_HandleTypeDef *_huart) : huart(_huart) {
    cobsDecodeReset();
    still_sending = false;
    rx_queue_idx = 0;
    packageDropCount = 0;

    mq_attr.name = NULL;
    mq_attr.attr_bits = 0;
    mq_attr.cb_mem = NULL;
    mq_attr.cb_size = 0;
    mq_attr.mq_mem = NULL;
    mq_attr.mq_size = 0;
    // init message queue
    mq = osMessageQueueNew(RX_QUEUE_SIZE - 1, sizeof(MgmtLinkMessage *),
                           &mq_attr);

    // enable interrupts
    registerCallback(IntType::HAL_UART_RxHalfCpltCallback);
    registerCallback(IntType::HAL_UART_RxCpltCallback);
    //registerCallback(IntType::HAL_UART_TxCpltCallback);
    registerCallback(IntType::HAL_UART_ErrorCallback);

    HAL_UART_Receive_DMA(huart, uart_rx_circ_buf, 2);
}

bool MgmtLink::handlePacketIsr(uint8_t idx, size_t length) {
    // here is our packet
    // returns true if the packet was inserted into the queue.
    // If it was not, drop the packet and overwrite the buffer as the queue is
    // full!
    if (length >= COBS_MIN_PAYLOAD_SIZE && length <= TX_MAX_PAYLOAD_SIZE + 4) {
        rx_queue[idx].len = length;
        // push pointer &rx_queue[idx] to message queue to thread
        // If it fails, there is nothing we can do here, so ignore return value.
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
        printf("MQ %i\n", length);
#endif
        // push pointer to queue if it is not full.
        MgmtLinkMessage *s = &rx_queue[idx];
        if (osMessageQueuePut(mq, &s, 0, 0) == osOK) {
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
            printf("Inserted into MQ.\n");
#endif
            return true;
        } else {
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
            printf("Failed to insert into MQ.\n");
#endif
            packageDropCount++;
        }
    } else {
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
        printf("!!!!!!!!!!! ELSE length\n");
#endif
    }
    return false;
}

void MgmtLink::cobsDecodeReset() {
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
    printf("cobsDecodeReset()\n");
#endif
    code = 0xff;
    block = 0;
    decode = rx_queue[rx_queue_idx].rx_packet_buf;
}

void MgmtLink::cobsDecodeIsr(uint8_t byte) {
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
    printf("%02X ", byte);
#endif
    // check max length
    if ((decode - rx_queue[rx_queue_idx].rx_packet_buf == COBS_BUFFER_SIZE) &&
        byte != 0x00) {
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
        printf("!!! BUF OVERUN\n");
#endif
        cobsDecodeReset();
    }

    if (block) {
        // we're currently decoding and should not get a 0
        if (byte == 0x00) {
            // probably found some garbage -> reset
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
            printf("!!! GARBAGE\n");
#endif
            cobsDecodeReset();
            return;
        }
        *decode++ = byte;
    } else {
        if (code != 0xff) {
            *decode++ = 0;
        }
        block = code = byte;
        if (code == 0x00) {
            // we're finished, reset everything and commit
            if (decode == rx_queue[rx_queue_idx].rx_packet_buf) {
                // we received nothing, just a 0x00
            } else {
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
                printf("!!! FOUND PACKET\n");
#endif
                // set back decode with one, as it gets post-incremented
                const size_t raw_len =
                    decode - 1 - rx_queue[rx_queue_idx].rx_packet_buf;
                if (handlePacketIsr(rx_queue_idx, raw_len)) {
                    // switch to next buffer for decoding if it was inserted
                    // into the queue.
                    rx_queue_idx++;
                    if (rx_queue_idx >= RX_QUEUE_SIZE)
                        rx_queue_idx = 0;
                }
            }
            cobsDecodeReset();
            return; // need to return here, because of block--
        }
    }
    block--;
}

bool MgmtLink::read(HiToLo *msg, uint32_t timeout) {
    //	blocking pull from message queue in thread
    MgmtLinkMessage *rx_buf = NULL;
    if (osMessageQueueGet(mq, &rx_buf, NULL, timeout) != osOK)
        return false;
    if (rx_buf == NULL) {
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
        printf("Rcvd Null ptr from MQ\n");
#endif
        return false;
    }
    uint8_t *rx_packet_buf = rx_buf->rx_packet_buf;
    size_t rx_packet_len = rx_buf->len;

    // check crc32 (CRC-32/JAMCRC)
    // NOTE: this could also be done earlier, but here it is in thread instead
    // of ISR
    uint32_t crc = calculateCrc(rx_packet_buf, rx_packet_len);

    if (crc) {
        // crc mismatch, drop packet
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
        printf("MgmtLink::read(): CRC failed\n");
#endif
        return false;
    } else {
        rx_packet_len = rx_packet_len - 4;
    }
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
    printf("MgmtLink::read(): CRC passed\n");
#endif

    pb_istream_t istream = pb_istream_from_buffer(rx_packet_buf, rx_packet_len);

    bool status = pb_decode(&istream, HiToLo_fields, msg);

    return status;
}

bool MgmtLink::write(LoToHi *msg) {

	while (huart->gState != HAL_UART_STATE_READY) {};

    pb_ostream_t ostream =
        pb_ostream_from_buffer(tx_packet_buf, TX_MAX_PAYLOAD_SIZE);

    bool status = pb_encode(&ostream, LoToHi_fields, msg);

    if (!status) {
        // couldn't encode
        return false;
    }

    size_t tx_payload_len = ostream.bytes_written;

    // add crc32 (CRC-32/JAMCRC)
    uint32_t crc = calculateCrc(tx_packet_buf, tx_payload_len);

    for (int byte_pos = 0; byte_pos < 4; ++byte_pos) {
        tx_packet_buf[tx_payload_len] = (uint8_t)crc & 0xFF;
        crc = crc >> 8;
        tx_payload_len++;
    }

    size_t tx_encode_len =
        cobsEncode(tx_packet_buf, tx_payload_len, encode_buf);

    if (HAL_OK == HAL_UART_Transmit_DMA(huart, encode_buf, tx_encode_len)) {
        return true;
    }
    return false;
}

void MgmtLink::HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huartisr) {
    if (huartisr == huart) {
        uint8_t rx_byte = uart_rx_circ_buf[0];
        cobsDecodeIsr(rx_byte);
    }
}

void MgmtLink::HAL_UART_RxCpltCallback(UART_HandleTypeDef *huartisr) {
    if (huartisr == huart) {
        uint8_t rx_byte = uart_rx_circ_buf[1];
        cobsDecodeIsr(rx_byte);
    }
}

/*
void MgmtLink::HAL_UART_TxCpltCallback(UART_HandleTypeDef *huartisr) {
    if (huartisr == huart) {
        still_sending = false;
    }
}*/

void MgmtLink::HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
#ifdef MGMTLINK_CPP_ENABLE_PRINTF
    printf("UART2 ERROR - ignoring...\n");
#endif
   // HAL_UART_DeInit(huart);
   // HAL_UART_Receive_DMA(huart, uart_rx_circ_buf, 2);
}
