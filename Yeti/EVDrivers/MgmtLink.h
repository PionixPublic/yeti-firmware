//
// MgmtLink.h
//
//  Created on: Mar 30, 2021
//      Author: aw@pionier-manufaktur.de
//

#ifndef SRC_EVDRIVERS_MGMTLINK_H_
#define SRC_EVDRIVERS_MGMTLINK_H_

#include "cmsis_os.h"

#include "../protobuf/hi2lo.pb.h"
#include "../protobuf/lo2hi.pb.h"
#include "../protobuf/common.pb.h"

#include "InterruptBase.h"

#define COBS_BUFFER_SIZE      256 /* this will give 254 bytes of payload */
#define COBS_PAYLOAD_SIZE     (COBS_BUFFER_SIZE - 2)
#define COBS_MIN_PAYLOAD_SIZE (4 + 1) /* 4 bytes for crc32 + min one byte */
#define TX_MAX_PAYLOAD_SIZE   (COBS_PAYLOAD_SIZE - 4) /* 4 bytes for crc32 */
#define RX_QUEUE_SIZE         3

typedef struct _MgmtLinkMessage {
    uint8_t rx_packet_buf[COBS_BUFFER_SIZE];
    size_t len;
} MgmtLinkMessage;

class MgmtLink : private InterruptBase {
public:
    MgmtLink(UART_HandleTypeDef *_huart);

    // Fills up msg and returns status of decoding
    // NOTE: if decoding fails, msg might be still altered!
    bool read(HiToLo *msg, uint32_t timeout);

    // Encodes a LoToHi message and sends it out
    // If the message cannot be encoded, it will return false
    // NOTE: if sending fails due to a previous sending
    //       is still on-going, it also will return false
    // FIXME: right now, we would need to poll in order to
    //        see, if we can send and we can not distinguish
    //        between a encoding failure ..
    bool write(LoToHi *msg);

private:
    // UART handle
    UART_HandleTypeDef *huart;

    // use a 2 byte ring-buffer, so we get interrupted for each single byte
    uint8_t uart_rx_circ_buf[2];

    // rx related
    MgmtLinkMessage rx_queue[RX_QUEUE_SIZE];
    uint8_t rx_queue_idx;
    // message queue to pass packets from isr to thread. Actually only a queue
    // of pointers.
    osMessageQueueAttr_t mq_attr;
    osMessageQueueId_t mq;

    // tx related
    uint8_t encode_buf[COBS_BUFFER_SIZE];
    uint8_t tx_packet_buf[COBS_BUFFER_SIZE];
    volatile bool still_sending;

    // cobs decoding related
    uint8_t code;
    uint8_t block;
    uint8_t *decode;

    uint32_t packageDropCount;

    // reset cobs decode state
    void cobsDecodeReset();

    // handle new incoming byte for decoding
    void cobsDecodeIsr(uint8_t byte);

    // handle framed cobs packet (runs inside ISR)
    bool handlePacketIsr(uint8_t idx, size_t length);

    // interrupt handlers
    virtual void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
    virtual void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
    //virtual void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
    virtual void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
};

#endif // SRC_EVDRIVERS_MGMTLINK_H_
