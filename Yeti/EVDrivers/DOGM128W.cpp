/*
 * DOGM128W.cpp
 *
 *  Created on: Jul 5, 2021
 *      Author: cornelius
 */

#include "DOGM128W.h"
#include "DOGM_fonts.h"
#include "bitmap_font_priv.h"
#include <cmsis_os2.h>
#include <stdio.h>
#include <string.h>

DOGM128W::DOGM128W(Gpio &cs_, Gpio &dc_, Gpio &reset_) :
    SpiDevice(DOGM128W_MAX_SPI_FREQ),
    Task("Display", 512 * 4, osPriorityLow),
    cs(cs_),
    dc(dc_),
    reset(reset_),
    first_buf(false) {
    setChipSelect(false);
}

DOGM128W::~DOGM128W() {}

void DOGM128W::clearDisplay() {
    uint8_t *buf = getFreeBuf();
    memset(buf, 0x00, BUF_SIZE);
    // memcpy(buf, font_Tamzen8x16 + 4 + 8 * i++, 128*8);
    // memcpy(buf, font + 16 * 40, 128*8);
}

void DOGM128W::printText(const char *text, unsigned int column,
                         unsigned int row, Alignment align) {
    uint8_t *buf = getFreeBuf();
    // font_Tamzen8x16
    int i = 0;
    while (true) {
        if (text[i] == 0)
            break;
        uint8_t *ptr = buf + 7 * i + row * 128 + column * 7;
        int idx = text[i] - ' ';
        if ((ptr - buf + 7) < BUF_SIZE && idx < FONT_CHARS * FONT_WIDTH)
            memcpy(ptr, fontData7x8[idx], 7);
        else
            break;
        i++;
    }
    /*
         // 4 corners
         buf[0] = 0x01;
         buf[126] = 0x01;
         buf[128*7] = 0x80;
         buf[128*7+127] = 0x80;*/
}

void DOGM128W::initialize() {

    acquireSpiBus(false);

    setReset(false);
    osDelay(10);
    setReset(true);
    osDelay(10);
    setReset(false); // inverted, should stay low for 100us
    osDelay(10);

    setChipSelect(true); // enable chip select
    sendCmd(0x40);
    sendCmd(0xA1);
    sendCmd(0xC0);
    sendCmd(0xA6);
    sendCmd(0xA2);
    sendCmd(0x2F);
    sendCmd(0xF8, 0x00);
    sendCmd(0x27);
    sendCmd(0x81, 0x16);
    sendCmd(0xAC, 0x00);
    // sendCmd(0xA5); // All pixels black, good for testing
    clearDisplay();
    sendCmd(0xAF); // Display on

    setChipSelect(false); // enable chip select

    releaseSpiBus();

    updateDisplay();
}

void DOGM128W::setChipSelect(bool enable) { (enable) ? cs.reset() : cs.set(); }

void DOGM128W::sendCmd(uint8_t cmd) { sendCmdHelper(cmd, NULL, 0); }

void DOGM128W::sendCmd(uint8_t cmd, uint8_t arg) {
    sendCmdHelper(cmd, &arg, 1);
}

void DOGM128W::sendCmdHelper(uint8_t cmd, uint8_t data[], uint16_t len) {
    waitForReadyToSend();

    enableCommand();
    spiTransmit(&cmd, 1);

    if (data) {
        //	enableData();
        spiTransmit(data, len);
    }
}

void DOGM128W::enableCommand() { dc.reset(); }

void DOGM128W::enableData() { dc.set(); }

void DOGM128W::waitForReadyToSend() {
    while (isReadyToTransmit() == false)
        ;
}

void DOGM128W::setReset(bool enable) { (enable) ? reset.reset() : reset.set(); }

void DOGM128W::sendData(uint8_t data[], uint16_t len) {
    waitForReadyToSend();

    enableData();
    // spiTransmitViaDma(data, len);
    spiTransmit(data, len, 0);
}

uint8_t *DOGM128W::getFreeBuf() {
    return first_buf ? double_buf : &(double_buf[BUF_SIZE]);
}

uint8_t *DOGM128W::getFullBuf() {
    return !first_buf ? double_buf : &(double_buf[BUF_SIZE]);
}

void DOGM128W::switchBuf() { first_buf = !first_buf; }

void DOGM128W::updateDisplay() {
    switchBuf();
    // wakeup display thread to copy buffer via SPI to display RAM
    resume();
}

/*
 * This thread actually updates the display RAM by copying the (double)
 * buffer via SPI if ready was signaled by calling updateDisplay() from
 * rendering thread.
 * */
void DOGM128W::main() {
    while (1) {
        // osDelay(1000);
        // we woke up, so display needs updating
        uint8_t *buf = getFullBuf();

        acquireSpiBus(false);

        // osDelay(1);
        // sendCmd(0x27);

        for (uint8_t page = 0; page < 8; ++page) {
            waitForReadyToSend();
            setChipSelect(true); // enable chip select
            // sendCmd(0x40);
            sendCmd(0xB0 | page); // 4 bit page addr
            sendCmd(0x10 | 0x00); // 4 MSBs of column addr
            sendCmd(0x00 | 0x00); // 4 LSBs of column addr
            // FIXME: maybe reconfigure display here all the time
            // to allow hot plugging and improve random data error stability
            setChipSelect(false); // enable chip select
            osDelay(1);
            setChipSelect(true); // enable chip select
            // osDelay(1);
            sendData(buf + page * 128, 128);
            waitForReadyToSend(); // FIXME using DMA here is useless if we busy
                                  // wait afterwards...
            setChipSelect(false); // enable chip select
            osDelay(1);
            // osDelay(1);
        }

        waitForReadyToSend();
        setChipSelect(false); // enable chip select

        releaseSpiBus();
        suspend();
    }
}
