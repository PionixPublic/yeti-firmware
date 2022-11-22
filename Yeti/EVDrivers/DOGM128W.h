/*
 * DOGM128W.h
 *
 *  Created on: Jul 5, 2021
 *      Author: cornelius
 */

#ifndef EVDRIVERS_DOGM128W_H_
#define EVDRIVERS_DOGM128W_H_

#include "Display.h"
#include "Gpio.h"
#include "SpiDevice.h"
#include "Task.h"

class DOGM128W : public SpiDevice, public Display, public Task {
public:
    DOGM128W(Gpio &cs, Gpio &dc, Gpio &reset);
    virtual ~DOGM128W();

    virtual void clearDisplay();
    virtual void printText(const char *text, unsigned int column,
                           unsigned int row, Alignment align = LEFT);

protected:
    void initialize();

private:
    Gpio cs;
    Gpio dc;
    Gpio reset;

    static const uint32_t DOGM128W_MAX_SPI_FREQ = 100000;
    static const uint16_t BUF_SIZE = 128 * 64 / 8;

    uint8_t double_buf[2 * BUF_SIZE];
    bool first_buf;

    void setChipSelect(bool enable);
    void sendCmd(uint8_t cmd);
    void sendCmd(uint8_t cmd, uint8_t arg);
    void sendCmdHelper(uint8_t cmd, uint8_t data[], uint16_t len);
    void enableCommand();
    void enableData();
    void waitForReadyToSend();
    void setReset(bool enable);
    void sendData(uint8_t data[], uint16_t len);
    void clearDisplayWithAcq();
    uint8_t *getFreeBuf();
    uint8_t *getFullBuf();
    void switchBuf();
    virtual void updateDisplay();

    virtual void main();
};

#endif /* EVDRIVERS_DOGM128W_H_ */
