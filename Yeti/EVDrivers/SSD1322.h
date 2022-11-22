//
// SSD1322.h
//
//  Created on: Mar 17, 2021
//      Author: aw@pionier-manufaktur.de
//

#ifndef SRC_EVDRIVERS_SSD1322_H_
#define SRC_EVDRIVERS_SSD1322_H_

#include "Display.h"
#include "Gpio.h"
#include "SpiDevice.h"

class SSD1322 : public SpiDevice, public Display {
public:
    SSD1322(Gpio &cs, Gpio &dc, Gpio &reset);
    const int COL_SIZE = 32;
    const int ROW_SIZE = 4;

    virtual void clearDisplay();
    //
    // text needs to be null-terminated
    // column: 0..31
    // row: 0..3
    //
    virtual void printText(const char *text, unsigned int column,
                           unsigned int row, Alignment align = LEFT);

protected:
    void initialize();

private:
    void sendCmd(uint8_t cmd);
    void sendCmd(uint8_t cmd, uint8_t arg);
    void sendCmd(uint8_t cmd, uint8_t arg1, uint8_t arg2);
    void sendData(uint8_t data[], uint16_t len);

    // WithAcq means, we are already acquired the spi bus
    void clearDisplayWithAcq();
    void enableCommand();
    void enableData();

    uint8_t *getNextFreeBuf();
    void sendCmdHelper(uint8_t cmd, uint8_t data[], uint16_t len);
    void setChipSelect(bool enable);
    void setReset(bool enable);
    void waitForReadyToSend();

    Gpio cs;
    Gpio dc;
    Gpio reset;

    static const uint32_t SSD1322_MAX_SPI_FREQ = 10000000;
    static const uint8_t BUF_SIZE = 64;

    uint8_t double_buf[2 * BUF_SIZE];
    bool first_buf;
};

#endif // SRC_EVDRIVERS_SSD1322_H_
