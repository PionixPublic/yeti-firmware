//
// SSD1322.cpp
//
//  Created on: Mar 17, 2021
//      Author: aw@pionier-manufaktur.de
//

#include "SSD1322.h"
#include "SSD1322_priv.h"
#include "bitmap_font_priv.h"

#include <cstring>

#include <cmsis_os2.h>
#include <main.h>

#include "Gpio.h"

SSD1322::SSD1322(Gpio &cs_, Gpio &dc_, Gpio &reset_) :
    SpiDevice(SSD1322_MAX_SPI_FREQ),
    cs(cs_),
    dc(dc_),
    reset(reset_),
    first_buf(false) {
    setChipSelect(false);
}

void SSD1322::initialize() {

    acquireSpiBus(false);

    setReset(false);
    setChipSelect(false); // no chip select
    enableData();         // data
    osDelay(10);
    setReset(true); // inverted, should stay low for 100us
    osDelay(1);
    setReset(false);
    osDelay(10);
    setChipSelect(true); // enable chip select
    enableCommand();     // enable command
    osDelay(10);

    // unlock commands
    sendCmd(ssd1322::set_command_lock, 0x12);

    // display off
    sendCmd(ssd1322::set_display_off);

    // set column start and end address (28 .. 91)
    sendCmd(ssd1322::set_column_address, 0x1C, 0x5B);

    // set row start and end address (0 .. 63)
    sendCmd(ssd1322::set_row_address, 0x00, 0x3F);

    // set display clock ~ 80 fps
    sendCmd(ssd1322::set_divider_and_frequency, 0x91);

    // 1/64 duty
    sendCmd(ssd1322::set_multiplex_ratio, 0x3F);

    // set display offset
    sendCmd(ssd1322::set_display_offset, 0x00);

    // set start line
    sendCmd(ssd1322::set_display_start_line, 0x00);

    // set remap format (0x06 normal, 0x14 rotated)
    sendCmd(ssd1322::set_remap_and_dual_com_line_mode, 0x06);

    // disable gpios
    sendCmd(ssd1322::set_gpio, 0x00);

    // enable internal vdd regulator
    sendCmd(ssd1322::set_function_selection, 0x01);

    // enable external vsl
    sendCmd(ssd1322::display_enhancement_a, 0xA0, 0xFD);

    // set segment output current
    sendCmd(ssd1322::set_contrast_current, 0x9F);

    // set scale factor of segment output current control
    sendCmd(ssd1322::master_current_control, 0x0F);

    // set default linear gray scale
    sendCmd(ssd1322::select_default_linear_grayscale_table);

    // set phase 1 as 5 clocks and phase 2 as 14 clocks
    sendCmd(ssd1322::set_phase_length, 0xE2);

    // enhance driving scheme capability
    sendCmd(ssd1322::display_enhancement_b, 0x20);

    // set pre?charge voltage level to 0.60*vcc
    sendCmd(ssd1322::set_precharge_voltage, 0x1F);

    // set second pre?charge period to 8 clocks
    sendCmd(ssd1322::set_second_precharge_period, 0x08);

    // set common pins deselect voltage level to 0.86*vcc
    sendCmd(ssd1322::set_vcomh_voltage, 0x07);

    // set normal display mode
    sendCmd(ssd1322::set_display_mode_normal);

    // disable partial display
    sendCmd(ssd1322::exit_partial_display);

    // turn on display
    sendCmd(ssd1322::set_display_on);

    clearDisplayWithAcq();

    setChipSelect(false);
    releaseSpiBus();
}

void SSD1322::clearDisplay() {
    acquireSpiBus();
    clearDisplayWithAcq();
    releaseSpiBus();
}

void SSD1322::clearDisplayWithAcq() {
    // set column start and end address (28 .. 91)
    sendCmd(ssd1322::set_column_address, 0x1C, 0x5B);

    // set row start and end address (0 .. 63)
    sendCmd(ssd1322::set_row_address, 0x00, 0x3F);

    sendCmd(ssd1322::write_ram_command);

    uint8_t *buf = getNextFreeBuf();

    memset(buf, 0, 64);

    for (int line = 0; line < 64; ++line) {
        for (int x = 0; x < 4; ++x) {
            sendData(buf, 64);
        }
    }
}

void SSD1322::printText(const char *text, unsigned int column, unsigned int row,
                        Alignment align) {
    acquireSpiBus();

    // calculate column offset
    int col_offset = 0;
    switch (align) {
    case LEFT:
        col_offset = column;
        break;
    case CENTER:
        col_offset = COL_SIZE / 2 - strlen(text) / 2 + column;
        break;
    case RIGHT:
        col_offset = COL_SIZE - strlen(text) - column;
        break;
    }

    if (col_offset < 0 || col_offset + 1 > COL_SIZE ||
        (int)row + 1 > ROW_SIZE) {
        return;
    }

    unsigned int chars_left = COL_SIZE - col_offset;

    // set row
    sendCmd(ssd1322::set_row_address, 0x0f * row, 0x0e + 0x0f * row);

    // four pixels per column, 4 bit per pixel
    uint8_t col_addr = 0x1C + col_offset * 2;

    for (const char *c = text; *c || chars_left < 0; ++c, --chars_left) {

        const uint8_t *chr = font + 15 * (*c - ' ');

        uint8_t *buf = getNextFreeBuf();

        uint8_t *ch_p = buf;
        for (int i = 0; i < 15; ++i) {
            uint8_t ch = chr[i];
            for (int j = 0; j < 4; ++j) {
                switch (ch & 0xC0) {
                case 0x00:
                    *ch_p = 0x00;
                    break;
                case 0x40:
                    *ch_p = 0x0d;
                    break;
                case 0x80:
                    *ch_p = 0xd0;
                    break;
                case 0xC0:
                    *ch_p = 0xdd;
                    break;
                }
                ch_p++;
                ch = ch << 2;
            }
        }

        sendCmd(ssd1322::set_column_address, col_addr, col_addr + 1);

        sendCmd(ssd1322::write_ram_command);

        sendData(buf, 60);

        col_addr += 2;
    }

    releaseSpiBus();
}

void SSD1322::sendCmd(uint8_t cmd) { sendCmdHelper(cmd, NULL, 0); }

void SSD1322::sendCmd(uint8_t cmd, uint8_t arg) { sendCmdHelper(cmd, &arg, 1); }

void SSD1322::sendCmd(uint8_t cmd, uint8_t arg1, uint8_t arg2) {
    uint8_t args[] = {arg1, arg2};
    sendCmdHelper(cmd, args, 2);
}

void SSD1322::sendData(uint8_t data[], uint16_t len) {
    waitForReadyToSend();

    enableData();
    spiTransmitViaDma(data, len);
}

void SSD1322::enableCommand() { dc.reset(); }

void SSD1322::enableData() { dc.set(); }

uint8_t *SSD1322::getNextFreeBuf() {
    first_buf = !first_buf;
    return first_buf ? double_buf : &(double_buf[BUF_SIZE]);
}

void SSD1322::sendCmdHelper(uint8_t cmd, uint8_t data[], uint16_t len) {
    waitForReadyToSend();

    enableCommand();
    spiTransmit(&cmd, 1);

    if (data) {
        enableData();
        spiTransmit(data, len);
    }
}

void SSD1322::setChipSelect(bool enable) { (enable) ? cs.reset() : cs.set(); }

void SSD1322::setReset(bool enable) { (enable) ? reset.reset() : reset.set(); }

void SSD1322::waitForReadyToSend() {
    while (isReadyToTransmit() == false)
        ;
}
