//
// SpiBus.h
//
//  Created on: Apr 12, 2021
//      Author: aw@pionier-manufaktur.de
//

#ifndef SRC_EVDRIVERS_SPIBUS_H_
#define SRC_EVDRIVERS_SPIBUS_H_

#include <vector>

#include "main.h"

#include "InterruptBase.h"
#include "SpiDevice.h"

// forward declaration
class SpiDevice;

struct SpiDeviceInfo {
    SpiDevice *dev;
    int prescaler;
};

// class SpiBus : private InterruptBase {
class SpiBus {
    friend class SpiDevice;

public:
    static const int MAX_DEV_COUNT = 4;
    SpiBus(SPI_HandleTypeDef &hspi, uint32_t spi_clock_speed);
    void initialize(std::vector<SpiDevice *> device_list);

protected:
    bool acquire(int dev_id, bool auto_cs = true);
    bool release(int dev_id, bool auto_cs = true);
    bool transmit(uint8_t *data, uint16_t size, uint32_t timeout);
    bool transmitViaDma(uint8_t *data, uint16_t size);
    bool transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size,
                    uint32_t timeout);
    bool isReadyToSend();

private:
    SPI_HandleTypeDef &_hspi;
    const uint32_t spi_clk_freq;
    int cur_dev;

    SpiDeviceInfo devices[MAX_DEV_COUNT];

    // void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
};

#endif // SRC_EVDRIVERS_SPIBUS_H_
