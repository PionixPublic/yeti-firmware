//
// SpiDevice.h
//
//  Created on: Apr 12, 2021
//      Author: aw@pionier-manufaktur.de
//

#ifndef SRC_EVDRIVERS_SPIDEVICE_H_
#define SRC_EVDRIVERS_SPIDEVICE_H_

#include "Gpio.h"
#include "SpiBus.h"

// forward declaration
class SpiBus;

class SpiDevice {
    friend SpiBus;

public:
    SpiDevice(uint32_t max_freq);

protected:
    const uint32_t max_freq;

    void initialize(SpiBus *spi_bus, int device_id);
    virtual void initialize() = 0;
    virtual void setChipSelect(bool enable) = 0;
    bool acquireSpiBus(bool auto_cs = true);
    bool releaseSpiBus(bool auto_cs = true);
    bool spiTransmit(uint8_t *data, uint16_t size, uint32_t timeout = 0);
    bool spiTransmitViaDma(uint8_t *data, uint16_t size);
    bool spiTransceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size,
                       uint32_t timeout = 0);
    bool isReadyToTransmit();

private:
    static const uint32_t DEFAULT_TIMEOUT = 1000; // 1 second
    int dev_id;
    SpiBus *spi_bus;
    bool _acquired;
    uint32_t _spi_timeout;
};

#endif // SRC_EVDRIVERS_SPIDEVICE_H_
