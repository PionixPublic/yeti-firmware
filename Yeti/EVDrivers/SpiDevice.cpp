//
// SpiDevice.cpp
//
//  Created on: Apr 12, 2021
//      Author: aw@pionier-manufaktur.de
//

#include "SpiDevice.h"

SpiDevice::SpiDevice(uint32_t max_freq_) :
    max_freq(max_freq_), _acquired(false), _spi_timeout(DEFAULT_TIMEOUT) {}

void SpiDevice::initialize(SpiBus *spi_bus_, int device_id) {
    spi_bus = spi_bus_;
    dev_id = device_id;
    initialize();
}

bool SpiDevice::acquireSpiBus(bool auto_cs) {
    _acquired = spi_bus->acquire(dev_id, auto_cs);
    return _acquired;
}

bool SpiDevice::releaseSpiBus(bool auto_cs) {
    // TODO: this needs more details
    while (!isReadyToTransmit())
        ;
    _acquired = false;
    return spi_bus->release(dev_id, auto_cs);
}

bool SpiDevice::spiTransmit(uint8_t *data, uint16_t size, uint32_t timeout_) {
    if (!_acquired)
        return false;
    uint32_t timeout = (timeout_ == 0) ? _spi_timeout : timeout_;
    return spi_bus->transmit(data, size, timeout);
}

bool SpiDevice::spiTransmitViaDma(uint8_t *data, uint16_t size) {
    if (!_acquired)
        return false;
    return spi_bus->transmitViaDma(data, size);
}

bool SpiDevice::spiTransceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size,
                              uint32_t timeout_) {
    if (!_acquired)
        return false;
    uint32_t timeout = (timeout_ == 0) ? _spi_timeout : timeout_;
    return spi_bus->transceive(tx_data, rx_data, size, timeout);
}

bool SpiDevice::isReadyToTransmit() {
    if (!_acquired)
        return false;
    return spi_bus->isReadyToSend();
}
