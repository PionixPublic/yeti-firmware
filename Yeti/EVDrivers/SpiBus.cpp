//
// SpiBus.cpp
//
//  Created on: Apr 12, 2021
//      Author: aw@pionier-manufaktur.de
//

#include "SpiBus.h"

SpiBus::SpiBus(SPI_HandleTypeDef &hspi, uint32_t spi_clk_freq_) :
    _hspi(hspi), spi_clk_freq(spi_clk_freq_), cur_dev(-1) {
    for (int i = 0; i < MAX_DEV_COUNT; ++i) {
        devices[i].dev = nullptr;
    }

    // registerCallback(IntType::HAL_SPI_TxCpltCallback);
}

void SpiBus::initialize(std::vector<SpiDevice *> device_list) {
    int next_dev_id = 0;
    for (auto dev : device_list) {
        uint32_t max_freq = dev->max_freq;

        uint32_t clk_freq = spi_clk_freq / 2;
        uint32_t baudrate_ctrl = 0;
        while (max_freq <= clk_freq && baudrate_ctrl != 7) {
            baudrate_ctrl++;
            clk_freq = clk_freq / 2;
        }

        devices[next_dev_id].dev = dev;
        devices[next_dev_id].prescaler = baudrate_ctrl;
#ifdef SPIBUS_CPP_ENABLE_PRINTF
        printf("SPI: selecting prescaler %u, freq %u\n",
               (unsigned int)baudrate_ctrl, (unsigned int)clk_freq);
#endif

        next_dev_id++;
        if (next_dev_id == MAX_DEV_COUNT)
            break;
    }

    for (int i = 0; i < MAX_DEV_COUNT; ++i) {
        if (devices[i].dev == nullptr)
            break;

        devices[i].dev->setChipSelect(false);
    }

    // HACK: we do this in order to initialize the GPIOs to their correct
    //       level, using 1 ms as timeout
    uint8_t dummy_byte = 'd';
    transmit(&dummy_byte, 1, 1);

    for (int i = 0; i < MAX_DEV_COUNT; ++i) {
        if (devices[i].dev == nullptr)
            break;

        devices[i].dev->initialize(this, i);
    }
}

bool SpiBus::acquire(int dev_id, bool auto_cs) {
    // this device has already acquired the bus
    if (cur_dev == dev_id)
        return true;
    else if (cur_dev != -1)
        return false;
    else {
        cur_dev = dev_id;

        // set new baud rate
        uint32_t cr1 = READ_REG(_hspi.Instance->CR1) & ~SPI_CR1_BR_Msk;
        WRITE_REG(_hspi.Instance->CR1,
                  cr1 | (devices[cur_dev].prescaler << SPI_CR1_BR_Pos));

        if (auto_cs)
            devices[cur_dev].dev->setChipSelect(true);
        return true;
    }
}

bool SpiBus::release(int dev_id, bool auto_cs) {
    if (cur_dev != dev_id)
        return false;
    else {
        if (auto_cs)
            devices[cur_dev].dev->setChipSelect(false);
        cur_dev = -1;
        return true;
    }
}

bool SpiBus::transmit(uint8_t *data, uint16_t size, uint32_t timeout) {
    if (HAL_OK == HAL_SPI_Transmit(&_hspi, data, size, timeout))
        return true;
    else
        return false;
}

bool SpiBus::transmitViaDma(uint8_t *data, uint16_t size) {
    if (HAL_OK == HAL_SPI_Transmit_DMA(&_hspi, data, size))
        return true;
    else
        return false;
}

bool SpiBus::transceive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size,
                        uint32_t timeout) {
    if (HAL_OK ==
        HAL_SPI_TransmitReceive(&_hspi, tx_data, rx_data, size, timeout))
        return true;
    else
        return false;
}

bool SpiBus::isReadyToSend() {
    return (HAL_SPI_GetState(&_hspi) == HAL_SPI_STATE_READY);
}

/*void SpiBus::HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi_) {
    if (hspi_ != &_hspi)
        return;
}*/
