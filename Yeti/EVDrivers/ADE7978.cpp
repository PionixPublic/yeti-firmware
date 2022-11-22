//
// ADE7978.cpp
//
//  Created on: Mar 19, 2021
//      Author: aw@pionier-manufaktur.de
//

#include "ADE7978.h"
#include "ADE7978_priv.h"

#include "cmsis_os2.h"
#include "main.h"

#include <cstdio>

ADE7978::ADE7978(Gpio &meter_cs_, Gpio &_int1, PowerSwitch *_powerSwitch) :
    SpiDevice(ADE7978_MAX_SPI_FREQ),
    meter_cs(meter_cs_),
    int1(_int1),
    powerSwitch(_powerSwitch) {
    setChipSelect(false);
}

void ADE7978::initialize() {
    acquireSpiBus();
    /* 1. Monitor the IRQ1 pin until it goes low, indicating that the
    RSTDONE interrupt is triggered.*/
    // FIXME: we assume we waited long enough here already...

    /* 2. When the ADE7978 starts functioning after power-up, the
    I2C port is the active serial port. If SPI communication is
    to be used, toggle the SS/HSA pin three times from high to
    low to select the SPI interface.
    */

    // toggle 3 times from high to low to select spi mode
    // after this routine, chip select is still enabled
    for (int i = 0; i < 3; ++i) {
        setChipSelect(false);
        osDelay(1);
        setChipSelect(true);
        osDelay(1);
    }

    releaseSpiBus();
}

void ADE7978::softReset() {
    // we don't want the chip select initially activated
    acquireSpiBus(false);

    // initiate SW reset
    _write_reg16(ade7978::CONFIG, ade7978::CONFIG_SWRST);

    /* 3. Read the STATUS1 register (Address 0xE503) to verify that
    Bit 15 (RSTDONE) is set to 1, and then write a 1 to the bit
    to clear it. The IRQ1 pin returns high. Because RSTDONE
    is an unmaskable interrupt, Bit 15 (RSTDONE) must be
    reset to 0 for the IRQ1 pin to return high.
    It is recommended that all other flags in the STATUS1 and
    STATUS0 registers also be reset by writing a 1 to all bits in
    the registers. */

    // poll on RSTDONE
    while (!(_read_reg32(ade7978::STATUS1) & ade7978::STATUS1_RSTDONE_Msk))
        ;

    // write 1 to all bits in STATUS0 and STATUS1
    _write_reg32(ade7978::STATUS0, 0xFFFFFFFF);
    _write_reg32(ade7978::STATUS1, 0xFFFFFFFF);

    // check CRC
    if (_read_reg32(ade7978::CHECKSUM) != ade7978::CHECKSUM_RESET_VAL) {
        // FIXME: should not happen
        while (1)
            ;
    }

    /* 4. If SPI communication is used, lock the port by writing any
    value to the CONFIG2 register. */
    // lock SPI, 'd' is for dummy
    _write_reg8(ade7978::CONFIG2, 'd');

    releaseSpiBus();
}

void ADE7978::setup() {
    // we don't want the chip select initially activated
    acquireSpiBus(false);

    /****************************************************************************
    5. Initialize the AIGAIN, BIGAIN, CIGAIN, and NIGAIN
    registers (Address 0x4380, Address 0x4383, Address 0x4386,
    and Address 0x4389, respectively).
    */
    // TODO implement me

    /****************************************************************************
    6. Start the DSP by writing 0x0001 to the run register (Address 0xE228).
    */
    _write_reg16(ade7978::RUN, ade7978::RUN_VAL_START);

    /****************************************************************************
    7. Initialize the DSP RAM-based registers located at
    Address 0x4380 to Address 0x43BF. Write the last register
    in the queue three times.
    */

#if 0
    // do some calibration:
    // FIXME: this is solely based on aw's board
    _write_reg32(ade7978::AVRMSOS, -3489);
    _write_reg32(ade7978::AIRMSOS, -24120);
#endif

    // set VLEVEL according to
    //    VLEVEL = V_FS/V_n * 4 * 10^6
    constexpr uint32_t v_level =
        static_cast<uint32_t>(VOLTAGE_FULLSCALE / VOLTAGE_NOMINAL_230) * 4 *
        1e6;
    _write_reg32(ade7978::VLEVEL, v_level);
    _write_reg32(ade7978::VLEVEL, v_level);
    _write_reg32(ade7978::VLEVEL, v_level);

    // set VNOM
    //    VNOM is not used by default, so we don't care about it by now

    /****************************************************************************
    8. Initialize the hardware-based configuration registers
    located at Address 0xE507 to Address 0xEA04 with the
    exception of the CFMODE register (see Step 11).
    */

    // select frequency (50Hz)
    _write_bit(ade7978::COMPMODE, ade7978::COMPMODE_SELFREQ_Msk,
               ade7978::COMPMODE_SELFREQ_50HZ);

    // INSEL: NIRMS register does not contain neutral current but sum of 3 phase
    // currents
    //_write_bit(ade7978::CONFIG, ade7978::CONFIG_INSEL_Msk,
    // ade7978::CONFIG_INSEL);

    // ZX_READY_PHASEA: output phase A Zero crossings on ZX/DREADY pin instead
    // of DREADY signal
    _write_bit(ade7978::CONFIG, ade7978::CONFIG_ZX_DREADY_Msk,
               ade7978::CONFIG_ZX_DREADY_PHASEA);
    /*
     * On the ADE7978, the selection of the second voltage channel
     * or the temperature sensor is based on Bits[3:0] (VN2_EN,
     * VC2_EN, VB2_EN, and VA2_EN) in the CONFIG3 register
     * (Address 0xE708).
     * Enable temperature reading: */
    _write_reg8(ade7978::CONFIG3, 0x00);
    // disable RSTREAD, so we do not have to keep book for accumulation right
    // now
    _clear_bit(ade7978::LCYCMODE, ade7978::LCYCMODE_RSTREAD_Msk);

    /****************************************************************************
    9. Enable the DSP RAM write protection by writing 0xAD to the
    internal 8-bit register located at Address 0xE7FE. Then write
    0x80 to the internal 8-bit register located at Address 0xE7E3.*/
    //    first write 0xAD to 0xE7FE
    //    second write 0x80 to 0xE7E3
    // FIXME: factor out to a special routine
    // FIXME2: we don't do this, so we don't have to unlock if we do not repower
#if 0
    _write_reg8(0xE7FE, 0xAD);
    _write_reg8(0xE7E3, 0x80);
#endif

    /****************************************************************************
     10. Read the energy registers xWATTHR, xVARHR,
    xFWATTHR, xFVARHR, and xVAHR to erase their contents
    and start energy accumulation from a known state.
    */

    // FIXME OR: set WTHR register
    //    WTHR = PMAX * f_s * 3600 * 10^n / (V_FS * I_FS * 2^27)
    //    default value is 3, leave it for now
    //    this gives accumulated energy in units of DEFAULT_POWER_UNIT

    /****************************************************************************
    11. Clear Bit 9 (CF1DIS), Bit 10 (CF2DIS), and Bit 11 (CF3DIS)
    in the CFMODE register (Address 0xE610) to enable pulses
    at the CF1, CF2, and CF3 pins
    */
    // TODO: implement for external calibration purposes

    /****************************************************************************
    12. Read back all ADE7978 registers to ensure that they are
    initialized with the desired values.
    */

#if 0
    _calibrate_debug();

    while (true) {
        printf("AVRMS: %f\AIRMSV: %f\tAWATTHR: %f\n",
               _32zp_24b_u(_read_reg32(ade7978::AVRMS)) * VOLTAGE_UNIT,
               _32zp_24b_u(_read_reg32(ade7978::AIRMS)) * CURRENT_UNIT,
               _32_s(_read_reg32(ade7978::AWATTHR)) * DEFAULT_ENERGY_UNIT);
        osDelay(1000);
    }
#endif

    releaseSpiBus();
}

uint32_t ADE7978::getVRMS(Line line) {
    if (line == Line::N)
        return 0;

    // we don't want the chip select initially activated
    acquireSpiBus(false);

    const uint32_t reg =
        _read_reg32(ade7978::AVRMS + 3 * static_cast<uint16_t>(line));
    releaseSpiBus();

    return _32zp_24b_s(reg);
}

uint32_t ADE7978::getIRMS(Line line) {
    // we don't want the chip select initially activated
    acquireSpiBus(false);
    const uint32_t reg =
        _read_reg32(ade7978::AIRMS + 3 * static_cast<uint16_t>(line));
    releaseSpiBus();

    return _32zp_24b_s(reg);
}

uint32_t ADE7978::getTEMP(Line line) {
    // we don't want the chip select initially activated
    acquireSpiBus(false);
    const uint32_t reg =
        _read_reg32(ade7978::ATEMP + static_cast<uint16_t>(line));
    releaseSpiBus();
    return _32zp_24b_s(reg);
}

int32_t ADE7978::getWATTHR(Line line) {
    if (line == Line::N)
        return 0;

    // we don't want the chip select initially activated
    acquireSpiBus(false);
    const uint32_t reg =
        _read_reg32(ade7978::AWATTHR + static_cast<uint16_t>(line));
    releaseSpiBus();

    return _32_s(reg);
}

int32_t ADE7978::getTotalWATTHR() {
    // we don't want the chip select initially activated
    acquireSpiBus(false);
    const int32_t reg = _32_s(_read_reg32(ade7978::AWATTHR)) +
                        _32_s(_read_reg32(ade7978::BWATTHR)) +
                        _32_s(_read_reg32(ade7978::CWATTHR));
    releaseSpiBus();

    return reg;
}

int32_t ADE7978::getWATT(Line line) {
    if (line == Line::N)
        return 0;

    // we don't want the chip select initially activated
    acquireSpiBus(false);
    const uint32_t reg =
        _read_reg32(ade7978::AWATT + static_cast<uint16_t>(line));
    releaseSpiBus();

    return _32se_s(reg);
}

uint32_t ADE7978::getSTATUS1() {
    // we don't want the chip select initially activated
    acquireSpiBus(false);
    const uint32_t reg = _read_reg32(ade7978::STATUS1);
    releaseSpiBus();

    return _32_u(reg);
}

uint16_t ADE7978::getPERIOD(Line line) {
    if (line == Line::N)
        return 0;
    // we don't want the chip select initially activated
    acquireSpiBus(false);
    const uint16_t reg =
        _read_reg16(ade7978::APERIOD + static_cast<uint16_t>(line));
    releaseSpiBus();
    return _16_u(reg);
}

void ADE7978::resetWATTHR() {
    // we don't want the chip select initially activated
    acquireSpiBus(false);

    _set_bit(ade7978::LCYCMODE, ade7978::LCYCMODE_RSTREAD_Msk);

    _read_reg32(ade7978::AWATTHR);
    _read_reg32(ade7978::BWATTHR);
    _read_reg32(ade7978::CWATTHR);

    _clear_bit(ade7978::LCYCMODE, ade7978::LCYCMODE_RSTREAD_Msk);

    releaseSpiBus();
}

void ADE7978::_calibrate_debug() {
    // just read the rms values and average, to get an offset
    const int iterations = 64;
    const int delay = 41;

    uint32_t volt_acc = 0;
    uint32_t curr_acc = 0;

    for (int i = 0; i < iterations; ++i) {
        osDelay(delay);
        volt_acc += _32zp_24b_u(_read_reg32(ade7978::AVRMS));
        curr_acc += _32zp_24b_u(_read_reg32(ade7978::AIRMS));
    }

    // power calibration needs to be done differently

#ifdef ADE7978_CPP_ENABLE_PRINTF
    uint32_t volt_os = pow(volt_acc / (double)iterations, 2) / 128; // 3489
    uint32_t curr_os = pow(curr_acc / (double)iterations, 2) / 128; // 24120
    printf("volt_os: %f\t curr_os: %f\n", volt_os * VOLTAGE_UNIT,
           curr_os * CURRENT_UNIT);
#endif
}

/* hard_limit in ampere RMS */
void ADE7978::set_over_current_limit(float hard_limit) {
    registerCallback(IntType::HAL_GPIO_EXTI_Callback);

    // we don't want the chip select initially activated
    acquireSpiBus(false);
    // write over current limit to ADE7978
    // OILVL is 24 bit unsigned (max 5 320 000). Transfer as 32bit with 8 MSB
    // padded with 0
    uint32_t hard_limit_24bit =
        hard_limit / CURRENT_UNIT *
        M_SQRT2; // note this is peak current and not rms, hence a factor of
                 // sqrt2 for the limit
    if (hard_limit_24bit >= 5320000)
        hard_limit_24bit = 5320000 - 2;
    // printf("Setting hard limit to %u\n", hard_limit_24bit);
    _write_reg32(ade7978::OILVL, hard_limit_24bit);

    // Set bit 17 (OI) in MASK1 register to enable IRQ1# (pg 52)
    _set_bit(ade7978::MASK1, (uint32_t)(1 << 17));

    releaseSpiBus();
}

void ADE7978::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // printf("INT triggered %i\n", GPIO_Pin);
    //  GPIO_EXTI2 (PB2 PIN) on Yeti R1
    if (GPIO_Pin == int1.getPin()) {
        // in interrupt read STATUS1 register and verify that bit 17 is set to 1
        // to ensure interrupt was caused by overcurrent.
        // printf("query status1\n");
        if (getSTATUS1() & ade7978::STATUS1_OI_Msk) {
            // printf("currents %f %f %f %f\n", getIRMS(Line::L1) *
            // CURRENT_UNIT,
            //        getIRMS(Line::L2) * CURRENT_UNIT,
            //        getIRMS(Line::L3) * CURRENT_UNIT,
            //        getIRMS(Line::N) * CURRENT_UNIT);
            //  emergency switch off relais
            // printf("off\n");
            if (powerSwitch)
                powerSwitch->emergencySwitchOff();
            // Write a 1 to Bit 17
            // (OI) in the STATUS1 register to clear Bit 17 and Bits[5:3]
            // (OIPHASE[2:0]) of the PHSTATUS register. The IRQ1 interrupt pin
            // returns high.
            // we don't want the chip select initially activated
            acquireSpiBus(false);
            // printf("clar\n");
            _set_bit(ade7978::STATUS1, (uint32_t)(1 << 17));
            _set_bit(ade7978::PHSTATUS,
                     (uint32_t)((1 << 3) | (1 << 4) | (1 << 5)));
            releaseSpiBus();
            // printf("released\n");
        }
    }
}

void ADE7978::setChipSelect(bool enable) {
    (enable) ? meter_cs.reset() : meter_cs.set();
}

void ADE7978::_build_spi_cmd(bool read, uint16_t addr) {
    _spi_tx[0] = read ? 0x01 : 0x00;
    _spi_tx[1] = static_cast<uint8_t>(addr >> 8);
    _spi_tx[2] = static_cast<uint8_t>(addr);
}

uint8_t ADE7978::_read_reg8(uint16_t addr) {
    setChipSelect(true);
    _build_spi_cmd(true, addr);

    // 1 byte r/w, 2 bytes address, 1 byte read data = 4 bytes
    spiTransceive(_spi_tx, _spi_rx, 4);

    setChipSelect(false);
    return _spi_rx[3];
}

uint16_t ADE7978::_read_reg16(uint16_t addr) {
    setChipSelect(true);
    _build_spi_cmd(true, addr);

    // 1 byte r/w, 2 bytes address, 2 byte read data = 5 bytes
    spiTransceive(_spi_tx, _spi_rx, 5);

    setChipSelect(false);
    return (_spi_rx[3] << 8) | (_spi_rx[4]);
}

uint32_t ADE7978::_read_reg32(uint16_t addr) {
    setChipSelect(true);
    _build_spi_cmd(true, addr);

    // 1 byte r/w, 2 bytes address, 4 byte read data = 7 bytes
    spiTransceive(_spi_tx, _spi_rx, 7);

    setChipSelect(false);
    return (_spi_rx[3] << 24) | (_spi_rx[4] << 16) | (_spi_rx[5] << 8) |
           (_spi_rx[6]);
}

void ADE7978::_write_reg8(uint16_t addr, uint8_t value) {
    setChipSelect(true);
    _build_spi_cmd(false, addr);
    _spi_tx[3] = value;

    // 1 byte r/w, 2 bytes address, 1 byte data = 4 bytes
    spiTransmit(_spi_tx, 4);

    setChipSelect(false);
}

void ADE7978::_write_reg16(uint16_t addr, uint16_t value) {
    setChipSelect(true);
    _build_spi_cmd(false, addr);
    _spi_tx[3] = static_cast<uint8_t>(value >> 8);
    _spi_tx[4] = static_cast<uint8_t>(value);

    // 1 byte r/w, 2 bytes address, 2 byte data = 5 bytes
    spiTransmit(_spi_tx, 5);

    setChipSelect(false);
}

void ADE7978::_write_reg32(uint16_t addr, uint32_t value) {
    setChipSelect(true);
    _build_spi_cmd(false, addr);
    _spi_tx[3] = static_cast<uint8_t>(value >> 24);
    _spi_tx[4] = static_cast<uint8_t>(value >> 16);
    _spi_tx[5] = static_cast<uint8_t>(value >> 8);
    _spi_tx[6] = static_cast<uint8_t>(value);

    // 1 byte r/w, 2 bytes address, 4 bytes data = 7 bytes
    spiTransmit(_spi_tx, 7);

    setChipSelect(false);
}

void ADE7978::_set_bit(uint16_t addr, uint8_t msk) {
    uint8_t reg = _read_reg8(addr);
    reg |= msk;
    _write_reg8(addr, reg);
}

void ADE7978::_set_bit(uint16_t addr, uint16_t msk) {
    uint16_t reg = _read_reg16(addr);
    reg |= msk;
    _write_reg16(addr, reg);
}

void ADE7978::_set_bit(uint16_t addr, uint32_t msk) {
    uint32_t reg = _read_reg32(addr);
    reg |= msk;
    _write_reg32(addr, reg);
}

void ADE7978::_clear_bit(uint16_t addr, uint8_t msk) {
    uint8_t reg = _read_reg8(addr);
    reg &= ~msk;
    _write_reg8(addr, reg);
}

void ADE7978::_clear_bit(uint16_t addr, uint16_t msk) {
    uint16_t reg = _read_reg16(addr);
    reg &= ~msk;
    _write_reg16(addr, reg);
}

void ADE7978::_clear_bit(uint16_t addr, uint32_t msk) {
    uint32_t reg = _read_reg32(addr);
    reg &= ~msk;
    _write_reg32(addr, reg);
}

void ADE7978::_write_bit(uint16_t addr, uint8_t msk, uint8_t val) {
    uint8_t reg = _read_reg8(addr);
    reg &= ~msk;
    reg |= val;
    _write_reg8(addr, reg);
}

void ADE7978::_write_bit(uint16_t addr, uint16_t msk, uint16_t val) {
    uint16_t reg = _read_reg16(addr);
    reg &= ~msk;
    reg |= val;
    _write_reg16(addr, reg);
}

void ADE7978::_write_bit(uint16_t addr, uint32_t msk, uint32_t val) {
    uint32_t reg = _read_reg32(addr);
    reg &= ~msk;
    reg |= val;
    _write_reg32(addr, reg);
}

const uint32_t ADE7978::_32zp_24b_u(uint32_t val) { return val; }

// 32 ZP = 28- or 24-bit signed or unsigned register that is transmitted as a
// 32-bit word with four MSBs or eight MSBs, respectively, padded with 0s.
const int32_t ADE7978::_32zp_24b_s(uint32_t val) {
    const uint32_t tmp = (val & (1 << 23)) ? val | 0xFF000000 : val;
    return static_cast<int32_t>(tmp);
}

const uint32_t ADE7978::_32zp_28b_u(uint32_t val) { return val; }

const int32_t ADE7978::_32zp_28b_s(uint32_t val) {
    const uint32_t tmp = (val & (1 << 27)) ? val | 0xF0000000 : val;
    return static_cast<int32_t>(tmp);
}

// 32 ZPSE = 24-bit signed register that is transmitted as a 32-bit word with
// four MSBs padded with 0s and sign extended to 28 bits
const int32_t ADE7978::_zpse_s(uint32_t val) {
    const uint32_t tmp = (val & (1 << 27)) ? val | 0xF0000000 : val;
    return static_cast<int32_t>(tmp);
}

const uint16_t ADE7978::_16zp_u(uint16_t val) { return val; }

const int32_t ADE7978::_32se_s(uint32_t val) {
    return static_cast<int32_t>(val);
}

const uint32_t ADE7978::_32_u(uint32_t val) { return val; }

const int32_t ADE7978::_32_s(uint32_t val) { return static_cast<int32_t>(val); }

const uint16_t ADE7978::_16_u(uint16_t val) { return val; }

const int16_t ADE7978::_16_s(uint16_t val) { return static_cast<int16_t>(val); }

const uint8_t ADE7978::_8_u(uint8_t val) { return val; }

const int8_t ADE7978::_8_s(uint8_t val) { return static_cast<int8_t>(val); }
