//
// ADE7978.h
//
//  Created on: Mar 19, 2021
//      Author: aw@pionier-manufaktur.de
// TODO: implement overcurrent/overvoltage detection once interrupt is connected
// on new HW.
//       this can be used to protect relais if switching onto short circuit
//

#ifndef SRC_EVDRIVERS_ADE7978_H_
#define SRC_EVDRIVERS_ADE7978_H_

#include "EVConfig.h"
#include "Gpio.h"
#include "SpiDevice.h"
#include "InterruptBase.h"
#include "PowerSwitch.h"

#define _USE_MATH_DEFINES // for C++
#include <cmath>

class ADE7978 : public SpiDevice, private InterruptBase {
public:
    ADE7978(Gpio &meter_cs, Gpio &_int1, PowerSwitch *_powerSwitch);
    void softReset();
    // FIXME: what type of setups do will we support?
    void setup();

    enum Line {
        L1 = 0,
        L2 = 1,
        L3 = 2,
        N = 3,
    };

    uint32_t getVRMS(Line line);
    uint32_t getTEMP(Line line);
    uint32_t getIRMS(Line line);
    int32_t getWATTHR(Line line);
    int32_t getWATT(Line line);
    uint16_t getPERIOD(Line line);
    uint32_t getSTATUS1();
    int32_t getTotalWATTHR();
    void resetWATTHR();

    void set_over_current_limit(float hard_limit);
    virtual void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

    // voltage ratio:
    // 1. voltage divider is 1 : 990 on R0, 1 : 801 on R1
    // 2. 0.5V adc (for voltage) input equals 5320000
#if YETI_RELEASE == 0
    static constexpr double VOLTAGE_PEAK = 990 * 0.5;
#elif YETI_RELEASE == 1
    static constexpr double VOLTAGE_PEAK = 801 * 0.5;
#endif
    static constexpr double VOLTAGE_UNIT = VOLTAGE_PEAK / 5320000;
    static constexpr double VOLTAGE_FULLSCALE = VOLTAGE_PEAK / M_SQRT2;
    static constexpr double VOLTAGE_NOMINAL_230 = 230;

    // current ratio:
    // 1. shunt is 0.0005 Ohm
    // 2. 31.25mV adc (for current) input equals 5320000
    static constexpr double CURRENT_PEAK = 0.03125 / 0.0005;
    static constexpr double CURRENT_UNIT = CURRENT_PEAK / 5320000.;
    static constexpr double CURRENT_FULLSCALE = CURRENT_PEAK / M_SQRT2;

    // power ratio:
    // WTHR = PMAX * f_s * 3600 * 10^n / (V_FS * I_FS * 2^27)
    // default setting for WTHR is 3
    static constexpr double DEFAULT_ENERGY_UNIT =
        (3 * VOLTAGE_FULLSCALE * CURRENT_FULLSCALE * (0x1 << 27)) /
        (26991271 * 1.024e6 * 3600);

    // period ratio:
    // T = xPERIOD[15:0]/256E3 (sec)
    static constexpr float PERIOD_UNIT = 1 / 256000.;

    static constexpr float TEMPERATURE_CELSIUS_SCALE = 8.72101e-5;
    static constexpr float TEMPERATURE_CELSIUS_OFFSET = -306.47;

    static constexpr float DEFAULT_POWER_UNIT =
        16. * VOLTAGE_FULLSCALE * CURRENT_FULLSCALE / 26991271.;

protected:
    void initialize();
    virtual void setChipSelect(bool enable);

private:
    Gpio meter_cs;
    Gpio int1;
    PowerSwitch *powerSwitch;

    static const uint32_t ADE7978_MAX_SPI_FREQ = 2500000;

    void _calibrate_debug();

    // private buffer for reading and writing commands
    uint8_t _spi_tx[8];
    uint8_t _spi_rx[8];

    //
    // helper functions
    //
    // setting up first 3 bytes for read or write @ addr
    void _build_spi_cmd(bool read, uint16_t addr);

    // reading
    uint8_t _read_reg8(uint16_t addr);
    uint16_t _read_reg16(uint16_t addr);
    uint32_t _read_reg32(uint16_t addr);

    // writing
    void _write_reg8(uint16_t addr, uint8_t value);
    void _write_reg16(uint16_t addr, uint16_t value);
    void _write_reg32(uint16_t addr, uint32_t value);

    // bit setting in registers
    // NOTE: simple function overloading might be also enough
    void _set_bit(uint16_t reg, uint8_t msk);
    void _set_bit(uint16_t reg, uint16_t msk);
    void _set_bit(uint16_t reg, uint32_t msk);
    void _clear_bit(uint16_t reg, uint8_t msk);
    void _clear_bit(uint16_t reg, uint16_t msk);
    void _clear_bit(uint16_t reg, uint32_t msk);
    void _write_bit(uint16_t reg, uint8_t msk, uint8_t val);
    void _write_bit(uint16_t reg, uint16_t msk, uint16_t val);
    void _write_bit(uint16_t reg, uint32_t msk, uint32_t val);

    // register conversion
    static const uint32_t _32zp_24b_u(uint32_t val);
    static const int32_t _32zp_24b_s(uint32_t val);
    static const uint32_t _32zp_28b_u(uint32_t val);
    static const int32_t _32zp_28b_s(uint32_t val);
    static const int32_t _zpse_s(uint32_t val);
    static const uint16_t _16zp_u(uint16_t val);
    static const int32_t _32se_s(uint32_t val);
    static const uint32_t _32_u(uint32_t val);
    static const int32_t _32_s(uint32_t val);
    static const uint16_t _16_u(uint16_t val);
    static const int16_t _16_s(uint16_t val);
    static const uint8_t _8_u(uint8_t val);
    static const int8_t _8_s(uint8_t val);
};

#endif // SRC_EVDRIVERS_ADE7978_H_
