/*
 * ControlPilot.h
 *
 *  Created on: 15.10.2021
 *  Author: cornelius
 *
 *  IEC 61851-1 compliant Control Pilot state machine
 *
 * This class provides:
 *  1) PWM out for CP +-12V signal to indicate max charge current (via HAL)
 *  2) PWM in to read back answer from car (via HAL)
 *  3) state machine for all CP states, driven by 1) the car and 2) externally
 *     by charger class 4) PP signal reading to determine cable max current if
 *     enabled
 *  4) controls power switches, motor locks and RCD
 *
 *  TODO:
 *  - implement PP reading
 *  - verify connector lock/unlocking
 *  - Run RCD self testing before charging? This is currently disabled as
 *    it was not working when 400V is connected.
 *  - Event PermanentFault is currently never sent.
 */

#ifndef SRC_EVDRIVERS_CONTROLPILOT_H_
#define SRC_EVDRIVERS_CONTROLPILOT_H_

#include "ControlPilot_HAL.h"
#include "PowerSwitch.h"
#include "Rcd.h"
#include "cmsis_os.h"
#include "main.h"
#include <queue>

class ControlPilot {
public:
    ControlPilot(ControlPilot_HAL &_control_pilot_hal,
                 PowerSwitch &_powerSwitch, Rcd &_rcd);
    virtual ~ControlPilot();

    void setThreePhases(bool n);
    bool getThreePhases();
    bool getThreePhasesConfirmed();

    void setHasVentilation(bool v);
    bool getHasVentilation();

    float getResidualCurrent();
    bool getPwmRunning();
    bool getEvSimplifiedMode();
    bool getRcdReclosingAllowed();
    float getCPHi();
    float getCPLo();
    float getSupply12V();
    float getSupplyN12V();

    // reqd for local regulations in IEC norm
    void setCountryCode(const char *iso3166_alpha2_code);

    void enable();
    void disable();

    // trigger replug sequence while charging to switch number of phases
    bool switchThreePhasesWhileCharging(bool n);

    bool getVentilatedChargingActive();
    bool isPowerOn();

    enum class CPState { Disabled, A, B, C, D, E, F, DF };
    CPState getCurrentState();

    enum class Event {
        CarPluggedIn,
        CarRequestedPower,
        PowerOn,
        PowerOff,
        CarRequestedStopPower,
        CarUnplugged,
        Error_E,
        Error_DF,
        Error_Relais,
        Error_RCD,
        Error_VentilationNotAvailable,
        Error_OverCurrent,
        EnterBCD,
        LeaveBCD,
        PermanentFault,
        EvseReplugStarted,
        EvseReplugFinished
    };

    void pwmOn(float dc);
    void pwmOff();
    void pwmF();
    void replug(unsigned int t);
    float getPwmDutyCycle();
    // Allow to power on relais. The ControlPilot class decides when to actually
    // switch on and off. Switiching off is always allowed independent of this
    // setting. The flag will be reset e.g. if car unplugs or if pwmOff()/pwmF()
    // is called
    void allowPowerOn(bool p);

    void forceUnlock();

    void rcdEnable();
    void rcdDisable();
    bool getRcdEnabled();

    void enableSimulation(bool e);
    bool simulationEnabled();
    void setSimulationData(float _sim_CPVoltage, float _sim_ppResistor,
                           bool _sim_diodeFail, bool _sim_errorE,
                           float _sim_rcdCurrent);

    std::queue<ControlPilot::Event> runStateMachine();
    const char *eventToString(ControlPilot::Event);
    const char *currentStateToString();

    void setOverCurrent(bool o, uint32_t timeout);

private:
    // References to external objects we use here
    ControlPilot_HAL &control_pilot_hal;
    PowerSwitch &powerSwitch;
    Rcd &rcd;

    CPState currentState;
    CPState lastState;

    bool teslaFilter_readFromCar(CPState *cp);
    CPState tesla_filter_last_cp{CPState::Disabled};
    uint32_t teslaTick{0};
    bool teslaTickStarted{false};

    bool tesla_last_pwm_running{false};
    uint32_t teslaPwmTimer{0};
    bool teslaPwmTimerStarted{false};

    bool readFromCar(CPState *cp);

    bool powerOn(std::queue<ControlPilot::Event> &events, bool threePhase);
    bool powerOff(std::queue<ControlPilot::Event> &events);
    void checkLock();

    void checkOverCurrent(std::queue<ControlPilot::Event> &events);

    bool overcurrent;
    uint32_t ocTimeoutTick;
    uint32_t ocTimeout;

    bool isVoltageInRange(float voltage, float center);

    void startTimer(uint32_t msecs);
    bool timerElapsed();
    uint32_t timerCountdown;
    uint32_t timerTick;

    bool simulation;
    float sim_CPVoltage;
    float sim_ppResistor;
    bool sim_diodeFail;
    bool sim_errorE;
    float sim_rcdCurrent;

    float cpHi;
    float cpLo;

    bool useThreePhase;
    bool useThreePhaseConfirmed;
    bool hasVentilation;
    bool ventilatedChargingActive;
    bool rcdReclosingAllowed;
    bool evSimplifiedMode;

    bool replugging_in_progress;
    bool replugging_in_progress_F;
    bool replugging_in_progress_HI;
    bool replugging_start;
    int replugging_timer_F;
    int replugging_timer_HI;

    uint32_t lockSwOffTick;
    float pwmDutyCycle;

    bool pwmRunning;
    bool lastPwmRunning;
    bool powerOnAllowed;
};

#endif // SRC_EVDRIVERS_CONTROLPILOT_H_
