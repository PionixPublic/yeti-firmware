/*
 * Charger.h
 *
 *  Created on: 08.03.2021
 *  Author: cornelius
 *
 *  IEC 61851-1 compliant AC/DC high level charging logic
 *
 * This class provides:
 *  1) Hi level state machine that is controlled by a) events from ControlPilot
 class
 *     and b) by external commands from higher levels
 *  2) HIL (part of the HIL is forwarded to ControlPilot)
 *
 * The state machine runs in its own (big) task. Once configured it can be run
 * in controlmode mode 0: hilevel can observe everything, but charger always
 * accepts any car and starts
 * charging immediately. When car is unplugged, the next charging session can be
 * started immediately.
 *
 * If control mode 1:
 * The charger waits in state WaitingForAuthentication forever. Send
 * Authenticate()
 * from hi level to start charging. After car is unplugged, it waits in
 * ChargingFinished forever (or in an error state if an error happens during
 * charging). Send restart() from hi level to allow a new charging session.
 *
 * Control mode 2: This class is merely a proxy to controlpilot
 * Charger TODOs:
 *  - switchThreePhasesWhileCharging: fix with new HW, switch back to
 *    floating CP signal and reconnect!
 *
 */

#ifndef SRC_EVDRIVERS_CHARGER_H_
#define SRC_EVDRIVERS_CHARGER_H_

#include "cmsis_os.h"
#include "main.h"

#include "ADE7978.h"
#include "Adc.h"
#include "ControlPilot.h"
#include "Display.h"
#include "Gpio.h"
#include "InterruptBase.h"
#include "OsMutexLockGuard.h"
#include "PowerSwitch.h"
#include "Rcd.h"
#include "Task.h"
#include "ControlMode.h"

#include <queue>
class RemoteControl;



class Charger : private InterruptBase, public Task {
public:
    Charger(ControlPilot &_control_pilot, Display *_display,
            ADE7978 &_power_meter);
    virtual ~Charger();

    // Public interface to configure Charger
    //
    // Call anytime also during charging, but call setters in this block at
    // least initially once.
    //

    // external input to charger (thread safe, no ISR)
    bool setMaxCurrent(float ampere);
    float getMaxCurrent();
    void setHardCurrentLimit(float ampere);

    // setThreePhases can be called anytime, but will only become valid at next
    // replug.
    void setThreePhases(bool n);
    bool getThreePhases();
    bool getThreePhasesConfirmed();

    // config option (set anytime)
    void setHasVentilation(bool v);
    bool getHasVentilation();

    // reqd for local regulations in IEC norm
    void setCountryCode(const char *iso3166_alpha2_code);


    bool enable();
    bool disable();
    // switch to next charging session after Finished
    bool restart();

    // Public interface during charging
    //
    // Call anytime, but mostly used during charging session
    //

    // receive keepalive message from hi level.
    void keepAliveFromHi();

    // call when in state WaitingForAuthentication
    void Authorize(bool a, const char *userid);
    bool getAuthorization();

    // trigger replug sequence while charging to switch number of phases
    bool switchThreePhasesWhileCharging(bool n);

    bool pauseCharging();
    bool resumeCharging();
    bool getPausedByEVSE();

    void rcdEnable();
    void rcdDisable();

    bool getVentilatedChargingActive();
    float getResidualCurrent();
    bool isPowerOn();

    // Public states for Hi Level
    enum class EvseState {
        Disabled,
        Idle,
        WaitingForAuthentication,
        Charging,
        ChargingPausedEV,
        ChargingPausedEVSE,
        Finished,
        Error,
        Faulted
    };

    enum class ErrorState {
        Error_E,
        Error_F,
        Error_DF,
        Error_Relais,
        Error_VentilationNotAvailable,
        Error_RCD,
		Error_OverCurrent
    };



    void setControlMode(ControlMode c);
    ControlMode getControlMode();

    // Logical states for Hi Level
    EvseState getCurrentState();
    ErrorState getErrorState();

    // Debug interface
    //
    // Read only access to some private members for debugging purposes via Hi
    // level only. Dont use unless you have to!
    //

    class DebugVars {
    public:
        bool evsePwmRunning;
        bool simplifiedMode;
        bool rcdReclosingAllowed;
        bool endState;
        float cpuTemperature;
        float maxCurrentCable;

        float evseCPHi;
        float evseCPLo;
        float supply12V;
        float supplyN12V;
        float pwmDutyCycle;

        bool rcdEnabled;

        ControlPilot::CPState currentState;

        bool simulation;
    };

    DebugVars getDebugVars();

    struct PowerMeterData {
        float vrms[3];
        float irms[4];
        float wattHr[3];
        float totalWattHr;
        float temp[4];
        float watt[3];
        float freq[3];
        bool phaseSeqError;
    };

    PowerMeterData getPowerMeterData();

    Display *getDisplay();

    // Simulator interface
    void enableSimulation(bool e);
    bool simulationEnabled();

    void setSimulationData(float cpVoltage, float ppResistor, float currents[4],
                           float voltages[3], float freq[3], float rcdCurrent,
                           bool diodeFail, bool errorE);

    // Proxy interface for control_mode == 2 (direct low level control)
    void pwmOn(float dc);
    void pwmOff();
    void pwmF();
    void replug(unsigned int t);
    void allowPowerOn(bool p);
    void forceUnlock();

    void setRemoteControl(RemoteControl *p);

private:
    // References to external objects we use here
    Display *display;
    ADE7978 &power_meter;
    ControlPilot control_pilot;

    // This is set after starting the charger and only needed for controlmode low.
    RemoteControl* remote_control;

    virtual void main();

    float maxCurrent;
    float hardCurrentLimit;
    const uint32_t softOverCurrentTimeout = 5000+1000; // IEC61851-1 A.6 Sequence 6: Car must react within 5 seconds to current change
    const uint32_t hardOverCurrentTimeout = 0;

    void checkOverCurrent();

    // This mutex locks all config type members
    osMutexId_t configMutex;
    const osMutexAttr_t configMutex_attributes = {.name = "configMutex"};

    // This mutex locks all state type members
    osMutexId_t stateMutex;
    const osMutexAttr_t stateMutex_attributes = {.name = "stateMutex"};

    EvseState currentState;
    ErrorState errorState;

    float ampereToDutyCycle(float ampere);

    void evseStateToString(char *str, EvseState s);

    void ISO_IEC_Coordination();

    void updateDisplay();

    void processCPEventsIndependent(ControlPilot::Event cp_event);
    void processCPEventsState(ControlPilot::Event cp_event);
    void runStateMachine();

    ControlMode controlMode;
    bool authorized;

    uint32_t lastPwmUpdate;
    uint32_t lastKeepAliveFromHi;

    void updatePowerMeter();
    float getVrms(ADE7978::Line line);
    float getIrms(ADE7978::Line line);
    float getwattHr(ADE7978::Line line);
    float getTotalWattHr();

    PowerMeterData powerMeterData;

    // simulation variables
    bool simulation;

    float sim_CPVoltage;
    float sim_ppResistor;
    float sim_currents[4];
    float sim_voltages[3];
    float sim_freq[3];
    float sim_rcdCurrent;
    bool sim_diodeFail;
    bool sim_errorE;
    float sim_wattHr[3];
    uint32_t sim_powermeter_lastTick;
    bool oddeven;
};



#endif // SRC_EVDRIVERS_CHARGER_H_
