
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/


#pragma once

#include "stdafx.h"
#include "RemoteDisplayWidget.h"
#define SPEED_DEFAULT_VALUE 20.
#define SPEED_MAX_VALUE 40.
#define SPEED_MIN_VALUE 0.
#define SPEED_INCREMENT_VALUE 5.

#define STEERING_ANGLE_DEFAULT_VALUE 50.
#define STEERING_ANGLE_MAX_VALUE 90.
#define STEERING_ANGLE_MIN_VALUE 0.
#define STEERING_ANGLE_INCREMENT_VALUE 10.
#define KEY_STOP_ALL 32  // space bar
#define KEY_INCREASE_SPEED 82  // R
#define KEY_DECREASE_SPEED 70  // F
#define KEY_INCREASE_ANGLE 69  // E
#define KEY_DECREASE_ANGLE 81  // Q
#define KEY_TOGGLE_LIGHTS 76  // L
#define KEY_TOGGLE_SIREN 72  // H

#define CID_KEYBOARD_REMOTE_FILTER "keyboard_remote.filter.user.aadc.cid"
#define LABEL_KEYBOARD_REMOTE_FILTER "KeyboardRemoteFilter"
#define KEYBOARD_DISABLED_MSG "keyboard control disabled"
class RemoteDisplayWidget;

class KeyboardRemoteFilter : public QObject, virtual public cQtUIFilter {
    Q_OBJECT
 public:
    ADTF_CLASS_ID_NAME(KeyboardRemoteFilter, CID_KEYBOARD_REMOTE_FILTER,
        LABEL_KEYBOARD_REMOTE_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem),
        REQUIRE_INTERFACE(adtf::services::IReferenceClock));
    object_ptr<IStreamType> pTypeSignalValue;

 public slots:
    void keyboardEnabledTrue();
    void keyboardEnabledFalse();

    void keyCmd(int k);

    void keyDriveFwdPressed();
    void keyDriveFwdReleased();

    void keyDriveBwdPressed();
    void keyDriveBwdReleased();

    void keyDriveLeftPressed();
    void keyDriveLeftReleased();

    void keyDriveRightPressed();
    void keyDriveRightReleased();

    void commandResetAll();

 private:
    tBool m_doKeyboardRemoteFilter;
    void driveFwdStart();
    void driveFwdStop();

    void driveBwdStart();
    void driveBwdStop();

    void driveLeftStart();
    void driveLeftStop();

    void driveRightStart();
    void driveRightStop();

    void resetAll();

 private:
    property_variable<tBool> m_propEnableConsoleOutput = tTrue;
    property_variable<tBool> m_propSetMaxSpeed = tFalse;
    property_variable<tBool> m_propMetricSpeedOutput = tTrue;
    property_variable<tFloat32> m_propMaxSpeed = 40;
    struct tSignalValueId {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;
    struct tBoolSignalValueId {
        tSize ui32ArduinoTimestamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;

    cPinWriter m_oWriterSpeed;
    cPinWriter m_oWriterSteering;
    cPinWriter m_oWriterLights;
    cPinWriter m_oWriterHazard;
    RemoteDisplayWidget* m_pWidget;
    object_ptr<adtf::services::IReferenceClock> m_pClock;

 public:
    KeyboardRemoteFilter();
    virtual ~KeyboardRemoteFilter();

 protected:
    bool keyboard_enabled;
    bool speed_front_enabled;
    bool speed_back_enabled;
    bool steer_left_enabled;
    bool steer_right_enabled;
    bool headlights_enabled;
    bool hazard_enabled;
    tFloat32 speed_target;
    tFloat32 steering_target;
    tFloat32 speed_actual;
    tFloat32 steering_actual;
    tSignalValue speed_signal;
    tSignalValue steering_signal;
    QWidget * CreateView() override;
    tVoid      ReleaseView() override;
    tResult Init(tInitStage eStage) override;
    tResult OnTimer() override;
    tResult Shutdown(cFilterLevelmachine::tInitStage eStage) override;
    tResult transmitSpeed(tFloat32 speedSignal);
    tResult transmitSteering(tFloat32 steerSignal);
    tResult toggleLights();
    tResult toggleHazard();
};
