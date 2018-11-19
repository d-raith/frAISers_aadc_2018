
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers, University of Freiburg
******************************************************************************/

#include "KeyboardRemoteFilter.h"
#include "RemoteDisplayWidget.h"
#include "ADTF3_helper.h"
ADTF_PLUGIN(LABEL_KEYBOARD_REMOTE_FILTER, KeyboardRemoteFilter);
KeyboardRemoteFilter::KeyboardRemoteFilter() {
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);
    RegisterPropertyVariable("Override max speed", m_propSetMaxSpeed);
    RegisterPropertyVariable("Max speed [0-100]", m_propMaxSpeed);
    RegisterPropertyVariable("Use metric speed output", m_propMetricSpeedOutput);

    keyboard_enabled = false;
    if (SPEED_DEFAULT_VALUE > SPEED_MAX_VALUE) {
        speed_target = SPEED_MAX_VALUE;
    } else {
        speed_target = SPEED_DEFAULT_VALUE;
    }
    steering_target = STEERING_ANGLE_DEFAULT_VALUE;
    speed_actual = 0;
    steering_actual = 0;
    speed_front_enabled = false;
    speed_back_enabled = false;
    steer_left_enabled = false;
    steer_right_enabled = false;
    headlights_enabled = false;
    hazard_enabled = false;
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
            "tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
            cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
            cString("f32Value"), m_ddlSignalValueId.value));
    } else {
        LOG_INFO("KeyboardRemote: No mediadescription for tSignalValue found!");
    }
    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
            "tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory,
            cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory,
            cString("bValue"), m_ddlBoolSignalValueId.bValue));
    } else {
        LOG_INFO("KeyboardRemote: No mediadescription for tBoolSignalValue found!");
    }
    create_pin(*this, m_oWriterSpeed, "speed", pTypeSignalValue);
    create_pin(*this, m_oWriterSteering, "steering", pTypeSignalValue);
    create_pin(*this, m_oWriterLights, "headlights", pTypeBoolSignalValue);
    create_pin(*this, m_oWriterHazard, "hazard lights", pTypeBoolSignalValue);
}

KeyboardRemoteFilter::~KeyboardRemoteFilter() {}

QWidget* KeyboardRemoteFilter::CreateView() {
    m_pWidget = new RemoteDisplayWidget(nullptr);
    tBool allSignalsConnected = tTrue;
    allSignalsConnected &=
        tBool(connect(m_pWidget->m_pEnableKeyboardButton, SIGNAL(clicked()),
            this, SLOT(keyboardEnabledTrue())));
    allSignalsConnected &=
        tBool(connect(m_pWidget->m_pDisableKeyboardButton, SIGNAL(clicked()),
            this, SLOT(keyboardEnabledFalse())));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(keyReceived(int)), this, SLOT(keyCmd(int))));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(keyFwdPressed()), this, SLOT(keyDriveFwdPressed())));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(keyBwdPressed()), this, SLOT(keyDriveBwdPressed())));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(keyLeftPressed()), this, SLOT(keyDriveLeftPressed())));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(keyRightPressed()), this,
            SLOT(keyDriveRightPressed())));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(keyFwdReleased()), this, SLOT(keyDriveFwdReleased())));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(keyBwdReleased()), this, SLOT(keyDriveBwdReleased())));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(keyLeftReleased()), this,
            SLOT(keyDriveLeftReleased())));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(keyRightReleased()), this,
            SLOT(keyDriveRightReleased())));
    allSignalsConnected &=
        tBool(connect(m_pWidget, SIGNAL(widgetOutOfFocus()), this,
            SLOT(commandResetAll())));

    if (!allSignalsConnected) {
        LOG_ERROR("KeyboardRemote: not all signals connected to their slots");
    } else {
        LOG_INFO("KeyboardRemote: all signals connected");
    }
    return m_pWidget;
}

tVoid KeyboardRemoteFilter::ReleaseView() {
    delete m_pWidget;
    m_pWidget = nullptr;
}

tResult KeyboardRemoteFilter::Init(tInitStage eStage) {
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    if (m_propSetMaxSpeed) {
        #undef SPEED_MAX_VALUE
        #define SPEED_MAX_VALUE m_propMaxSpeed
    }
    RETURN_NOERROR;
}

tResult KeyboardRemoteFilter::Shutdown(cFilterLevelmachine::tInitStage eStage) {
    return cQtUIFilter::Shutdown(eStage);
}
tResult KeyboardRemoteFilter::OnTimer() {
    RETURN_NOERROR;
}
void KeyboardRemoteFilter::keyCmd(int key) {
    if (keyboard_enabled) {
        switch (key) {
            case KEY_STOP_ALL:
                LOG_WARNING("KeyboardRemote: EMERGENCY STOP");
                KeyboardRemoteFilter::resetAll();
                break;

            case KEY_INCREASE_SPEED:
                if (speed_target + SPEED_INCREMENT_VALUE <= SPEED_MAX_VALUE) {
                    speed_target += SPEED_INCREMENT_VALUE;
                }
                if (m_propEnableConsoleOutput) LOG_INFO(A_UTILS_NS::cString::Format(
                        "KeyboardRemote: speed value: %f", speed_target));
                driveFwdStart();
                driveBwdStart();
                break;

            case KEY_DECREASE_SPEED:
                if (speed_target - SPEED_INCREMENT_VALUE >= SPEED_MIN_VALUE) {
                    speed_target -= SPEED_INCREMENT_VALUE;
                }
                if (m_propEnableConsoleOutput) LOG_INFO(A_UTILS_NS::cString::Format(
                        "KeyboardRemote: speed value: %f", speed_target));
                driveFwdStart();
                driveBwdStart();
                break;

            case KEY_INCREASE_ANGLE:
                if (steering_target + STEERING_ANGLE_INCREMENT_VALUE
                    <= STEERING_ANGLE_MAX_VALUE) {
                    steering_target += STEERING_ANGLE_INCREMENT_VALUE;
                }
                if (m_propEnableConsoleOutput) LOG_INFO(A_UTILS_NS::cString::Format(
                        "KeyboardRemote: steering value: %f", steering_target));
                driveLeftStart();
                driveRightStart();
                break;

            case KEY_DECREASE_ANGLE:
                if (steering_target - STEERING_ANGLE_INCREMENT_VALUE
                    >= STEERING_ANGLE_MIN_VALUE) {
                    steering_target -= STEERING_ANGLE_INCREMENT_VALUE;
                }
                if (m_propEnableConsoleOutput) LOG_INFO(A_UTILS_NS::cString::Format(
                        "KeyboardRemote: steering value: %f", steering_target));
                driveLeftStart();
                driveRightStart();
                break;

            case KEY_TOGGLE_LIGHTS:
                !headlights_enabled ? headlights_enabled = true : headlights_enabled = false;
                toggleLights();
                if (m_propEnableConsoleOutput) LOG_INFO("KeyboardRemote: lights toggled");
                break;

            case KEY_TOGGLE_SIREN:
                if (m_propEnableConsoleOutput) LOG_INFO("KeyboardRemote: siren toggled");
                break;

            default :
                break;
        }
    } else {
    }
}
void KeyboardRemoteFilter::keyDriveFwdPressed() {
    speed_front_enabled = true;
    driveFwdStart();
}

void KeyboardRemoteFilter::keyDriveFwdReleased() {
    speed_front_enabled = false;
    driveFwdStop();
}

void KeyboardRemoteFilter::keyDriveBwdPressed() {
    speed_back_enabled = true;
    driveBwdStart();
}

void KeyboardRemoteFilter::keyDriveBwdReleased() {
    speed_back_enabled = false;
    driveBwdStop();
}

void KeyboardRemoteFilter::keyDriveLeftPressed() {
    steer_left_enabled = true;
    driveLeftStart();
}

void KeyboardRemoteFilter::keyDriveLeftReleased() {
    steer_left_enabled = false;
    driveLeftStop();
}

void KeyboardRemoteFilter::keyDriveRightPressed() {
    steer_right_enabled = true;
    driveRightStart();
}

void KeyboardRemoteFilter::keyDriveRightReleased() {
    steer_right_enabled = false;
    driveRightStop();
}

void KeyboardRemoteFilter::commandResetAll() {
    KeyboardRemoteFilter::resetAll();
    LOG_WARNING("KeyboardRemote: Reset triggered. Probably outOfFocus");
}
void KeyboardRemoteFilter::driveFwdStart() {
    if (keyboard_enabled && speed_front_enabled) {
        if (m_propEnableConsoleOutput) LOG_INFO("KeyboardRemote: drive forward");
        speed_actual = speed_target;
        KeyboardRemoteFilter::transmitSpeed(speed_actual);
    }
}

void KeyboardRemoteFilter::driveFwdStop() {
    if (keyboard_enabled) {
        speed_actual = 0;
        KeyboardRemoteFilter::transmitSpeed(speed_actual);
    }
}

void KeyboardRemoteFilter::driveBwdStart() {
    if (keyboard_enabled && speed_back_enabled) {
        if (m_propEnableConsoleOutput) LOG_INFO("KeyboardRemote: drive backward");
        speed_actual = -speed_target;
        KeyboardRemoteFilter::transmitSpeed(speed_actual);
    }
}

void KeyboardRemoteFilter::driveBwdStop() {
    if (keyboard_enabled) {
        speed_actual = 0;
        KeyboardRemoteFilter::transmitSpeed(speed_actual);
    }
}

void KeyboardRemoteFilter::driveLeftStart() {
    if (keyboard_enabled && steer_left_enabled) {
        if (m_propEnableConsoleOutput) LOG_INFO("KeyboardRemote: steer left");
        steering_actual = -steering_target;
        KeyboardRemoteFilter::transmitSteering(steering_actual);
    }
}

void KeyboardRemoteFilter::driveLeftStop() {
    if (keyboard_enabled) {
        steering_actual = 0;
        KeyboardRemoteFilter::transmitSteering(steering_actual);
    }
}

void KeyboardRemoteFilter::driveRightStart() {
    if (keyboard_enabled && steer_right_enabled) {
        if (m_propEnableConsoleOutput) LOG_INFO("KeyboardRemote: steer right");
        steering_actual = steering_target;
        KeyboardRemoteFilter::transmitSteering(steering_actual);
    }
}

void KeyboardRemoteFilter::driveRightStop() {
    if (keyboard_enabled) {
        steering_actual = 0;
        KeyboardRemoteFilter::transmitSteering(steering_actual);
    }
}

void KeyboardRemoteFilter::resetAll() {
    keyboard_enabled = false;
    speed_front_enabled = false;
    speed_back_enabled = false;
    steer_left_enabled = false;
    steer_right_enabled = false;
    headlights_enabled = false;
    hazard_enabled = true;
    if (m_propEnableConsoleOutput) LOG_INFO("KeyboardRemote: keyboard deactivated");

    tFloat32 speed_actual = 0;
    KeyboardRemoteFilter::transmitSpeed(speed_actual);

    tFloat32 steering_actual = 0;
    KeyboardRemoteFilter::transmitSteering(steering_actual);
    KeyboardRemoteFilter::toggleLights();
    KeyboardRemoteFilter::toggleHazard();
}

void KeyboardRemoteFilter::keyboardEnabledTrue() {
    keyboard_enabled = true;
    hazard_enabled = false;
    KeyboardRemoteFilter::toggleHazard();
    if (m_propEnableConsoleOutput) LOG_INFO("KeyboardRemote: keyboard activated");
}

void KeyboardRemoteFilter::keyboardEnabledFalse() {
    keyboard_enabled = false;
    if (m_propEnableConsoleOutput) LOG_INFO("KeyboardRemote: keyboard deactivated");
}
tResult KeyboardRemoteFilter::transmitSpeed(tFloat32 speed_transmitted) {
    if (m_propMetricSpeedOutput) {
        speed_signal.f32Value = (speed_transmitted / 100) * 16;
    } else {
        speed_signal.f32Value = speed_transmitted;
    }
    transmitSignalValue(m_oWriterSpeed, m_pClock->GetStreamTime(), m_SignalValueSampleFactory,
        m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, speed_signal.f32Value);
    RETURN_NOERROR;
}
tResult KeyboardRemoteFilter::transmitSteering(tFloat32 steering_transmitted) {
    steering_signal.f32Value = steering_transmitted;
    transmitSignalValue(m_oWriterSteering, m_pClock->GetStreamTime(), m_SignalValueSampleFactory,
        m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, steering_signal.f32Value);
    RETURN_NOERROR;
}
tResult KeyboardRemoteFilter::toggleLights() {
    transmitBoolSignalValue(
        m_oWriterLights, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory,
        m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0,
        m_ddlBoolSignalValueId.bValue, headlights_enabled);
    RETURN_NOERROR;
}
} */
tResult KeyboardRemoteFilter::toggleHazard() {
    transmitBoolSignalValue(
        m_oWriterHazard, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory,
        m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0,
        m_ddlBoolSignalValueId.bValue, hazard_enabled);
    RETURN_NOERROR;
}
