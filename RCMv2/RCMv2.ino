//   This program is template code for programming small esp32 powered wifi controlled robots.
//   https://github.com/rcmgames/RCMv2
//   for information see this page: https://github.com/RCMgames

/**
UNCOMMENT ONE OF THE FOLLOWING LINES DEPENDING ON WHAT HARDWARE YOU ARE USING
Remember to also choose the "environment" for your microcontroller in PlatformIO
*/
// #define RCM_HARDWARE_VERSION RCM_ORIGINAL // versions 1, 2, 3, and 3.1 of the original RCM hardware // https://github.com/RCMgames/RCM_hardware_documentation_and_user_guide
// #define RCM_HARDWARE_VERSION RCM_4_V1 // version 1 of the RCM 4 // https://github.com/RCMgames/RCM-Hardware-V4
// #define RCM_HARDWARE_VERSION RCM_BYTE_V2 // version 2 of the RCM BYTE // https://github.com/RCMgames/RCM-Hardware-BYTE
#define RCM_HARDWARE_VERSION RCM_NIBBLE_V1 // version 1 of the RCM Nibble // https://github.com/RCMgames/RCM-Hardware-Nibble
// #define RCM_HARDWARE_VERSION RCM_D1_V1 // version 1 of the RCM D1 // https://github.com/RCMgames/RCM-Hardware-D1
// #define RCM_HARDWARE_VERSION ALFREDO_NOU2_NO_VOLTAGE_MONITOR // voltageComp will always report 10 volts https://www.alfredosys.com/products/alfredo-nou2/
// #define RCM_HARDWARE_VERSION ALFREDO_NOU2_WITH_VOLTAGE_MONITOR // modified to add resistors VIN-30k-D36-10k-GND https://www.alfredosys.com/products/alfredo-nou2/
// #define RCM_HARDWARE_VERSION ALFREDO_NOU3 // https://www.alfredosys.com/products/alfredo-nou3/

/**
uncomment one of the following lines depending on which communication method you want to use
*/
#define RCM_COMM_METHOD RCM_COMM_EWD // use the normal communication method for RCM robots

#include "rcm.h" //defines pins

// set up motors and anything else you need here
// See this page for how to set up servos and motors for each type of RCM board:
// https://github.com/RCMgames/useful-code/tree/main/boards
// See this page for information about how to set up a robot's drivetrain using the JMotor library
// https://github.com/joshua-8/JMotor/wiki/How-to-set-up-a-drivetrain

/**
Bits of code that might be helpful when starting a program with an original RCM board
https://github.com/RCMgames/RCM-Hardware-Nibble
// from https://github.com/RCMgames/useful-code/tree/main/boards
*/

JEncoderAS5048bI2C encoder1 = JEncoderAS5048bI2C(false, 1.0, 0x48, 10000, 100, true);
JEncoderAS5048bI2C encoder2 = JEncoderAS5048bI2C(true, 1.0, 0x50, 10000, 100, true);

// all the motor drivers
JMotorDriverTMC7300 motor1Driver = JMotorDriverTMC7300(portD);
JMotorDriverTMC7300 motor2Driver = JMotorDriverTMC7300(portB);

// TODO: do floats cause problems if the wheels have turned many times?
float local_left_pos = 0;
float local_right_pos = 0;
float local_left_vel = 0;
float local_right_vel = 0;

float local_left_motor_power = 0; // -1 to 1
float local_right_motor_power = 0;

// --- Free-space calibration model ---
// Fit these from free-space measurements:
// v_free = slope * controller_position + intercept
float left_free_slope = 6.67f;
float left_free_intercept = 0.0f;

float right_free_slope = 6.67f;
float right_free_intercept = 0.0f;

// --- Haptic tuning ---
float k_base = 0.00f;        // baseline stiffness in free space
float k_vel_gain = 3.00f;    // how strongly velocity deficit changes stiffness
float k_min = 0.0f;          // allow fully light feel
float k_max = 100.0f;          // safety cap
float b_damping = 0.00f;     // small damping for stability

// Filtering / smoothing
float vel_alpha = 0.20f;     // low-pass filter for received wheel velocity
float k_smooth = 0.50f;      // low-pass filter for k updates

// State
float filtered_remote_left_vel = 0.0f;
float filtered_remote_right_vel = 0.0f;
float k_left_eff = 0.20f;
float k_right_eff = 0.20f;
int left_motion_sign = 0;
int right_motion_sign = 0;

//
// --- Tilt based resistance method ---
//
float k_base_accel = 0.5;       // baseline spring stiffness
float k_terrain = 2.0;    // how much car velocity scales stiffness
float b_damping_accel = 0.05;    // damping coefficient for velocity
float acc_alpha = 0.05;       // IMU low-pass filter weight (lower = smoother, more lag)

float filtered_accel_x = 0;

int32_t car_micros = 0;
float car_batteryVoltage = 0;
boolean car_button = false;
// will add accelerometer from car

float remote_left_pos = 0;
float remote_right_pos = 0;
float remote_left_vel = 0;
float remote_right_vel = 0;
float remote_imu_accel_x = 0;

void Enabled()
{
    // WRITE CONTROLS HERE!!
    /*
    inputs:
    * enabled (true if car is connected)
    * local_left_pos, local_right_pos (wheel positions)
    * local_left_vel, local_right_vel (wheel velocities)
    *
    *
    outputs:
    * local_left_motor_power, local_right_motor_power, motor3Val, motor4Val (floats between -1 and 1 that control the motors)
    *
    *
    */

    RSLcolor = (car_button ? CRGB(255, 255, 255) : (voltageComp.getSupplyVoltage() < 7.0 ? CRGB(150, 0, 5) : CRGB(250, 45, 0)));

    // low-pass filter IMU to remove high-frequency noise
    filtered_accel_x = acc_alpha * remote_imu_accel_x + (1.0 - acc_alpha) * filtered_accel_x;

    // Smooth the incoming car-side wheel velocities
    filtered_remote_left_vel  = vel_alpha * remote_left_vel  + (1.0f - vel_alpha) * filtered_remote_left_vel;
    filtered_remote_right_vel = vel_alpha * remote_right_vel + (1.0f - vel_alpha) * filtered_remote_right_vel;

    // ****************** CONTROL CODE *****************




    // -------------------------------------------------------------------------
    // tilt based resistance method
    // -------------------------------------------------------------------------

    // // position to velocity with spring that gets stronger as car inclines up / accelerates forwards
    // float k = k_base_accel + k_terrain * filtered_accel_x;

    // // TODO: TEST THIS AND TRY TO FIX (currently, k<0 instance is oscillatory)
    // // if car is tilted downwards too much, controller pushes user in forward direction, goes to infinity if controller let go
    // // Potential fix (design choice!):
    // // if k < 0 because car downhill enough, only oppose if moving backwards, otherwise turn motor off to let wheels spin freely
    // if (k < 0) {
    //     if (local_left_vel > 0) {
    //         local_left_motor_power  = -k * local_left_pos  - b_damping_accel * local_left_vel;
    //     } else {
    //         local_left_motor_power = 0;
    //     }
    //     if (local_right_vel > 0) {
    //         local_right_motor_power = -k * local_right_pos - b_damping_accel * local_right_vel;
    //     } else {
    //         local_right_motor_power = 0;
    //     }
    // } else {
    //     // if you want to run normally, with downwards tilt issue, just run the below 2 lines without if statement
    //     local_left_motor_power  = -k * local_left_pos  - b_damping_accel * local_left_vel;
    //     local_right_motor_power = -k * local_right_pos - b_damping_accel * local_right_vel;
    // }





    // -------------------------------------------------------------------------
    // calibration code: pos to velocity (unilateral)
    // -------------------------------------------------------------------------

    // local_left_motor_power = -k_base * local_left_pos - b_damping * local_left_vel;
    // local_right_motor_power = -k_base * local_right_pos - b_damping * local_right_vel;





    // -------------------------------------------------------------------------
    // Wheel-by-wheel impedance estimate:
    //   1) Predict the free-space wheel velocity from controller displacement
    //   2) Compare actual car wheel velocity to that prediction
    //   3) If actual < expected -> increase stiffness
    //      If actual > expected -> decrease stiffness (bounded at 0)
    // -------------------------------------------------------------------------

    const float v_deadband = 0.02f;
    const float error_deadband = 0.01f;
    const float dir_epsilon = 0.001f;

    // Predicted free-space velocities from your fitted lines
    float left_expected_vel  = left_free_slope  * local_left_pos  + left_free_intercept;
    float right_expected_vel = right_free_slope * local_right_pos + right_free_intercept;

    // Determine expected direction for each wheel with a little memory near zero
    int left_dir = 0;
    if (fabsf(left_expected_vel) > v_deadband) {
        left_dir = (left_expected_vel > 0.0f) ? 1 : -1;
        left_motion_sign = left_dir;
    } else if (fabsf(local_left_pos) > v_deadband) {
        left_dir = (local_left_pos > 0.0f) ? 1 : -1;
        left_motion_sign = left_dir;
    } else {
        left_dir = left_motion_sign;
    }

    int right_dir = 0;
    if (fabsf(right_expected_vel) > v_deadband) {
        right_dir = (right_expected_vel > 0.0f) ? 1 : -1;
        right_motion_sign = right_dir;
    } else if (fabsf(local_right_pos) > v_deadband) {
        right_dir = (local_right_pos > 0.0f) ? 1 : -1;
        right_motion_sign = right_dir;
    } else {
        right_dir = right_motion_sign;
    }

    // Measure velocity "along the expected travel direction"
    float left_meas_along  = (left_dir  == 0) ? 0.0f : (left_dir  * filtered_remote_left_vel);
    float right_meas_along = (right_dir == 0) ? 0.0f : (right_dir * filtered_remote_right_vel);

    // Compare against expected speed magnitude
    float left_speed_error  = fabsf(left_expected_vel)  - left_meas_along;
    float right_speed_error = fabsf(right_expected_vel) - right_meas_along;

    if (fabsf(left_speed_error) < error_deadband) {
        left_speed_error = 0.0f;
    }
    if (fabsf(right_speed_error) < error_deadband) {
        right_speed_error = 0.0f;
    }

    // Convert velocity deficit to stiffness target
    float left_k_target  = k_base + k_vel_gain * left_speed_error;
    float right_k_target = k_base + k_vel_gain * right_speed_error;

    left_k_target  = constrain(left_k_target,  k_min, k_max);
    right_k_target = constrain(right_k_target, k_min, k_max);

    // Smooth k so the controller does not chatter
    k_left_eff  = k_left_eff  + k_smooth * (left_k_target  - k_left_eff);
    k_right_eff = k_right_eff + k_smooth * (right_k_target - k_right_eff);

    // Final haptic rendering:
    // stiffer when impeded, lighter when free, always stable because k >= 0
    local_left_motor_power  = -k_left_eff  * local_left_pos  - b_damping * local_left_vel;
    local_right_motor_power = -k_right_eff * local_right_pos - b_damping * local_right_vel;
  




    // set motors (all methods)
    motor1Driver.set(local_left_motor_power);
    motor2Driver.set(local_right_motor_power);

}

void Enable()
{
    motor1Driver.enable();
    motor2Driver.enable();


}

void Disable()
{
    motor1Driver.disable();
    motor2Driver.disable();
}

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    encoder1.useCustomWire(Wire1);
    encoder2.useCustomWire(Wire1);
    Wire1.begin();
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you don't also need to call leftMotorController.run())
    encoder1.run();
    encoder2.run();

    local_left_pos = encoder1.getPos();
    local_right_pos = encoder2.getPos();
    local_left_vel = encoder1.getVel();
    local_right_vel = encoder2.getVel();

    Serial.print(local_left_pos);
    Serial.print(", \t");
    Serial.print(filtered_remote_left_vel);
    Serial.print(", \t");
    Serial.print(local_right_pos);
    Serial.print(", \t");
    Serial.print(filtered_remote_right_vel);
    Serial.print(", \t");
    Serial.print(k_left_eff);
    Serial.print(", \t");

    Serial.println(filtered_accel_x);


}

#if RCM_COMM_METHOD == RCM_COMM_EWD
void WifiDataToParse()
{
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)

    enabled = EWD::recvBl();
    car_batteryVoltage = EWD::recvFl();
    car_micros = EWD::recvIn();
    car_button = EWD::recvBl();
    remote_left_pos = EWD::recvFl();
    remote_right_pos = EWD::recvFl();
    remote_left_vel = EWD::recvFl();
    remote_right_vel = EWD::recvFl();
    remote_imu_accel_x = EWD::recvFl();
    // get car acceleration data
}
void WifiDataToSend()
{
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
    EWD::sendBl(true); // enabled
    EWD::sendFl(voltageComp.getSupplyVoltage());
    EWD::sendIn(micros());
    EWD::sendBl(digitalRead(0) == 0);
    EWD::sendFl(local_left_pos);
    EWD::sendFl(local_right_pos);
    EWD::sendFl(local_left_vel);
    EWD::sendFl(local_right_vel);
}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "BEJM_controller";
    EWD::routerPassword = "hapticsBEJM";
    EWD::routerPort = 25210;
    EWD::communicateWithIP = "192.168.4.1";
    EWD::resendTimeout = 20;
    EWD::signalLossTimeout = 750;
}
#endif

#include "rcmutil.h"
