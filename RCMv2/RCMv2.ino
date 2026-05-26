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
float k_vel_gain = 5.00f;    // how strongly velocity deficit changes stiffness
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


// --- Improved velocity-impedance tuning ---
float left_expected_vel_f = 0.0f;
float right_expected_vel_f = 0.0f;

float left_err_f = 0.0f;
float right_err_f = 0.0f;

// how quickly the expected free-space velocity model moves
float expected_alpha = 0.03f;

// how quickly the error estimate moves
float err_alpha = 0.07f;

// how strongly controller motion suppresses stiffness updates
float motion_scale = 0.25f;   // larger = less suppression

// faster rise, slower fall is usually nice for haptics
float k_up_rate = 0.15f;
float k_down_rate = 0.05f;




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
    // Wheel-by-wheel impedance estimate with motion-gating
    //
    // Goal:
    //   - use free-space velocity model from controller position
    //   - compare to measured wheel velocity
    //   - suppress update while controller is moving quickly
    //   - low-pass the expected velocity and the error to avoid spikes
    // -------------------------------------------------------------------------

    const float v_deadband = 0.02f;
    const float error_deadband = 0.01f;

    // Raw free-space predictions from controller position
    float left_expected_raw  = left_free_slope  * local_left_pos  + left_free_intercept;
    float right_expected_raw = right_free_slope * local_right_pos + right_free_intercept;

    // Slowly filter the expected free-space velocity so it does NOT jump instantly
    left_expected_vel_f  += expected_alpha * (left_expected_raw  - left_expected_vel_f);
    right_expected_vel_f += expected_alpha * (right_expected_raw - right_expected_vel_f);

    // Determine expected travel direction with memory near zero
    int left_dir = 0;
    if (fabsf(left_expected_vel_f) > v_deadband) {
        left_dir = (left_expected_vel_f > 0.0f) ? 1 : -1;
        left_motion_sign = left_dir;
    } else if (fabsf(local_left_pos) > v_deadband) {
        left_dir = (local_left_pos > 0.0f) ? 1 : -1;
        left_motion_sign = left_dir;
    } else {
        left_dir = left_motion_sign;
    }

    int right_dir = 0;
    if (fabsf(right_expected_vel_f) > v_deadband) {
        right_dir = (right_expected_vel_f > 0.0f) ? 1 : -1;
        right_motion_sign = right_dir;
    } else if (fabsf(local_right_pos) > v_deadband) {
        right_dir = (local_right_pos > 0.0f) ? 1 : -1;
        right_motion_sign = right_dir;
    } else {
        right_dir = right_motion_sign;
    }

    // Measured car velocity projected along expected direction
    float left_meas_along  = (left_dir  == 0) ? 0.0f : (left_dir  * filtered_remote_left_vel);
    float right_meas_along = (right_dir == 0) ? 0.0f : (right_dir * filtered_remote_right_vel);

    // Velocity deficit: positive means the car is moving slower than expected
    float left_err_raw  = fabsf(left_expected_vel_f)  - left_meas_along;
    float right_err_raw = fabsf(right_expected_vel_f) - right_meas_along;

    // Low-pass the error so transient changes do not spike k
    left_err_f  += err_alpha * (left_err_raw  - left_err_f);
    right_err_f += err_alpha * (right_err_raw - right_err_f);

    // Deadband around zero to avoid chatter
    if (fabsf(left_err_f) < error_deadband) {
        left_err_f = 0.0f;
    }
    if (fabsf(right_err_f) < error_deadband) {
        right_err_f = 0.0f;
    }

    // Motion gate based on how fast the controller handle is moving.
    // When the user is moving the controller quickly, reduce adaptation.
    // This prevents the system from mistaking normal transients for load.
    float controller_motion = 0.5f * (fabsf(local_left_vel) + fabsf(local_right_vel));
    float motion_gate = 1.0f / (1.0f + controller_motion / motion_scale);

    // Map error to stiffness target.
    // Positive error -> stiffer.
    // Negative error -> lighter, but not below k_min.
    float left_k_target  = k_base + motion_gate * k_vel_gain * left_err_f;
    float right_k_target = k_base + motion_gate * k_vel_gain * right_err_f;

    left_k_target  = constrain(left_k_target,  k_min, k_max);
    right_k_target = constrain(right_k_target, k_min, k_max);

    // Asymmetric smoothing: stiffness can rise faster than it falls.
    // This helps contrast without making free space feel heavy.
    float left_k_delta = left_k_target - k_left_eff;
    if (left_k_delta > 0.0f) {
        left_k_delta = fminf(left_k_delta, k_up_rate);
    } else {
        left_k_delta = fmaxf(left_k_delta, -k_down_rate);
    }
    k_left_eff += left_k_delta;

    float right_k_delta = right_k_target - k_right_eff;
    if (right_k_delta > 0.0f) {
        right_k_delta = fminf(right_k_delta, k_up_rate);
    } else {
        right_k_delta = fmaxf(right_k_delta, -k_down_rate);
    }
    k_right_eff += right_k_delta;

    // Final haptic rendering
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
