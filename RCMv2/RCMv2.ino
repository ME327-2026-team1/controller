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

JEncoderAS5048bI2C encoder1 = JEncoderAS5048bI2C(false, 1.0, 0x50, 20000, 200, true); // left
JEncoderAS5048bI2C encoder2 = JEncoderAS5048bI2C(true, 1.0, 0x48, 20000, 200, true); // right

// all the motor drivers
JMotorDriverTMC7300 motor1Driver = JMotorDriverTMC7300(portB); // left
JMotorDriverTMC7300 motor2Driver = JMotorDriverTMC7300(portD); // right

SemaphoreHandle_t encoderMutex; // used to check whether it's save to edit encoder variables (multi threading)

// TODO: do floats cause problems if the wheels have turned many times?
float local_left_pos = 0;
float local_right_pos = 0;
float local_left_vel = 0;
float local_right_vel = 0;

float local_left_motor_power = 0; // -1 to 1
float local_right_motor_power = 0;

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

    local_left_motor_power = (remote_left_pos / 2.0 - local_left_pos) * 0.09;
    local_right_motor_power = (remote_right_pos / 2.0 - local_right_pos) * 0.09;

    RSLcolor = (car_button ? CRGB(255, 255, 255) : (voltageComp.getSupplyVoltage() < 7.0 ? CRGB(150, 0, 5) : CRGB(250, 45, 0)));

    // set motors
    motor1Driver.set(-local_left_motor_power);
    motor2Driver.set(-local_right_motor_power);
}

void Enable()
{
    motor1Driver.enable();
    motor2Driver.enable();
    // encoder1.zeroCounter();
    // encoder2.zeroCounter();
}

void Disable()
{
    motor1Driver.disable();
    motor2Driver.disable();
}

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    Wire1.setClock(1000000);

    encoderMutex = xSemaphoreCreateMutex();

    encoder1.useCustomWire(Wire1);
    encoder2.useCustomWire(Wire1);
    Wire1.begin();

    xTaskCreatePinnedToCore( // create task to run encoder tasks
        encoderTaskFunction, /* Function to implement the task */
        "EncoderTask", /* Name of the task */
        15000, /* Stack size in words */
        NULL, /* Task input parameter */
        0, /* Priority of the task */
        NULL, /* Task handle. */
        0 /* Core on which task should run */
    );
}

void Always()
{
    // always runs if void loop is running, JMotor run() functions should be put here
    // (but only the "top level", for example if you call drivetrainController.run() you don't also need to call leftMotorController.run())

    if (xSemaphoreTake(encoderMutex, 1) == pdTRUE) {
        local_left_pos = encoder1.getPos();
        local_right_pos = encoder2.getPos();
        local_left_vel = encoder1.getVel();
        local_right_vel = encoder2.getVel();
        xSemaphoreGive(encoderMutex);
    }

    if(digitalRead(0) == 0) {
        encoder1.zeroCounter();
        encoder2.zeroCounter();
    }

    Serial.print(local_left_pos);
    Serial.print(", \t");
    Serial.print(local_left_vel);
    Serial.print(", \t");
    Serial.print(local_right_pos);
    Serial.print(", \t");
    Serial.print(local_right_vel);
    Serial.print(", \t");
    Serial.print(remote_left_pos);
    Serial.print(", \t");
    Serial.print(remote_right_pos);
    Serial.println();
    delay(1);
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

void encoderTaskFunction(void* pvParameters)
{
    while (true) { // infinite loop

        if (xSemaphoreTake(encoderMutex, 1) == pdTRUE) {
            encoder2.run();
            encoder1.run();
            xSemaphoreGive(encoderMutex);
        }
        delay(1);
    }
}

#include "rcmutil.h"
