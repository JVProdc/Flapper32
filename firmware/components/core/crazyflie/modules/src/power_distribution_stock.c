

#include <string.h>

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#define DEBUG_MODULE "PWR_DIST"
#include "debug_cf.h"

static bool motorSetEnable = false;


static float pitch_ampl = 0.4f; // 1 = full servo stroke
// static float thrust;
static uint16_t act_max = 65535;


struct flapperConfig_s {
  uint8_t pitchServoNeutral;
  uint8_t yawServoNeutral;
  int8_t rollBias;
  uint16_t maxThrust;
};

struct flapperConfig_s flapperConfig = {
  .pitchServoNeutral = 50,
  .yawServoNeutral = 50,
  .rollBias = 0,
  .maxThrust = 60000,
};


static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  
  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_BLDC1, 0);
  motorsSetRatio(MOTOR_BLDC2, 0);
  motorsSetRatio(SERVO1, 0);
  motorsSetRatio(SERVO2, 0);
}

uint16_t scaleThrottleToPWM(int16_t thrust)
{
    // Map thrust (0-1000) to PWM range (1000-2000µs)
    return (uint16_t)(1000 + (thrust * 1000 / 1000));
}




void powerDistribution(const control_t *control)
{   
    motorsInit(platformConfigGetMotorMapping());
    memset(&motorPowerSet, 0, sizeof(motorPowerSet));  // Ensure all values start at 0

    int16_t r = control->roll / 2.0f;   // Roll: Differential thrust control
    int16_t p = control->pitch / 2.0f;  // Pitch: Servo 1 control
    int16_t y = control->yaw / 2.0f;    // Yaw: Servo 2 control

    // Calculate power for motors (roll + thrust)
    // motorPower.m1 = limitThrust(control->thrust + r);  // Motor 1 (right)
    // motorPower.m2 = limitThrust(control->thrust - r);  // Motor 2 (left)

    motorPower.m2 = limitThrust(control->thrust + r);
    motorPower.m1 = limitThrust(control->thrust - r);
  
    // Assign servo positions
    motorPower.m3 = flapperConfig.pitchServoNeutral*act_max / 100.0f + pitch_ampl * control->pitch; // pitch servo
    motorPower.m4 = flapperConfig.yawServoNeutral*act_max / 100.0f - control->yaw; // yaw servo

    if (motorSetEnable)
    {
        motorsSetRatio(MOTOR_BLDC1, motorPowerSet.m1);
        motorsSetRatio(MOTOR_BLDC2, motorPowerSet.m2);
        motorsSetRatio(SERVO1, motorPowerSet.m3);
        motorsSetRatio(SERVO2, motorPowerSet.m4);
    }
    else
    {
        // Apply idle thrust as a safety mechanism
        if (motorPower.m1 < idleThrust) motorPower.m1 = idleThrust;
        if (motorPower.m2 < idleThrust) motorPower.m2 = idleThrust;

        if (motorPower.m3 < idleThrust) motorPower.m3 = idleThrust;
        if (motorPower.m4 < idleThrust) motorPower.m4 = idleThrust;

        motorsSetRatio(MOTOR_BLDC1, motorPower.m1);
        motorsSetRatio(MOTOR_BLDC2, motorPower.m2);
        motorsSetRatio(SERVO1, motorPower.m3);
        motorsSetRatio(SERVO2, motorPower.m4);
    }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

PARAM_GROUP_START(powerDist)
PARAM_ADD(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)




// void powerDistribution(const control_t *control)
// {
//     // Calculate the roll and pitch effects
//     int16_t r = control->roll / 2.0f;   // Roll affects motors for differential thrust (thrust control)
//     int16_t p = control->pitch / 2.0f;  // Pitch affects servo control for pitch axis movement

//     // Motor 1 and Motor 2 are responsible for generating thrust and controlling roll
//     motorPower.m1 = limitThrust(control->thrust + r + control->yaw);  // Motor 1 (right motor)
//     motorPower.m2 = limitThrust(control->thrust - r - control->yaw);  // Motor 2 (left motor)

//     // Servo 1 controls pitch motion
//     motorPower.m3 = limitThrust(p + control->thrust);  // Servo 1 (pitch control)

//     // Servo 2 controls yaw motion
//     motorPower.m4 = limitThrust(control->yaw + control->thrust);  // Servo 2 (yaw control)

//     // Set the values for motors and servos based on the above calculations
//     if (motorSetEnable)
//     {
//         motorsSetRatio(MOTOR_BLDC1, motorPowerSet.m1);
//         motorsSetRatio(MOTOR_BLDC2, motorPowerSet.m2);
//         motorsSetRatio(SERVO1, motorPowerSet.m3);
//         motorsSetRatio(SERVO2, motorPowerSet.m4);
//     }
//     else
//     {
//         // If motor set is not enabled, apply the idle thrust as a minimum value for safety
//         if (motorPower.m1 < idleThrust) motorPower.m1 = idleThrust;
//         if (motorPower.m2 < idleThrust) motorPower.m2 = idleThrust;
//         if (motorPower.m3 < idleThrust) motorPower.m3 = idleThrust;
//         if (motorPower.m4 < idleThrust) motorPower.m4 = idleThrust;

//         motorsSetRatio(MOTOR_BLDC1, motorPower.m1);
//         motorsSetRatio(MOTOR_BLDC2, motorPower.m2);
//         motorsSetRatio(SERVO1, motorPower.m3);
//         motorsSetRatio(SERVO2, motorPower.m4);
//     }
// }








// #include <string.h>

// #include "power_distribution.h"

// #include <string.h>
// #include "log.h"
// #include "param.h"
// #include "num.h"
// #include "platform.h"
// #include "motors.h"
// #define DEBUG_MODULE "PWR_DIST"
// #include "debug_cf.h"

// static bool motorSetEnable = false;

// static float pitch_ampl = 0.4f; // 1 = full servo stroke
// // static float thrust;
// static uint16_t act_max = 65535;


// // #define MIN_PWM 1000
// // #define MAX_PWM 2000

// static struct {
//   uint32_t m1;
//   uint32_t m2;
//   uint32_t m3;
//   uint32_t m4;
// } motorPower;

// static struct {
//   uint16_t m1;
//   uint16_t m2;
//   uint16_t m3;
//   uint16_t m4;
// } motorPowerSet;


// struct flapperConfig_s {
//   uint8_t pitchServoNeutral;
//   uint8_t yawServoNeutral;
//   int8_t rollBias;
//   uint16_t maxThrust;
// };

// struct flapperConfig_s flapperConfig = {
//   .pitchServoNeutral = 50,
//   .yawServoNeutral = 50,
//   .rollBias = 0,
//   .maxThrust = 60000,
// };

// #define DEFAULT_IDLE_THRUST 10000

// static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

// void powerDistributionInit(void)
// {
//   motorsInit(platformConfigGetMotorMapping());
// }

// bool powerDistributionTest(void)
// {
//   bool pass = true;

//   pass &= motorsTest();
  
//   return pass;
// }

// #define limitThrust(VAL) limitUint16(VAL)


// // static int16_t limitThrust(int16_t thrust)
// // {
// //     if (thrust < MIN_PWM)
// //         return MIN_PWM;
// //     if (thrust > MAX_PWM)
// //         return MAX_PWM;
// //     return thrust;
// // }

// void powerStop()
// {
//   motorsSetRatio(MOTOR_BLDC1, 0);
//   motorsSetRatio(MOTOR_BLDC2, 0);
//   motorsSetRatio(SERVO1, 0);
//   motorsSetRatio(SERVO2, 0);
// }

// // uint16_t scaleThrottleToPWM(int16_t thrust)
// // {
// //     // Map thrust (0-1000) to PWM range (1000-2000µs)
// //     thrust = limitThrust(thrust); // Ensure thrust stays in the valid range
// //     return (uint16_t)(1000 + ((thrust * 1000) / 1000)); // Linear mapping
// // }

// // static int16_t scaleThrottleToPWM(int16_t throttle)
// // {
// //     return (throttle < MIN_PWM) ? MIN_PWM : ((throttle > MAX_PWM) ? MAX_PWM : throttle);
// // }




// void powerDistribution(const control_t *control)
// {   
//     motorsInit(platformConfigGetMotorMapping());
//     // memset(&motorPowerSet, 0, sizeof(motorPowerSet));  // Ensure all values start at 0

//     int16_t r = control->roll / 2.0f;   // Roll: Differential thrust control
//     int16_t p = control->pitch / 2.0f;  // Pitch: Servo 1 control
//     int16_t y = control->yaw / 2.0f;    // Yaw: Servo 2 control

//     // // Calculate power for motors (roll + thrust)
//     motorPower.m1 = limitThrust(control->thrust + r);  // Motor 1 (right)
//     motorPower.m2 = limitThrust(control->thrust - r);  // Motor 2 (left)


//     // // motorPower.m1 =  0.5f * control->roll + thrust * (1.0f + flapperConfig.rollBias / 100.0f); // left motor
//     // // motorPower.m2 = -0.5f * control->roll + thrust * (1.0f - flapperConfig.rollBias / 100.0f); // right motor
    
    

//     memset(&motorPowerSet, 0, sizeof(motorPowerSet)); // Clear all motor power sets

    


//     // Servo control: pitch and yaw
//     uint32_t servo1Position = flapperConfig.pitchServoNeutral * act_max / 100.0f + pitch_ampl * p; // Servo 1 (pitch)
//     uint32_t servo2Position = flapperConfig.yawServoNeutral * act_max / 100.0f - y;               // Servo 2 (yaw)

//     // Scale BLDC thrust to PWM ratio
//     // motorPower.m1 = scaleThrottleToPWM(limitThrust(control->thrust + r));
//     // motorPower.m2 = scaleThrottleToPWM(limitThrust(control->thrust - r));

//     // Assign servo positions
//     motorPower.m3 = flapperConfig.pitchServoNeutral*act_max / 100.0f + pitch_ampl * control->pitch; // pitch servo
//     motorPower.m4 = flapperConfig.yawServoNeutral*act_max / 100.0f - control->yaw; // yaw servo



//     if (motorSetEnable)
//     {
//         motorsSetRatio(MOTOR_BLDC1, motorPowerSet.m1);
//         motorsSetRatio(MOTOR_BLDC2, motorPowerSet.m2);
//         motorsSetRatio(SERVO1, motorPowerSet.m3);
//         motorsSetRatio(SERVO2, motorPowerSet.m4);
//     }
//     else
//     {
//         // Apply idle thrust as a safety mechanism
//         if (motorPower.m1 < idleThrust) motorPower.m1 = idleThrust;
//         if (motorPower.m2 < idleThrust) motorPower.m2 = idleThrust;

//         if (motorPower.m3 < idleThrust) motorPower.m3 = idleThrust;
//         if (motorPower.m4 < idleThrust) motorPower.m4 = idleThrust;

//         motorsSetRatio(MOTOR_BLDC1, motorPower.m1);
//         motorsSetRatio(MOTOR_BLDC2, motorPower.m2);
//         motorsSetRatio(SERVO1, motorPower.m3);
//         motorsSetRatio(SERVO2, motorPower.m4);
//     }
// }

// PARAM_GROUP_START(motorPowerSet)
// PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
// PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
// PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
// PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
// PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
// PARAM_GROUP_STOP(motorPowerSet)

// PARAM_GROUP_START(powerDist)
// PARAM_ADD(PARAM_UINT32, idleThrust, &idleThrust)
// PARAM_GROUP_STOP(powerDist)

// LOG_GROUP_START(motor)
// LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
// LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
// LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
// LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
// LOG_GROUP_STOP(motor)




// // void powerDistribution(const control_t *control)
// // {
// //     // Calculate the roll and pitch effects
// //     int16_t r = control->roll / 2.0f;   // Roll affects motors for differential thrust (thrust control)
// //     int16_t p = control->pitch / 2.0f;  // Pitch affects servo control for pitch axis movement

// //     // Motor 1 and Motor 2 are responsible for generating thrust and controlling roll
// //     motorPower.m1 = limitThrust(control->thrust + r + control->yaw);  // Motor 1 (right motor)
// //     motorPower.m2 = limitThrust(control->thrust - r - control->yaw);  // Motor 2 (left motor)

// //     // Servo 1 controls pitch motion
// //     motorPower.m3 = limitThrust(p + control->thrust);  // Servo 1 (pitch control)

// //     // Servo 2 controls yaw motion
// //     motorPower.m4 = limitThrust(control->yaw + control->thrust);  // Servo 2 (yaw control)

// //     // Set the values for motors and servos based on the above calculations
// //     if (motorSetEnable)
// //     {
// //         motorsSetRatio(MOTOR_BLDC1, motorPowerSet.m1);
// //         motorsSetRatio(MOTOR_BLDC2, motorPowerSet.m2);
// //         motorsSetRatio(SERVO1, motorPowerSet.m3);
// //         motorsSetRatio(SERVO2, motorPowerSet.m4);
// //     }
// //     else
// //     {
// //         // If motor set is not enabled, apply the idle thrust as a minimum value for safety
// //         if (motorPower.m1 < idleThrust) motorPower.m1 = idleThrust;
// //         if (motorPower.m2 < idleThrust) motorPower.m2 = idleThrust;
// //         if (motorPower.m3 < idleThrust) motorPower.m3 = idleThrust;
// //         if (motorPower.m4 < idleThrust) motorPower.m4 = idleThrust;

// //         motorsSetRatio(MOTOR_BLDC1, motorPower.m1);
// //         motorsSetRatio(MOTOR_BLDC2, motorPower.m2);
// //         motorsSetRatio(SERVO1, motorPower.m3);
// //         motorsSetRatio(SERVO2, motorPower.m4);
// //     }
// // }






