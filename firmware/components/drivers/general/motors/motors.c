 /*
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * motors.c - Motor driver
 *
 */

#include <stdbool.h>


//cfclcient

//FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stm32_legacy.h"
#include "motors.h"
#include "pm_esplane.h"
#include "log.h"
#define DEBUG_MODULE "MOTORS"

#include "debug_cf.h"


#include "esp_log.h" // Add ESP-IDF logging

#define DEBUG_MODULE "MOTORS"


static uint16_t motorsConvBitsTo16(uint16_t bits);
static uint16_t motorsConv16ToBits(uint16_t bits);


uint32_t motor_ratios[] = {0, 0, 0, 0};

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

const MotorPerifDef **motorMap; /* Current map configuration */

const uint32_t MOTORS[] = {MOTOR_BLDC1, MOTOR_BLDC2, SERVO1, SERVO2};

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5};

static bool isInit = false;
static bool isTimerInit = false;

ledc_channel_config_t motors_channel[NBR_OF_MOTORS] = {
    {
        .channel = PWM_CHANNEL_BLDC1,
        .duty = 0,
        .gpio_num = MOTOR_BLDC1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = PWM_CHANNEL_BLDC2,
        .duty = 0,
        .gpio_num = MOTOR_BLDC2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    },
    {
        .channel = PWM_CHANNEL_SERVO1,
        .duty = 0,
        .gpio_num = SERVO1_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0 // Use a separate timer for servos if required HERE
    },
    {
        .channel = PWM_CHANNEL_SERVO2,
        .duty = 0,
        .gpio_num = SERVO2_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0 // Use a separate timer for servos if required HERE
    },
};

/* Private functions */

static uint16_t motorsConvBitsTo16(uint16_t bits)
{
    return ((bits) << (16 - MOTORS_PWM_BITS));
}

static uint16_t motorsConv16ToBits(uint16_t bits)
{
    return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

bool pwm_timmer_init()
{
    if (isTimerInit) {
        // First to init will configure it
        return TRUE;
    }

    /*
     * Prepare and set configuration of timers
     * that will be used by MOTORS Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = MOTORS_PWM_BITS, // resolution of PWM duty
        .freq_hz = 50,					// frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE, // timer mode
        .timer_num = LEDC_TIMER_0,			// timer index
    };

    // Set configuration of timer0 for high speed channels
    if (ledc_timer_config(&ledc_timer) == ESP_OK) {
        isTimerInit = TRUE;
        return TRUE;
    }

    return FALSE;
}





/* Public functions */




// //Test spin:
void motorsTestSpin(void)
{
    const uint32_t min_ticks = (MOTORS_PWM_PERIOD * 5) / 100;  
    const uint32_t mid_ticks = (MOTORS_PWM_PERIOD * 10) / 100; 
    const uint32_t ramp_duration_ms = 2000; 
    const uint32_t steps = 100; 
    const uint32_t ramp_delay = pdMS_TO_TICKS(ramp_duration_ms / steps); 


    for (uint32_t step = 0; step <= steps; step++) {
        uint32_t ticks = min_ticks + step * (mid_ticks - min_ticks) / steps;
        for (int i = 0; i < 2; i++) {
            ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, ticks);
            ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
        }
        vTaskDelay(ramp_delay);
    }

    for (uint32_t step = 0; step <= steps; step++) {
        uint32_t ticks = mid_ticks - step * (mid_ticks - min_ticks) / steps;
        for (int i = 0; i < 2; i++) {
            ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, ticks);
            ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
        }
        vTaskDelay(ramp_delay);
    }
 
    for (int i = 0; i < 2; i++) {
        ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, min_ticks);
        ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
    }

    vTaskDelay(3000);

   
}


void servoTestSpin(void)
{
    // Standard servo PWM range (50Hz frequency)
    const uint32_t min_ticks = (MOTORS_PWM_PERIOD * 5) / 100;  // PWM for 0 degrees (1 ms pulse)
    const uint32_t max_ticks = (MOTORS_PWM_PERIOD * 10) / 100; // PWM for 180 degrees (2 ms pulse)
    const uint32_t steps = 180;                                // Number of steps for smooth transition
    const uint32_t ramp_delay = pdMS_TO_TICKS(20);             // 20 ms delay per step

    // Sweep from 0° to 180°
    for (uint32_t angle = 0; angle <= 90; angle++) {
        uint32_t ticks = min_ticks + ((angle * (max_ticks - min_ticks)) / 90); // Map angle to PWM range
        for (int i = 2; i < NBR_OF_MOTORS; i++) {
            ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, ticks);
            ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
        }
        vTaskDelay(ramp_delay);
    }

    // Sweep back from 180° to 0°
    for (uint32_t angle = 90; angle >= 0; angle--) {
        uint32_t ticks = min_ticks + ((angle * (max_ticks - min_ticks)) / 90); // Map angle to PWM range
        for (int i = 2; i < NBR_OF_MOTORS; i++) {
            ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, ticks);
            ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
        }
        vTaskDelay(ramp_delay);
    }

    // Reset servos to 0° position
    for (int i = 2; i < NBR_OF_MOTORS; i++) {
        ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, min_ticks);
        ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);
    }
}





//Initialization. Will set all motors ratio to 0%
void motorsInit(const MotorPerifDef **motorMapSelect)
{
    int i;

    if (isInit) {
        return;
    }

    DEBUG_PRINT("Initializing motors...\n");

    motorMap = motorMapSelect;

    if (pwm_timmer_init() != TRUE) {
        DEBUG_PRINT("PWM timer initialization failed!\n");
        return;
    }

    for (i = 0; i < NBR_OF_MOTORS; i++) {
        if (ledc_channel_config(&motors_channel[i]) != ESP_OK) {
            DEBUG_PRINT("PWM channel %d initialization failed!\n", i);
            return;
        }
    }

    DEBUG_PRINT("All motors initialized. Starting arming sequence...\n");

    // Arming Sequence for BLDC Motors
    for (i = 0; i < 2; i++) {
        uint32_t arm_ticks = (MOTORS_PWM_PERIOD * 1) / 100; // 1% (1 µs pulse width)
        ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, arm_ticks);
        ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);

        DEBUG_PRINT("Motor %d arming: PWM ticks = %u\n", i, arm_ticks);
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds for arming

    DEBUG_PRINT("Arming complete. Setting motors to idle (1000 µs).\n");

    // Set to idle (1000 µs)
    for (i = 0; i < 2; i++) {
        uint32_t min_ticks = (MOTORS_PWM_PERIOD * 5) / 100; // 5%
        ledc_set_duty(motors_channel[i].speed_mode, motors_channel[i].channel, min_ticks);
        ledc_update_duty(motors_channel[i].speed_mode, motors_channel[i].channel);

        DEBUG_PRINT("Motor %d set to idle: PWM ticks = %u\n", i, min_ticks);
    }

    isInit = true;

    DEBUG_PRINT("Initialization complete. Starting test spin...\n");
    DEBUG_PRINT("Test spin completed.\n");
}








void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    for (int i = 0; i < NBR_OF_MOTORS; i++) {
        ledc_stop(motors_channel[i].speed_mode, motors_channel[i].channel, 0);
    }
}

bool motorsTest(void)
{
    int i;

    for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++) {
        if (motorMap[i]->drvType == BRUSHED) {
#ifdef ACTIVATE_STARTUP_SOUND
            motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4) / 20);
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            motorsBeep(MOTORS[i], false, 0, 0);
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
            motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            motorsSetRatio(MOTORS[i], 0);
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
        }
    }

    return isInit;
}

// Ithrust is thrust mapped for 65536 <==> 60 grams





void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
    if (isInit) {
        uint16_t duty_ticks;

        ASSERT(id < NBR_OF_MOTORS);

       
        if (id == SERVO1 || id == SERVO2) {
            uint32_t angle = (ithrust * 180) / UINT16_MAX;
            uint32_t min_ticks = (MOTORS_PWM_PERIOD * 5) / 100;
            uint32_t max_ticks = (MOTORS_PWM_PERIOD * 10) / 100;
            duty_ticks = min_ticks + ((angle * (max_ticks - min_ticks)) / 180);
        } else {
            // duty_ticks = (ithrust * MOTORS_PWM_PERIOD) / UINT16_MAX;
            // BLDC Motor: Map thrust to PWM signal (1ms to 2ms)
            uint32_t min_ticks = (MOTORS_PWM_PERIOD * 5) / 100;  // 5% duty cycle (1ms pulse)
            uint32_t max_ticks = (MOTORS_PWM_PERIOD * 10) / 100; // 10% duty cycle (2ms pulse)
            duty_ticks = min_ticks + ((ithrust * (max_ticks - min_ticks)) / UINT16_MAX);  // Map ithrust to duty cycle
        }

        // Set PWM duty cycle for the selected motor
        ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, duty_ticks);
        ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);

        // Store the ratio for debugging/logging purposes
        motor_ratios[id] = duty_ticks;
    }
}







int motorsGetRatio(uint32_t id)
{
    int ratio;
    ASSERT(id < NBR_OF_MOTORS);
    ratio = motorsConvBitsTo16((uint16_t)ledc_get_duty(motors_channel[id].speed_mode, motors_channel[id].channel));
    return ratio;
}

void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
    uint32_t freq_hz = 50;
    ASSERT(id < NBR_OF_MOTORS);
    if (ratio != 0) {
        ratio = (uint16_t)(0.05*(1<<16));
    }
    
    if (enable) {
        freq_hz = frequency;
    }
    
    ledc_set_freq(LEDC_LOW_SPEED_MODE,LEDC_TIMER_0,freq_hz);
    ledc_set_duty(motors_channel[id].speed_mode, motors_channel[id].channel, (uint32_t)motorsConv16ToBits(ratio));
    ledc_update_duty(motors_channel[id].speed_mode, motors_channel[id].channel);
}

// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
    motorsBeep(MOTOR_BLDC1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(MOTOR_BLDC2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(SERVO1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    motorsBeep(SERVO2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency) / 20);
    vTaskDelay(M2T(duration_msec));
    motorsBeep(MOTOR_BLDC1, false, frequency, 0);
    motorsBeep(MOTOR_BLDC2, false, frequency, 0);
    motorsBeep(SERVO1, false, frequency, 0);
    motorsBeep(SERVO2, false, frequency, 0);
}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes)
{
    int i = 0;
    uint16_t note;     // Note in hz
    uint16_t duration; // Duration in ms

    do
    {
      note = notes[i++];
      duration = notes[i++];
      motorsPlayTone(note, duration);
    } while (duration != 0);
}
LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1_pwm, &motor_ratios[0])
LOG_ADD(LOG_UINT32, m2_pwm, &motor_ratios[1])
LOG_ADD(LOG_UINT32, m3_pwm, &motor_ratios[2])
LOG_ADD(LOG_UINT32, m4_pwm, &motor_ratios[3])
LOG_GROUP_STOP(pwm)