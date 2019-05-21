/**
*    ||          ____  _ __
* +------+      / __ )(_) /_______________ _____  ___
* | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
* +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
*  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
*
* Crazyflie control firmware
*
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
* This code mainly interfacing the PWM peripheral lib of ST.
*/

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include "motors.h"
#include "pm.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#ifdef BRUSHLESS_PROTO_DECK_MAPPING
// HW defines for prototype brushless deck
//M1 and M3 refers to servos and M4 to the DC motor

#define MOTORS_TIM_M1_PERIF       RCC_APB1Periph_TIM3 // TIM3_CH2
#define MOTORS_TIM_M1             TIM3
#define MOTORS_TIM_M1_DBG         DBGMCU_TIM3_STOP
#define M1_TIM_SETCOMPARE         TIM_SetCompare2
#define M1_TIM_GETCAPTURE         TIM_GetCapture2
#define M1_TIM_OC_INIT            TIM_OC2Init
#define M1_TIM_OC_PRE_CFG         TIM_OC2PreloadConfig

#define MOTORS_TIM_M3_PERIF       RCC_APB1Periph_TIM2 // TIM2_CH4
#define MOTORS_TIM_M3             TIM2
#define MOTORS_TIM_M3_DBG         DBGMCU_TIM2_STOP
#define M3_TIM_SETCOMPARE         TIM_SetCompare4
#define M3_TIM_GETCAPTURE         TIM_GetCapture4
#define M3_TIM_OC_INIT            TIM_OC4Init
#define M3_TIM_OC_PRE_CFG         TIM_OC4PreloadConfig

#define MOTORS_TIM_M4_PERIF       RCC_APB1Periph_TIM4
#define MOTORS_TIM_M4             TIM4
#define MOTORS_TIM_M4_DBG         DBGMCU_TIM4_STOP
#define M4_TIM_SETCOMPARE         TIM_SetCompare4
#define M4_TIM_GETCAPTURE         TIM_GetCapture4
#define M4_TIM_OC_INIT            TIM_OC4Init
#define M4_TIM_OC_PRE_CFG         TIM_OC4PreloadConfig

#define MOTORS_GPIO_M1_PERIF         RCC_AHB1Periph_GPIOB // PB5
#define MOTORS_GPIO_M1_PORT          GPIOB
#define MOTORS_GPIO_M1_PIN           GPIO_Pin_5
#define MOTORS_GPIO_AF_M1_PIN        GPIO_PinSource5
#define MOTORS_GPIO_AF_M1            GPIO_AF_TIM3

#define MOTORS_GPIO_M3_PERIF         RCC_AHB1Periph_GPIOA // PA3
#define MOTORS_GPIO_M3_PORT          GPIOA
#define MOTORS_GPIO_M3_PIN           GPIO_Pin_3
#define MOTORS_GPIO_AF_M3_PIN        GPIO_PinSource3
#define MOTORS_GPIO_AF_M3            GPIO_AF_TIM2

#define MOTORS_GPIO_M4_PERIF         RCC_AHB1Periph_GPIOB
#define MOTORS_GPIO_M4_PORT          GPIOB
#define MOTORS_GPIO_M4_PIN           GPIO_Pin_9 // TIM4_CH4
#define MOTORS_GPIO_AF_M4_PIN        GPIO_PinSource9
#define MOTORS_GPIO_AF_M4            GPIO_AF_TIM4
#else
// Mapping of brushed controller timers. Brushless controller can still be activated using
// the same mapping, PWM then needs to be inverted.

#ifdef BRUSHLESS_MOTORCONTROLLER
// The brushed motor drivers (pull-down mosfet) inverses the output. Compensate for that.
#define BRUSHLESS_INVERSED_POLARITY
#endif

#define MOTORS_TIM_M1_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M1             TIM2
#define MOTORS_TIM_M1_DBG         DBGMCU_TIM2_STOP
#define M1_TIM_SETCOMPARE         TIM_SetCompare2
#define M1_TIM_GETCAPTURE         TIM_GetCapture2
#define M1_TIM_OC_INIT            TIM_OC2Init
#define M1_TIM_OC_PRE_CFG         TIM_OC2PreloadConfig


#define MOTORS_TIM_M2_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M2             TIM2
#define MOTORS_TIM_M2_DBG         DBGMCU_TIM2_STOP
#define M2_TIM_SETCOMPARE         TIM_SetCompare4
#define M2_TIM_GETCAPTURE         TIM_GetCapture4
#define M2_TIM_OC_INIT            TIM_OC4Init
#define M2_TIM_OC_PRE_CFG         TIM_OC4PreloadConfig

#define MOTORS_TIM_M3_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M3             TIM2
#define MOTORS_TIM_M3_DBG         DBGMCU_TIM2_STOP
#define M3_TIM_SETCOMPARE         TIM_SetCompare1
#define M3_TIM_GETCAPTURE         TIM_GetCapture1
#define M3_TIM_OC_INIT            TIM_OC1Init
#define M3_TIM_OC_PRE_CFG         TIM_OC1PreloadConfig

#define MOTORS_TIM_M4_PERIF       RCC_APB1Periph_TIM4
#define MOTORS_TIM_M4             TIM4
#define MOTORS_TIM_M4_DBG         DBGMCU_TIM4_STOP
#define M4_TIM_SETCOMPARE         TIM_SetCompare4
#define M4_TIM_GETCAPTURE         TIM_GetCapture4
#define M4_TIM_OC_INIT            TIM_OC4Init
#define M4_TIM_OC_PRE_CFG         TIM_OC4PreloadConfig

#define MOTORS_GPIO_M1_PERIF         RCC_AHB1Periph_GPIOB
#define MOTORS_GPIO_M1_PORT          GPIOA
#define MOTORS_GPIO_M1_PIN           GPIO_Pin_1 // TIM2_CH2
#define MOTORS_GPIO_AF_M1_PIN        GPIO_PinSource1
#define MOTORS_GPIO_AF_M1            GPIO_AF_TIM2

#define MOTORS_GPIO_M2_PERIF         RCC_AHB1Periph_GPIOB
#define MOTORS_GPIO_M2_PORT          GPIOB
#define MOTORS_GPIO_M2_PIN           GPIO_Pin_11 // TIM2_CH4
#define MOTORS_GPIO_AF_M2_PIN        GPIO_PinSource11
#define MOTORS_GPIO_AF_M2            GPIO_AF_TIM2

#define MOTORS_GPIO_M3_PERIF         RCC_AHB1Periph_GPIOA
#define MOTORS_GPIO_M3_PORT          GPIOA
#define MOTORS_GPIO_M3_PIN           GPIO_Pin_15 // TIM2_CH1
#define MOTORS_GPIO_AF_M3_PIN        GPIO_PinSource15
#define MOTORS_GPIO_AF_M3            GPIO_AF_TIM2

#define MOTORS_GPIO_M4_PERIF         RCC_AHB1Periph_GPIOB
#define MOTORS_GPIO_M4_PORT          GPIOB
#define MOTORS_GPIO_M4_PIN           GPIO_Pin_9 // TIM4_CH4
#define MOTORS_GPIO_AF_M4_PIN        GPIO_PinSource9
#define MOTORS_GPIO_AF_M4            GPIO_AF_TIM4

#endif



//Define the pins M5 and M6 to be put VCC
#define MOTORS_TIM_M5_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M5             TIM2
#define MOTORS_TIM_M5_DBG         DBGMCU_TIM2_STOP
#define M5_TIM_SETCOMPARE         TIM_SetCompare2
#define M5_TIM_GETCAPTURE         TIM_GetCapture2
#define M5_TIM_OC_INIT            TIM_OC2Init
#define M5_TIM_OC_PRE_CFG         TIM_OC2PreloadConfig

#define MOTORS_TIM_M7_PERIF       RCC_APB1Periph_TIM2
#define MOTORS_TIM_M7             TIM2
#define MOTORS_TIM_M7_DBG         DBGMCU_TIM2_STOP
#define M7_TIM_SETCOMPARE         TIM_SetCompare1
#define M7_TIM_GETCAPTURE         TIM_GetCapture1
#define M7_TIM_OC_INIT            TIM_OC1Init
#define M7_TIM_OC_PRE_CFG         TIM_OC1PreloadConfig

#define MOTORS_GPIO_M5_PERIF         RCC_AHB1Periph_GPIOB
#define MOTORS_GPIO_M5_PORT          GPIOA
#define MOTORS_GPIO_M5_PIN           GPIO_Pin_1 // TIM2_CH2
#define MOTORS_GPIO_AF_M5_PIN        GPIO_PinSource1
#define MOTORS_GPIO_AF_M5            GPIO_AF_TIM2

#define MOTORS_GPIO_M7_PERIF         RCC_AHB1Periph_GPIOA
#define MOTORS_GPIO_M7_PORT          GPIOA
#define MOTORS_GPIO_M7_PIN           GPIO_Pin_15 // TIM2_CH1
#define MOTORS_GPIO_AF_M7_PIN        GPIO_PinSource15
#define MOTORS_GPIO_AF_M7            GPIO_AF_TIM2





/* Utils Conversion macro */
#ifdef BRUSHLESS_MOTORCONTROLLER
#define C_BITS_TO_16(X) (0xFFFF * (X - MOTORS_PWM_CNT_FOR_1MS) / MOTORS_PWM_CNT_FOR_1MS)
#define C_16_TO_BITS(X) (MOTORS_PWM_CNT_FOR_1MS + ((X * MOTORS_PWM_CNT_FOR_1MS) / 0xFFFF))
#define C_BITS_TO_16_(X) ((X)<<(16-MOTORS_PWM_BITS1))
#define C_16_TO_BITS_(X) ((X)>>(16-MOTORS_PWM_BITS1)&((1<<MOTORS_PWM_BITS1)-1))
#else
#define C_BITS_TO_16(X) ((X)<<(16-MOTORS_PWM_BITS))
#define C_16_TO_BITS(X) ((X)>>(16-MOTORS_PWM_BITS)&((1<<MOTORS_PWM_BITS)-1))
#endif

const int MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };
static bool isInit = false;

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit()
{
	if (isInit)
		return;

	//Init structures
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure1;
	GPIO_InitTypeDef GPIO_InitStructure2;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure1;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure1;

	//Clock the gpio and the timers
	RCC_AHB1PeriphClockCmd(MOTORS_GPIO_M1_PERIF |
		MOTORS_GPIO_M3_PERIF | MOTORS_GPIO_M4_PERIF, ENABLE);

	RCC_APB1PeriphClockCmd(MOTORS_TIM_M1_PERIF |
		MOTORS_TIM_M3_PERIF | MOTORS_TIM_M4_PERIF, ENABLE);

	RCC_AHB1PeriphClockCmd(MOTORS_GPIO_M5_PERIF |
		MOTORS_GPIO_M7_PERIF, ENABLE);
	RCC_APB1PeriphClockCmd(MOTORS_TIM_M5_PERIF |
		MOTORS_TIM_M7_PERIF, ENABLE);


	// Configure the GPIO for the timer output for 2 servos
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin = MOTORS_GPIO_M1_PIN;
	GPIO_Init(MOTORS_GPIO_M1_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = MOTORS_GPIO_M3_PIN;
	GPIO_Init(MOTORS_GPIO_M3_PORT, &GPIO_InitStructure);

	//Timer for the DC motor
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure2.GPIO_Pin = MOTORS_GPIO_M4_PIN;
	GPIO_Init(MOTORS_GPIO_M4_PORT, &GPIO_InitStructure2);

	//Timers for the Vcc pins
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure1.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure1.GPIO_Pin = MOTORS_GPIO_M5_PIN;
	GPIO_Init(MOTORS_GPIO_M5_PORT, &GPIO_InitStructure1);
	GPIO_InitStructure1.GPIO_Pin = MOTORS_GPIO_M7_PIN;
	GPIO_Init(MOTORS_GPIO_M7_PORT, &GPIO_InitStructure1);


	//Map timers to alternate functions.
	GPIO_PinAFConfig(MOTORS_GPIO_M1_PORT, MOTORS_GPIO_AF_M1_PIN, MOTORS_GPIO_AF_M1);
	GPIO_PinAFConfig(MOTORS_GPIO_M3_PORT, MOTORS_GPIO_AF_M3_PIN, MOTORS_GPIO_AF_M3);
	GPIO_PinAFConfig(MOTORS_GPIO_M4_PORT, MOTORS_GPIO_AF_M4_PIN, MOTORS_GPIO_AF_M4);

	//Timer configuration for servos
	TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(MOTORS_TIM_M1, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(MOTORS_TIM_M3, &TIM_TimeBaseStructure);

	//Timer configuration for DC motor
	TIM_TimeBaseStructure1.TIM_Period = MOTORS_PWM_PERIOD1;
	TIM_TimeBaseStructure1.TIM_Prescaler = MOTORS_PWM_PRESCALE1;
	TIM_TimeBaseStructure1.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure1.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure1.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(MOTORS_TIM_M4, &TIM_TimeBaseStructure1);

	// PWM channels configuration for servos
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
#ifdef BRUSHLESS_INVERSED_POLARITY
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#else
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
#endif

	// PWM channels configuration for DC motor
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure1.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure1.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure1.TIM_Pulse = 0;
	TIM_OCInitStructure1.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure1.TIM_OCIdleState = TIM_OCIdleState_Set;


	// Configure Output Compare for PWM (servos)
	M1_TIM_OC_INIT(MOTORS_TIM_M1, &TIM_OCInitStructure);
	M1_TIM_OC_PRE_CFG(MOTORS_TIM_M1, TIM_OCPreload_Enable);
	M3_TIM_OC_INIT(MOTORS_TIM_M3, &TIM_OCInitStructure);
	M3_TIM_OC_PRE_CFG(MOTORS_TIM_M3, TIM_OCPreload_Enable);

	// Configure Output Compare for PWM (DC motor)
	M4_TIM_OC_INIT(MOTORS_TIM_M4, &TIM_OCInitStructure1);
	M4_TIM_OC_PRE_CFG(MOTORS_TIM_M4, TIM_OCPreload_Enable);

#ifdef BRUSHLESS_MOTORCONTROLLER
	TIM_Cmd(MOTORS_TIM_M1, ENABLE);
	TIM_Cmd(MOTORS_TIM_M3, ENABLE);
	TIM_Cmd(MOTORS_TIM_M4, ENABLE);
#else
	{
		int tempCR1_M1_2_3;
		int tempCR1_M4;

		// Try to sync counters...
		tempCR1_M1_2_3 = MOTORS_TIM_M1->CR1 | TIM_CR1_CEN;
		tempCR1_M4 = MOTORS_TIM_M4->CR1 | TIM_CR1_CEN;
		//Enable the timer
		portDISABLE_INTERRUPTS();
		MOTORS_TIM_M1->CR1 = tempCR1_M1_2_3;
		MOTORS_TIM_M4->CR1 = tempCR1_M4;
		portENABLE_INTERRUPTS();
	}
#endif


	//Enable the timer PWM outputs
	TIM_CtrlPWMOutputs(MOTORS_TIM_M1, ENABLE);
	TIM_CtrlPWMOutputs(MOTORS_TIM_M3, ENABLE);
	TIM_CtrlPWMOutputs(MOTORS_TIM_M4, ENABLE);

	// Halt timer during debug halt.
	DBGMCU_APB2PeriphConfig(MOTORS_TIM_M1_DBG, ENABLE);
	DBGMCU_APB2PeriphConfig(MOTORS_TIM_M3_DBG, ENABLE);
	DBGMCU_APB2PeriphConfig(MOTORS_TIM_M4_DBG, ENABLE);

	//Setting two pins to always output Vcc
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	GPIO_SetBits(GPIOA, GPIO_Pin_15);

	isInit = true;
}

bool motorsTest(void)
{
#ifndef BRUSHLESS_MOTORCONTROLLER
	int i;
	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{
		motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
		vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
		motorsSetRatio(MOTORS[i], 0);
		vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
	}
#endif
	return isInit;
}

#ifdef ENABLE_THRUST_BAT_COMPENSATED
// Ithrust is thrust mapped for 65536 <==> 60g
void motorsSetRatio(int id, uint16_t ithrust)
{
	float thrust = ((float)ithrust / 65536.0f) * 60;
	float volts = -0.0006239 * thrust * thrust + 0.088 * thrust + 0.069;
	float supply_voltage = pmGetBatteryVoltage();
	float percentage = volts / supply_voltage;
	percentage = percentage > 1.0 ? 1.0 : percentage;
	uint16_t ratio = percentage * UINT16_MAX;

	switch (id)
	{
	case MOTOR_M1:
		M1_TIM_SETCOMPARE(MOTORS_TIM_M1, C_16_TO_BITS(ratio));
		break;
	case MOTOR_M2:
		break;
	case MOTOR_M3:
		M3_TIM_SETCOMPARE(MOTORS_TIM_M3, C_16_TO_BITS(ratio));
		break;
	case MOTOR_M4:
		M4_TIM_SETCOMPARE(MOTORS_TIM_M4, C_16_TO_BITS_(ratio));
		break;
	}
}
#else
void motorsSetRatio(int id, uint16_t ratio)
{
	switch (id)
	{
	case MOTOR_M1:
		M1_TIM_SETCOMPARE(MOTORS_TIM_M1, C_16_TO_BITS(ratio));
		break;
	case MOTOR_M2:
		break;
	case MOTOR_M3:
		M3_TIM_SETCOMPARE(MOTORS_TIM_M3, C_16_TO_BITS(ratio));
		break;
	case MOTOR_M4:
		M4_TIM_SETCOMPARE(MOTORS_TIM_M4, C_16_TO_BITS_(ratio));
		break;
	}
}
#endif

int motorsGetRatio(int id)
{
	switch (id)
	{
	case MOTOR_M1:
		return C_BITS_TO_16(M1_TIM_GETCAPTURE(MOTORS_TIM_M1));
	case MOTOR_M3:
		return C_BITS_TO_16(M3_TIM_GETCAPTURE(MOTORS_TIM_M3));
	case MOTOR_M4:
		return C_BITS_TO_16_(M4_TIM_GETCAPTURE(MOTORS_TIM_M4));
	}

	return -1;
}

#ifdef MOTOR_RAMPUP_TEST
// FreeRTOS Task to test the Motors driver with a rampup of each motor alone.
void motorsTestTask(void* params)
{
	int step = 0;
	float rampup = 0.01;

	motorsSetupMinMaxPos();
	motorsSetRatio(MOTOR_M4, 1 * (1 << 16) * 0.0);
	motorsSetRatio(MOTOR_M3, 1 * (1 << 16) * 0.0);
	motorsSetRatio(MOTOR_M1, 1 * (1 << 16) * 0.0);
	vTaskDelay(M2T(1000));

	while (1)
	{
		vTaskDelay(M2T(100));

		motorsSetRatio(MOTOR_M4, 1 * (1 << 16) * rampup);
		motorsSetRatio(MOTOR_M3, 1 * (1 << 16) * rampup);
		motorsSetRatio(MOTOR_M1, 1 * (1 << 16) * rampup);

		rampup += 0.001;
		if (rampup >= 0.1)
		{
			if (++step>3) step = 0;
			rampup = 0.01;
		}
	}
}
#else
// FreeRTOS Task to test the Motors driver
void motorsTestTask(void* params)
{
	static const int sequence[] = { 0.1*(1 << 16), 0.15*(1 << 16), 0.2*(1 << 16), 0.25*(1 << 16) };
	int step = 0;

	//Wait 3 seconds before starting the motors
	vTaskDelay(M2T(3000));

	while (1)
	{
		motorsSetRatio(MOTOR_M4, sequence[step % 4]);
		motorsSetRatio(MOTOR_M3, sequence[(step + 1) % 4]);
		motorsSetRatio(MOTOR_M1, sequence[(step + 3) % 4]);

		if (++step>3) step = 0;

		vTaskDelay(M2T(1000));
	}
}
#endif
