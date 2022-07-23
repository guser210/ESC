/*
 * main.cpp
 *
 *  Created on: July 18, 2022
 *      Author: gvargas
 */

#include "main.h"

extern "C"
{
extern TIM_HandleTypeDef htim1;		// Signal input capture.
extern TIM_HandleTypeDef htim3;		// PWM driver.
extern TIM_HandleTypeDef htim7;		// Generic timer.
extern TIM_HandleTypeDef htim17;	// commutator bump timer.
extern  ADC_HandleTypeDef hadc1;	// battery voltages.
}

uint8_t newDmaSignal = 0;
uint16_t dmaSignal[16] = {0};
uint16_t dmaSignalNormalized[16] = {0};
uint16_t motorSpeed = 0;
uint16_t motorSpeedCurrent = 0;


//////////////////////////////////////////////////////////////////////
const uint16_t dmaPulse = 1;
const uint16_t dmaPulseReload = 1024;
volatile uint16_t dmaBuffer[dmaPulse] = {0};
volatile uint16_t resolution = dmaPulse * dmaPulseReload;
void setDutyCycle(uint16_t dc)
{
	if( dc > resolution) dc = resolution;

	dmaBuffer[0] = dc;
}


const uint32_t zcOff[6] = {//
		ZC_B_Pin, // B
		ZC_A_Pin, // A
		ZC_C_Pin, // C
		ZC_B_Pin, // B
		ZC_A_Pin, // A
		ZC_C_Pin, // C
};

const uint32_t odLow[6] = {//
		OD_C_Pin, // C
		OD_C_Pin, // C
		OD_A_Pin, // A
		OD_A_Pin, // A
		OD_B_Pin, // B
		OD_B_Pin, // B
};
const uint32_t odHigh[6] = {//
		OD_A_Pin, // A
		OD_B_Pin, // B
		OD_B_Pin, // B
		OD_C_Pin, // C
		OD_C_Pin, // C
		OD_A_Pin, // A
};

const uint32_t odOff[6] = {//
		OD_B_Pin, // B
		OD_A_Pin, // A
		OD_C_Pin, // C
		OD_B_Pin, // B
		OD_A_Pin, // A
		OD_C_Pin, // C
};

const uint32_t ccOff[6] = {// TIME3->CCER For pwm output, RM0454 page 495.
		1<<8, // B
		1<<4, // A
		1<<12, // C
		1<<8, // B
		1<<4, // A
		1<<12, // C
};

const uint32_t diOff[6] = {// TIM3->DIER for PWM output RM0454 page .
		1<<11, // B
		1<<10, // A
		1<<12, // C
		1<<11, // B
		1<<10, // A
		1<<12, // C
};

const GPIO_TypeDef* zcPortOff[6] = {//
		(GPIO_TypeDef*)ZC_B_GPIO_Port, // B
		(GPIO_TypeDef*)ZC_A_GPIO_Port, // A
		(GPIO_TypeDef*)ZC_C_GPIO_Port, // C
		(GPIO_TypeDef*)ZC_B_GPIO_Port, // B
		(GPIO_TypeDef*)ZC_A_GPIO_Port, // A
		(GPIO_TypeDef*)ZC_C_GPIO_Port, // C
};
volatile uint32_t resetImrFlags = 0;
volatile uint8_t powerStep = 0;
volatile uint8_t powerStepCurrent = 0;

const uint8_t rising[2][6] = {{1,0,1,0,1,0},
							  {0,1,0,1,0,1}};

volatile uint8_t reverse = 0;

#define zcPinOn ZC_A_Pin | ZC_B_Pin | ZC_C_Pin
inline void commutate()
{

	// go to next power step.
	powerStepCurrent++;
	powerStepCurrent %= 6;

	if( reverse)
		powerStep = 5 - powerStepCurrent;
	else
		powerStep = powerStepCurrent; // forward.

	if( rising[reverse][powerStep])
	{
	// if zc rising
	//		enable zc rising interrupt
		EXTI->FTSR1 = 0;
		EXTI->RTSR1 = zcOff[powerStep];
	//		enable odLow
		GPIOA->BSRR = odLow[powerStep];
		GPIOA->BRR = odOff[powerStep]; // disable odOff

		TIM3->CCER |= (ccOff[powerStep]);
	//		enable dma on timer 3.
		TIM3->DIER |= (diOff[powerStep]);
	}else
	{
	// else
	//		enable zc falling int.
		EXTI->RTSR1 = 0;
		EXTI->FTSR1 = zcOff[powerStep];
	//		enable odHigh.
		GPIOA->BRR = odOff[powerStep]; // disable odOff
		GPIOA->BSRR = odHigh[powerStep];

		TIM3->CCER &= ~(ccOff[powerStep]);
		//		disable dma on timer 3
		TIM3->DIER &= ~(diOff[powerStep]);

	}


	// add delay of 2 us.
//	TIM7->CNT = 0;
	//while(TIM7->CNT < 128); // 2 us.

	// enable wakeup with interrupt.
	EXTI->IMR1 = resetImrFlags | zcOff[powerStep];
	// clear any zc pending interrupts.
	EXTI->FPR1 = zcPinOn;//(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin);
	EXTI->RPR1 = zcPinOn; //(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin);
	// reset timer17 for bump motor.
	TIM17->CNT = 0;
}

volatile uint16_t checkRising = 0;
volatile uint16_t checkFalling = 0;
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	TIM7->CNT = 0;

	while( TIM7->CNT < checkRising)
		if( (zcPortOff[powerStep]->IDR & zcOff[powerStep]) == 0)
			return;

//	if( powerStep == 0 || powerStep == 3)
//		LED_GPIO_Port->BSRR = LED_Pin;

	commutate();
}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{

	TIM7->CNT = 0;

	while( TIM7->CNT < checkFalling)
		if( (zcPortOff[powerStep]->IDR & zcOff[powerStep]) )
			return;

//	if( powerStep == 0 || powerStep == 3)
//		LED_GPIO_Port->BRR = LED_Pin;

	commutate();
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim == &htim17 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		if( reverse == 0)
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		if( motorSpeedCurrent > 0)
		{// Bump
			commutate();
		}
}

//////////////////////////////////////////////////////////////////////


uint16_t divClosest( uint16_t a, uint16_t b)
{
	return (a + b/ 2) / b;
}

volatile uint8_t frequencyType = 0;// 0 = Proshot, 1=PWM.
void processDmaSignalPWM()
{
	newDmaSignal = 0;

	uint8_t found = 0;

	uint16_t pwm1 = divClosest( dmaSignal[2] - dmaSignal[1] , 8);
	uint16_t pwm2 = divClosest(dmaSignal[1] - dmaSignal[0] , 8);

	pwm1 = pwm1 < pwm2 ? pwm1 : pwm2;

	if( pwm1 < 900 || pwm1 > 2010)
	{
		frequencyType = 0;
		return;
	}

	motorSpeed = pwm1 > 1000 && pwm1 < 2010 ? (pwm1 + 24) - 1000 : 0;
	dmaSignal[0] = motorSpeed;


	return;


}
void processDmaSignal()
{
	newDmaSignal = 0;

	for( int index = 0; index < 15; index++)
		dmaSignalNormalized[index] = (dmaSignal[index + 1] - dmaSignal[index]) - 8;

	uint16_t pwm1 = divClosest( dmaSignal[2] - dmaSignal[1] , 8);
	uint16_t pwm2 = divClosest(dmaSignal[1] - dmaSignal[0] , 8);

	pwm1 = pwm1 < pwm2 ? pwm1: pwm2;
	if( pwm1 > 998 && pwm1 < 2010)
	{
		frequencyType = 1;
		return;
	}

	for( int index = 0; index <= 10; index++)
	{
		if( dmaSignalNormalized[index] < 100)
			continue;

		uint16_t speed = dmaSignalNormalized[index + 1]<<6 |
					 dmaSignalNormalized[index + 3]<<2 |
					 dmaSignalNormalized[index + 5]>>2;
		if(  speed > 2000)
			return; // invalid value.


		motorSpeed = speed > 0 && speed < 1025 ? speed : 0;
		break;
	}

}

//void processDmaSignal()
//{
//	newDmaSignal = 0;
//
//	for( int index = 0; index < 15; index++)
//		dmaSignalNormalized[index] = divClosest( dmaSignal[index + 1] - dmaSignal[index], 8) - 8;
//
//
//	for( int index = 0; index <= 10; index++)
//	{
//		if( dmaSignalNormalized[index] < 100)
//			continue;
//
//		motorSpeed = dmaSignalNormalized[index + 1]<<6 |
//					 dmaSignalNormalized[index + 3]<<2 |
//					 dmaSignalNormalized[index + 5]>>2;
//
//		break;
//	}
//
//}

void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *htim)
{
	if(htim == &htim1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		newDmaSignal = 1;
}

void setup()
{
	EXTI->FTSR1 = 0;
	EXTI->RTSR1 = 0;
	HAL_Delay(1000);
	resetImrFlags = EXTI->IMR1;
	resetImrFlags &= ~(ZC_A_Pin | ZC_B_Pin | ZC_C_Pin );
	EXTI->IMR1 = resetImrFlags;

	HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dmaSignal, 16);



	if( HAL_TIM_Base_Start(&htim3) != HAL_OK)
		Error_Handler();

	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t*)dmaBuffer, dmaPulse);
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t*)dmaBuffer, dmaPulse);
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t*)dmaBuffer, dmaPulse);


	__HAL_TIM_SET_COMPARE(&htim17,TIM_CHANNEL_1,250);
	HAL_TIM_OC_Start_IT(&htim17, TIM_CHANNEL_1);

}

inline uint16_t map(int x, int in_min, int in_max, int out_min, int out_max)
{
	if (x < in_min)	x = in_min;

	if (x > in_max) x = in_max;

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

volatile double batteryVoltages = 0.0f;
volatile double adcValue = 0;
volatile uint16_t fallingPaddingValue = 0;

uint8_t memorySettings[32] = {0};
int sizeOfSettings = sizeof(memorySettings);
void readMemory(unsigned char* data, int size, int location){
	location *= 8;

	for( int index = 0; index < size; index++){
		data[index] =  (unsigned int)(*(uint64_t*)(0x0801F800 + location));
		location += 8;
	}

}


void writeMemory(unsigned char* data, int size, int location){

	readMemory(memorySettings, sizeOfSettings, 0);

	size = size > sizeOfSettings ? sizeOfSettings : size;
	location = location >= sizeOfSettings ? sizeOfSettings : location;

	for( int index = 0; index < size; index++){
		location = location >= sizeOfSettings ? sizeOfSettings : location;
		memorySettings[location++] = data[index];
	}

	FLASH_EraseInitTypeDef epage;
	epage.TypeErase = FLASH_TYPEERASE_PAGES; // FLASH_TYPEERASE_PAGES
	epage.Page = 63;
	epage.NbPages = 1;


	uint32_t error = 0;
	HAL_StatusTypeDef ret = HAL_FLASH_Unlock();

	ret = HAL_FLASHEx_Erase(&epage, &error);
	for( int index = 0; index < sizeOfSettings; index++){
		ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F800 + (index * 8), memorySettings[index]);
	}

	ret = HAL_FLASH_Unlock();

}
void maincpp()
{
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start(&hadc1);
	if( HAL_TIM_Base_Start(&htim7) != HAL_OK)
		Error_Handler();

	HAL_Delay(500);
	setup();


	uint8_t direction = 0;

	uint8_t memorySetting[1] = {0};
//
//	writeMemory(data, 1, 0);

	readMemory(memorySettings, sizeOfSettings, 0);
	if( memorySettings[0] != 1)
		memorySettings[0] = 0;

	reverse = memorySettings[0];

	frequencyType = 0;

	while(1)
	{
		if((REV_GPIO_Port->IDR & REV_Pin) == 0)
		{// switch direction when button pressed.
			LED_GPIO_Port->BRR = LED_Pin;
			HAL_Delay(300);
			LED_GPIO_Port->BSRR = LED_Pin;

			if( direction == 1)
				direction = 0;
			else
				direction = 1;

			memorySetting[0] = direction;
			writeMemory(memorySetting, 1, 0);

			while((REV_GPIO_Port->IDR & REV_Pin) == 0)
				HAL_Delay(100);
			LED_GPIO_Port->BRR = LED_Pin;
		}

		if( reverse == 1)
			LED_GPIO_Port->BSRR = LED_Pin;

		if( newDmaSignal)
		{
			if( frequencyType == 0)
				processDmaSignal();
			else
				processDmaSignalPWM();

//			if( GPIOA->IDR & REV_Pin )
//				direction = 1;
//			else
//				direction = 0;
			if( direction != reverse )
			{
				motorSpeedCurrent = 0;
				setDutyCycle(motorSpeedCurrent);
				reverse = direction;
				HAL_Delay(100);
			}

			adcValue = HAL_ADC_GetValue(&hadc1);

			if( adcValue > 0)
			{
				//											rb/rt
				batteryVoltages = adcValue*(3.3 / 4095.0)/(10000.0/110000.0);

				fallingPaddingValue = map(batteryVoltages,8,25,1,400);
			}

			if( motorSpeed != motorSpeedCurrent)
			{
				if( (motorSpeed - motorSpeedCurrent) > 20)
					motorSpeedCurrent += 20;
				else
					motorSpeedCurrent = motorSpeed;

				if( motorSpeedCurrent < 150 )
				{
					//checkRising = 500;// 1288;
//					checkRising = 1300 - map(batteryVoltages,8,21,1,1000);
					checkRising = 1400 - map(batteryVoltages,8,21,1,
							map(motorSpeedCurrent, 1,200,2,1250)

							);
				}
				else
				{
					checkRising =  map(1024 - motorSpeedCurrent,0,1024,1,320);
				}
				checkFalling =  map(1024 - motorSpeedCurrent,0,1024,1,1256);
				setDutyCycle(motorSpeedCurrent);
			}

			if( frequencyType == 1) // PWM
			{
				TIM1->CNT = 0;
				HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dmaSignal, 3);
			}
			else
			{
				TIM1->CNT = 0;
				HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dmaSignal, 16);
			}

		}

	}
}


