#include "stm32f10x.h"
#include "delay.h"
#include "i2c_gen.h"


#define LED_PORT		GPIOB
#define LED_PORT_RCC	RCC_APB2Periph_GPIOB
#define LED_PIN_GREEN	GPIO_Pin_2	// schematic typo, GREEN led is lit when PB2 is high.
#define LED_PIN_RED		GPIO_Pin_0

#define LED_GREEN_ON()	GPIO_SetBits(LED_PORT, LED_PIN_GREEN)
#define LED_GREEN_OFF()	GPIO_ResetBits(LED_PORT, LED_PIN_GREEN)
#define LED_RED_ON()	GPIO_SetBits(LED_PORT, LED_PIN_RED)
#define LED_RED_OFF()	GPIO_ResetBits(LED_PORT, LED_PIN_RED)

#define OP_PORT			GPIOA
#define OP_PORT_RCC		RCC_APB2Periph_GPIOA
#define OP_PIN			GPIO_Pin_8


#define SVC_PORT		GPIOA
#define SVC_PORT_RCC	RCC_APB2Periph_GPIOA
#define SVC_PIN			GPIO_Pin_7
#define SVC_PIN_PS		GPIO_PinSource7
#define SVC_PORT_SOURCE GPIO_PortSourceGPIOA


#define AN_PORT			GPIOA
#define AN_PORT_RCC		RCC_APB2Periph_GPIOA
#define AN_PIN_BUZZER	GPIO_Pin_4

#define AN_ADCx			ADC1
#define AN_ADCx_RCC		RCC_APB2Periph_ADC1
#define AN_CHx			ADC_Channel_4

#define EXTIn_IRQn		EXTI9_5_IRQn
#define EXTI_Linex		EXTI_Line7


#define MODE_CAL		0
#define MODE_WRK		1

#define COMP_REFV		0.99F
#define ADC_REFV		3.3F
#define POT_STEPS		128U
#define POT_RES			100000U
#define POT_STEP		POT_RES/POT_STEPS
#define OP_R1_R			1000U	// R30, DA7 InvA -> GND
#define PAR_R2_R		51000U  // R25, OutA -> InvA
#define VOLT_DIV		2		// ADC voltage divider factor

#define POT_ADDR		0x5E

void InitRCC(void);
void InitGPIO(void);
void InitADC(void);
void InitIT(void);
void InitI2C(void);
void WriteI2C(uint8_t addr, uint8_t reg, uint8_t data);
void SetPotValue(uint8_t step);
float GetTargetK(float vin);
float GetTargetR(float k);
uint8_t GetWStep(float r);
float GetPotR(float tr);
float GetADCVolt(uint16_t av);
void PotWrite(uint8_t data);

volatile uint16_t trig_level;
volatile uint8_t mode = MODE_WRK;



void EXTI9_5_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Linex) != RESET) {
		if (GPIO_ReadInputDataBit(SVC_PORT, SVC_PIN) == SET) {
			mode = MODE_CAL;
		} else {
			mode = MODE_WRK;
		}
		EXTI_ClearITPendingBit(EXTI_Linex);
	}
}




int main(void)
{

	float tempf;

	InitRCC();
	InitGPIO();
	InitADC();

	I2CInit();
	InitIT();
	Delay_Init(24);

	// reset pot value
	PotWrite(0x00);


    while(1) {
    	if (mode == MODE_CAL) {
    		LED_GREEN_ON();
    		LED_RED_OFF();
    		uint16_t av = ADC_GetConversionValue(AN_ADCx);
    		if (av > trig_level) {
    			trig_level = av;
    			tempf = GetADCVolt(trig_level)*VOLT_DIV; // get voltage from piezo
    			tempf = GetTargetK(tempf);		// get target amp coefficient
    			tempf = GetTargetR(tempf);		// get R2 for opamp feedback
    			tempf = GetPotR(tempf);			// get target resistance for pot as part of R2
    			PotWrite(GetWStep(tempf));		// write step to pot
    		}

    	} else {
    		// turn on red for working mode indication, turn green if mcu gets opamp output high
    		LED_RED_ON();
    		if (GPIO_ReadInputDataBit(OP_PORT, OP_PIN) == SET) {
    			LED_GREEN_ON();
    			delay_ms(500);
    		} else {
    			LED_GREEN_OFF();
    		}


    	}

    }

}



void InitRCC(void) {
	RCC_APB2PeriphClockCmd((LED_PORT_RCC | AN_PORT_RCC | AN_ADCx_RCC | LED_PORT_RCC | SVC_PORT_RCC | I2Cx_PORT_RCC | OP_PORT_RCC), ENABLE);
	RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);
}

void InitGPIO(void) {
	GPIO_InitTypeDef port;

	port.GPIO_Pin = LED_PIN_GREEN | LED_PIN_RED;
	port.GPIO_Mode = GPIO_Mode_Out_PP;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_PORT, &port);

	port.GPIO_Pin = AN_PIN_BUZZER;
	port.GPIO_Mode = GPIO_Mode_AIN;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(AN_PORT, &port);

	port.GPIO_Pin = SVC_PIN;
	port.GPIO_Mode = GPIO_Mode_IPD;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SVC_PORT, &port);

	port.GPIO_Pin = OP_PIN;
	port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	port.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OP_PORT, &port);



}

void InitADC(void) {
	ADC_InitTypeDef adc;

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	adc.ADC_Mode = ADC_Mode_Independent;
	adc.ADC_NbrOfChannel = 1;
	adc.ADC_ScanConvMode = DISABLE;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_ContinuousConvMode = ENABLE;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_Init(AN_ADCx, &adc);

	ADC_RegularChannelConfig(AN_ADCx, AN_CHx, 1, ADC_SampleTime_7Cycles5);
	ADC_Cmd(AN_ADCx, ENABLE);

	ADC_ResetCalibration(AN_ADCx);

	while (ADC_GetResetCalibrationStatus(AN_ADCx)) {};
	ADC_StartCalibration(AN_ADCx);

	while (ADC_GetCalibrationStatus(AN_ADCx));
	ADC_SoftwareStartConvCmd(AN_ADCx,ENABLE);


}




void InitIT(void) {
	NVIC_InitTypeDef nvis;
	EXTI_InitTypeDef exts;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	nvis.NVIC_IRQChannel = EXTIn_IRQn;
	nvis.NVIC_IRQChannelCmd = ENABLE;
	nvis.NVIC_IRQChannelPreemptionPriority = 0;
	nvis.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvis);

	GPIO_EXTILineConfig(SVC_PORT_SOURCE,SVC_PIN_PS);

	exts.EXTI_Line = EXTI_Linex;
	exts.EXTI_LineCmd = ENABLE;
	exts.EXTI_Mode = EXTI_Mode_Interrupt;
	exts.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&exts);

	__enable_irq();
}




void PotWrite(uint8_t data) {
	I2C_Start();
	I2C_TXInit(POT_ADDR);
	I2C_WriteByte(data);
	I2C_Stop();
}

float GetTargetK(float vin) {
	// get target coefficient
	return ((float)COMP_REFV/vin);
}



float GetTargetR(float k) {
	// get target resistance
	return (k - 1) * (float)OP_R1_R;
}

float GetPotR(float tr) {
	return ((float)PAR_R2_R * tr) / ((float)PAR_R2_R - tr);
}

uint8_t GetWStep(float r) {
	// get step for pot
	uint32_t ps = POT_STEP;
	return (uint8_t)((uint32_t)r/(uint32_t)ps);
}

float GetADCVolt(uint16_t av) {
	// get voltage
	return (ADC_REFV * ((float)(av)/(float)0xFFF));;
}

