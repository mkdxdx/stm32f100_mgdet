/*
 * Согласно техзаданию нужно получить срабатывание на канале B ОУ DA7 который работает в режиме компаратора с опорным напряжением 0.99В.
 *
 * К неинвертирующему входу подключён канал А, который работает в режиме неинвертирующего усилителя, коеффициент которого
 * нужно выставить так, чтобы на выходе получить напряжение равное или выше опорному канала B.
 *
 * От пьезоэлемента через выпрямляющую схему и делитель (/2) сигнал поступает на АЦП микроконтроллера, который исходя из замера должен вычислить
 * и выставить сопротивление верхней части обратной связи ОУ так, чтобы получить нужный коеффициент усиления канала А.
 *
 * Верхняя часть (вых А -> инв. вход А) обратной связи состоит из резистора на 51КОм и цифрового потенциометра, сопротивление которого
 * нужно узнать и которое варьируется от 0 до 100КОм с шагом в 780Ом на разряд (127 шагов возможно)
 *
 * Целевой коефициент после замера на АЦП вычисляется по формуле:
 * k = Vref/Vin,
 * где Vref - опорное напряжение компаратора (0.99В), Vin - напряжение, замеренное на канале АЦП
 *
 * Так как коефициент усиления на неинвертирующем усилителя вычисляется по формуле:
 * k = 1+(R2/R1) и R1 у нас известно (1КОм), необходимо вычислить значение R2 исходя из целевого коефициента.
 *
 * R2 = (k - 1) * R1
 *
 * Получив R2 как значение верхней части усилителя, я получил общее значение сопротивления, которое состоит из параллельных резисторов
 * один из которых известен (51КОм) и второй является потенциометром, сопротивление которого нужно задать чтобы получить нужный коеффициент
 * усиления.
 *
 * Так как формула сопротивления параллельных резисторов (одна из) выглядит так:
 *
 * Rп = (R2*Rx)/(R2+Rx)
 *
 * Где Rx - неизвестное сопротивление потенциометра, Rп - известное целевое сопротивление, R2 - известное значение (51КОм)
 * , можно вывести формулу сопротивления для потенциометра:
 *
 * Rx = (R2*Rп)/(R2-Rп)
 *
 * Полученное значение Rx необходимо записать в цифровой потенциометр для установки коеффициента усиления.
 * Цифровой потенциометр работает с шагом в ~780Ом, необходимо вычислить число шагов для записи в его регистр:
 *
 * Steps = Rx/StepV
 *
 * где Steps - нужное количество шагов, Rx полученное сопротивление, StepV - значение шага на потенциометре.
 *
 * К примеру, на АЦП поступил сигнал напряжением в 0.1V. Целевой коеффициент будет равен:
 *
 * k = 0.99/0.1 = 9,9
 *
 * Получим целевое сопротивление верхней части обратной связи:
 *
 * R2 = (9,9 - 1) * 1000 = 8900 Ом
 *
 * Получим значение потенциометра исходя из цепи параллельных резисторов:
 *
 * Rx = (51000 * 8900)/(51000 - 8900) = 10781 Ом
 *
 * Получим значение регистра потенциометра:
 *
 * Steps = 10781/780 = 14
 *
 * Проверим результат:
 *
 * Значение потенциометра при 14 шагах станет:
 * 14*780 = 10920 Ом
 * Верхняя часть обратной связи усилителя будет иметь сопротивление:
 * (51000*10920)/(51000+10920) = 8994,2 Ом
 * Коефициент усиления канала А будет равен:
 * 1 + (8994,2/1000) = 9,9942
 * Выходное напряжение канала А будет равно:
 * 9,9942 * 0,1 = 0,99942
 * Компаратор канала В подаст сигнал на МК и зажгётся зелёный светодиод вместе с красным (штатный режим)
 *

 *
 */


#include "stm32f10x.h"
#include "delay.h"
#include "i2c_gen.h"

#include "stm32f10x_i2c.h"

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

