#include <stdint.h>
#include <math.h>
#include "stm32f3xx_hal.h"
#include "main.h"
#include "functions.h"

uint16_t ADC_Measure[32] = {};
uint16_t i_buf[32];
const uint32_t DWA_DO_11=2048;
/*to dzia³a*/
void Measure_and_send(uint16_t Sine[32])
{

	HAL_ADC_Start_DMA(&hadc1,&ADC_Measure,32);
	for(int i=0; i<32; i++)
	{
		i_buf[i] = ADC_Measure[i];
	}
	HAL_UART_Transmit(&huart1, &i_buf,64,200 );
	uint32_t sum=0;

	for(int i=0;i <= 32; i++)
	{
		//sum=sum+(Sine[i]-DWA_DO_11)*ADC_Measure[i];
	}
	HAL_ADC_Stop_DMA(&hadc1);
	//uint32_t ADC_Liczba=sum/32;
	//HAL_UART_Transmit(&huart1, &ADC_Liczba,4,200 );
}
/*prawdopodobnie dzia³a*/
void Change_Freq_Manualy(uint16_t Sine[32],uint16_t PSC_VALUE)
{

	    TIM6->PSC = PSC_VALUE; // putting new PSC value to TIM6 register

}

void Impedance_auto_measure(uint16_t F_s,uint16_t F_e,uint16_t F_ch,uint16_t Sinet[32])
{


	if( ((F_e - F_s)%F_ch) == 0 && F_s>=1 && F_e<=50 && F_e != 0 && F_s != 0)
	{
		uint16_t	Calc_PSC_VALUE=0;
		uint16_t    Real_Freq = 0;

		for(int i=0; i <= round((F_e - F_s)/F_ch);i=i+1)
		{

		Calc_PSC_VALUE=round(2000/(F_s+i*F_ch))-1; // freq=Clock/(period-1)*(prescaler-1)
		Real_Freq=(uint16_t)round((20000/(Calc_PSC_VALUE+1))*100);

		/*--Change Freq--*/
		 Change_Freq_Manualy(Sinet,Calc_PSC_VALUE);
		 /*----------------------*/
		/*wysy³anie danych w postaci:
		 * Frequency,Data space
		 * Send(freq)
		 * Send(Data)
		 * Send("/")
		 */
		HAL_UART_Transmit(&huart1,(uint8_t) &Calc_PSC_VALUE,2,1000);
		//HAL_UART_Transmit(&huart1,",",1,200);
		//Measure_and_send(Sinet);
		//HAL_UART_Transmit(&huart1,"/",1,200);



		}
	}
}





