/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"	//inclus�o das defini��es do microcontrolador
#include <stdio.h>		//necess�rio para usar as fun��es printf() e scanf()
#include "Utility.h"	//arquivo de fun��es �teis

#define ARM_MATH_CM4	//defini��o da plataforma Cortex-M4 para a biblioteca arm_math.h
#include "arm_math.h"	//fun��es matem�ticas da ARM (incluindo FFT)

//Defini��es
#define FFT_BUFFER_SIZE	2048				//quantidade de amostras para c�lculo da FFT (pot�ncia de 2)
#define OFFSET			2048			//valor m�dio do conversor ADC

//Declara��o de vari�veis
arm_rfft_fast_instance_f32 fftHandler;	//estrutura ARM necess�ria para c�lculo da FFT
float fftBufIn[FFT_BUFFER_SIZE];		//buffer com as amostras de entrada
float fftBufOut[FFT_BUFFER_SIZE];		//buffer com o resultado da FFT
float fftBufMag[FFT_BUFFER_SIZE/2]; 	//buffer dos valores do m�dulo da FFT

//Prot�tipo de fun��es
void Configure_GPIO(void);		//configura��o dos pinos de GPIO
void Configure_ADC(void);		//configura��o do conversor AD
void Configure_Timer3(void);	//configura��o da base de tempo para amostragem


int main(void)
{
	Utility_Init();		//configura o sistema de clock
	USART1_Init();		//inicializa a USART
	Configure_GPIO();	//configura o pino de entrada anal�gica (PA0)
	Configure_ADC();	//configura o conversor AD
	Configure_Timer3();	//configura a base de tempo para amostragem de PA0

	printf("\n--------  Decifrador de Corda de Viol�o  --------\n\n");


	//printf("\n--------  Exemplo de aplica��o da FFT  --------\n\n");
	Delay_ms(1000);

	//Inicializa��o da estrutura ARM para FFT
	arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);

	//Liga o Timer3 para iniciar as convers�es
	TIM3->CR1 |= TIM_CR1_CEN;	//habilita o Timer3 para gera��o dos gatilhos de convers�o em PA0

	uint16_t fft_pointer = 0;	//ponteiro para o buffer de amostras do sinal de entrada
	while(1)
	{

		if(ADC1->SR & ADC_SR_EOC)	//aguarda a nova amostra estar dispon�vel
		{
			int16_t sample = (ADC1->DR) - OFFSET;	//faz a leitura do valor convertido;
			fftBufIn[fft_pointer] = sample;			//armazena a amostra no buffer
			++fft_pointer;							//atualiza o ponteiro

			if(fft_pointer == FFT_BUFFER_SIZE)		//verifica se o buffer est� cheio
			{
				fft_pointer = 0;	//reinicia o ponteiro do array

				//Execu��o da FFT
				//fftBufOut � formado por pares (n, n+1)
				//O elemento n � a parte real da FFT
				//O elemento n+1 � a parte imagin�ria da FFT
				arm_rfft_fast_f32(&fftHandler, fftBufIn, fftBufOut, 0);

				//printf("M�dulo da FFT:\n\n");
				//C�lculo do m�dulo da FFT
				for(uint16_t index = 0; index < FFT_BUFFER_SIZE; index += 2)
				{
					fftBufMag[index/2] = sqrt((fftBufOut[index] * fftBufOut[index]) + (fftBufOut[index + 1] * fftBufOut[index + 1]));

					//printf("%.1f\n", fftBufMag[index/2]/10000);	//divis�o por 10000 para reduzir os valores apresentados
				}

				Delay_ms(2000);

				float max = fftBufMag[1];
				int indice = 0;

				for(uint16_t index = 1; index < 20; index++)
				{
					if(fftBufMag[index] > max)
					{
						max = fftBufMag[index];
						indice = index;
						//printf("%f\n", max);
					 }
				}
				//printf("%d\n", indice);
				if (indice == 14)
				{
					printf("1� Corda E4\n\n");
					GPIOA->ODR |= 1 << 6;    //Liga LED pra 1� corda (E4)
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 11)
				{
					printf("2� Corda B3\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR |= 1 << 5;     //Liga LED pra 2� corda (B3)
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 8)
				{
					printf("3� Corda G3\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR |= 1 << 4;     //Liga LED pra 3� corda (G3)
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 6)
				{
					printf("4� Corda D3\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR |= 1 << 3;     //Liga LED pra 4� corda (D3)
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 5)
				{
					printf("5� Corda A2\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR |= 1 << 2;     //Liga LED pra 5� corda (A2)
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 7)
				{
					printf("6� Corda E2\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR |= 1 << 1;          //Liga LED pra 6� corda (E2)

				}
				else
				{
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR &= ~(1 << 1);

				}

				}

			}
	}
}


//Configura��o dos pinos de GPIO
void Configure_GPIO(void)
{
	//Configura��o dos pinos de GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	//habilita o clock do GPIOA
	GPIOA->MODER |= 0b11;					//pino PA0 como entrada anal�gica do ADC
	GPIOA->MODER |= 0b01 << 2;              //pino PA1 como sa�da digital
	GPIOA->MODER |= 0b01 << 4;              //pino PA2 como sa�da digital
	GPIOA->MODER |= 0b01 << 6;              //pino PA3 como sa�da digital
	GPIOA->MODER |= 0b01 << 8;              //pino PA4 como sa�da digital
	GPIOA->MODER |= 0b01 << 10;             //pino PA5 como sa�da digital
	GPIOA->MODER |= 0b01 << 12;             //pino PA6 como sa�da digital
}

//Configura��o do conversor ADC
void Configure_ADC(void)
{
	//Configura��o do ADC1
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	//liga o clock da interface digital do ADC1
	ADC->CCR |= 0b01 << 16;				//prescaler /4
	ADC1->SQR1 &= ~(0xF << 20);			//convers�o de apenas um canal
	ADC1->SQR3 &= ~(0x1F);				//sele��o do canal a ser convertido (IN_0)
	ADC1->SMPR2 = 0b001;				//sampling time do canal = (15)cycles*(1/ADCCLK) = 714,29 ns. Conversion time de cada canal = (15+12)cycles*(1/ADCCLK) = 1,28 us
	ADC1->CR2 |= 0b1000 << 24;			//seleciona TIM3_TRGO como fonte de gatilho do ADC1
	ADC1->CR2 |= ADC_CR2_EXTEN_0;		//habilita disparo externo em rising edge no ADC1
	ADC1->CR2 |= ADC_CR2_ADON;			//liga o conversor ADC1
}

//O Timer3 como base de tempo (taxa de amostragem de 48ksps)
void Configure_Timer3(void)
{
	//Configura��o do Timer3 para gera��o de gatilhos
	//Timer3 gera pulsos de gatilho (TRGO) em cada overflow
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;		//liga o clock do Timer3
	TIM3->PSC = 0;							//prescaler /1
	TIM3->EGR = TIM_EGR_UG;					//update event para escrever o valor de PSC
	TIM3->CR2 = TIM_CR2_MMS_1;				//master mode 010: update
	TIM3->ARR = 1749;						//overflow do contador a cada 20,83 us
	TIM3->CR1 |= TIM_CR1_URS;				//overflow gera um gatilho em TRGO
}
