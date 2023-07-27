/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"	//inclusão das definições do microcontrolador
#include <stdio.h>		//necessário para usar as funções printf() e scanf()
#include "Utility.h"	//arquivo de funções úteis

#define ARM_MATH_CM4	//definição da plataforma Cortex-M4 para a biblioteca arm_math.h
#include "arm_math.h"	//funções matemáticas da ARM (incluindo FFT)

//Definições
#define FFT_BUFFER_SIZE	2048				//quantidade de amostras para cálculo da FFT (potência de 2)
#define OFFSET			2048			//valor médio do conversor ADC

//Declaração de variáveis
arm_rfft_fast_instance_f32 fftHandler;	//estrutura ARM necessária para cálculo da FFT
float fftBufIn[FFT_BUFFER_SIZE];		//buffer com as amostras de entrada
float fftBufOut[FFT_BUFFER_SIZE];		//buffer com o resultado da FFT
float fftBufMag[FFT_BUFFER_SIZE/2]; 	//buffer dos valores do módulo da FFT

//Protótipo de funções
void Configure_GPIO(void);		//configuração dos pinos de GPIO
void Configure_ADC(void);		//configuração do conversor AD
void Configure_Timer3(void);	//configuração da base de tempo para amostragem


int main(void)
{
	Utility_Init();		//configura o sistema de clock
	USART1_Init();		//inicializa a USART
	Configure_GPIO();	//configura o pino de entrada analógica (PA0)
	Configure_ADC();	//configura o conversor AD
	Configure_Timer3();	//configura a base de tempo para amostragem de PA0

	printf("\n--------  Decifrador de Corda de Violão  --------\n\n");


	//printf("\n--------  Exemplo de aplicação da FFT  --------\n\n");
	Delay_ms(1000);

	//Inicialização da estrutura ARM para FFT
	arm_rfft_fast_init_f32(&fftHandler, FFT_BUFFER_SIZE);

	//Liga o Timer3 para iniciar as conversões
	TIM3->CR1 |= TIM_CR1_CEN;	//habilita o Timer3 para geração dos gatilhos de conversão em PA0

	uint16_t fft_pointer = 0;	//ponteiro para o buffer de amostras do sinal de entrada
	while(1)
	{

		if(ADC1->SR & ADC_SR_EOC)	//aguarda a nova amostra estar disponível
		{
			int16_t sample = (ADC1->DR) - OFFSET;	//faz a leitura do valor convertido;
			fftBufIn[fft_pointer] = sample;			//armazena a amostra no buffer
			++fft_pointer;							//atualiza o ponteiro

			if(fft_pointer == FFT_BUFFER_SIZE)		//verifica se o buffer está cheio
			{
				fft_pointer = 0;	//reinicia o ponteiro do array

				//Execução da FFT
				//fftBufOut é formado por pares (n, n+1)
				//O elemento n é a parte real da FFT
				//O elemento n+1 é a parte imaginária da FFT
				arm_rfft_fast_f32(&fftHandler, fftBufIn, fftBufOut, 0);

				//printf("Módulo da FFT:\n\n");
				//Cálculo do módulo da FFT
				for(uint16_t index = 0; index < FFT_BUFFER_SIZE; index += 2)
				{
					fftBufMag[index/2] = sqrt((fftBufOut[index] * fftBufOut[index]) + (fftBufOut[index + 1] * fftBufOut[index + 1]));

					//printf("%.1f\n", fftBufMag[index/2]/10000);	//divisão por 10000 para reduzir os valores apresentados
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
					printf("1º Corda E4\n\n");
					GPIOA->ODR |= 1 << 6;    //Liga LED pra 1º corda (E4)
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 11)
				{
					printf("2º Corda B3\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR |= 1 << 5;     //Liga LED pra 2º corda (B3)
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 8)
				{
					printf("3º Corda G3\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR |= 1 << 4;     //Liga LED pra 3º corda (G3)
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 6)
				{
					printf("4º Corda D3\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR |= 1 << 3;     //Liga LED pra 4º corda (D3)
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 5)
				{
					printf("5º Corda A2\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR |= 1 << 2;     //Liga LED pra 5º corda (A2)
					GPIOA->ODR &= ~(1 << 1);

				}
				else if (indice == 7)
				{
					printf("6º Corda E2\n\n");
					GPIOA->ODR &= ~(1 << 6);
					GPIOA->ODR &= ~(1 << 5);
					GPIOA->ODR &= ~(1 << 4);
					GPIOA->ODR &= ~(1 << 3);
					GPIOA->ODR &= ~(1 << 2);
					GPIOA->ODR |= 1 << 1;          //Liga LED pra 6º corda (E2)

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


//Configuração dos pinos de GPIO
void Configure_GPIO(void)
{
	//Configuração dos pinos de GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	//habilita o clock do GPIOA
	GPIOA->MODER |= 0b11;					//pino PA0 como entrada analógica do ADC
	GPIOA->MODER |= 0b01 << 2;              //pino PA1 como saída digital
	GPIOA->MODER |= 0b01 << 4;              //pino PA2 como saída digital
	GPIOA->MODER |= 0b01 << 6;              //pino PA3 como saída digital
	GPIOA->MODER |= 0b01 << 8;              //pino PA4 como saída digital
	GPIOA->MODER |= 0b01 << 10;             //pino PA5 como saída digital
	GPIOA->MODER |= 0b01 << 12;             //pino PA6 como saída digital
}

//Configuração do conversor ADC
void Configure_ADC(void)
{
	//Configuração do ADC1
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	//liga o clock da interface digital do ADC1
	ADC->CCR |= 0b01 << 16;				//prescaler /4
	ADC1->SQR1 &= ~(0xF << 20);			//conversão de apenas um canal
	ADC1->SQR3 &= ~(0x1F);				//seleção do canal a ser convertido (IN_0)
	ADC1->SMPR2 = 0b001;				//sampling time do canal = (15)cycles*(1/ADCCLK) = 714,29 ns. Conversion time de cada canal = (15+12)cycles*(1/ADCCLK) = 1,28 us
	ADC1->CR2 |= 0b1000 << 24;			//seleciona TIM3_TRGO como fonte de gatilho do ADC1
	ADC1->CR2 |= ADC_CR2_EXTEN_0;		//habilita disparo externo em rising edge no ADC1
	ADC1->CR2 |= ADC_CR2_ADON;			//liga o conversor ADC1
}

//O Timer3 como base de tempo (taxa de amostragem de 48ksps)
void Configure_Timer3(void)
{
	//Configuração do Timer3 para geração de gatilhos
	//Timer3 gera pulsos de gatilho (TRGO) em cada overflow
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;		//liga o clock do Timer3
	TIM3->PSC = 0;							//prescaler /1
	TIM3->EGR = TIM_EGR_UG;					//update event para escrever o valor de PSC
	TIM3->CR2 = TIM_CR2_MMS_1;				//master mode 010: update
	TIM3->ARR = 1749;						//overflow do contador a cada 20,83 us
	TIM3->CR1 |= TIM_CR1_URS;				//overflow gera um gatilho em TRGO
}
