#include <stm32f10x.h>
#include <stdio.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static unsigned int LEDState = 0;
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        if((char)USART_ReceiveData(USART1) == '1')
            LEDState = 2;
 
        if((char)USART_ReceiveData(USART1) == '0')
            LEDState = 1;
    }
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}



static xSemaphoreHandle trg = NULL;


static void USART_SendString(USART_TypeDef* USARTx, char* s)
{
    while(*s)
    {
        while(!USART_GetFlagStatus(USARTx, USART_FLAG_TC));
        USART_SendData(USARTx, *s);
        s++;
    }
}


void triggertask(void *pvParameters)
{
	
    while(1) {
			if(LEDState==2){
      xSemaphoreGive(trg);
			//vTaskDelay(1000); 
				LEDState=0;
			}
}

}

void sendertask(void *pvParameters)
{
    while(1) {
			
				if(xSemaphoreTake(trg,portMAX_DELAY)){	
					
					USART_SendString(USART1,"deneme \r\n");
			  GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
        vTaskDelay(1000/portTICK_RATE_MS);
  
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
        vTaskDelay(1000/portTICK_RATE_MS);
				}
				
        
 }
}


//-----------------------------------------
int main()
{ 
 SystemInit();
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
USART_ClockInitTypeDef USART_ClockInitStructure; 
NVIC_InitTypeDef NVIC_InitStructure;

//----------------------------------------------------------------	
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
 
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
   GPIO_Init(GPIOA, &GPIO_InitStructure);
   
 
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_Parity                = USART_Parity_No;
    USART_InitStructure.USART_StopBits              = USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  = USART_Mode_Tx | USART_Mode_Rx;
 
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
 
    NVIC_InitStructure.NVIC_IRQChannel          = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd       = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
    NVIC_Init(&NVIC_InitStructure);
	
	
	
	
	
 //GPIO
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //GPIOA pin 0 output push pull
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_Init(GPIOA, &GPIO_InitStructure);
 
 
 
 
 
 

	vSemaphoreCreateBinary(trg);
	
 //Create task to blink gpio
 xTaskCreate(triggertask, (const char *)"triggertask", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, NULL);
 xTaskCreate(sendertask, (const char *)"sendertask", configMINIMAL_STACK_SIZE, (void *)NULL, tskIDLE_PRIORITY, NULL);
 //Start the scheduler
 vTaskStartScheduler();
 
 //Should never reach here
 while(10);
}




