#include "ESP8266.h"
#include "main.h"
#include "ringbuf.h"

#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
 
 extern uint8_t BufRx[128];
 
 extern UART_HandleTypeDef huart1;
 extern UART_HandleTypeDef huart2;
 
 extern struct ringbuf Ring;

 extern bool conection_set;

ESP_StatusTypeDef ESP8266_init(UART_HandleTypeDef *uart)
{

	HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_GPIO_Pin, GPIO_PIN_SET);
	
	HAL_Delay(1000);
	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC); // habilita interrupção UART2
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC); // habilita interrupção UART1
	
	

	if(ESP_AT_TEST(uart)!= ESP_OK)
	{
		return ESP_ERROR;
	}
	
	memset(BufRx, 0, sizeof(BufRx)); //reseta bufer para evitar ler lixo
	
	HAL_UART_Transmit_IT(uart, CWMODE, sizeof(CWMODE));
	HAL_Delay(1000);
	
	#ifdef RECEPTEC
	HAL_UART_Transmit_IT(uart, AT_CIPAP, sizeof(AT_CIPAP));
	HAL_Delay(2000);
	#endif
	
	HAL_UART_Transmit_IT(uart, CIPMUX, sizeof(CIPMUX));
	HAL_Delay(5000);
	
	#ifdef COLETEC
	
		//HAL_UART_Transmit_IT(uart, CWJAP, sizeof(CWJAP));
	
		memset(BufRx, 0, sizeof(BufRx)); //reseta bufer para evitar ler lixo
		
		if(ESP_CONEXAO_TCP(uart, "9.0.0.0", "1001")!= ESP_OK)
		{
			return ESP_ERROR;
		}
	
//		while(1){
//	  ESP_ENVIA_DADOS( uart, (char*) teste);
//		HAL_Delay(300);
//		}
		
	#endif
		
		
	#ifdef RECEPTEC
		
		#ifdef PRIMEIRA_GRAVACAO
		memset(BufRx, 0, sizeof(BufRx)); //reseta bufer para evitar ler lixo
		HAL_UART_Transmit(uart, CWSAP, sizeof(CWSAP),1000);
		#endif
		
		memset(BufRx, 0, sizeof(BufRx));
		HAL_UART_Transmit_IT(uart, CIPSERVER, sizeof(CIPSERVER));
		HAL_Delay(200);
		
		ESP_AGUARDA_CONEXAO_TCP();
			
		//HAL_UART_Transmit_IT(uart, CIFSR, sizeof(CIFSR));

		//HAL_UART_Transmit_IT(uart, CWLIF , sizeof(CWLIF));
		
		
		
	#endif

	return ESP_OK;
}


ESP_StatusTypeDef ESP_AT_TEST(UART_HandleTypeDef *uart)
{


	ringbuf_read_buffer(&Ring, ringbuf_read_available(&Ring), BufRx);
	memset(BufRx, 0, sizeof(BufRx));
	
	HAL_UART_Transmit_IT(uart, AT, sizeof(AT));
	HAL_Delay(5);                         

	ringbuf_read_buffer(&Ring, ringbuf_read_available(&Ring), BufRx);

		if(strcmp((const char*)BufRx, (const char*) AT_RESPONSE_OK) == 0)
	{
		return ESP_OK;
	}
	
	return ESP_ERROR;
	
}

ESP_StatusTypeDef ESP_CONEXAO_TCP(UART_HandleTypeDef *uart, char *IP, char *Port)
{
	
	char cipstart[38];

	strcpy (cipstart,"AT+CIPSTART="); strcat(cipstart,"0,\"TCP\",\""); strcat(cipstart, IP); //cria comando AT para conexao tcp
	strcat (cipstart,"\","); strcat(cipstart, (char*) Port); strcat (cipstart,"\r\n");


	ringbuf_read_buffer(&Ring, ringbuf_read_available(&Ring), BufRx); //lê ring para preparação de recepção da nova mensagem
	memset(BufRx, 0, sizeof(BufRx)); //reseta buf

	HAL_UART_Transmit_IT(uart, (uint8_t*) cipstart, sizeof(cipstart));
	HAL_Delay(300);
	
	ringbuf_read_buffer(&Ring, ringbuf_read_available(&Ring), BufRx); //lê ring 

	if(strstr((char*)BufRx, (const char*) TCP_RESPONSE_OK) == 0)
	{
		return ESP_ERROR;
	}
	
	return ESP_OK;
}

#ifdef RECEPTEC
void ESP_AGUARDA_CONEXAO_TCP(void)
{
		
	ringbuf_read_buffer(&Ring, ringbuf_read_available(&Ring), BufRx); //lê ring para preparação de recepção da nova mensagem
	memset(BufRx, 0, sizeof(BufRx)); //reseta buf
		
	while(ringbuf_read_available(&Ring)<10); // Aguarda mensagem confirmando conexão TCP
	
	HAL_Delay(10); 
	
	ringbuf_read_buffer(&Ring, ringbuf_read_available(&Ring), BufRx); //lê ring para preparação de recepção da nova mensagem
		
	if(strstr((char*)BufRx, (const char*) RECEPTEC_TCP_RESPONSE_OK) == 0) // valor zero representa nao encontrar mensagem de ok
	{
		while(1);
	}
	
	conection_set = true; //libera envio para receptec;
	
	HAL_UART_Transmit_IT(&huart1, (uint8_t*) TCP_CONECTADO_SOFTEC , sizeof(TCP_CONECTADO_SOFTEC)-1); //envia para softec mensagem de conectado
	
}

	#endif

ESP_StatusTypeDef ESP_ENVIA_DADOS(UART_HandleTypeDef *uart, char *data, int size_msg)
{
	char aux[4];
	char cipsend[180];
	
	sprintf(aux, "%i", size_msg);
	
	strcpy (cipsend,"AT+CIPSEND=0,");  strcat(cipsend, aux); strcat (cipsend,"\r\n"); //cria comando AT para envio da mensagem
	
	ringbuf_read_buffer(&Ring, ringbuf_read_available(&Ring), BufRx); //lê ring para preparação de recepção da nova mensagem
	memset(BufRx, 0, sizeof(BufRx)); //reseta buf
		


	HAL_UART_Transmit_IT(uart, (uint8_t*) cipsend, strlen(cipsend));
	
	while(ringbuf_read_available(&Ring)<24){} // Aguarda mensagem pedindo envio
		
		HAL_Delay(2);
		
	ringbuf_read_buffer(&Ring, ringbuf_read_available(&Ring), BufRx); //lê ring 

	
		if(strstr((char*)BufRx, (const char*) CIPSEND_AGUARDA) != 0) //caso encontre mensagem pedido de envio da mensagem
		{
		if(HAL_UART_Transmit_IT(uart, (uint8_t*) data, size_msg) == HAL_OK) // envia mensagem
		{
		  return ESP_OK;
		}
		
	}
	
	return ESP_ERROR;
	
}
