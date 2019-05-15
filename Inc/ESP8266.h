
#include "stm32l4xx_hal.h"
#include "main.h"
#include "string.h"

#define          ESP_EN_GPIO_Pin   GPIO_PIN_6
#define         ESP_EN_GPIO_Port   GPIOA
#define                 ESP_UART          

#define                       AT   "AT\r\n"
#define           AT_RESPONSE_OK   "AT\r\r\n\r\nOK\r\n"
#define          TCP_RESPONSE_OK   "0,CONNECT\r\n\r\nOK\r\n"
#define RECEPTEC_TCP_RESPONSE_OK   "0,CONNECT\r\n"
#define          CIPSEND_AGUARDA   "OK\r\n>"
#define                    CIFSR   "AT+CIFSR\r\n"
#define                    CWLIF   "AT+CWLIF\r\n"
#define                   CIPMUX   "AT+CIPMUX=1\r\n"
#define                     UART   "AT+UART=115200,8,1,0,0\r\n"
#define                  RFPOWER   "AT+RFPOWER=82\r\n"

#ifdef COLETEC

#define                   CWMODE   "AT+CWMODE=1\r\n"
#define                    CWJAP   "AT+CWJAP=\"Atletec\",\"IEFESUFC\"\r\n"
#define                 CIPSTART   "AT+CIPSTART=0,\"TCP\",\"9.0.0.0\",1001\r\n"
#define                   CIPSTA   "AT+CIPSTA?\r\n"
#define                  CIPSEND   "AT+CIPSEND=0,4\r\n"
#define 	             WIFI_RESPONSE   "WIFI CONNECTED"

#endif

#ifdef RECEPTEC

#define						   CWDHCPS_DEF   "AT+CWDHCPS_DEF=1,2000,\"9.0.0.0\",\"9.0.0.100\"\r\n"
#define						      AT_CIPAP   "AT+CIPAP=\"9.0.0.0\",\"9.0.0.0\",\"255.255.255.0\"\r\n"
#define 							    CWMODE   "AT+CWMODE=3\r\n"
#define                    CWSAP   "AT+CWSAP=\"Atletec\",\"IEFESUFC\",5,3\r\n"
#define                CIPSERVER   "AT+CIPSERVER=1,1001\r\n"
#define     TCP_CONECTADO_SOFTEC   "~CONNECT~"

#endif
/**
  * @brief  ESP Status structures definition
  */
	
typedef enum
{
	
  ESP_OK       = 0x00,
  ESP_ERROR    = 0x01,
  ESP_BUSY     = 0x02,
  ESP_TIMEOUT  = 0x03
	
} ESP_StatusTypeDef;

ESP_StatusTypeDef ESP8266_init(UART_HandleTypeDef *uart);
ESP_StatusTypeDef ESP_AT_TEST(UART_HandleTypeDef *uart);
ESP_StatusTypeDef ESP_CONEXAO_TCP(UART_HandleTypeDef *uart, char *IP, char *Port);

#ifdef RECEPTEC

	void ESP_AGUARDA_CONEXAO_TCP(void);

#endif

ESP_StatusTypeDef ESP_ENVIA_DADOS(UART_HandleTypeDef *uart, char *data, int size_msg);
