// Trabalho Prático 3 de Programação de Periféricos
// Guilherme Martins Specht
// Última atualização: 27/11/2024

/*
 * MQTT demo application for ustack-stm32
 * 
 * start the MQTT/UDP bridge (mqtt_udp) before running this application!
 */

// PINOS
// PBO = Temperatura e umidade (DHT11)
// PB1 = Luminosidade
// PB13 = Relay 1
// PB12 = Relay 2 
// PB8 = PWM

#include <stm32f4xx_conf.h>
#include <hal.h>
#include <usart.h>
#include <ustack.h>
#include "hw_res.h"
#include "dht.h"

uint8_t eth_frame[FRAME_SIZE];
uint8_t mymac[6] = {0x0e, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t myip[4];
uint8_t mynm[4];
uint8_t mygw[4];

/* Tópicos do MQTT */
#define SENSOR_TEMPERATURA		"testtopic/gms/sensor_temperatura"
#define SENSOR_LUMINOSIDADE		"testtopic/gms/sensor_luminosidade"
#define SENSOR_UMIDADE	        "testtopic/gms/sensor_umidade"
#define MANUAL_DIMERIZACAO		"testtopic/gms/manual_dimerizacao"
#define LIMITES_DIMERIZACAO	    "testtopic/gms/limites_dimerizacao"
#define MANUAL_RELAY_1			"testtopic/gms/manual_relay_1"
#define MANUAL_RELAY_2			"testtopic/gms/manual_relay_2"
#define LIMITES_RELAY_1	        "testtopic/gms/limites_relay_1"
#define LIMITES_RELAY_2	        "testtopic/gms/limites_relay_2"

/* ADC library */
void analog_config();
void adc_config(void);
void adc_channel(uint8_t channel);
uint16_t adc_read();

/* PWM library */
void pwm_config();

/* sensors parameters */
const float F_VOLTAGE = 635.0;		// 590 ~ 700mV typical diode forward voltage
const float T_COEFF = -2.0;			// 1.8 ~ 2.2mV change per degree Celsius
const float V_RAIL = 3300.0;		// 3300mV rail voltage
const float ADC_MAX = 4095.0;		// max ADC value
const int ADC_SAMPLES = 1024;		// ADC read samples
const int REF_RESISTANCE = 3500;

/* Variáveis de sensores e controle */
float temperatura, luminosidade = 0;
volatile int sensor_poll = 0;

/* Variáveis de limites e controle manual */
int limite_max_lux = -1;
int limite_min_lux = -1;
int lux_manual = 0;

int limite_max_relay1 = -1;
int limite_min_relay1 = -1;
int relay1_manual = 0;

int limite_max_relay2 = -1;
int limite_min_relay2 = -1;
int relay2_manual = 0;

/* Variáveis de Cálculo para Controle de Dimmer */
float dif_max_min_lim,  dif_atual_inf, lux_int;
float div, multi;
int mult_int;
int cont;
char converter[100], converter2[100];

/* sensor data is polled every 10 seconds */
void TIM2_IRQHandler()
{
	static uint16_t secs = 0;
	
	/* Checks whether the TIM2 interrupt has occurred or not */
	if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
		if (++secs > 10) {
			secs = 0;
			sensor_poll = 1;
		}
		/* Clears the TIM2 interrupt pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void tim2_init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	/* Enable clock for TIM2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	/* TIM2 initialization overflow every 1000ms
	 * TIM2 by default has clock of 84MHz
	 * Here, we must set value of prescaler and period,
	 * so update event is 1Hz or 1000ms
	 * Update Event (Hz) = timer_clock / ((TIM_Prescaler + 1) * (TIM_Period + 1))
	 * Update Event (Hz) = 84MHz / ((8399 + 1) * (9999 + 1)) = 1 Hz
	 */
	TIM_TimeBaseInitStruct.TIM_Prescaler = 8399;
	TIM_TimeBaseInitStruct.TIM_Period = 9999;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	
	/* TIM2 initialize */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	/* Enable TIM2 interrupt */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* Start TIM2 */
	TIM_Cmd(TIM2, ENABLE);
	
	/* Nested vectored interrupt settings */
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOC Peripheral clock enable. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* configure board LED as output */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	//GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOC Peripheral clock enable. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* configure board LED as output */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

/* this function is called asynchronously (on MQTT messages) */
int32_t app_udp_handler(uint8_t *packet)
{
	uint8_t dst_addr[4];
	uint16_t src_port, dst_port;
	struct ip_udp_s *udp = (struct ip_udp_s *)packet;
	char *datain, *dataval, *dataval2;
	char data[256];
	char primeiro_param[5], segundo_param[5];
	
	src_port = ntohs(udp->udp.dst_port);
	dst_port = ntohs(udp->udp.src_port);

	if (ntohs(udp->udp.dst_port) == UDP_DEFAULT_PORT) {
		memcpy(dst_addr, udp->ip.src_addr, 4);
		
		datain = (char *)packet + sizeof(struct ip_udp_s);
		datain[ntohs(udp->udp.len) - sizeof(struct udp_s)] = '\0';
		
		if (strstr(datain, LIMITES_DIMERIZACAO)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			dataval2 = strstr(datain, " ") + 1;

			char *token = strtok(dataval2, ",");

			if (token != NULL) {
				strcpy(primeiro_param, token);

				token = strtok(NULL, ",");

				if (token != NULL) {
					strcpy(segundo_param, token);
				} 
			} 

			limite_max_lux = atoi(segundo_param);
			limite_min_lux = atoi(primeiro_param);
			
			/* print received data and its origin */
			sprintf(data, "LIMITES_DIMERIZACAO: [%s] hello %d.%d.%d.%d ", dataval,
			udp->ip.src_addr[0], udp->ip.src_addr[1], udp->ip.src_addr[2], udp->ip.src_addr[3]);
			
			/* send data back, just bacause we can (this is not needed) */
			memcpy(packet + sizeof(struct ip_udp_s), data, strlen(data));
			udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
		}
		if (strstr(datain, MANUAL_DIMERIZACAO)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			
			/* print received data and its origin */
			sprintf(data, "MANUAL_DIMERIZACAO: [%s] hello %d.%d.%d.%d", dataval, 
			udp->ip.src_addr[0], udp->ip.src_addr[1], udp->ip.src_addr[2], udp->ip.src_addr[3]);

			lux_manual = atoi(dataval);

			if (lux_manual > 999) lux_manual = 999;
			else if (lux_manual < 0) lux_manual = 0;
			
			/* send data back, just bacause we can (this is not needed) */
			memcpy(packet + sizeof(struct ip_udp_s), data, strlen(data));
			udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
		}
		
		if (strstr(datain, LIMITES_RELAY_1)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			
			/* print received data and its origin */
			sprintf(data, "LIMITES_RELAY_1: [%s] hello %d.%d.%d.%d", dataval, 
			udp->ip.src_addr[0], udp->ip.src_addr[1], udp->ip.src_addr[2], udp->ip.src_addr[3]);

			char *token = strtok(dataval, ",");

			if (token != NULL) {
				strcpy(primeiro_param, token);
				
				token = strtok(NULL, ",");

				if (token != NULL) {
					strcpy(segundo_param, token);
				} 
			} 
			
			limite_min_relay1 = atoi(primeiro_param);
			limite_max_relay1 = atoi(segundo_param);
			
			/* send data back, just bacause we can (this is not needed) */
			memcpy(packet + sizeof(struct ip_udp_s), data, strlen(data));
			udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
		}
		
		if (strstr(datain, MANUAL_RELAY_1)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			
			/* print received data and its origin */
			sprintf(data, "MANUAL_RELAY_1: [%s] hello %d.%d.%d.%d", dataval, 
			udp->ip.src_addr[0], udp->ip.src_addr[1], udp->ip.src_addr[2], udp->ip.src_addr[3]);

			relay1_manual = atoi(dataval);

			if (relay1_manual > 1) relay1_manual = 1;
			else if (relay1_manual < 0) relay1_manual = 0;
			
			/* send data back, just bacause we can (this is not needed) */
			memcpy(packet + sizeof(struct ip_udp_s), data, strlen(data));
			udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
		}
		
		if (strstr(datain, LIMITES_RELAY_2)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			
			/* print received data and its origin */
			sprintf(data, "LIMITES_RELAY_2: [%s] hello %d.%d.%d.%d", dataval, 
			udp->ip.src_addr[0], udp->ip.src_addr[1], udp->ip.src_addr[2], udp->ip.src_addr[3]);
			
			char *token = strtok(dataval, ",");

			if (token != NULL) {
				strcpy(primeiro_param, token);
				token = strtok(NULL, ",");
				if (token != NULL) {
					strcpy(segundo_param, token);
				}
			}

			limite_min_relay2 = atoi(primeiro_param);
			limite_max_relay2 = atoi(segundo_param);
			
			/* send data back, just bacause we can (this is not needed) */
			memcpy(packet + sizeof(struct ip_udp_s), data, strlen(data));
			udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
		}
		
		if (strstr(datain, MANUAL_RELAY_2)){
			/* skip topic name */
			dataval = strstr(datain, " ") + 1;
			
			/* print received data and its origin */
			sprintf(data, "MANUAL_RELAY_2: [%s] hello %d.%d.%d.%d", dataval, 
			udp->ip.src_addr[0], udp->ip.src_addr[1], udp->ip.src_addr[2], udp->ip.src_addr[3]);

			relay2_manual = atoi(dataval);

			if (relay2_manual > 1) relay2_manual = 1;
			else if (relay2_manual < 0) relay2_manual = 0;
			
			/* send data back, just bacause we can (this is not needed) */
			memcpy(packet + sizeof(struct ip_udp_s), data, strlen(data));
			udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
		}
	}
	
	return 0;
}


/* these functions are called when timer 2 generates an interrupt */
void sensor1_data(uint8_t *packet, char *topic, float val)
{
	uint8_t dst_addr[4] = {172, 31, 69, 1};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	char buf[30];
	
	ftoa(val, buf, 6);
	sprintf(data, "PUBLISH %s %s", topic, buf);
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}

void sensor2_data(uint8_t *packet, char *topic, float val)
{
	uint8_t dst_addr[4] = {172, 31, 69, 1};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	char buf[30];
	
	ftoa(val, buf, 6);
	sprintf(data, "PUBLISH %s %s", topic, buf);
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}


/* this function is used to register a topic */
void setup_topic(uint8_t *packet, char *topic)
{
	uint8_t dst_addr[4] = {172, 31, 69, 1};
	uint16_t src_port = UDP_DEFAULT_PORT, dst_port = 8888;
	char data[256];
	
	strcpy(data, "TOPIC ");
	strcat(data, topic);
	
	strcpy((char *)packet + sizeof(struct ip_udp_s), data);
	udp_out(dst_addr, src_port, dst_port, packet, sizeof(struct udp_s) + strlen(data));
}


/* resolve ARP stuff */
void net_setup(uint8_t *packet)
{
	uint16_t i;
	
	for (i = 0; i < 100; i++) {
		netif_recv(packet);
		delay_ms(100);
	}

	setup_topic(packet, "");
	delay_ms(250);
	netif_recv(packet);
	setup_topic(packet, "");
	delay_ms(250);
	netif_recv(packet);
}


float temperature()
{
	float temp = 0.0;
	float voltage;
	
	for (int i = 0; i < ADC_SAMPLES; i++) {
		voltage = adc_read() * (V_RAIL / ADC_MAX);
		temp += ((voltage - F_VOLTAGE) / T_COEFF);
	}
	
	return (temp / ADC_SAMPLES);
}

float luminosity()
{
	float voltage, lux = 0.0, rldr;
	
	for (int i = 0; i < ADC_SAMPLES; i++) {
		voltage = adc_read() * (V_RAIL / ADC_MAX);
		rldr = (REF_RESISTANCE * (V_RAIL - voltage)) / voltage;
		lux += 500 / (rldr / 650);
	}
	
	return (lux / ADC_SAMPLES);
}

float task_1(void)
{	
		adc_channel(ADC_Channel_8);
		temperatura = temperature();
	
	return temperatura;
}

float task_2(void)
{
		
		adc_channel(ADC_Channel_9);
		luminosidade = luminosity();
	
	return luminosidade;
}

int main(void)
{
	uint8_t *packet = eth_frame + sizeof(struct eth_s);
	uint16_t len;

	analog_config();
	adc_config();
	pwm_config();
	
	/* setup LED */
	led_init();
	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	/* setup timer 2 */
	tim2_init();
	
	/* setup network stack */
	if_setup();
	config(mymac + 2, USTACK_IP_ADDR);
	config(myip, USTACK_IP_ADDR);
	config(mynm, USTACK_NETMASK);
	config(mygw, USTACK_GW_ADDR);
	
	net_setup(packet);
	udp_set_callback(app_udp_handler);
	
	/* create MQTT topics */
	setup_topic(packet, SENSOR_TEMPERATURA);
	setup_topic(packet, SENSOR_LUMINOSIDADE);
	setup_topic(packet, SENSOR_UMIDADE);
	setup_topic(packet, MANUAL_DIMERIZACAO);
	setup_topic(packet, LIMITES_DIMERIZACAO);
	setup_topic(packet, MANUAL_RELAY_1);
	setup_topic(packet, MANUAL_RELAY_2);
	setup_topic(packet, LIMITES_RELAY_1);
	setup_topic(packet, LIMITES_RELAY_2);
	
	
	/* application loop */
	while (1) { 
		len = netif_recv(packet); // Recebe pacotes da rede.

		if (len > 0) {
			ip_in(myip, packet, len); // Processa pacotes recebidos e encaminha para a pilha de rede.
		}

		if (sensor_poll) { // Verifica se é hora de realizar uma nova leitura dos sensores.
			sensor_poll = 0; // Reseta a flag para evitar múltiplas leituras seguidas.
			sensor1_data(packet, "testtopic/gms/sensor_temperatura", task_1()); 
			sensor2_data(packet, "testtopic/gms/sensor_luminosidade", task_2()); 
		}

		/* Controle manual da luminosidade (dimensão do dimmer) */
		if (limite_min_lux != -1 && limite_max_lux != -1) { 
			ftoa(luminosidade, converter, 5); 
			lux_int = atoi(converter);
			
			dif_max_min_lim = (limite_max_lux - limite_min_lux); // Calcula a diferença entre o limite máximo e mínimo.
			dif_atual_inf = (luminosidade - limite_min_lux); // Calcula a diferença entre o valor atual e o limite inferior.
			div = (dif_atual_inf / dif_max_min_lim); // Determina a proporção da luminosidade dentro do intervalo configurado.
			multi = (div * 999); // Ajusta a proporção para o valor do PWM (de 0 a 999).

			ftoa(multi, converter2, 5);
			mult_int = atoi(converter2);
			cont = mult_int; // Atualiza o valor do PWM.

			if (luminosidade < limite_min_lux) {
				cont = -6; // Define um valor indicativo.
				TIM4->CCR3 = 0; 
			} 
			else if (luminosidade > limite_max_lux) { 
				cont = -7; // Define outro valor indicativo.
				TIM4->CCR3 = 999; 
			}
			else {
				TIM4->CCR3 = cont;
			}

			delay_ms(1);
		} 
		else { 
			cont = 0; // Define o valor padrão.
			TIM4->CCR3 = lux_manual;
			delay_ms(1);
		}

		/* Controle manual do relay 1 */
		if (limite_min_relay1 != -1 && limite_max_relay1 != -1) {
			if (temperatura > limite_min_relay1 && temperatura < limite_max_relay1) {
				GPIO_SetBits(GPIOB, GPIO_Pin_13);
			} else {
				GPIO_ResetBits(GPIOB, GPIO_Pin_13);
			}
		} else {
			if (relay1_manual == 1) {
				GPIO_SetBits(GPIOB, GPIO_Pin_13);
			} else {
				GPIO_ResetBits(GPIOB, GPIO_Pin_13);
			}
		}

		/* Controle manual do relay 2 */
		if (limite_min_relay2 != -1 && limite_max_relay2 != -1) { /
			if (temperatura > limite_min_relay2 && temperatura < limite_max_relay2) {
				GPIO_SetBits(GPIOB, GPIO_Pin_12); 
			} else {
				GPIO_ResetBits(GPIOB, GPIO_Pin_12);
			}
		} else { 
			if (relay2_manual == 1) {
				GPIO_SetBits(GPIOB, GPIO_Pin_12); 
			} else {
				GPIO_ResetBits(GPIOB, GPIO_Pin_12);
			}
		}
	}
	
	return 0;
}
