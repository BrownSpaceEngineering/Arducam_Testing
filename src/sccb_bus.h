#include <atmel_start.h>
#ifndef BSE_FSW_TEMPLATE_SCCB_BUS_H
#define BSE_FSW_TEMPLATE_SCCB_BUS_H

#ifndef _SCCB_BUS_H_
#define _SCCB_BUS_H_

#ifdef __cplusplus
extern "C"{
#endif

#define sda_port        GPIO(GPIO_PORTA, 8)
#define scl_port        GPIO(GPIO_PORTA, 9)


#define SCCB_SIC_H()      digitalWrite(scl_port,HIGH)	 	//SCL H
#define SCCB_SIC_L()      digitalWrite(scl_port,LOW)		 	//SCL H

#define SCCB_SID_H()      digitalWrite(sda_port,HIGH)   //SDA	H
#define SCCB_SID_L()      digitalWrite(sda_port,LOW)    //SDA	H
#define SCCB_DATA_IN      pinMode(sda_port, INPUT);
#define SCCB_DATA_OUT     pinMode(sda_port, OUTPUT);

#define SCCB_SID_STATE	  digitalRead(sda_port)

void sccb_bus_init(void);
void sccb_bus_start(void);
void sccb_bus_stop(void);
void sccb_bus_send_noack(void);
void sccb_bus_send_ack(void);
unsigned char sccb_bus_write_byte(unsigned char data);
unsigned char sccb_bus_read_byte(void);

extern unsigned char I2C_TIM;

#ifdef __cplusplus
}
#endif
#endif /* _SCCB_BUS_H_ */

/******************* (C) COPYRIGHT 2015 WangBao *****END OF FILE****/

#endif //BSE_FSW_TEMPLATE_SCCB_BUS_H