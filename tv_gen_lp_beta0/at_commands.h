/*
 * at_commands.h
 *
 *  Created on: Jan 12, 2022
 *      Author: AS
 */

#ifndef F2837X_COMMON_INCLUDE_AT_COMMANDS_H_
#define F2837X_COMMON_INCLUDE_AT_COMMANDS_H_

#include "stdint.h"
#include "stdio.h"
#include <string.h>

#define STR 1
#define NUM 0
#define LEN 2
#define STA 1
#define AP 2

#define AT(x)                                                       (x == 0 ? 0 : ((x) == 1 ? "AT\r\n" : 4))
#define AT_UART_Q(x)                                                (x == 0 ? 1 : ((x) == 1 ? "AT+UART=?\r\n" : 11))
#define AT_CWMODE_STA(x)                                            (x == 0 ? 2 : ((x) == 1 ? "AT+CWMODE_CUR=1\r\n" : 17))
#define AT_CIPSTART_UDP(x)                                          (x == 0 ? 3 : ((x) == 1 ? "AT+CIPSTART=\"UDP\",\"192.168.4.1\",7,7200\r\n" : 40))
#define AT_CIPSTART_TCP(x)                                          (x == 0 ? 4 : ((x) == 1 ? "AT+CIPSTART=\"TCP\",\"192.168.4.2\",7,7200\r\n" : 40))
#define AT_CWLAP_SET(x)                                             (x == 0 ? 5 : ((x) == 1 ? "AT+CWLAP=\"Wi-Fi\",\"06:ea:56:a0:47:81\",5,0,10,200\r\n" : 49))
#define AT_CWLAP_EXE(x)                                             (x == 0 ? 6 : ((x) == 1 ? "AT+CWLAP\r\n" : 10))
#define AT_CWLAPOPT(x)                                              (x == 0 ? 7 : ((x) == 1 ? "AT+CWLAPOPT=1,7\r\n" : 17))
#define AT_CWQAP(x)                                                 (x == 0 ? 8 : ((x) == 1 ? "AT+CWQAP\r\n" : 10))
#define AT_CIPMODE_TR(x)                                            (x == 0 ? 9 :  ((x) == 1 ? "AT+CIPMODE=1\r\n" : 14))
#define AT_CIPSEND_TR(x)                                            (x == 0 ? 10 : ((x) == 1 ? "AT+CIPSEND\r\n" : 12))
#define AT_CWMODE_AP(x)                                             (x == 0 ? 11 : ((x) == 1 ? "AT+CWMODE_CUR=2\r\n" : 17))
#define AT_CIPMUX_MULT(x)                                           (x == 0 ? 12 : ((x) == 1 ? "AT+CIPMUX=1\r\n" : 13))
#define AT_CIPSERVER_AP(x)                                          (x == 0 ? 13 : ((x) == 1 ? "AT+CIPSERVER=1,7\r\n" : 18))
//#define AT_CWSAP_CUR(x)                                           (x == 0 ? 14 : ((x) == 1 ? "AT+CWSAP=\"ReasonanceUpdateFirmware_11.1\",\"100@101@\",5,3\r\n" : 57 ))//(x == 0 ? 14 : ((x) == 1 ? "AT+CWSAP=\"REASONANCE_GEN_3000.0_4.0_000TV\",\"3000zvxw\",5,3\r\n" : 59 ))
#define AT_CWSAP_CUR(x)                                             (x == 0 ? 14 : ((x) == 1 ? "AT+CWSAP=\"REASONANCE_GEN_3000.0_4.0_000TV\",\"00000000\",5,3\r\n" : 59 ))////#define AT_CWSAP_CUR(x)                                             (x == 0 ? 14 : ((x) == 1 ? "AT+CWSAP=\"REASONANCE_GEN_3000.0_4.0_000TV\",\"3000zvxw\",5,3\r\n" : 59 ))// (x == 0 ? 14 : ((x) == 1 ? "AT+CWSAP=\"REASONANCE_GEN_3000.0_4.0_000TV\",\"3000zvxw\",5,3\r\n" : 59 ))
//#define AT_CWSAP_CUR(x)                                           (x == 0 ? 14 : ((x) == 1 ? "AT+CWSAP=\"ReasonanceUpdateFirmware_10.0\",\"100@101@\",5,3\r\n" : 57 ))//test
#define AT_CIPSTATUS(x)                                             (x == 0 ? 15 : ((x) == 1 ? "AT+CIPSTATUS\r\n" : 14 ))
//#define AT_CWSTATE(x)                                             (x == 0 ? 16 : ((x) == 1 ? "AT+CWSTATE?\r\n" : 13 ))
#define AT_CIPSEND_STA_ID0(x)                                       (x == 0 ? 17 : ((x) == 1 ? "AT+CIPSEND=0,4\r\n" : 16 ))//(x == 0 ? 17 : ((x) == 1 ? "AT+CIPSEND=0,64\r\n" : 17 ))
#define AT_CWMODE_AP_STA(x)                                         (x == 0 ? 18 : ((x) == 1 ? "AT+CWMODE_CUR=3\r\n" : 17))
#define AT_CWSAP_TERMINAL(x)                                        (x == 0 ? 19 : ((x) == 1 ? "AT+CWSAP=\"ReasonanceKickChargeTerminal\",\"100@101@\",11,3\r\n" : 57))// (x == 0 ? 19 : ((x) == 1 ? "AT+CWSAP=\"ReasonanceKickChargeTerminal\",\"100@101@\",5,3\r\n" : 56))
#define AT_CWJAP_TERMINAL(x)                                        (x == 0 ? 20 : ((x) == 1 ? "AT+CWJAP=\"ReasonanceHub\",\"100@101@\"\r\n" : 37))//(x == 0 ? 20 : ((x) == 1 ? "AT+CWJAP=\"ReasonanceUpdateFirmware_11.1\",\"100@101@\"\r\n" : 53))//(x == 0 ? 20 : ((x) == 1 ? "AT+CWJAP=\"ReasonanceKickChargeTerminal\",\"100@101@\"\r\n" : 52))
#define AT_CIPSTART_TCP_TERMINAL(x)                                 (x == 0 ? 21 : ((x) == 1 ? "AT+CIPSTART=\"TCP\",\"192.168.4.2\",8,7200\r\n" : 40))
#define AT_CIPSEND_STA_ID1(x)                                       (x == 0 ? 22 : ((x) == 1 ? "AT+CIPSEND=1,64\r\n" : 17 ))
#define AT_CIPSTART_MMQT(x)                                         (x == 0 ? 23 : ((x) == 1 ? "AT+CIPSTART=0,\"TCP\",\"driver.cloudmqtt.com\",18793\r\n" : 50))//(x == 0 ? 23 : ((x) == 1 ? "AT+CIPSTART=0,\"TCP\",\"driver.cloudmqtt.com\",18737\r\n" : 50))//(x == 0 ? 23 : ((x) == 1 ? "AT+CIPSTART=\"TCP\",\"driver.cloudmqtt.com\",18737\r\n" : 48))
#define AT_ESP8266_MMQT_GET_T_MMQT(x)                               (x == 0 ? 24 : ((x) == 1 ? "GET /get?cid=2097&key=FKAwfl&p1=2,80\r\n" : 38))
#define AT_CIPSTART_TCP_0(x)                                        (x == 0 ? 25 : ((x) == 1 ? "AT+CIPSTART=0,\"TCP\",\"192.168.4.2\",7,7200\r\n" : 42))
#define AT_MQTT_CONNECT_BROKER(x)                                   (x == 0 ? 26 : ((x) == 1 ? "AT+CIPSTART=0,\"TCP\",\"192.168.4.2\",7,7200\r\n" : 42))
#define AT_CIPSEND_STA_ID0_MQTT_CONNECT(x)                          (x == 0 ? 27 : ((x) == 1 ? "AT+CIPSEND=0,78\r\n" : 18 ))//(x == 0 ? 27 : ((x) == 1 ? "AT+CIPSEND=0,54\r\n" : 17 ))
#define AT_CIPSEND_STA_ID0_MQTT_DATA(x)                             (x == 0 ? 28 : ((x) == 1 ? "AT+CIPSEND=0,46\r\n" : 17 ))
#define AT_UART_CONFIG(x)                                           (x == 0 ? 29 : ((x) == 1 ? "AT+UART_CUR=2500000,8,1,0,0\r\n" : 29 ))//(x == 0 ? 29 : ((x) == 1 ? "AT+UART_CUR=2304000,8,1,0,0\r\n" : 29 ))
#define AT_CIPSEND_STA_ID0_MQTT_SUBSCRIPTION(x)                     (x == 0 ? 30 : ((x) == 1 ? "AT+CIPSEND=0,37\r\n" : 17 ))//(x == 0 ? 30 : ((x) == 1 ? "AT+CIPSEND=0,18\r\n" : 17 ))
#define AT_CIPCLOSE(x)                                              (x == 0 ? 31 : ((x) == 1 ? "AT+CIPCLOSE=1\r\n" : 15 ))
#endif /* F2802X_COMMON_INCLUDE_AT_COMMANDS_H_ */
