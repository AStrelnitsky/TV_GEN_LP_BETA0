/*
 * gen.h
 *
 *  Created on: 25 џэт. 2023 у.
 *      Author: AS
 */

#ifndef GEN_H_
#define GEN_H_

#include "DSP28x_Project.h"
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include "main.h"
#include "IQmathLib.h"
//#include <xdc/std.h>

#define SCI_RX_SIZE 128
#define SCI_TX_SIZE 128
#define SCI_AT_STRING_SIZE 140

#define RECT_KICKSCOOTER_BASE_NAME "REASONANCE_RECT_0250.0"
#define TERMINAL_BASE_NAME "REASONANCE_RECT_0250.0"
#define LOST_CONNECTION_TIMEOUT 50//10
#define DEVICE_LOST_TIMEOUT 2000

typedef enum
{
    MQTT_CONNECT_TO_SERVER = 0,
    MQTT_CONNECT_TO_SERVER_TX,
    MQTT_CONNECT_TO_SERVER_OK,
    MMQT_SEND_TX,
    MMQT_SEND_OK,
    MMQT_CONNECT_TX,
    MMQT_CONNECT_OK,
    MMQT_PREPARE_DATA_SEND_TX,
    MMQT_PREPARE_DATA_SEND_OK,
    MMQT_DATA_SEND_TX,
    MMQT_DATA_SEND_OK,
    MMQT_SUBSCRIBE_TX,
    MMQT_SUBSCRIBE_OK,
    MMQT_READY_TO_STATUS,
    MMQT_READY_TO_CLOSE_DEVICE,
    MMQT_STATUS_OK,
    MMQT_IDLE,
    MMQT_SUBSCRIBE_SEND_OK
}MQTT_STATES;
typedef struct
{
    MQTT_STATES state_machine;
    size_t  p_len, connect_p_len, subscribe_p_len;
    char send_str[20];
    uint16_t send_len;
    uint16_t swicth;
    uint16_t reset;
    //uint16_t freq, freq_h;
    float freq, freq_h;
    uint16_t duty;
    uint16_t hand_control;
    uint16_t eeprom;
    uint16_t f1, f2;
    float ul, uh;
    float k1,k2,k3;
    uint32_t  lost_connection_timer;
    float kp_u, ki_u, af, pi_limit_hi, pi_limit_low;
    uint16_t dD;
}MQTT_CLIENT;
typedef struct
{
    uint16_t len;
    char data[4];
    char *str;
}MQTT_VAR;
typedef struct
{
    _iq limit_hi, limit_low, in_old, in_new, accum, out, T;
}INTEGRATOR_Q;
typedef struct
{
    _iq ki,kp,kd;
    _iq T;
    _iq accum;
    _iq in, out;
    INTEGRATOR_Q integrator;

}PI_REG;
typedef enum
{
    LOOP_DISABLE = 0,
    LOOP_ENABLE
}LOOPS_ENABLE;
typedef struct
{
    _iq in, out, err, fbk, k_err;
    //STRUCT_SOFT_START ss;
}LOOP;
typedef enum
{
    CHARGE_IDLE = 0,
    CHARGE_CC,
    CHARGE_CV
}CHARGE_MODE;
typedef enum
{
    CHARGER_DISABLE = 0,
    CHARGER_ENABLE
}CHARGE_STATUS;
typedef struct
{
    CHARGE_MODE mode;
    CHARGE_STATUS status;
    float battery_voltge;
    float battery_max_voltage;
    float charge_current;
    float stop_charge_current;
    float battery_max_current;
}CHARGER_STRUCT;
typedef enum
{
    GSTATE_IDLE = 0,
    GSTATE_INTIT,
    GSTATE_OFFLINE,
    GSTATE_ONLINE,
    GSTATE_ERROR

}GEN_STATE;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct
{
  uint16_t GPIO20_THD_EXT_T2_High_R;//        0
  uint16_t GPIO21_THD_Vpwr_Low_R;//           1
  uint16_t GPIO22_THD_Vpwr_High_R;//          2
  uint16_t GPIO23_THD_Ipwr_High_R;//          3
  uint16_t GPIO28_THD_HB1_V_High_R;//         4
  uint16_t GPIO29_THD_HB1_I_High_R;//         5
  uint16_t GPIO30_THD_HB1_Iiso_High_R;//      6
  uint16_t GPIO31_THD_HB1_T_High_R;//         7
  uint16_t GPIO34_THD_HB2_V_High_R;//         8
  uint16_t GPIO39_THD_HB2_I_High_R;//         9
  uint16_t GPIO44_THD_HB2_Iiso_High_R;//      10
  uint16_t GPIO45_THD_HB2_T_High_R;//         11
  uint16_t GPIO54_THD_Ext_T_High_R;//         12

  uint16_t THD_EXT_T2_High_R_set_flag;
  uint16_t THD_EXT_T2_High_R_reset_flag;
  uint16_t THD_Vpwr_Low_R_set_flag;
  uint16_t THD_Vpwr_Low_R_reset_flag;
  uint16_t THD_Vpwr_High_R_set_flag;
  uint16_t THD_Vpwr_High_R_reset_flag;
  uint16_t THD_Ipwr_High_R_set_flag;
  uint16_t THD_Ipwr_High_R_reset_flag;
  uint16_t THD_HB1_V_High_R_set_flag;
  uint16_t THD_HB1_V_High_R_reset_flag;
  uint16_t THD_HB1_I_High_R_set_flag;
  uint16_t THD_HB1_I_High_R_reset_flag;
  uint16_t THD_HB1_Iiso_High_R_set_flag;
  uint16_t THD_HB1_Iiso_High_R_reset_flag;
  uint16_t THD_HB1_T_High_R_set_flag;
  uint16_t THD_HB1_T_High_R_reset_flag;
  uint16_t THD_HB2_V_High_R_set_flag;
  uint16_t THD_HB2_V_High_R_reset_flag;
  uint16_t THD_HB2_I_High_R_set_flag;
  uint16_t THD_HB2_I_High_R_reset_flag;
  uint16_t THD_HB2_Iiso_High_R_set_flag;
  uint16_t THD_HB2_Iiso_High_R_reset_flag;
  uint16_t THD_HB2_T_High_R_set_flag;
  uint16_t THD_HB2_T_High_R_reset_flag;
  uint16_t THD_Ext_T_High_R_set_flag;
  uint16_t THD_Ext_T_High_R_reset_flag;
} GPIO_THD;
typedef struct
{
  uint16_t GPIO_Log_Fan_On;             //53
  uint16_t GPIO_Log_Vpwr_Disable2;      //52
  uint16_t GPIO_Log_SafeTime_R;         //51
  uint16_t GPIO_Log_PWM_Absent_R;       //50
  uint16_t GPIO_Log_Error_RST2;         //24
  uint16_t GPIO_Log_LED_Red_On;         //19
  uint16_t GPIO_Log_LED_Green_On;       //19
  uint16_t GPIO_Log_Bat_Error;          //5

  uint16_t Log_Fan_On_set_flag;
  uint16_t Log_Fan_On_reset_flag;
  uint16_t Log_Vpwr_Disable2_set_flag;
  uint16_t Log_Vpwr_Disable2_reset_flag;
  uint16_t Log_SafeTime_R_set_flag;
  uint16_t Log_SafeTime_R_reset_flag;
  uint16_t Log_PWM_Absent_R_set_flag;
  uint16_t Log_PWM_Absent_R_reset_flag;
  uint16_t Log_Error_RST2_set_flag;
  uint16_t Log_Error_RST2_reset_flag;
  uint16_t Log_LED_Red_On_set_flag;
  uint16_t Log_LED_Red_On_reset_flag;
  uint16_t Log_LED_Green_On_set_flag;
  uint16_t Log_LED_Green_On_reset_flag;
  uint16_t Log_Bat_Error_set_flag;
  uint16_t Log_Bat_Error_reset_flag;

} GPIO_LOGS;
typedef struct
{

    float limit_hi, limit_low, in_old, in_new, accum, out, T;

}INTEGRATOR;

typedef struct
{
    INTEGRATOR integrator, integrator_loop;
    float kp,T;
    float in,out_s,out;
    float err,err_fb;
}APERIODIC;

typedef struct
{
    uint16_t OCC_OUT_PULSE;
    uint16_t OCC_OUT_MEAN;
    uint16_t OCCP_OUT_counter;
    uint16_t OCCM_OUT_counter;
    uint16_t OCCM_OUT_count_limit;
    uint16_t OCCP_OUT_count_limit;
   //uint16_t OVCM_OUT;
    uint16_t OVCP_OUT;
    uint16_t OVCP_OUT_counter;
    uint16_t OVCP_OUT_count_limit;
    uint16_t global_fault;
    uint16_t UVCP_IN;
    uint16_t UVCP_limit;
    uint16_t UVCP_tolerance;
    uint16_t OVCP_IN;
    uint16_t OVCP_limit;
    uint16_t OVCP_tolerance;
    uint16_t OVERTEMP;
}GEN_FAULTS;
typedef struct
{
    uint16_t Log_Error_RST_flag;
    uint16_t Log_PWM_Dis_R;
    uint32_t logs_couner;
}LOGS_R_STRUCT;
typedef struct
{
    uint16_t Log_OT_Tint;
    uint16_t Log_OC_Ihb_ls;
    uint16_t Log_OC_Idc;
    uint16_t Log_OV_Vres;
    uint16_t Log_GPIO28;
    uint16_t Log_GPIO29;
    uint16_t Log_OC_Ihb;
    uint16_t Log_OV_Vhb;
}LOGS_STRUCT;
/* Parameters (default storage) */
typedef struct
{
  int16_t amp_i_out;
  int16_t amp_u_out;
  float hb1_i;
  float hb2_i;
  uint16_t adc_isr_counter;
  uint32_t soft_start_counter;
  float soft_start_step;
  float soft_start_step_f;
  float amp_u;
  uint16_t ramp_counter_limit;
  float mean_current_limit;
  float mean_i_out_A_filt;
  float mean_i_out_A;
  float mean_u_out_V_filt;
  float mean_u_out_V;
  float amp_i_out_A;
  float amp_u_out_V;
  float amp_safe;
  GEN_FAULTS faults;

  float AMP,F,F_old;
  float k1,k2;
  uint16_t sm_status;
  //POWER_ENCODER encoder;
  float u_obs;
  float v_fbk;
  float v_int;
  uint16_t d_window;
  uint16_t d_counter;
  uint16_t d_result;
  uint16_t d_old;
  float leds_duty[3];
  LOGS_R_STRUCT logs_r;
  LOGS_STRUCT logs;
  uint16_t detector;
  PI_REG pi_u;
  LOOPS_ENABLE u_out_loop_en;
  LOOP u_out_loop;
} GEN_STRUCT ;
/* Block parameters (default storage) */
extern GEN_STRUCT gen;
typedef struct
{
    char str[SCI_AT_STRING_SIZE];
    uint16_t len;
    uint16_t tx_flag;
    uint16_t console;
}AT_COMMAND;
typedef enum
{
    ESP8266_IS_NOT_CONFIGURED = 0,
    ESP8266_IS_UDP_CLIENT,
    ESP8266_IS_TCP_CLIENT,
    ESP8266_IS_TCP_SERVER

}ESP8266_CONFIG_STATUS;
typedef enum
{
    ESP8266_IDLE = 0,
    ESP8266_READY,
    ESP8266_MUX_SET_TX,
    ESP8266_MUX_SET_RX,
    ESP8266_MUX_SET_OK,
    ESP8266_MUX_SET_ERROR,
    ESP8266_CIPSTART_UDP_TX,
    ESP8266_CIPSTART_UDP_RX,
    ESP8266_CIPSTART_UDP_OK,
    ESP8266_CIPSTART_UDP_ERROR,
    ESP8266_CIPSTART_UDP_TX_1,
    ESP8266_CIPSTART_UDP_RX_1,
    ESP8266_CIPSTART_UDP_OK_1,
    ESP8266_CIPSTART_UDP_ERROR_1,
    ESP8266_CIPSTART_UDP_TO_STATION_TX,
    ESP8266_CIPSTART_UDP_TO_STATION_RX,
    ESP8266_CIPSTART_UDP_TO_STATION_OK,
    ESP8266_CIPSTART_UDP_TO_STATION_ERROR,
    ESP8266_CIPSTART_UDP_TO_TERMINAL_TX,
    ESP8266_CIPSTART_UDP_TO_TERMINAL_RX,
    ESP8266_CIPSTART_UDP_TO_TERMINAL_OK,
    ESP8266_CIPSTART_UDP_TO_TERMINAL_ERROR,
    ESP8266_CIPSEND_UDP_TO_TERMINAL_TX,
    ESP8266_CIPSEND_UDP_TO_TERMINAL_RX,
    ESP8266_CIPSEND_UDP_TO_TERMINAL_OK,
    ESP8266_CIPSEND_UDP_TO_TERMINAL_ERROR,
    ESP8266_CIPMUX_TX,
    ESP8266_CIPMUX_RX,
    ESP8266_CIPMUX_OK,
    ESP8266_CIPMUX_ERROR,
    ESP8266_AT_TX,
    ESP8266_AT_RX,
    ESP8266_AT_OK,
    ESP8266_AT_ERROR,
    ESP8266_CIPMODE_TX,
    ESP8266_CIPMODE_RX,
    ESP8266_CIPMODE_OK,
    ESP8266_CIPMODE_ERROR,
    ESP8266_CIPSEND_TX,
    ESP8266_CIPSEND_RX,
    ESP8266_CIPSEND_OK,
    ESP8266_CIPSEND_ERROR,
    ESP8266_BUSY,
    ESP8266_SERVER_LISTENING_TERMINAL,
    ESP8266_SERVER_LISTENING_STATION,
    ESP8266_SERVER_SENDING_TERMINAL,
    ESP8266_SERVER_SENDING_STATION,
    ESP8266_CIPSERVER_TX,
    ESP8266_CIPSERVER_RX,
    ESP8266_CIPSERVER_OK,
    ESP8266_CIPSERVER_ERROR,
    ESP8266_CIPSERVER_LISTENING,
    ESP8266_CIPSEND_TERMINAL_TX,
    ESP8266_CIPSEND_TERMINAL_RX,
    ESP8266_CIPSEND_STATION_TX,
    ESP8266_CIPSEND_STATION_RX,
    ESP8266_CWMODE_AP_TX,
    ESP8266_CWMODE_AP_RX,
    ESP8266_CWMODE_AP_OK,
    ESP8266_CWSAP_AP_TX,
    ESP8266_CWSAP_AP_RX,
    ESP8266_CWSAP_AP_OK,
    ESP8266_CIPSTATUS_TX,
    ESP8266_CIPSTATUS_RX,
    ESP8266_CIPSTATUS_OK,
    ESP8266_CIPSTATUS_ERROR,
    SENT_DATA_TX,
    SENT_DATA_OK,
    ESP8266_CIPSTART_TCP_TO_STATION_TX,
    ESP8266_CIPSTART_TCP_TO_STATION_RX,
    ESP8266_CIPSTART_TCP_TO_STATION_OK,
    ESP8266_CWJAP_TERMINAL_TX,
    ESP8266_CWJAP_TERMINAL_OK,
    ESP8266_INIT_FINISH_TX,
    ESP8266_INIT_FINISH_WITH_TERMINAL,
    ESP8266_INIT_FINISH_WITH_NO_TERMINAL,
    ESP8266_CIPSTART_TCP_TO_TERMINAL_TX,
    SENT_DATA_TERMINAL_TX,
    SENT_DATA_TERMINAL_OK,
    ESP8266_MMQT_CONNECT_OK,
    ESP8266_MMQT_CONNECT_TX,
    ESP8266_MMQT_GET_OK,
    ESP8266_MMQT_GET_TX,
    ESP8266_MMQT_SEND_OK,
    ESP8266_MMQT_SEND_TX,
    ESP8266_UART_CONFIG,
    ESP8266_UART_CONFIG_TX

}CONNECT_STATUS;
typedef enum
{
    TCP_CLIENT_IS_NOT_CONNECTED = '0',
    TCP_CLIENT_IS_CONNECTED = '1',
    TCP_CLIENT_IS_DISCONNECTED = '2'
}TCP_CLIENT_STATUS;
typedef struct
{
    char port[5];
    uint16_t port_size;
    char *ip[11];
    char channel;
    TCP_CLIENT_STATUS status;
    uint16_t lost_connection;
}TCP_CLIENT;
typedef enum
{
    SERIAL_MODE_AT = 0,
    SERIAL_MODE_DATA

}SERIAL_MODE;
typedef struct
{
    CONNECT_STATUS c_status;
    SERIAL_MODE s_mode;
    char AT_tx_buff[SCI_TX_SIZE];
    char AT_rx_buff[SCI_RX_SIZE];
    uint16_t sci_write_flag;
    ESP8266_CONFIG_STATUS config_status;
    uint16_t first_reqest;
    uint16_t reset_status;
    uint16_t reset_counter;
    TCP_CLIENT station[2];
    TCP_CLIENT terminal;
    TCP_CLIENT device;
}ESP8266;
inline uint16_t timer_arrays_update(float amp, float freq, uint16_t * top, uint16_t * bottom)
{
    uint16_t base = 0xFFFF;

    if(freq < 100000.0)
    {
        freq = 100000.0;
    }
    if(amp > 1.0)
    {
        amp = 1.0;
    }
    base = (uint16_t)(CPU_FREQ / freq);
    top[1] = base - (uint16_t)(amp*base)/2;
    //top[1] = base - (uint16_t)(amp*base)/4;
    bottom[0] = base - top[1];

    return base;
}
#endif /* GEN_H_ */
