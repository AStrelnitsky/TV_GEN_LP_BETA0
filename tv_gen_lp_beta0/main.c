

/**
 * main.c
 */
#include "DSP28x_Project.h"
#include "main.h"
#include "math.h"
//#include "std.h"
#include "stdint.h"
#include "stdio.h"
#include "stddef.h"
#include "gen.h"
#include "CLAShared.h"
#include "at_commands.h"
#include "DSP28xx_SciUtil.h"
#include "buffer.h"
#include "errors.h"
#include "message.h"
#include "parser.h"
#include "serialiser.h"
#include "DSP2803x_GlobalPrototypes.h"
#include "IQmathLib.h"
#include "SFO_V6.h"
//#include "F2803x_Cla_defines.h"
//
#pragma DATA_SECTION(isr_counter,   "Cla1ToCpuMsgRAM");
//#pragma DATA_SECTION(sensors,   "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(volts,   "Cla1ToCpuMsgRAM");

#pragma DATA_SECTION(i_filt_counter,   "Cla1ToCpuMsgRAM");

#pragma DATA_SECTION(i_in_filt,   "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(i_in_filt_counter,   "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION (seq_counter, "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION (vhb, "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION (sns, "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION (ihb_ls, "Cla1ToCpuMsgRAM");
#define LOWPASS 1
#define PERIOD_DIVIDER_TIM 1
#define PERIOD_MULTIPLIER_TIM 32
#define BURST_FREQ 100

#pragma DATA_SECTION(fCoeffs,"Cla1ToCpuMsgRAM");
float fCoeffs[FILTER_LEN];//{0.0212, 0.00212, 0.00213, 0.00212, 0.0212};// {0.0625, 0.25, 0.375, 0.25, 0.0625};//{0.0625, 0.25, 0.375, 0.25, 0.0625};
#pragma DATA_SECTION(fDelayLine,"Cla1ToCpuMsgRAM");
float fDelayLine[FILTER_LEN];
#pragma DATA_SECTION(fDelayLine_VDC,"Cla1ToCpuMsgRAM");
float fDelayLine_VDC[FILTER_LEN];
//#pragma DATA_SECTION(fDelayLine_VRES,"Cla1ToCpuMsgRAM");
//float fDelayLine_VRES[FILTER_LEN];

#pragma DATA_SECTION(fDelayLine_VDC,"Cla1ToCpuMsgRAM");
//extern float fCoeffs_VDC[FILTER_LEN] = {0.0625, 0.25, 0.375, 0.25, 0.0625};
float32 volts[16];
//float32 sensors[16];
//float32 X[FILTER_LEN];
//Uint16 VoltRaw[PERIOD_DIVIDER];
Uint16 isr_counter;
Uint16 cla_step = 1;
Uint16 i_filt_counter;
float i_in_filt;
Uint16 i_in_filt_counter;
Uint16 seq_counter;
Uint16 vhb[16];
int16 sns[16];
int16 ihb_ls[16];
uint16_t mqtt_shift[16];
uint16_t mqtt_shift_counter = 0;
int cpu_temp = 0;//_IQ(0.0);
uint16_t vres_detector = 0;
//uint16_t rect_temp = 0;
//int bridge_temp = 0;
//
// The following will be placed in the CPU to CLA
// message RAM.  The main CPU can write to and read from
// this RAM.  The CLA can only read from it
//
// A is the filter coefficients passed by the main
// CPU to the CLA.
//
//#pragma DATA_SECTION(A,          "CpuToCla1MsgRAM");

#if LOWPASS
//float fCoeffs[FILTER_LEN] = {0.0214, 0.025, 0.025, 0.025, 0.0214};// {0.0625, 0.25, 0.375, 0.25, 0.0625};//{0.0625, 0.25, 0.375, 0.25, 0.0625};
#elif HIGHPASS
    float32 A [FILTER_LEN] = {0.0625L, -0.25L, 0.375L, -0.25L, 0.0625L};
#endif
#define PWM1_INT_ENABLE  0

#define PWM1_TIMER_TBPRD   0x1FFF
//FROM EXAMPLE/////
#define ADC_SAMPLE_PERIOD   1500
#define PWM_PERIOD          30000
#define PWM_DUTY_CYCLE      15000
#define ADC_BUF_LEN         64
////////////////////
#define BATTERY_VOLTAGE 380.0
#define U_KP_PI 0.92
#define READ_ROM 0x33
#define USER_DEBUG 1
#define WIFI_TIMER_OVERFLOW 10//0x200//0x1FF//0x1FF
#define WIFI_INIT_TIMER_OVERFLOW 0x250//0x1FF
#define STATUS_Q_MQTT 100//200//10//20//10//100

//__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
//__interrupt void cpu_timer2_isr(void);
__interrupt void cla1_isr7(void);
__interrupt void adc_isr(void);
__interrupt void sci_isr(void);

void init_GPIO (void);
void init_CPUT_timer (void);
void InitEPwmTimer1(void);
void InitEPwmTimer2(void);
void InitEPwmTimer3(void);
void InitEPwmTimer4(void);
void InitEPwmTimer5(void);
void gen_initial_conditions(void);
void init_adc(void);
void init_cla(void);
void reprom_processing(void);
uint16_t reprom_init_pulse_thread(void);
void initWiFi_station(void);
void wifi_init(void);
void esp8266Reset(void);
void init_SCI(void);
//void init_SCI_fast(void);
void sciReadBuff(void);
void debug_console(void);
void atCommandSend(char * str, uint16_t len);
uint16_t stringSeeker(const char * str_src, const char * str_tg, uint16_t len_1, uint16_t len_2, uint16_t * num);
uint16_t waitATReceive(const char * at, uint16_t len, uint16_t timeout);
void logs_processing(GEN_STRUCT * obj);
void atCommandManager(uint16_t command);
void mqtt_message_constructor(uint16_t * msg, uint16_t * raw, uint16_t raw_len, MQTT_VAR * var );
uint16_t connection_manager(void);
void device_manager(void);
void mqtt_manager(void);
void number_to_string (uint16_t num, MQTT_VAR * var);
void mqtt_parser(char * data);
uint16_t waitATReceive_nonbloking(const char * at, uint16_t len, uint16_t timeout);
uint16_t waitATReceive_nonbloking_service(const char * at, uint16_t len, uint16_t timeout);
uint16_t IPD_parser(const char * str);
uint16_t atStatusParser(const char * str);
uint16_t AT_parser(const char * str);
void portSeeker(const char * str_src, uint16_t len_1,  ESP8266 * esp);
_iq loopProcessing(LOOP * loop, PI_REG * pi, LOOPS_ENABLE flag, _iq norm);
inline _iq piProcessing(PI_REG * pi);
_iq integratorProcessing(INTEGRATOR_Q *i);
void resetLoops(GEN_STRUCT * d);
uint32_t main_counter = 0;
uint16_t SINTOP1[2];
uint16_t SINBOT1[2];
Uint16 SampleCount;
//Uint16 AdcBuf[ADC_BUF_LEN];
//Uint16 AdcFiltBuf[ADC_BUF_LEN];
//char recbuff[100];
uint16_t d = 0;
uint32_t acqps = 6;
uint16_t ap = 1;
uint16_t len;
const char *s1;
const char *s_1310 = "\r\n";
const char *s_13_10 = "\n";

uint16_t mqtt_test_flag = 0;
uint16_t mqtt_message_pointer = 0;

const char * str_open = "{";
const char * str_var1 = "\"val1\":";
const char * str_var2 = "\"val2\":";
const char * str_var3 = "\"val3\":";
const char * str_var4 = "\"val4\":";
const char * str_var5 = "\"val5\":";
const char * str_var6 = "\"val6\":";
const char * str_var7 = "\"val7\":";
const char * str_var8 = "\"val8\":";
const char * str_var9 = "\"val9\":";
const char * str_var10 = "\"val10\":";
const char * str_var11 = "\"val11\":";
const char * str_var12 = "\"val12\":";

const char * str_comma = ",";
const char * str_close = "}";
const char * s1_p;
uint16_t crc = 0;
char data_tx[64], data_rx[64], data_rx_service[64], data_rx_mqtt[64];//, data_rx_1[64];

extern Uint16 Cla1funcsLoadStart;
extern Uint16 Cla1funcsLoadSize;
extern Uint16 Cla1funcsRunStart;
extern Uint32 Cla1Prog_Start;

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
uint16_t detection_counter = 0;
//int min_int(uint16_t * data, uint16_t len);
//int mean_int(uint16_t * data, uint16_t len);
GEN_STRUCT gen;
DS2502R reprom;
ESP8266 esp8266;
AT_COMMAND atCommand_tx;//, atCommand_rx;

uint16_t mqtt_publish_common[140];
MQTT_CLIENT mqtt;
/*
uint16_t mqtt_connect[] =            {
                                     0x10, 0x34, 0x00, 0x06, 0x4d, 0x51, 0x49, 0x73, 0x64, 0x70, 0x03, 0xc2, 0x00, 0x3c, 0x00, 0x0e,
                                     0x6d, 0x71, 0x74, 0x74, 0x78, 0x5f, 0x65, 0x33, 0x64, 0x32, 0x64, 0x36, 0x62, 0x36, 0x00, 0x08,
                                     0x70, 0x6a, 0x78, 0x76, 0x7a, 0x72, 0x64, 0x61, 0x00, 0x0c, 0x4d, 0x49, 0x53, 0x56, 0x77, 0x35,
                                     0x39, 0x6c, 0x4f, 0x5f, 0x39, 0x6e
                                     };*/
/*
uint16_t mqtt_connect[] =            {
                                     0x10, 0x55, 0x00, 0x04, 0x4d, 0x51, 0x54, 0x54, 0x04, 0xc2, 0x00, 0xc2, 0x00, 0x0a, 0x00, 0x31,
                                     0x36, 0x63, 0x36, 0x64, 0x64, 0x30, 0x37, 0x34, 0x2d, 0x35, 0x64, 0x37, 0x63, 0x2d, 0x34, 0x39,
                                     0x31, 0x61, 0x78, 0x76, 0x7a, 0x72, 0x64, 0x61, 0x00, 0x0c, 0x4d, 0x49, 0x53, 0x56, 0x77, 0x35,
                                     0x39, 0x6c, 0x4f, 0x5f, 0x39, 0x6e
                                     };
                                     */
uint16_t mqtt_connect[200];
/*
uint16_t mqtt_subscribe_send[] =     {
                                     0x82, 0x10, 0x00, 0xfe, 0x00, 0x0b, 0x74, 0x76, 0x5f, 0x6d, 0x75, 0x73, 0x65, 0x75, 0x6d, 0x5f,
                                     0x31, 0x00
                                     };


uint16_t mqtt_subscribe_send[] =     {
                                     0x82, 0x10, 0x00, 0xfe, 0x00, 0x0b, 0x74, 0x76, 0x5f, 0x6d, 0x75, 0x73, 0x65, 0x75, 0x6d, 0x5f,
                                     0x63, 0x00
                                     };
*/
uint16_t mqtt_subscribe_send[48];

uint16_t base = 1000;
uint16_t base_led = 1000;
uint16_t remote_data[64];
mqtt_parser_t parser;
mqtt_serialiser_t serialiser;
mqtt_message_t message;
mqtt_message_t connect_message;
mqtt_message_t publish_message;
mqtt_message_t subscribe_message;
char str_out[64];
MQTT_VAR mqtt_var[3];
char digit_buf[3];
uint16_t digit_len;
uint16_t val_p = 0;
uint16_t var_counter;
uint16_t b = 0;
uint32_t cnt = 0;
uint16_t reprom_state = 0;
uint32_t read_id_wait = 0;
uint16_t wifi_init_flag = 0;
float32 apm = 0.0;
float32 burst_duty = 0.0;
uint16_t duty_fine = 0;
uint16_t prd_fine = 0;
uint16_t id_state = 0;
uint16_t e_detector = 0;
uint16_t uh = 18;
uint16_t ul = 24;
uint16_t idc = 0;
uint32_t idc_acc = 0;
uint16_t tim_counter = 0;
int MEP_ScaleFactor;
uint16_t publish_counter = 0;
//
// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
//
volatile struct EPWM_REGS *ePWM[PWM_CH] =
             {  &EPwm1Regs, &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs};
//char recbuff[SCI_RX_SIZE];
int main(void)
{
        int i = 0;
        Uint16 RamfuncsLoadSize;
        int32 reset_flag = 0;
        InitSysCtrl();
        DINT;
        //
        // Initialize the PIE control registers to their default state.
        // The default state is all PIE interrupts disabled and flags
        // are cleared.
        // This function is found in the DSP2803x_PieCtrl.c file.
        //
        InitPieCtrl();
        //
        // Disable CPU interrupts and clear all CPU interrupt flags:
    //    //
        IER = 0x0000;
        IFR = 0x0000;
        //
        // Initialize the PIE vector table with pointers to the shell Interrupt
        // Service Routines (ISR).
        // This will populate the entire table, even if the interrupt
        // is not used in this example.  This is useful for debug purposes.
        // The shell ISR routines are found in DSP2803x_DefaultIsr.c.
        // This function is found in DSP2803x_PieVect.c.
        //
        InitPieVectTable();

       // RamfuncsLoadSize = RamfuncsLoadStart - RamfuncsLoadEnd;
      //  memcpy((uint16_t *)&RamfuncsRunStart,(uint16_t *)&RamfuncsLoadStart, (unsigned long)&RamfuncsLoadSize);

        EALLOW;
        PieVectTable.CLA1_INT7 = &cla1_isr7;
        PieVectTable.SCIRXINTA = &sci_isr;
        EDIS;
        init_GPIO();
      //  EALLOW;             // This is needed to write to EALLOW protected register
      //  PieVectTable.ADCINT1 = &adc_isr;
      //  EDIS;    // This is needed to disable write to EALLOW protected registers
       // PieCtrlRegs.PIEIER11.bit.INTx7 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx1 = 1; //SCI
        IER |= M_INT11;
        IER |= M_INT9; //SCI
        EINT;   // Enable Global interrupt INTM
        ERTM;

       //initWiFi_station();

        gen_initial_conditions();

        init_cla();
        init_adc();

        base = timer_arrays_update(gen.AMP, (4*gen.F), SINTOP1, SINBOT1);
        base_led =  (uint16_t)(base * PERIOD_DIVIDER_TIM);

        InitEPwmTimer1();
        InitEPwmTimer2();
        InitEPwmTimer3();
        InitEPwmTimer4();
        //InitEPwmTimer5();

        init_CPUT_timer();

        PieCtrlRegs.PIEIER11.bit.INTx7 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
       // while (read_id_wait < 210)
        reprom.id[0] = '~';//reprom.id[0];
        reprom.id[1] = '>';//reprom.id[1];
        reprom.id[2] = '0';//reprom.id[2];
        reprom.id[3] = '0';//reprom.id[3];
        reprom.id[4] = '0';//reprom.id[4];
        reprom.id[5] = '0';//reprom.id[5];

      //  while (id_state < 7)
     //   {
    //        GpioDataRegs.GPATOGGLE.bit.GPIO4 |= 1;
    //        ++read_id_wait;
    //    }

        //ConfigCpuTimer(&CpuTimer1, 60, 50000);

        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
        PieCtrlRegs.PIEIER11.bit.INTx7 = 0;
        //IER |= M_INT11;
        PieCtrlRegs.PIEIER9.bit.INTx1 = 0;

        PieCtrlRegs.PIEIER9.bit.INTx1 = 1;

        GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
        initWiFi_station();

        wifi_init_flag = 1;
        CpuTimer1.RegsAddr->PRD.all = 29999;//CpuTimer1.RegsAddr->PRD.all = 599999;
        gen.F = 71600.0;
        while(1)
        {
            GpioDataRegs.GPATOGGLE.bit.GPIO4 |= 1;
            logs_processing(&gen);
            connection_manager();
            uh = (uint16_t)(mqtt.uh);
            ul = (uint16_t)(mqtt.ul);

            if(volts[ADC_TINT] > 90.0)
            {
               burst_duty = 0.0;
            }
            if(cpu_temp > 120)
            {
               burst_duty = 0.0;
            }
            if((data_rx[6] > 85) || (data_rx[7] > 85) || ((volts[ADC_TEXT]) > 85.0) )
            {
                burst_duty = 0.0;
            }
            SFO();

        }
}
void init_GPIO(void)
{
    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1; //SETUP LED BLUE
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1; //SETUP LED RED
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1; //SETUP LED GREEN

    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; // SETUP AS EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1; // SETUP AS EPWM1B


    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // SETUP AS EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; // SETUP AS EPWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1; // SETUP AS EPWM3A

    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1; // SETUP DQ AS OUTPUT
    GpioDataRegs.GPASET.bit.GPIO5 = 1; // SETUP DQ HIGH

    GpioCtrlRegs.GPADIR.bit.GPIO6 = 0; //SETUP EXTERNAL GPIO6 AS INPUT
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 0; //SETUP EXTERNAL GPIO8 AS INPUT

    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1; //SETUP WIFI RESET GPIO9 AS OUTPUT
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //SETUP WIFI RESET IS INACTIVE

    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1; //SETUP WIFI GPIO0 AS OUTPUT
    GpioDataRegs.GPASET.bit.GPIO31= 1; //SETUP WIFI GPIO0 SET

    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0; // SETUP SCI RX AS INPUT
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1; // SETUP SCI TX AS OUTPUT

    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2; // GPIO7 AS SCI RX
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2; // GPIO12 AS SCI TX

    GpioCtrlRegs.GPADIR.bit.GPIO11 = 0; // SETUP OD HALL AS INPUT
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 0; // SETUP Log_PFC_Rsrv_R AS INPUT

    GpioCtrlRegs.GPADIR.bit.GPIO18 = 0; // SETUP Log_PFC_Error_R AS INPUT

    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;// SETUP Log_OT_Tint AS INPUT
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;// SETUP Log_OC_Ihb_ls AS INPUT

    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1; //SETUP Log_ERROR_RST AS OUTPUT
    GpioDataRegs.GPASET.bit.GPIO22 = 1; //SETUP NON CLEARED LATCHES

    GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;// SETUP Log_OC_Idc AS INPUT
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;// SETUP Log_OC_Vres AS INPUT

    GpioCtrlRegs.GPADIR.bit.GPIO28 = 0;// SETUP Log_GPIO28 AS INPUT
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 0;// SETUP Log_GPIO29 AS INPUT

    //GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;// SETUP Log_PWM_Dis_R AS INPUT
    //GpioDataRegs.GPASET.bit.GPIO30 = 1;// SET TO HIGH Log_PWM_Dis_R

    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;// SETUP Log_OC_Ihb AS INPUT
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;// SETUP Log_OV_Vhb AS INPUT
    EDIS;

}
void init_CPUT_timer(void)
{
    EALLOW;  // This is needed to write to EALLOW protected registers
   // PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.TINT1 = &cpu_timer1_isr;
   // PieVectTable.TINT2 = &cpu_timer2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    InitCpuTimers();

  //  ConfigCpuTimer(&CpuTimer0, 60, 100000);
      ConfigCpuTimer(&CpuTimer1, 60, 100000);
 //   ConfigCpuTimer(&CpuTimer2, 60, 100000);

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in DSP2803x_CpuTimers.h), the
    // below settings must also be updated.
    //
   // CpuTimer0Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0MP
    CpuTimer1Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0
 //   CpuTimer2Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0
    //
    // Step 5. User specific code, enable interrupts:
    //
    //
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:
    //
    //IER |= M_INT1;
    IER |= M_INT13;
   // IER |= M_INT14;
    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
}
__interrupt void cpu_timer1_isr(void)
{
    _iq d_b = _IQ(0.0);
    vres_detector = data_rx[4] + (((uint16_t)data_rx[5]) << 8);
    if(tim_counter < 48)
    {
        if(volts[ADC_IDC] > 0.0)
        {
           idc_acc  += (uint16_t)(volts[ADC_IDC]*10.0);
        }
        ++tim_counter;
    }
    else
    {
        idc = idc_acc / tim_counter;
        tim_counter = 0;
        idc_acc = 0;
    }
   // rect_temp = (uint16_t)data_rx[6];
    if(mqtt.swicth == 0)
    {
        /*
        if(volts[ADC_VDC] >= 280.0)
        {
            if(e_detector == 0)
            {
               burst_duty = 0.35;
            }
            else
            {
               burst_duty = 0.96;
            }
        }
        else if(volts[ADC_VDC] < 200.0)
        {
          e_detector = 0;
          burst_duty = 0.0;
        }
        */
        if(idc <= 4)
        {
            if(esp8266.station[1].status == TCP_CLIENT_IS_NOT_CONNECTED)
            {
              //  if(esp8266.station[1].lost_connection > 20)
             //   {
                    gen.leds_duty[0] = 0.2;
                    burst_duty = 0.03;
            //    }
            //    else
            //    {
            //        ++(esp8266.station[1].lost_connection);
            //    }
            }
            else if(esp8266.station[1].status == TCP_CLIENT_IS_CONNECTED)
            {
                if(!(cnt % 10))
                {
                    if(esp8266.station[1].lost_connection < 20)
                    {
                        gen.leds_duty[0] = 0.0;
                        gen.leds_duty[2] = 0.2;
                        gen.u_out_loop.in = V_NOM;
                        gen.u_out_loop.fbk = _IQ(vres_detector);
                        gen.u_out_loop_en = LOOP_ENABLE;
                        d_b = loopProcessing(&(gen.u_out_loop), &(gen.pi_u), gen.u_out_loop_en, V_NOM);
                        burst_duty = _IQtoF(d_b);
                    }
                    else
                    {
                        gen.leds_duty[0] = 0.2;
                        gen.leds_duty[2] = 0.0;
                    }
                    ++(esp8266.station[1].lost_connection);
                }
            }
        }
        else
        {
            if(esp8266.station[1].status == TCP_CLIENT_IS_CONNECTED)
            {
              if(!(cnt % 10))
              {
                  if(esp8266.station[1].lost_connection < 20)
                  {
                     gen.leds_duty[0] = 0.0;
                     gen.leds_duty[2] = 0.2;
                  }
                  else
                  {
                     gen.leds_duty[0] = 0.2;
                     gen.leds_duty[2] = 0.0;
                  }
                     ++(esp8266.station[1].lost_connection);
              }
            }
            burst_duty = 0.7;
        }
    }
    else
    {
        if(mqtt.duty <= 1)
        {
            mqtt.duty = 0;
        }
        burst_duty = 0.01*(mqtt.duty);
    }
    if(CpuTimer1.InterruptCount >= BURST_FREQ)
    {
        CpuTimer1.InterruptCount = 0;
        gen.AMP = 0.95;//0.48;
    }
    if(CpuTimer1.InterruptCount < BURST_FREQ)
    {
        if((CpuTimer1.InterruptCount) >= ((burst_duty)*(BURST_FREQ)))
        {
            gen.AMP = 0.0;//apm
        }
        else
        {
            //vres_detector = (uint16_t)(volts[ADC_VRES]);
        }
        CpuTimer1.InterruptCount++;
    }

   cpu_temp = GetTemperatureC(sns[0]);

   ++cnt;
   base = timer_arrays_update(gen.AMP, (4*gen.F), SINTOP1, SINBOT1);
   base_led =  0x48;//(uint16_t)(base);//(uint16_t)(base * PERIOD_DIVIDER);

   EALLOW;
   EPwm1Regs.TBPRD = (uint16_t)(base);

   EPwm1Regs.TBPRDHR = prd_fine;

   EPwm1Regs.CMPA.half.CMPA = SINBOT1[0];
  // EPwm1Regs.CMPA.all = SINBOT1[0] + (duty_fine << 8);
   EPwm1Regs.CMPB = SINTOP1[1];

   EPwm2Regs.TBPRD = base_led;
   EPwm2Regs.CMPA.half.CMPA = (gen.leds_duty[0])*(EPwm2Regs.TBPRD);
   EPwm2Regs.CMPB = (gen.leds_duty[1])*(EPwm2Regs.TBPRD);

   EPwm3Regs.TBPRD = base_led;
   EPwm3Regs.CMPA.half.CMPA = (gen.leds_duty[2])*(EPwm3Regs.TBPRD);

   if(wifi_init_flag == 1)
   {
       EPwm4Regs.TBPRD = (base * PERIOD_MULTIPLIER_TIM);
   }
   else
   {
       EPwm4Regs.TBPRD = (base / PERIOD_DIVIDER_TIM);
   }
   EPwm4Regs.CMPA.half.CMPA = (EPwm4Regs.TBPRD / 2);

   EDIS;

  // debug_console();
}
__interrupt void cpu_timer2_isr(void)
{
   EALLOW;
   CpuTimer2.InterruptCount++;
   //
   // The CPU acknowledges the interrupt.
   //
   EDIS;
}
void InitEPwmTimer1(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;
    //
    // Disable Sync
    //
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 11;//10;  // Pass through
    //
    // Initially disable Free/Soft Bits
    //
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 0;
    //EPwm1Regs.TBPRD = PWM1_TIMER_TBPRD;          // Set up PWM1 Period

    EPwm1Regs.TBPRD = (uint16_t)(base);
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;   // Count up mode

    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;//ET_CTR_ZERO;    // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = PWM1_INT_ENABLE; // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;          // Generate INT on 1st event
    EPwm1Regs.TBCTR = 0x0000;                    // Clear timer counter

    //EPwm1Regs.CMPA.half.CMPA = 0;//PWM1_TIMER_TBPRD / 2;//;
    //EPwm1Regs.CMPB = 0;
   // EPwm1Regs.CMPA.half.CMPAHR = (1 << 8);

    EPwm1Regs.CMPA.half.CMPA = SINBOT1[0];
    EPwm1Regs.CMPB = SINTOP1[1];

    //EPwm1Regs.TZSEL.bit.CBC1 = 1;
    //EPwm1Regs.TZCTL.bit.TZA = 2;
   // EPwm1Regs.TZCTL.bit.TZB = 2;
    //EPwm1Regs.AQCTLA.all = 0x0024;
    //EPwm1Regs.AQCTLB.all = 0x0018;

    EPwm1Regs.AQCTLA.all = 0x0090;
    EPwm1Regs.AQCTLB.all = 0x0600;

    EALLOW;
    EPwm1Regs.HRCNFG.all = 0x0;
    EPwm1Regs.HRCNFG.bit.EDGMODE = HR_REP;      //MEP control on Rising edge
    EPwm1Regs.HRCNFG.bit.CTLMODE = HR_CMP;
    EPwm1Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
    EPwm1Regs.HRPCTL.bit.HRPE = 0x1;
    EPwm1Regs.HRCNFG.bit.AUTOCONV = 1;
        //
        // Enable TBPHSHR sync (required for updwn count HR control)
        //
    EPwm1Regs.HRPCTL.bit.TBPHSHRLOADE = 1;
        //
        // Turn on high-resolution period control
        //
    EPwm1Regs.HRPCTL.bit.HRPE = 1;
    EDIS;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Start all the timers synced
    EDIS;
}

void InitEPwmTimer2(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;
    //
    // Disable Sync
    //
    EPwm2Regs.TBCTL.bit.SYNCOSEL = 11;  // Pass through
    //
    // Initially disable Free/Soft Bits
    //
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0;

    //EPwm2Regs.TBPRD = PWM1_TIMER_TBPRD;          // Set up PWM1 Period
    EPwm2Regs.TBPRD = base_led;

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;   // Count up mode
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = PWM1_TIMER_TBPRD / 2; // Enable INT
    EPwm2Regs.TBCTR = 0x0000;                    // Clear timer counter

    //EPwm2Regs.CMPA.half.CMPA = 0;//;
    //EPwm2Regs.CMPB = 0;
    EPwm2Regs.CMPA.half.CMPA = (gen.leds_duty[0])*(EPwm2Regs.TBPRD);
    EPwm2Regs.CMPB = (gen.leds_duty[1])*(EPwm2Regs.TBPRD);

    EPwm2Regs.AQCTLA.all = 0x0090;
    EPwm2Regs.AQCTLB.all = 0x0090;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Start all the timers synced
    EDIS;
}
void InitEPwmTimer3(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;
    //
    // Disable Sync
    //
    EPwm3Regs.TBCTL.bit.SYNCOSEL = 11;  // Pass through
    //
    // Initially disable Free/Soft Bits
    //
    EPwm3Regs.TBCTL.bit.FREE_SOFT = 0;
    //EPwm3Regs.TBPRD = PWM1_TIMER_TBPRD;          // Set up PWM1 Period
    EPwm3Regs.TBPRD = base_led;

    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;   // Count up mode
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = PWM1_TIMER_TBPRD / 2; // Enable INT
    EPwm3Regs.TBCTR = 0x0000;                    // Clear timer counter

    //EPwm3Regs.CMPA.half.CMPA = 0;//;
    EPwm3Regs.CMPA.half.CMPA = (gen.leds_duty[2])*(EPwm3Regs.TBPRD);
    EPwm3Regs.CMPB = 0;

    EPwm3Regs.AQCTLA.all = 0x0090;
    //EPwm2Regs.AQCTLB.all = 0x0090;
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Start all the timers synced
    EDIS;
}
void InitEPwmTimer4(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;
    //
    // Disable Sync
    //
    EPwm4Regs.TBCTL.bit.SYNCOSEL = 11;  // Pass through
    //
    // Initially disable Free/Soft Bits
    //
    EPwm4Regs.TBCTL.bit.FREE_SOFT = 0;
    //EPwm4Regs.TBPRD = PWM1_TIMER_TBPRD;          // Set up PWM1 Period

    EPwm4Regs.TBPRD = (base / PERIOD_DIVIDER_TIM);

    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;   // Count up mode
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;

    EPwm4Regs.ETSEL.bit.SOCAEN = TB_ENABLE;
    EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;//ET_CTR_PRD;//ET_CTRU_CMPA;
    EPwm4Regs.ETPS.bit.SOCAPRD = ET_2ND;
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;//ET_CTRU_CMPA;
    EPwm4Regs.ETSEL.bit.INTEN = 0; // Enable INT
    EPwm4Regs.TBCTR = 0x0000;                    // Clear timer counter

    //EPwm4Regs.CMPA.half.CMPA = 0;//;
    EPwm4Regs.CMPA.half.CMPA = (EPwm4Regs.TBPRD / 2);
    EPwm4Regs.CMPB = 0;

    //EPwm4Regs.AQCTLA.all = 0x0090;
    //EPwm2Regs.AQCTLB.all = 0x0090;
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Start all the timers synced
    EDIS;
}
void InitEPwmTimer5(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;
    //
    // Disable Sync
    //
    EPwm5Regs.TBCTL.bit.SYNCOSEL = 11;  // Pass through
    //
    // Initially disable Free/Soft Bits
    //
    EPwm5Regs.TBCTL.bit.FREE_SOFT = 0;
    //EPwm4Regs.TBPRD = PWM1_TIMER_TBPRD;          // Set up PWM1 Period

    EPwm5Regs.TBPRD = (base * PERIOD_DIVIDER_TIM);

    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;   // Count up mode
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;

    EPwm5Regs.ETSEL.bit.SOCAEN = TB_ENABLE;
    EPwm5Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;//ET_CTR_PRD;//ET_CTRU_CMPA;
    EPwm5Regs.ETPS.bit.SOCAPRD = ET_2ND;
    EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;//ET_CTRU_CMPA;
    EPwm5Regs.ETSEL.bit.INTEN = 0; // Enable INT
    EPwm5Regs.TBCTR = 0x0000;                    // Clear timer counter

    //EPwm4Regs.CMPA.half.CMPA = 0;//;
    EPwm5Regs.CMPA.half.CMPA = 0;
    EPwm5Regs.CMPB = 0;

    //EPwm4Regs.AQCTLA.all = 0x0090;
    //EPwm2Regs.AQCTLB.all = 0x0090;
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   // Start all the timers synced
    EDIS;
}
void gen_initial_conditions(void)
{

    memset((uint16_t *)(&gen),0, sizeof(GEN_STRUCT));
    memset((uint16_t *)(&reprom),0, sizeof(DS2502R));

    gen.AMP = 0.0;//46;//0.65;
    gen.F = 144000.0;//208000.0;
    gen.leds_duty[0] = 0.2;
    gen.leds_duty[1] = 0.2;
    gen.leds_duty[2] = 0.2;

    reprom.time_scale = 1;
    reprom.ID = 0;
    reprom.CRC = 0;
    reprom.family = 0;
    reprom.init_cnt = 0;
    reprom.ack_cnt = 0;
    reprom.init_delay = 210;//110;//50;//100;//500;
    reprom.ID_read_flag = 1;
    reprom.init_ack = 0;
    reprom.ack_delay = 8;//2;//15;
    reprom.read_cnt = 0;
    reprom.start_write_cnt = 0;
    reprom.start_write_cnt_1 = 0;
    reprom.start_write_cnt_2 = 0;
    reprom.read_delay = 20;//8;//80;//0;
    reprom.start_read_cnt = 0;
    reprom.start_read_delay = 2;//1;//40;
    reprom.start_write_delay = 2;//1;
    reprom.write_cnt = 0;
    reprom.write_delay = 2;//1;//14;
    reprom.init_release = 200;//100;//50;//100;
    reprom.start_release_cnt = 0;
    reprom.start_release_delay = 4;//3;//30;
    reprom.read_window = 4;//2;
    reprom.state = 0;
    reprom.ID = 0;

    /*
    reprom.time_scale = 1;
    reprom.ID = 0;
    reprom.CRC = 0;
    reprom.family = 0;
    reprom.init_cnt = 0;
    reprom.ack_cnt = 0;
    reprom.init_delay = 50;//110;//50;//100;//500;
    reprom.ID_read_flag = 1;
    reprom.init_ack = 0;
    reprom.ack_delay = 8;//2;//15;
    reprom.read_cnt = 0;
    reprom.start_write_cnt = 0;
    reprom.start_write_cnt_1 = 0;
    reprom.start_write_cnt_2 = 0;
    reprom.read_delay = 8;//80;//0;
    reprom.start_read_cnt = 0;
    reprom.start_read_delay = 1;//40;
    reprom.start_write_delay = 1;
    reprom.write_cnt = 0;
    reprom.write_delay = 1;//14;
    reprom.init_release = 50;//100;//50;//100;
    reprom.start_release_cnt = 0;
    reprom.start_release_delay = 3;//30;
    reprom.read_window = 2;
    reprom.state = 0;
    reprom.ID = 0;
    atCommand_tx.tx_flag = 0;
    */
    mqtt_parser_init(&parser);
    mqtt_serialiser_init(&serialiser);
    mqtt_message_init(&message);
    mqtt_message_init(&connect_message);
    mqtt_message_init(&subscribe_message);

    message.common.type = MQTT_TYPE_PUBLISH;
    message.common.dup = 0;
    message.common.qos = 0;
    message.common.retain = 0;
    message.publish.type = 0x03;//
    message.publish.dup = 0;
    message.publish.qos = 0;
    message.publish.retain = 0;
    message.publish.topic_name.data = (uint16_t *)("a_tv_wpt__t");
    message.publish.topic_name.length = 11;
    message.publish.content.data = (uint16_t *)("{\"val1\":74,\"val2\":34,\"val3\":88}");
    message.publish.content.length = 31;

    connect_message.common.type = MQTT_TYPE_CONNECT;
    connect_message.connect.client_id.data = (uint16_t *)("kick_tv_wpt__test_client_id0000000xxxxxx");
    connect_message.connect.client_id.length = 40;
    connect_message.connect.password.data = (uint16_t *)("d4HU1TGw2Acj");
    connect_message.connect.password.length = 12;
    connect_message.connect.protocol_name.data = (uint16_t *)("MQTT");
    connect_message.connect.protocol_name.length = 4;
    connect_message.connect.protocol_version = 4;
    connect_message.connect.username.data = (uint16_t *)("kpcbiovv");
    connect_message.connect.username.length = 8;
    connect_message.connect.flags.clean_session = 1;
    connect_message.connect.flags.password_follows = 1;
    connect_message.connect.flags.username_follows = 1;
    connect_message.connect.flags.will_retain = 0;
    connect_message.connect.flags.will_qos = 0;
    connect_message.connect.flags.will = 0;
    connect_message.connect.keep_alive = 10;
    connect_message.common.length = mqtt_serialiser_size(&serialiser, &connect_message) - 2;

    mqtt.connect_p_len  = connect_message.common.length + 2;
    mqtt.state_machine = MQTT_CONNECT_TO_SERVER;
    mqtt.uh = 22.0;
    mqtt.ul = 23.0;
    mqtt.swicth = 0;
    mqtt.k1 = 24.0;
    mqtt.k2 = 22.0;
    mqtt_serialiser_write(&serialiser, &connect_message, &mqtt_connect[0], mqtt.connect_p_len); //-2??

    subscribe_message.common.type = MQTT_TYPE_SUBSCRIBE;
    subscribe_message.common.qos = MQTT_QOS_EXACTLY_ONCE;
   // subscribe_message.common.
    subscribe_message.subscribe.topics->name.data = (uint16_t *)("WPTC2023/tv_wpt_xxxxxx/control");
    subscribe_message.subscribe.topics->name.length = 30;
    subscribe_message.subscribe.message_id = 14598;
    subscribe_message.subscribe.qos = MQTT_QOS_AT_MOST_ONCE;///MQTT_QOS_AT_LEAST_ONCE;

    subscribe_message.common.length = 35;//mqtt_serialiser_size(&serialiser, &subscribe_message) - 2;

    mqtt.subscribe_p_len  = subscribe_message.common.length + 2;
    mqtt.state_machine = MQTT_CONNECT_TO_SERVER;
    subscribe_message.common.qos = MQTT_QOS_AT_LEAST_ONCE;
    mqtt_serialiser_write(&serialiser, &subscribe_message, &mqtt_subscribe_send[0], mqtt.subscribe_p_len); //-2??
    memset(remote_data, 0, sizeof(remote_data));
    gen.detector = 0;
    wifi_init_flag = 0;

   // memset(data_tx,0,64);//char data_tx[64], data_rx[64], data_rx_service[64], data_rx_mqtt[64];
    memset(data_rx,0,64);
    memset(data_rx_service,0,64);
    memset(data_rx_mqtt,0,64);
    esp8266.station[0].status = TCP_CLIENT_IS_NOT_CONNECTED;
    esp8266.station[1].status = TCP_CLIENT_IS_NOT_CONNECTED;
    gen.pi_u.kd = _IQ(0.0);
    gen.pi_u.ki = _IQ(0.1);
    gen.pi_u.kp = _IQ(0.2);
    gen.pi_u.out = _IQ(0.0);
    gen.pi_u.kd = _IQ(0.0);
    gen.pi_u.ki = _IQ(0.4);
    gen.pi_u.kp = _IQ(0.2);
    gen.pi_u.out = _IQ(0.0);

    resetLoops(&gen);
}
void init_adc(void)
{
    //
    // Assumes ADC clock is already enabled in InitSysCtrl();
    //
    //
    // Call the InitAdc function in the DSP2803x_Adc.c file
    //
    // This function calibrates and powers up the ADC to
    // into a known state.
    //
    InitAdc();
    //AdcOffsetSelfCal();
    //
    // Specific ADC configuration for this example
    //
    // ADC interrupt will trigger early - before the ADC conversion starts
    // Enable ADCINT7
    // Disable ADC 7 continuous mode
    // Set the SOC1 channel select to ADCINTA2.
    // This interrupt triggers task 7
    // which is where the CLA FIR filter is located
    //
    EALLOW;

    //
    // ADC interrupt comes early (end of sample window)
    // SOC1 will trigger ADCINT7
    // Enable ADCINT7
    // Disable ADCINT7 Continuous mode
    //
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 0;
    AdcRegs.INTSEL7N8.bit.INT7SEL   = 0x0F;
    AdcRegs.INTSEL7N8.bit.INT7E     = 1;
    AdcRegs.INTSEL7N8.bit.INT7CONT  = 1;//0;
    AdcRegs.ADCCTL1.bit.TEMPCONV = 1;
    //
    // set SOC1 channel select to ADCINA2
    // set SOC1 start trigger on EPWM1A interrupt
    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 5;// Internal Temp Sensor 0//ADC_Ihb_Is2;
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 8;

    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 1;//ADC_Ihb_Is2;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 2;// ADC_Vhb
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC3CTL.bit.CHSEL    = 3;// Tint
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC4CTL.bit.CHSEL    = 4;// ADC_Ihb2;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC4CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC5CTL.bit.CHSEL    = 5;// ADC_Vres 2;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC5CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC6CTL.bit.CHSEL    = 6;// ADC_Text 2;
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC6CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC7CTL.bit.CHSEL    = 7;// ADC_Vdc
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC7CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC8CTL.bit.CHSEL    = 8; //ADC_Idc //2;
    AdcRegs.ADCSOC8CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC8CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC9CTL.bit.CHSEL    = 9;// ADC_PFC_Temp //2;
    AdcRegs.ADCSOC9CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC9CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC10CTL.bit.CHSEL    = 10; //ADC_PFC_Vin
    AdcRegs.ADCSOC10CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC10CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC11CTL.bit.CHSEL    = 11;//ADC_PFC_Iin 2;
    AdcRegs.ADCSOC11CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC11CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC12CTL.bit.CHSEL    = 12;// ADC_Version 2;
    AdcRegs.ADCSOC12CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC12CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC13CTL.bit.CHSEL    = 13;
    AdcRegs.ADCSOC13CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC13CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC14CTL.bit.CHSEL    = 14;
    AdcRegs.ADCSOC14CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC14CTL.bit.ACQPS    = 8;//6;

    AdcRegs.ADCSOC15CTL.bit.CHSEL    = 15;
    AdcRegs.ADCSOC15CTL.bit.TRIGSEL  = 0x0B; //EPWM4_SOCA
    AdcRegs.ADCSOC15CTL.bit.ACQPS    = 8;//6;
    EDIS;
}
void init_cla(void)
{
    //
    // This code assumes the CLA clock is already enabled in
    // the call to InitSysCtrl();
    //
    // EALLOW: is needed to write to EALLOW protected registers
    // EDIS: is needed to disable write to EALLOW protected registers
    //
    // Initialize the interrupt vectors for Task 7 (CLA FIR Filter)
    // and for Task 8 (FIR filter initialization)
    //
    // The symbols used in this calculation are defined in the CLA
    // assembly code and in the CLAShared.h header file
    //
    EALLOW;
    Cla1Regs.MVECT7 = (Uint16)((Uint32)&Cla1Task7 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT8 = (Uint16)((Uint32)&Cla1Task8 - (Uint32)&Cla1Prog_Start);

    //
    // Task 7 has the option to be started by either EPWM7_INT or ADCINT7
    // In this case we will allow ADCINT7 to start CLA Task 7
    //
    Cla1Regs.MPISRCSEL1.bit.PERINT7SEL = CLA_INT7_ADCINT7;

    //
    // Copy the CLA program code from its load address to the CLA program
    // memory. Once done, assign the program memory to the CLA
    //
    // Make sure there are at least two SYSCLKOUT cycles between assigning
    // the memory to the CLA and when an interrupt comes in
    //
    memcpy((uint16_t *)&Cla1funcsRunStart, (uint16_t *)&Cla1funcsLoadStart, (unsigned long)&Cla1funcsLoadSize);

    Cla1Regs.MMEMCFG.bit.PROGE = 1;

    //
    // Enable the IACK instruction to start a task
    // Enable the CLA interrupt 8 and interrupt 7
    //
    Cla1Regs.MCTL.bit.IACKE = 1;
    Cla1Regs.MIER.all = (M_INT8 | M_INT7);

    //
    // Force CLA task 8 using the IACK instruction
    // Task 8 will initialize the filter input delay
    // line to zero (X[0] - X[4]).
    //
    // No need to wait, the task will finish by the time
    // we configure the ePWM and ADC modules
    //
    Cla1ForceTask8();
}
__interrupt void cla1_isr7()
{
    //
    // Clear the ADC interrupt flag so the next SOC can occur
    // Clear the IACK bits so another interrupt
    // can be taken
    // Toggle GPIO for status
    //
    GpioDataRegs.GPATOGGLE.bit.GPIO2 = 1;

    AdcRegs.ADCINTFLGCLR.bit.ADCINT7 = 1;
    PieCtrlRegs.PIEACK.all = 0xFFFF;
    if(wifi_init_flag == 1)
    {
        switch(isr_counter)
        {
            case 0:
            {
                           EALLOW;
                           AdcRegs.ADCSOC0CTL.bit.CHSEL     = 6;//2//ADC_Ihb_Is2;
                           AdcRegs.ADCSOC1CTL.bit.CHSEL     = 6;//2;//ADC_Ihb_Is2;
                           AdcRegs.ADCSOC2CTL.bit.CHSEL     = 6;//2;// ADC_Vhb
                           AdcRegs.ADCSOC3CTL.bit.CHSEL     = 6;//2;// Tint
                           AdcRegs.ADCSOC4CTL.bit.CHSEL     = 6;//2;// ADC_Ihb2;
                           AdcRegs.ADCSOC5CTL.bit.CHSEL     = 6;//2;// ADC_Vres 2;
                           AdcRegs.ADCSOC6CTL.bit.CHSEL     = 6;//2;// ADC_Text 2;
                           AdcRegs.ADCSOC7CTL.bit.CHSEL     = 6;//2;// ADC_Vdc
                           AdcRegs.ADCSOC8CTL.bit.CHSEL     = 6;//2; //ADC_Idc //2;
                           AdcRegs.ADCSOC9CTL.bit.CHSEL     = 6;//2;// ADC_PFC_Temp //2;
                           AdcRegs.ADCSOC10CTL.bit.CHSEL    = 6;//2; //ADC_PFC_Vin
                           AdcRegs.ADCSOC11CTL.bit.CHSEL    = 6;//2;//ADC_PFC_Iin 2;
                           AdcRegs.ADCSOC12CTL.bit.CHSEL    = 6;//2;// ADC_Version 2;
                           AdcRegs.ADCSOC13CTL.bit.CHSEL    = 6;//2;
                           AdcRegs.ADCSOC14CTL.bit.CHSEL    = 6;//2;
                           AdcRegs.ADCSOC15CTL.bit.CHSEL    = 6;//2;
                           EDIS;
            }
            break;
            case 1:
            {
                           EALLOW;
                           AdcRegs.ADCSOC0CTL.bit.CHSEL     = 5;//ADC_Ihb_Is2;
                           AdcRegs.ADCSOC1CTL.bit.CHSEL     = 1;//ADC_Ihb_Is2;
                           AdcRegs.ADCSOC2CTL.bit.CHSEL     = 2;// ADC_Vhb
                           AdcRegs.ADCSOC3CTL.bit.CHSEL     = 3;// Tint
                           AdcRegs.ADCSOC4CTL.bit.CHSEL     = 4;// ADC_Ihb2;
                           AdcRegs.ADCSOC5CTL.bit.CHSEL     = 0;// ADC_Vres 2;
                           AdcRegs.ADCSOC6CTL.bit.CHSEL     = 6;// ADC_Text 2;
                           AdcRegs.ADCSOC7CTL.bit.CHSEL     = 7;// ADC_Vdc
                           AdcRegs.ADCSOC8CTL.bit.CHSEL     = 8; //ADC_Idc //2;
                           AdcRegs.ADCSOC9CTL.bit.CHSEL     = 9;// ADC_PFC_Temp //2;
                           AdcRegs.ADCSOC10CTL.bit.CHSEL    = 10; //ADC_PFC_Vin
                           AdcRegs.ADCSOC11CTL.bit.CHSEL    = 11;//ADC_PFC_Iin 2;
                           AdcRegs.ADCSOC12CTL.bit.CHSEL    = 12;// ADC_Version 2;
                           AdcRegs.ADCSOC13CTL.bit.CHSEL    = 13;
                           AdcRegs.ADCSOC14CTL.bit.CHSEL    = 14;
                           AdcRegs.ADCSOC15CTL.bit.CHSEL    = 15;
                           EDIS;
            }
            break;
            default: break;

        }
    }
    id_state = reprom.state;
    if(reprom.state != 7)
    {
        reprom_processing();
    }
    else
    {
        read_id_wait = 5000;
    }
}
void reprom_processing(void)
{
    uint16_t i = 0;
    if(main_counter < 200)
      {
          ++main_counter;
      }
      if(main_counter == 100)
      {
          reprom.state = 1;
          reprom.ID = 0;
      }
      if(reprom.state != 7)
      {
             switch (reprom.state)
             {
                 case 0:
                 {
                     GpioCtrlRegs.GPADIR.bit.GPIO5 |= 1;
                     //GpioCtrlRegs.GPAODR.bit.GPIO5 |= 1;
                     GpioDataRegs.GPASET.bit.GPIO5 |= 1;

                     reprom.ID_read_flag = 2;
                     reprom.init_cnt = 0;
                     reprom.ack_cnt = 0;
                     reprom.init_cnt_1 = 0;
                     reprom.start_write_cnt = 0;
                     reprom.write_cnt = 0;
                     reprom.read_rom_counter = 0;
                 }
                 break;
                 case 1:
                 {
                     if(reprom_init_pulse_thread())
                     {
                         reprom.state = 2;
                     }
                     else
                     {
                         reprom.state = 1;
                     }
                 }
                 break;
                 case 2:
                 {
                     if( reprom.ack_cnt < reprom.ack_delay )
                     {
                         ++reprom.ack_cnt;
                         reprom.init_ack = GpioDataRegs.GPADAT.bit.GPIO5;
                     }
                     else
                     {
                         reprom.state = 3;
                        // reprom.ID = 1;
                        // reprom.state = 0;
                     }
                 }
                 break;
                 case 3:
                 {
                     if(reprom.init_cnt_1 <= reprom.init_release)
                     {
                         ++(reprom.init_cnt_1);
                     }
                     else
                     {
                         reprom.state = 4;
                     }
                 }
                 break;
                 case 4:
                 {
                     if(!(reprom.init_ack))
                     {
                         EALLOW;
                         GpioCtrlRegs.GPADIR.bit.GPIO5 |= 1;
                         EDIS;
                         GpioDataRegs.GPASET.bit.GPIO5 = 1;
                         reprom.start_write_cnt = 0;
                         reprom.start_write_cnt_1 = 0;
                         reprom.start_write_cnt = 0;
                         reprom.read_rom_counter = 0;
                         d = (READ_ROM >> (reprom.read_rom_counter)) & 0x01;
                         reprom.state = 5;
                         //reprom.ID = 1;
                         //reprom.state = 0;
                     }
                 }
                 break;
                 case 5:
                 {
                     if (reprom.read_rom_counter < 8 )
                     {
                         GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
                         GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
                         if(reprom.start_write_cnt < reprom.start_write_delay)
                         {
                             GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
                             ++reprom.start_write_cnt;
                         }
                         else
                         {
                             if(reprom.write_cnt < reprom.write_delay)
                             {
                                 ++reprom.write_cnt;
                                 if(d)
                                 {
                                     GpioDataRegs.GPASET.bit.GPIO5 = 1;
                                 }
                                 else
                                 {
                                     GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
                                 }
                             }
                             else
                             {
                                  if(reprom.start_write_cnt_1 < reprom.start_write_delay)
                                  {
                                      ++reprom.start_write_cnt_1;
                                      GpioDataRegs.GPASET.bit.GPIO5 = 1;
                                  }
                                  else
                                  {
                                      reprom.write_cnt = 0;
                                      ++reprom.read_rom_counter;
                                      d = (READ_ROM >> (reprom.read_rom_counter)) & 0x01;
                                      reprom.start_write_cnt = 0;
                                      reprom.start_write_cnt_1 = 0;
                                  }
                             }
                         }
                      }
                     else
                     {
                         reprom.read_rom_counter = 0;
                         reprom.ID = 0;
                         reprom.state = 6;
                     }
                 }
                 break;
                 case 6:
                 {
                     if(reprom.read_rom_counter < 64)
                     {
                        EALLOW;
                        GpioCtrlRegs.GPADIR.bit.GPIO5 |= 1;
                        EDIS;
                        if( reprom.start_read_cnt < reprom.start_read_delay)
                        {
                            ++reprom.start_read_cnt;
                            GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
                        }
                        else
                        {
                            reprom.start_read_cnt = 0;
                            GpioDataRegs.GPASET.bit.GPIO5 = 1;
                            EALLOW;
                            GpioCtrlRegs.GPADIR.bit.GPIO5 &= 0;
                            EDIS;
                            GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
                            GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
                            for(reprom.read_cnt = 0; reprom.read_cnt < reprom.read_delay; ++reprom.read_cnt)
                            {
                                if(reprom.read_cnt < reprom.read_window)
                                {
                                       GpioDataRegs.GPASET.bit.GPIO2 = 1;
                                       GpioDataRegs.GPASET.bit.GPIO3 = 1;
                                       d = GpioDataRegs.GPADAT.bit.GPIO5;
                                       if(d)
                                       {
                                         d = 1;
                                       }
                                       else
                                       {
                                         d = 0;
                                       }
                                       if(reprom.read_rom_counter < 8)
                                       {
                                          reprom.family |= d << (reprom.read_rom_counter);
                                       }
                                       else if((reprom.read_rom_counter) < 56)
                                       {
                                          reprom.ID |= (d << ((reprom.read_rom_counter) - 8));
                                       }
                                       else
                                       {
                                          reprom.CRC |= (d << ((reprom.read_rom_counter) - 56));
                                       }
                                 }
                                 else
                                 {
                                       GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
                                       GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
                                 }
                               }
                              reprom.read_cnt = 0;
                              for(reprom.start_release_cnt = 0;reprom.start_release_cnt < reprom.start_release_delay; ++reprom.start_release_cnt)
                              {
                                  ;
                              }
                              reprom.start_release_cnt = 0;
                              ++reprom.read_rom_counter;
                           }
                     }
                     else
                     {
                             reprom_state = 1;
                             reprom.state = 7;
                             reprom.id[0] = (char)(reprom.ID & 0xFF);//3;
                             reprom.id[1] = (char)((reprom.ID >> 8) & 0xFF);//3;
                             reprom.id[2] = (char)((reprom.ID >> 16) & 0xFF);
                             reprom.id[3] = (char)((reprom.ID >> 24) & 0xFF);
                             reprom.id[4] = (char)((reprom.ID >> 32) & 0xFF);
                             reprom.id[5] = (char)((reprom.ID >> 40) & 0xFF);

                             for(i = 0; i < 6; ++i)
                             {
                                 if(reprom.id[i] == 0)
                                 {
                                     reprom.id[i] = 48;
                                 }
                                 else if(reprom.id[i] >= 127)
                                 {
                                     reprom.id[i] = (reprom.id[i]/2) - 1;
                                     //reprom.id[i] = (reprom.id[i] - 127);
                                 }
                                 crc -= reprom.id[i];
                             }
                             reprom.id[i] = crc & 0x0FF ;

                     }
                     break;
                 }
                 default: break;
             }
      }

}

uint16_t reprom_init_pulse_thread(void)
{
    uint16_t out = 0;
    uint16_t init_delay_c = reprom.init_delay/reprom.time_scale;
    if (reprom.init_cnt <= init_delay_c)
    {
           EALLOW;
           GpioCtrlRegs.GPADIR.bit.GPIO5 |= 1;
           EDIS;
           ++(reprom.init_cnt);
           GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
           out = 0;
    }
    else
    {
      //  reprom.init_cnt = 0;
        GpioDataRegs.GPASET.bit.GPIO5 = 1;
        EALLOW;
        GpioCtrlRegs.GPADIR.bit.GPIO5 &= 0;
        EDIS;
        out = 1;
    }
    return out;
}
__interrupt void adc_isr(void)
{
    //
    // Discard ADCRESULT0 as part of the workaround to the
    // 1st sample errata for rev0
    //
    GpioDataRegs.GPATOGGLE.bit.GPIO2 |= 1;
    GpioDataRegs.GPATOGGLE.bit.GPIO3 |= 1;
    GpioDataRegs.GPATOGGLE.bit.GPIO4 |= 1;

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}

void initWiFi_station(void)
{
    uint16_t init = 1;
    uint16_t init_hack = 1;
    const char *s_at;
    const char *s_at_1;
    long wifi_timer = 0;
   // while(init == 1)
   // {
   //     init = waitATReceive(s_at,5,10*WIFI_TIMER_OVERFLOW);
   // }
    wifi_init();
    esp8266Reset();
    while(init)
    {
            init ^= atCommand_tx.console; // FOR THE TEST
            if(wifi_timer < 10)
            {
                ++wifi_timer;
            }
            else
            {
                wifi_timer = 0;
                esp8266.c_status = ESP8266_IDLE;
            }
            switch(esp8266.c_status)
            {
                case ESP8266_IDLE:
                {
                    wifi_init();
                    esp8266Reset();
                    s_at = "ready";
                    esp8266.c_status = ESP8266_UART_CONFIG;

                    init = waitATReceive(s_at,5,20*WIFI_INIT_TIMER_OVERFLOW);

                    if(init == 2)
                    {
                        esp8266.c_status = ESP8266_UART_CONFIG;
                    }
                    else
                    {
                        esp8266.c_status = ESP8266_IDLE;
                        wifi_timer = 0;
                    }

                }
                break;
                case ESP8266_UART_CONFIG:
                {
                           s_at = "OK";
                           ///SETUP AP_STA MODE///////
                           atCommand_tx.tx_flag = 1; //FOR TEST
                           esp8266.c_status = ESP8266_UART_CONFIG_TX;
                           atCommandManager((int)AT_UART_CONFIG(NUM));  //FOR TEST
                           init = waitATReceive(s_at,2,20*WIFI_INIT_TIMER_OVERFLOW);
                           if(init == 2)
                           {
                                  init_SCI_fast();
                                  esp8266.c_status = ESP8266_READY;
                           }
                           else
                           {
                                  esp8266.c_status = ESP8266_IDLE;
                                  wifi_timer = 0;
                           }
                }
                break;
                case ESP8266_READY:
                {
                    s_at = "=3\r\n\r\nOK";
                    ///SETUP AP_STA MODE///////
                    atCommand_tx.tx_flag = 1;
                    esp8266.c_status = ESP8266_CWMODE_AP_TX;
                    atCommandManager((int)AT_CWMODE_AP_STA(NUM));
                    init = waitATReceive(s_at,8,5*WIFI_INIT_TIMER_OVERFLOW);
                    if(init == 2)
                    {
                       esp8266.c_status = ESP8266_CWMODE_AP_OK;
                    }
                    else
                    {
                       esp8266.c_status = ESP8266_IDLE;
                       wifi_timer = 0;
                    }
                }
                break;
                case ESP8266_CWMODE_AP_OK:
                            {
                                //s_at = "\"3000zvxw\",5,3\r\n\r\nOK";
                                s_at = "5,3\r\n\r\nOK";
                                //s_at = "AT+CWSAP=\"ReasonanceUpdateFirmware_11.1\",\"100@101@\",5,3\r\n\r\nOK";// test
                                ///SETUP AP MODE///////
                                atCommand_tx.tx_flag = 1;
                                esp8266.c_status = ESP8266_CWSAP_AP_TX;
                                atCommandManager((int)AT_CWSAP_CUR(NUM));
                                init = waitATReceive(s_at,9, 20*WIFI_INIT_TIMER_OVERFLOW);
                                if(init == 2)
                                {
                                   esp8266.c_status = ESP8266_CWSAP_AP_OK;
                                }
                                else
                                {
                                   esp8266.c_status = ESP8266_IDLE;
                                   wifi_timer = 0;

                                }
                            }
                            break;
                case ESP8266_CWSAP_AP_OK:
                {
                      s_at = "T+CIPMUX=1\r\n\r\nOK";
                      atCommand_tx.tx_flag = 1;
                      esp8266.c_status = ESP8266_CIPMUX_TX;
                      atCommandManager((int)AT_CIPMUX_MULT(NUM));
                      init = waitATReceive(s_at,16,5*WIFI_INIT_TIMER_OVERFLOW);
                      if(init == 2)
                      {
                         esp8266.c_status = ESP8266_CIPMUX_OK;
                      }
                      else
                      {
                         esp8266.c_status = ESP8266_IDLE;
                         wifi_timer = 0;
                      }
                }
                break;
                case ESP8266_CIPMUX_OK:
                {

                              s_at = "T+CIPSERVER=1,7\r\n\r\nOK";
                              ///SETUP MUX MODE///////
                              atCommand_tx.tx_flag = 1;
                              esp8266.c_status = ESP8266_CIPSERVER_TX;
                              atCommandManager((int)AT_CIPSERVER_AP(NUM));
                              init = waitATReceive(s_at,21,5*WIFI_INIT_TIMER_OVERFLOW);
                              if(init == 2)
                              {
                                 esp8266.c_status = ESP8266_CIPSERVER_OK;
                               }
                               else
                               {
                                  esp8266.c_status = ESP8266_IDLE;
                                  wifi_timer = 0;
                               }

                  //  esp8266.c_status = ESP8266_CIPSERVER_OK;
                    /*
                       s_at = "CONNECT\r\n\r\nOK";
                       atCommand_tx.tx_flag = 1;
                       esp8266.c_status = ESP8266_CIPSERVER_TX;
                       atCommandManager((int)AT_CIPSTART_UDP(NUM));
                       init = waitATReceive(s_at,13,5*WIFI_INIT_TIMER_OVERFLOW);
                       if(init == 2)
                       {
                          esp8266.c_status = ESP8266_CIPSERVER_OK;
                       }
                       else
                       {
                          esp8266.c_status = ESP8266_IDLE;
                          wifi_timer = 0;
                       }
                       */

                }
                break;
                case ESP8266_CIPSERVER_OK:
                {
                                s_at = "CONNECTED";
                                //s_at_1 = "CONECTED";
                                s_at_1 = "OK";
                                ///TRY TO CONNECT TO TERMIAL
                                atCommand_tx.tx_flag = 1;
                                esp8266.c_status = ESP8266_CWJAP_TERMINAL_TX;
                                atCommandManager((int)AT_CWJAP_TERMINAL(NUM));

                                init_hack = waitATReceive(s_at_1, 2, 50*WIFI_INIT_TIMER_OVERFLOW);
                                init = waitATReceive(s_at, 9, 50*WIFI_INIT_TIMER_OVERFLOW);
                                if((init == 2) || (init_hack == 2))
                                {
                                   esp8266.c_status = ESP8266_CWJAP_TERMINAL_OK;
                                   EPwm4Regs.TBPRD = (base * PERIOD_MULTIPLIER_TIM);
                                   init = 0;
                                //   atCommand_tx.console = 1;
                                }
                                else
                                {
                                    esp8266.c_status = ESP8266_IDLE;
                                   // wifi_timer = 0;
                                }
                 }
                break;
                default : break;
            }
        }
}
void wifi_init(void)
{
    init_SCI();
    esp8266.config_status = ESP8266_IS_NOT_CONFIGURED;
    esp8266.s_mode = SERIAL_MODE_AT;
    esp8266.c_status = ESP8266_IDLE;
    esp8266.reset_counter = 0;
    memset(&(esp8266.AT_rx_buff), 0, SCI_RX_SIZE);
    memset(&(esp8266.AT_tx_buff), 0, SCI_RX_SIZE);
    esp8266.first_reqest = 0;
    esp8266.reset_status = 0;
    atCommand_tx.len = 4;
    memcpy(&(atCommand_tx.str[0]), AT(0), 4);
    atCommand_tx.console = 0;
}
void esp8266Reset(void)
{
    uint16_t reset_flag = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
    SciaRegs.SCICTL1.bit.SWRESET = 0;
    asm(" NOP");
    asm(" NOP");
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
    SciaRegs.SCICTL1.bit.SWRESET = 1;
    while(reset_flag) // HARDWARE RESET ESP8266
    {
            if (reset_flag)
            {
                if((esp8266.reset_counter) < 10000)//if((esp8266.reset_counter) < 200)
                   {
                        PieCtrlRegs.PIEIER9.bit.INTx1 = 0;
                       GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
                       esp8266.c_status = ESP8266_BUSY;
                       ++(esp8266.reset_counter);
                   }
                   else
                   {
                       PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
                       reset_flag = 0;
                       GpioDataRegs.GPASET.bit.GPIO9 = 1;
                       esp8266.c_status = ESP8266_IDLE; // CHANGE TO "ESP8266_BUSY" FOR THE RECONFIG
                       esp8266.reset_counter = 0;
                       break;
                   }
             }
     }
}


void init_SCI(void) // TO LIB
{
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
  SysCtrlRegs.LOSPCP.all = 0;

  SciaRegs.SCICCR.bit.STOPBITS = 0;
  SciaRegs.SCICCR.bit.PARITY = 0;
  SciaRegs.SCICCR.bit.PARITYENA = 0;
  SciaRegs.SCICCR.bit.LOOPBKENA = 0;
  SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0;
  SciaRegs.SCICCR.bit.SCICHAR = 7;
  SciaRegs.SCICTL1.bit.RXERRINTENA = 0;
  SciaRegs.SCICTL1.bit.SWRESET = 1;
  SciaRegs.SCICTL1.bit.TXENA = 1;
  SciaRegs.SCICTL1.bit.RXENA = 1;
  SciaRegs.SCIHBAUD = 0U;//0U;//10U;
  SciaRegs.SCILBAUD = 63U;///63U;//43U;

  SciaRegs.SCIPRI.bit.FREE = 3;
  SciaRegs.SCIFFCT.bit.ABDCLR = 0;
  SciaRegs.SCIFFCT.bit.CDC = 0;
  SciaRegs.SCIFFTX.bit.SCIRST = 1;
  SciaRegs.SCIFFTX.bit.SCIFFENA = 1;
  SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
  SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
  SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
  SciaRegs.SCIFFRX.bit.RXFFIL = 1;
  SciaRegs.SCICTL1.bit.RXERRINTENA = 1;
  SciaRegs.SCIPRI.bit.SOFT = 3;
  EDIS;
}


void init_SCI_fast(void)
{
      EALLOW;
      SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
      SysCtrlRegs.LOSPCP.all = 0;

      asm(" NOP");
      SciaRegs.SCICCR.bit.STOPBITS = 0;
      SciaRegs.SCICCR.bit.PARITY = 0;
      SciaRegs.SCICCR.bit.PARITYENA = 0;
      SciaRegs.SCICCR.bit.LOOPBKENA = 0;
      SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0;
      SciaRegs.SCICCR.bit.SCICHAR = 7;
      SciaRegs.SCICTL1.bit.RXERRINTENA = 0;
      SciaRegs.SCICTL1.bit.SWRESET = 1;
      SciaRegs.SCICTL1.bit.TXENA = 1;
      SciaRegs.SCICTL1.bit.RXENA = 1;
      SciaRegs.SCIHBAUD = 0U;//10U;
      SciaRegs.SCILBAUD = 2;//216;//43U;

      SciaRegs.SCIPRI.bit.FREE = 3;
      SciaRegs.SCIFFCT.bit.ABDCLR = 0;
      SciaRegs.SCIFFCT.bit.CDC = 0;
      SciaRegs.SCIFFTX.bit.SCIRST = 1;
      SciaRegs.SCIFFTX.bit.SCIFFENA = 1;
      SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
      SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
      SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
      SciaRegs.SCIFFRX.bit.RXFFIL = 1;
      SciaRegs.SCICTL1.bit.RXERRINTENA = 1;
      SciaRegs.SCIPRI.bit.SOFT = 3;
      EDIS;
}

void sciReadBuff(void)
{
           char recbuff[SCI_RX_SIZE];
           int errFlg = NOERROR;
           {
              //int i;
             // for (i = 0; i < SCI_RX_SIZE; i++)
             // {
             //   recbuff[i] = 0;
             // }
              memset(recbuff, 0, SCI_RX_SIZE);
              errFlg = scia_rcv(recbuff, SCI_RX_SIZE, LONGLOOP, 1);
              //errFlg = scia_rcv(recbuff, SCI_RX_SIZE, SHORTLOOP, 1);
            //  if(errFlg != TIMEOUT)
          //    {
                 memcpy( &(esp8266.AT_rx_buff[0]), recbuff, (SCI_RX_SIZE));
           //   }
           }
           return;
}

void debug_console(void)
{
#ifdef USER_DEBUG
const char *s_at_ad = "/r/n";
uint16_t ap = 1;
    uint16_t len;
    const char *s1;
    const char *s_1310 = "\r\n";
    uint16_t n = 0;
    /*
    while(ap)
    {
        ap = atCommand_tx.console;
        sciReadBuff();
        if(atCommand_tx.tx_flag)
        {
            s1 = (char*) (&(atCommand_tx.str[0]));
            if(stringSeeker(s1,s_1310,32,2, &n))
            {
                len  = n + 2;
                atCommand_tx.tx_flag = 0;
                atCommandSend(&(atCommand_tx.str[0]), len);
            }
        }
    }
    */
    if(atCommand_tx.console)
    {
            //waitATReceive(s_at_ad,2, 50*WIFI_TIMER_OVERFLOW);
          //  printf("last received message:\r\n");
           // printf( "%s", esp8266.AT_rx_buff);
          //  fgets(atCommand_tx.str, 32, stdin);

            s1 = (char*) (&(atCommand_tx.str[0]));
            if(mqtt_test_flag == 0)
            {
                if(stringSeeker(s1,s_1310,32,2, &n))
                {
                    if(atCommand_tx.tx_flag)
                    {
                        //atCommand_tx.str[n] = 13;
                        //atCommand_tx.str[n + 1] = 10;
                        len  = n + 2;
                        atCommand_tx.len = len;
                        atCommand_tx.tx_flag = 0;
                        atCommandSend(&(atCommand_tx.str[0]), len);
                    }
                }
            }
    }
#endif
}

uint16_t waitATReceive (const char * at, uint16_t len, uint16_t timeout)
{

    uint16_t ptr = 1;
    long wifi_timer = 0;
    uint16_t wait = 1;
    const char *s1;
    uint16_t n = 0;
    while(wait)
    {
        s1 = (char*) (esp8266.AT_rx_buff);
        if(!stringSeeker(s1,at,110,len, &n))
        {
            ++wifi_timer;
            if(wifi_timer >= timeout)
            {
               wait = 0;
               break;
            }
        }
        else
        {
           ptr = 2;
           break;
        }
    }
    return ptr;

}
void atCommandSend(char * str, uint16_t len)
{
    scia_xmit(str, len, 1);
}
uint16_t stringSeeker(const char * str_src, const char * str_tg, uint16_t len_1, uint16_t len_2, uint16_t * num)
{

    const char *cut_string;
    uint16_t i = 0;
    for(i = 0; i < len_1; i++ )
    {
        cut_string = &str_src[i];
        if(!strncmp(cut_string, str_tg, len_2))
        {
            *num = i;
            return 1;
        }
    }
    return 0;

}
__interrupt void sci_isr(void)
{
    uint16_t n, at_status;
    const char *s1;
    const char *s2 = "AT+";
    const char *s_0 = "D,0";
    const char *s_1 = "D,1";
    const char *s_2 = "D,2";
    const char *s_3 = "D,3";
    const char *s_dis = "US:";
    const char *s_send_ok = "ND OK";
    sciReadBuff();
    EALLOW;
    SciaRegs.SCICTL1.bit.SWRESET = 0;
    asm ("NOP");
    SciaRegs.SCICTL1.bit.SWRESET = 1;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack

    EDIS;

    s1 = (char*) (esp8266.AT_rx_buff);

    if(stringSeeker(s1, s_dis, 32, 3, &n))
    {
        atStatusParser(s1);
    }
    if(stringSeeker(s1, s2, 8, 3, &n))
    {
        at_status = AT_parser(s1);
    }

    if (stringSeeker(s1, s_0, 10, 3, &n))
    {
      memcpy(&data_rx_mqtt[0], &esp8266.AT_rx_buff[2], 64);
    }
    if (stringSeeker(s1, s_1, 10, 3, &n))
    {
        IPD_parser(s1);
    }
    else if (stringSeeker(s1, s_2, 10, 3, &n))
    {
        IPD_parser(s1);
    }
    else if (stringSeeker(s1, s_3, 10, 3, &n))
    {
        IPD_parser(s1);
    }

    if (stringSeeker(s1, s_send_ok, 16, 5, &n))
    {
        data_rx_service[1] = 'S';
    }

}
void logs_processing(GEN_STRUCT * obj)
{
    if((obj->logs_r.Log_Error_RST_flag))
    {
        if(obj->logs_r.logs_couner < 20)
        {
            GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
            ++(obj->logs_r.logs_couner);
        }
        else
        {
            obj->logs_r.Log_Error_RST_flag = 0;
            GpioDataRegs.GPASET.bit.GPIO22 = 1;
            obj->logs_r.logs_couner = 0;
        }
    }
    if((obj->logs_r.Log_PWM_Dis_R))
    {
            if(obj->logs_r.logs_couner < 20)
            {
                GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
                ++(obj->logs_r.logs_couner);
            }
            else
            {
                obj->logs_r.Log_PWM_Dis_R = 0;
                GpioDataRegs.GPASET.bit.GPIO30 = 1;
                obj->logs_r.logs_couner = 0;
            }
    }
    obj->logs.Log_GPIO28 = GpioDataRegs.GPADAT.bit.GPIO28;
    obj->logs.Log_GPIO29 = GpioDataRegs.GPADAT.bit.GPIO29;
    obj->logs.Log_OC_Idc = GpioDataRegs.GPADAT.bit.GPIO23;
    obj->logs.Log_OC_Ihb = GpioDataRegs.GPBDAT.bit.GPIO32;
    obj->logs.Log_OC_Ihb_ls = GpioDataRegs.GPADAT.bit.GPIO21;
    obj->logs.Log_OT_Tint = GpioDataRegs.GPADAT.bit.GPIO20;
    obj->logs.Log_OV_Vhb = GpioDataRegs.GPBDAT.bit.GPIO33;
    obj->logs.Log_OV_Vres = GpioDataRegs.GPADAT.bit.GPIO24;
}
void atCommandManager(uint16_t command)
{
    switch(command)
    {
        case ((int) AT_CIPSTART_TCP(NUM)):
        {
            atCommand_tx.len = AT_CIPSTART_TCP(LEN);
            memcpy(&(atCommand_tx.str[0]), AT_CIPSTART_TCP(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CWLAP_SET(NUM)):
        {
             atCommand_tx.len = AT_CWLAP_SET(LEN);
             memcpy(&(atCommand_tx.str[0]), AT_CWLAP_SET(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CWLAP_EXE(NUM)):
        {
            atCommand_tx.len = AT_CWLAP_EXE(LEN);
            memcpy(&(atCommand_tx.str[0]), AT_CWLAP_EXE(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CWLAPOPT(NUM)):
        {
             atCommand_tx.len = AT_CWLAPOPT(LEN);
             memcpy(&(atCommand_tx.str[0]), AT_CWLAPOPT(STR), atCommand_tx.len);
        }
        break;

        case ((int) AT_CIPSEND_TR(NUM)):
        {
             atCommand_tx.len = AT_CIPSEND_TR(LEN);
             memcpy(&(atCommand_tx.str[0]), AT_CIPSEND_TR(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CWMODE_AP(NUM)):
        {
             atCommand_tx.len = AT_CWMODE_AP(LEN);
             memcpy(&(atCommand_tx.str[0]), AT_CWMODE_AP(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CIPMUX_MULT(NUM)):
        {
              atCommand_tx.len = AT_CIPMUX_MULT(LEN);
              memcpy(&(atCommand_tx.str[0]), AT_CIPMUX_MULT(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CIPSERVER_AP(NUM)):
        {
              atCommand_tx.len = AT_CIPSERVER_AP(LEN);
              memcpy(&(atCommand_tx.str[0]), AT_CIPSERVER_AP(STR), atCommand_tx.len);
        }
        break;

        case ((int) AT_CWSAP_CUR(NUM)):
        {
               atCommand_tx.len = AT_CWSAP_CUR(LEN);
               memcpy(&(atCommand_tx.str[0]), AT_CWSAP_CUR(STR), atCommand_tx.len);
               atCommand_tx.str[34] = '~';//reprom.id[0];
               atCommand_tx.str[35] = '>';//reprom.id[1];
               atCommand_tx.str[36] = '0';//reprom.id[2];
               atCommand_tx.str[37] = '0';//reprom.id[3];
               atCommand_tx.str[38] = '0';//reprom.id[4];
               atCommand_tx.str[39] = '0';//reprom.id[5];
        }
        break;
        case ((int) AT_CIPSTATUS(NUM)):
        {
               atCommand_tx.len = AT_CIPSTATUS(LEN);
               memcpy(&(atCommand_tx.str[0]), AT_CIPSTATUS(STR), atCommand_tx.len);
        }
        break;
        /*
        case ((int) AT_CWSTATE(NUM)):
        {
               atCommand_tx.len = AT_CWSTATE(LEN);
               memcpy(&(atCommand_tx.str[0]), AT_CWSTATE(STR), atCommand_tx.len);
        }
        break;
        */
        case ((int)AT_CIPSEND_STA_ID0(NUM)):
        {
                atCommand_tx.len = AT_CIPSEND_STA_ID0(LEN);
                memcpy(&(atCommand_tx.str[0]), AT_CIPSEND_STA_ID0(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CWMODE_AP_STA(NUM)):
        {
                atCommand_tx.len = AT_CWMODE_AP_STA(LEN);
                memcpy(&(atCommand_tx.str[0]), AT_CWMODE_AP_STA(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CWJAP_TERMINAL(NUM)):
        {
                atCommand_tx.len = AT_CWJAP_TERMINAL(LEN);
                memcpy(&(atCommand_tx.str[0]), AT_CWJAP_TERMINAL(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CIPSTART_TCP_TERMINAL(NUM)):
        {
                atCommand_tx.len = AT_CIPSTART_TCP_TERMINAL(LEN);
                memcpy(&(atCommand_tx.str[0]), AT_CIPSTART_TCP_TERMINAL(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CIPSEND_STA_ID1(NUM)) :
        {
                atCommand_tx.len = AT_CIPSEND_STA_ID1(LEN);
                memcpy(&(atCommand_tx.str[0]), AT_CIPSEND_STA_ID1(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CIPSTART_MMQT(NUM)) :
        {
                atCommand_tx.len = AT_CIPSTART_MMQT(LEN);
                memcpy(&(atCommand_tx.str[0]), AT_CIPSTART_MMQT(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_ESP8266_MMQT_GET_T_MMQT(NUM)):
        {
            atCommand_tx.len = AT_ESP8266_MMQT_GET_T_MMQT(LEN);
            memcpy(&(atCommand_tx.str[0]), AT_ESP8266_MMQT_GET_T_MMQT(STR), atCommand_tx.len);
        }
        break;
        case ((int)AT_CIPSEND_STA_ID0_MQTT_CONNECT(NUM)):
        {
            atCommand_tx.len = AT_CIPSEND_STA_ID0_MQTT_CONNECT(LEN);
            memcpy(&(atCommand_tx.str[0]), AT_CIPSEND_STA_ID0_MQTT_CONNECT(STR), atCommand_tx.len);
        }
        break;
        case ((int)AT_CIPSEND_STA_ID0_MQTT_DATA(NUM)):
        {
            atCommand_tx.len = mqtt.send_len;//AT_CIPSEND_STA_ID0_MQTT_DATA(LEN);
            memcpy(&(atCommand_tx.str[0]), mqtt.send_str, atCommand_tx.len);
        }
        break;
        case ((int)AT_UART_CONFIG(NUM)):
        {
             atCommand_tx.len = AT_UART_CONFIG(LEN);
             memcpy(&(atCommand_tx.str[0]), AT_UART_CONFIG(STR), atCommand_tx.len);
        }
        break;
        case ((int)AT_CIPSEND_STA_ID0_MQTT_SUBSCRIPTION(NUM)):
        {
             atCommand_tx.len = AT_CIPSEND_STA_ID0_MQTT_SUBSCRIPTION(LEN);
             memcpy(&(atCommand_tx.str[0]), AT_CIPSEND_STA_ID0_MQTT_SUBSCRIPTION(STR), atCommand_tx.len);
        }
        break;
        case ((int)AT_CIPCLOSE(NUM)):
        {
            atCommand_tx.len = AT_CIPCLOSE(LEN);
            memcpy(&(atCommand_tx.str[0]), AT_CIPCLOSE(STR), atCommand_tx.len);
        }
        break;
        case ((int) AT_CIPSTART_UDP(NUM)):
        {
            atCommand_tx.len = AT_CIPSTART_UDP(LEN);
            memcpy(&(atCommand_tx.str[0]), AT_CIPSTART_UDP(STR), atCommand_tx.len);
        }
        break;
        default : break;
    }
    if(atCommand_tx.tx_flag)
    {
        uint16_t len = atCommand_tx.len;
        atCommand_tx.tx_flag = 0;
        atCommandSend(&(atCommand_tx.str[0]), len);
    }
}
uint16_t connection_manager(void)
{
    uint16_t err = 1;
    const char * str_end = "\r\n";
    const char *str_send = "AT+CIPSEND=0,";
    uint16_t len = 0;
    uint16_t num = 0;
    if(atCommand_tx.console != 1 )
    {
                    /*
                    if(isr_counter == 1)
                    {
                        if(mqtt_shift_counter < 16)
                        {
                            //remote_data[0] = mqtt_shift[mqtt_shift_counter] / 10;
                            ++mqtt_shift_counter;
                        }
                        else
                        {
                           // remote_data[0] = mqtt_shift[mqtt_shift_counter];// / 10;
                            remote_data[0] = (mean_int(mqtt_shift, 16) / 10);//(min_int(mqtt_shift, 16) / 10);
                            memcpy(mqtt_shift, vhb, sizeof(vhb));
                            mqtt_shift_counter = 0;

                        }
                    }
                    */

                    remote_data[0] = (uint16_t)(burst_duty*100.0);//(uint16_t)(volts[ADC_IDC]*100.0);//(sns[ADC_IHB_LS)/10);//(int)(volts[ADC_TINT]);//bridge_temp;//(uint16_t)sensors[9];//tv_voltage;
                    remote_data[1] = cpu_temp;//(uint16_t)(gen.AMP*100.0); //out_gen_current
                    remote_data[2] = (uint16_t)(volts[ADC_VDC]);//(sns[ADC_IDC] - 2000);//(uint16_t)(volts[ADC_VDC] / 10.0); //t_tv_board
                    remote_data[3] = (uint16_t)(volts[ADC_TINT]);//(uint16_t)(volts[ADC_IHB_LS]*10.0);
                    remote_data[4] = vres_detector;//(uint16_t)(volts[ADC_VRES]);
                    remote_data[5] = idc;
                    remote_data[6]  = (uint16_t)data_rx[6];///(uint16_t)volts[ADC_IHB_LS];
                    remote_data[7]  = (uint16_t)data_rx[7];
                    remote_data[8]  = (uint16_t)(volts[ADC_TEXT]);

                    if(esp8266.c_status == ESP8266_CWJAP_TERMINAL_OK)
                    {   if(MMQT_SUBSCRIBE_OK == (mqtt.state_machine))
                        {
                           switch(var_counter)
                           {
                                     case 0:
                                     {
                                         message.publish.topic_name.data = (uint16_t *)("a_tv_wpt__t");
                                     }
                                     break;
                                     case 1:
                                     {
                                         message.publish.topic_name.data = (uint16_t *)("b_tv_wpt__m");
                                     }
                                     break;
                                     case 2:
                                     {
                                         message.publish.topic_name.data = (uint16_t *)("c_tv_wpt__v");
                                         remote_data[5] = 0;
                                     }
                                     break;
                                     case 3:
                                     {
                                          message.publish.topic_name.data = (uint16_t *)("d_tv_wpt__k");
                                     }
                                     break;
                                     case 4:
                                     {
                                          message.publish.topic_name.data = (uint16_t *)("e_tv_wpt__q");
                                     }
                                     break;
                                     default: break;
                            }
                            //mqtt_message_constructor(&message.publish.content.data[0], &remote_data[5*var_counter], 5, &mqtt_var[0]);
                            mqtt_message_constructor(&message.publish.content.data[0], &remote_data[9*var_counter], 9, &mqtt_var[0]);
                            mqtt.p_len  = mqtt_serialiser_size(&serialiser, &message);
                            memset(mqtt_publish_common, 0, sizeof(mqtt_publish_common));
                            mqtt_serialiser_write(&serialiser,&message,&mqtt_publish_common[0], mqtt.p_len); //-2??
                            num = mqtt.p_len;
                            if((num / 100) > 0)
                            {
                                     digit_len = 3;
                                     digit_buf[0] = 48 + (num / 100);
                                     digit_buf[1] = 48 + ((num % 100) / 10);
                                     digit_buf[2] = 48 + (num % 10);
                            }
                            else if((num / 10) > 0)
                            {
                                     digit_len = 2;
                                     digit_buf[0] = 48 + (num / 10);
                                     digit_buf[1] = 48 + (num % 10);
                            }
                            else
                            {
                                     digit_len = 1;
                                     digit_buf[0] = 48 + num;
                            }
                            memcpy(&mqtt.send_str[len], str_send, 13);
                            len += 13;
                            switch (digit_len)
                            {
                                     case 1 :
                                     {
                                         mqtt.send_str[len++] = digit_buf[0];
                                     }
                                     break;
                                     case 2 :
                                     {
                                         mqtt.send_str[len++] = digit_buf[0];
                                         mqtt.send_str[len++] = digit_buf[1];
                                     }
                                     break;
                                     case 3 :
                                     {
                                         mqtt.send_str[len++] = digit_buf[0];
                                         mqtt.send_str[len++] = digit_buf[1];
                                         mqtt.send_str[len++] = digit_buf[2];
                                     }
                                     break;
                                     default: break;
                              }
                              memcpy(&mqtt.send_str[len], str_end, 2);
                              mqtt.send_len = len + 2;
                        }
                        if(cnt >= STATUS_Q_MQTT)
                        {
                            mqtt_manager();
                            cnt = 1;
                        }
                        else
                        {
                            device_manager();
                        }
                    }
    }
    mqtt_parser(&data_rx_mqtt[0]);
    return err;
}
void mqtt_message_constructor(uint16_t * msg, uint16_t * raw, uint16_t raw_len, MQTT_VAR * var )
{
       uint16_t num = 0;
       uint16_t len = 0;
       memcpy(&str_out[len], str_open, 1);
       len += 1;
       for(val_p = 0; val_p < raw_len; val_p++)
       {
            num = raw[val_p];
            if((num / 100) > 0)
            {
                var[val_p].len = 3;
                var[val_p].data[0] = 48 + (num / 100);
                var[val_p].data[1] = 48 + ((num % 100) / 10);
                var[val_p].data[2] = 48 + (num % 10);
            }
            else if((num / 10) > 0)
            {
                var[val_p].len = 2;
                var[val_p].data[0] = 48 + (num / 10);
                var[val_p].data[1] = 48 + (num % 10);
            }
            else
            {
                var[val_p].len = 1;
                var[val_p].data[0] = 48 + num;
            }
               switch(val_p)
               {
                   case 0:
                   {
                       memcpy(&str_out[len], str_var1, 7);
                       len += 7;
                   }
                   break;
                   case 1:
                   {
                       memcpy(&str_out[len], str_var2, 7);
                       len += 7;
                   }
                   break;
                   case 2:
                   {
                       memcpy(&str_out[len], str_var3, 7);
                       len += 7;
                   }
                   break;
                   case 3:
                   {
                       memcpy(&str_out[len], str_var4, 7);
                       len += 7;
                   }
                   break;
                   case 4:
                   {
                       memcpy(&str_out[len], str_var5, 7);
                       len += 7;
                   }
                   break;
                   case 5:
                   {
                       memcpy(&str_out[len], str_var6, 7);
                       len += 7;
                   }
                   break;
                   case 6:
                   {
                       memcpy(&str_out[len], str_var7, 7);
                       len += 7;
                   }
                   break;
                   case 7:
                   {
                       memcpy(&str_out[len], str_var8, 7);
                       len += 7;
                   }
                   break;
                   case 8:
                   {
                       memcpy(&str_out[len], str_var9, 7);
                       len += 7;
                   }
                   break;
                   case 9:
                   {
                       memcpy(&str_out[len], str_var10, 8);
                       len += 8;
                   }
                   break;
                   case 10:
                   {
                       memcpy(&str_out[len], str_var11, 8);
                       len += 8;
                   }
                   break;
                   case 11:
                   {
                       memcpy(&str_out[len], str_var12, 8);
                       len += 8;
                   }
                   break;
                   default:break;
               }

               if(var[val_p].len == 3)
               {
                   str_out[len++] = var[val_p].data[0];
                   str_out[len++] = var[val_p].data[1];
                   str_out[len++] = var[val_p].data[2];
               }
               else if(var[val_p].len == 2)
               {
                   str_out[len++] = var[val_p].data[0];
                   str_out[len++] = var[val_p].data[1];
               }
               else if(var[val_p].len == 1)
               {
                   str_out[len++] = var[val_p].data[0];
               }
               memchr(&str_out[len], var[val_p].data[0], var[val_p].len);
               //len += (var[val_p].len - 1);
               memcpy(&str_out[len++], str_comma, 1);
       }
       memcpy(&str_out[--len], str_close, 1);
       message.publish.content.data = (char *)str_out;
       message.publish.content.length = len + 1;
}
void number_to_string (uint16_t num, MQTT_VAR * var)
{
    if((num / 100) > 0)
    {
        var->len = 3;
        var->data[2] = num / 100;
        var->data[1] = num / 10;
        var->data[0] = num % 10;
    }
    else if((num / 10) > 0)
    {
        var->len = 2;
        var->data[1] = num / 10;
        var->data[0] = num % 10;
    }
    else
    {
        var->len = 1;
        var->data[0] = num;
    }
    memcpy(var->str, var->data, var->len);
}
void mqtt_manager(void)
{
    const char *s;
    const char *s1;
    uint16_t init = 3;
    uint16_t init1 = 3;
    switch(mqtt.state_machine)
    {
                case MQTT_CONNECT_TO_SERVER:
                {
                               //gen.leds_duty[0] = 0.0;
                               s = "0,CONNECT";
                               s1 = "ALREADY";
                               init = waitATReceive_nonbloking_service(s, 9, 2*WIFI_TIMER_OVERFLOW);
                              // init1 = waitATReceive_nonbloking_service(s1, 7, 2*WIFI_TIMER_OVERFLOW);
                               //if((init == 2) || (init1 == 2))
                               if((init == 2) || (data_rx_service[0] == 'A'))
                               {
                                   data_rx_service[0] = 0;
                                   mqtt.state_machine = MQTT_CONNECT_TO_SERVER_OK;
                               }
                               else if(init == 3)
                               {
                                   if(mqtt.lost_connection_timer >= LOST_CONNECTION_TIMEOUT)
                                   {
                                       memset(data_rx_mqtt,0,sizeof(data_rx_mqtt));
                                       data_rx_service[1] = 0;
                                       mqtt.lost_connection_timer = 0;
                                   }
                                   else
                                   {
                                       ++mqtt.lost_connection_timer;
                                   }
                                   atCommand_tx.tx_flag = 1;
                                   atCommandManager((int)AT_CIPSTART_MMQT(NUM));

                               }
                }
                break;
                case MQTT_CONNECT_TO_SERVER_OK:
                {
                                s = ">";
                                data_rx_service[1] = 1;
                                init = waitATReceive_nonbloking(s,1, 5*WIFI_TIMER_OVERFLOW);
                                if((init == 2) || (data_rx_service[0] == '>'))
                                {
                                    data_rx_service[0] = 0;
                                    mqtt.state_machine = MMQT_SEND_OK;
                                }
                                else if(init == 3)
                                {
                                   // atCommand_tx.console = 1;
                                    atCommand_tx.tx_flag = 1;
                                    atCommandManager((int)AT_CIPSEND_STA_ID0_MQTT_CONNECT(NUM));
                                }
                }
                break;
                case MMQT_SEND_OK:
                {

                                   // memcpy(&(atCommand_tx.str[0]), mqtt_connect, sizeof(mqtt_connect));
                                    memcpy(&(mqtt_connect[48]), &(reprom.id[0]),6);
                                    memcpy(&(atCommand_tx.str[0]), &(mqtt_connect[0]), mqtt.connect_p_len);
                                    atCommand_tx.len = mqtt.connect_p_len;//sizeof(mqtt_connect);
                                    mqtt.state_machine = MMQT_CONNECT_TX;
                                    atCommand_tx.tx_flag = 1;
                                    atCommandSend(&(atCommand_tx.str[0]), atCommand_tx.len);
                }
                break;
                case MMQT_CONNECT_TX:
                {
                                    s = "IPD";
                                    init = waitATReceive_nonbloking(s,3, 5*WIFI_TIMER_OVERFLOW);
                                    if(init == 2)
                                    {
                                       data_rx_service[1] = 0;
                                       mqtt.state_machine = MMQT_CONNECT_OK; // AS TEST
                                       //atCommand_tx.console = 1;
                                    }
                                    else if(init == 3)
                                    {
                                        mqtt.state_machine = MMQT_CONNECT_OK; // AS TEST
                                    }
                }
                break;
                /////////////////////////////////////////////////////////
                case MMQT_CONNECT_OK:
                {

                     s = ">";
                     init = waitATReceive_nonbloking_service(s,1, WIFI_TIMER_OVERFLOW);
                     if(init == 2)
                     {
                         data_rx_service[0] = 0;
                         mqtt.state_machine = MMQT_SUBSCRIBE_SEND_OK;
                     }
                     else if(init == 3)
                     {
                       // atCommand_tx.console = 1;
                          atCommand_tx.tx_flag = 1;
                          atCommandManager((int)AT_CIPSEND_STA_ID0_MQTT_SUBSCRIPTION(NUM));
                     }

                   // mqtt.state_machine = MMQT_SUBSCRIBE_SEND_OK;
                }
                break;
                case MMQT_SUBSCRIBE_SEND_OK:
                {

                    //memcpy(&(atCommand_tx.str[0]), mqtt_subscribe_send, sizeof(mqtt_subscribe_send));
                    memcpy(&(mqtt_subscribe_send[22]), (reprom.id),6);
                    memcpy(&(atCommand_tx.str[0]), mqtt_subscribe_send, mqtt.subscribe_p_len);
                    atCommand_tx.tx_flag = 1;
                    atCommand_tx.len = mqtt.subscribe_p_len;//sizeof(mqtt_subscribe_send);
                    mqtt.state_machine = MMQT_SUBSCRIBE_TX;
                    atCommandSend(&(atCommand_tx.str[0]), atCommand_tx.len);

                   // mqtt.state_machine = MMQT_SUBSCRIBE_TX;
                }
                break;
                case MMQT_SUBSCRIBE_TX:
                {

                    s = "IPD";
                    init = waitATReceive_nonbloking(s,3, 10*WIFI_TIMER_OVERFLOW);
                    if(init == 2)
                    {
                       data_rx_service[1] = 0;
                       mqtt.state_machine = MMQT_SUBSCRIBE_OK;
                    }

                   // mqtt.state_machine = MMQT_SUBSCRIBE_OK;
                }
                break;
                ////////////////////////////////////////////////////////////////////////
                case MMQT_SUBSCRIBE_OK:
                {
                    //gen.leds_duty[0] = 0.2;

                        s = ">";
                        init = waitATReceive_nonbloking_service(s,1, WIFI_TIMER_OVERFLOW);
                        if(init == 2)
                        {
                             data_rx_service[0] = 0;
                             mqtt.state_machine = MMQT_PREPARE_DATA_SEND_OK;
                        }
                        else if(init == 3)
                        {
                                publish_counter = 0;
                                atCommand_tx.tx_flag = 1;
                                atCommandManager((int)AT_CIPSEND_STA_ID0_MQTT_DATA(NUM));
                        }

                  //  atCommand_tx.tx_flag = 1;
                   // atCommandManager((int)AT_CIPSEND_STA_ID0_MQTT_DATA(NUM));
                   // mqtt.state_machine = MMQT_PREPARE_DATA_SEND_TX;

                }
                break;
                case MMQT_PREPARE_DATA_SEND_TX:
                {
                    s = ">";
                    init = waitATReceive_nonbloking_service(s,1, WIFI_TIMER_OVERFLOW);
                    if(init == 2)
                    {
                            data_rx_service[0] = 0;
                            mqtt.state_machine = MMQT_PREPARE_DATA_SEND_OK;
                    }
                    else if(init == 3)
                    {
                            mqtt.state_machine = MMQT_PREPARE_DATA_SEND_OK;
                    }
                }
                break;
                case MMQT_PREPARE_DATA_SEND_OK:
                {

                    memcpy(&mqtt_publish_common[9],reprom.id,6);
                    memcpy(&(atCommand_tx.str[0]), mqtt_publish_common, mqtt.p_len);
                    atCommand_tx.tx_flag = 1;
                    atCommand_tx.len = mqtt.p_len;
                    mqtt.state_machine = MMQT_DATA_SEND_TX;
                    atCommandSend(&(atCommand_tx.str[0]), atCommand_tx.len);
                }
                break;
                case MMQT_DATA_SEND_TX:
                {
                    s = "S";
                    init = waitATReceive_nonbloking_service(s,1, WIFI_TIMER_OVERFLOW);
                    if(init == 2)
                    {
                        data_rx_service[1] = 0;
                        mqtt.state_machine = MMQT_DATA_SEND_OK;
                    }
                    else if(init == 3)
                    {
                       data_rx_service[1] = 0;
                        mqtt.state_machine = MMQT_DATA_SEND_OK;
                    }
                }
                break;
                case MMQT_DATA_SEND_OK:
                {
                    if(var_counter < 0)
                    {
                       ++var_counter;
                    }
                    else
                    {
                       var_counter = 0;
                    }
                    //mqtt.state_machine = MMQT_SUBSCRIBE_OK;
                    atCommand_tx.tx_flag = 1;
                    atCommandManager((int) AT_CIPSTATUS(NUM)); //FOR THE TEST
                    mqtt.state_machine = MMQT_READY_TO_STATUS;
                }
                break;
                case MMQT_READY_TO_STATUS:
                {
                       if(esp8266.station[1].status == TCP_CLIENT_IS_DISCONNECTED)
                       {
                          atCommand_tx.tx_flag = 1;
                          atCommandManager((int) AT_CIPCLOSE(NUM)); //FOR THE TEST
                          mqtt.state_machine = MMQT_READY_TO_CLOSE_DEVICE;
                       }
                       else
                       {
                           mqtt.state_machine = MMQT_SUBSCRIBE_OK;
                       }
                }
                break;
                case MMQT_READY_TO_CLOSE_DEVICE:
                {
                    mqtt.state_machine = MMQT_SUBSCRIBE_OK;
                    esp8266.station[1].status = TCP_CLIENT_IS_NOT_CONNECTED;
                }
                break;
                default:
                {
                    mqtt.state_machine = MQTT_CONNECT_TO_SERVER;
                    break;
                }
    }
   // s = "FAIL";
   // init = waitATReceive_nonbloking(s,4, 2*WIFI_TIMER_OVERFLOW);
   // if(init == 2)
   // {
   //     mqtt.state_machine = MMQT_SUBSCRIBE_OK;
   // }
    s = "0,CLOSED";
    init = waitATReceive_nonbloking(s,8, 20*WIFI_TIMER_OVERFLOW);
    if(init == 2)
    {
        mqtt.state_machine = MQTT_CONNECT_TO_SERVER;
    }
    s = "link is not";
    init = waitATReceive_nonbloking(s,11, 20*WIFI_TIMER_OVERFLOW);
    if(init == 2)
    {
        mqtt.state_machine = MQTT_CONNECT_TO_SERVER;
    }
}
void mqtt_parser(char * data)
{
    uint16_t kptr = 0;
   // const char * s1_p;
    const char *str_on = "s_on";
    const char *str_off = "s_off";
   // const char *str_h_on = "tv_museum_ch_on";
  //  const char *str_h_off = "tv_museum_ch_off";
   // const char *str_rst = "rst=";
   // const char *str_f = "{f=:";
    const char *str_d = "{d=:0.";

   // const char *str_eeprom = "eeprom";
   // const char *str_k1 = "k1:=";
   // const char *str_k2 = "k2:=";
   // const char *str_ki = "ki:=";
  //  const char *str_kp = "kp:=";
    const char *str_uh = "uh:=";
    const char *str_ul = "ul:=";
  //  const char *str_af = "af:=";
  //  const char *str_dD = "dD:=";
  //  const char *str_k3 = "k3:=";

    s1_p = (char*) (data_rx_mqtt);


    if(stringSeeker(s1_p, str_on, 50, 4, &kptr))
    {
        mqtt.swicth = 1;
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }

    else if(stringSeeker(s1_p, str_off, 50, 5, &kptr))
    {
        mqtt.swicth = 0;
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
        resetLoops(&gen);
    }
    /*
    else if(stringSeeker(s1_p, str_rst, 32, 4, &kptr))
    {
        mqtt.swicth = 0;
        mqtt.reset = 1;
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    else if(stringSeeker(s1_p, str_f, 32, 4, &kptr))
    {
        mqtt.freq = (data_rx_mqtt[kptr + 6] - 48)*1000.0 + (data_rx_mqtt[kptr + 7] - 48)*100.0 + (data_rx_mqtt[kptr + 8] - 48)*10.0 + (data_rx_mqtt[kptr + 9] - 48);
        mqtt.freq_h = (data_rx_mqtt[kptr + 5] - 48);
        //gen.F = 1.0*(mqtt.freq_h) + 10000.0*mqtt.freq + 100000.0;
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
*/
    else if(stringSeeker(s1_p, str_d, 52, 6, &kptr))
    {
        if(data_rx_mqtt[kptr + 7] == 125)
        {
            mqtt.duty = (data_rx_mqtt[kptr + 6] - 48)*10 ;
        }
        else
        {
            mqtt.duty = (data_rx_mqtt[kptr + 6] - 48)*10 + (data_rx_mqtt[kptr + 7] - 48) ;
        }
       // burst_duty = 0.01*(mqtt.duty);
        //gen.AMP = 0.01*(mqtt.duty);
        //apm = gen.AMP;

        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    /*
    else if(stringSeeker(s1, str_h_on, 32, 15, &kptr))
    {
        mqtt.hand_control = 1;
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    else if(stringSeeker(s1, str_h_off, 32, 16, &kptr))
    {
        mqtt.hand_control = 0;
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    else if(stringSeeker(s1, str_eeprom, 32, 6, &kptr))
    {
        //global_data_2.ipc_out[26] = 1;
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }

    else if(stringSeeker(s1, str_k1, 54, 4, &kptr))
    {
        mqtt.k1 = (data_rx_mqtt[kptr + 4] - 48)*1.0 + (data_rx_mqtt[kptr + 6] - 48)*0.1 + (data_rx_mqtt[kptr + 7] - 48)*0.01;
       // memcpy(&global_data_2.ipc_out[6], (uint16_t *)(&mqtt.k1), 2);
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    else if(stringSeeker(s1, str_k2, 54, 4, &kptr))
    {
        mqtt.k2 = (data_rx_mqtt[kptr + 4] - 48)*1.0 + (data_rx_mqtt[kptr + 6] - 48)*0.1 + (data_rx_mqtt[kptr + 7] - 48)*0.01;
        if(data_rx_mqtt[kptr + 3] == 45)
        {
            mqtt.k2 = -1.0*mqtt.k2;
        }
      //  memcpy(&global_data_2.ipc_out[8], (uint16_t *)(&mqtt.k2), 2);
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }

    else if(stringSeeker(s1, str_ki, 32, 4, &kptr))
    {
        mqtt.ki_u = (data_rx_mqtt[kptr + 7] - 48)*0.01 + (data_rx_mqtt[kptr + 8] - 48)*0.001 + (data_rx_mqtt[kptr + 9] - 48)*0.0001;
       // memcpy(&global_data_2.ipc_out[10], (uint16_t *)(&mqtt.ki_u), 2);
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    else if(stringSeeker(s1, str_kp, 32, 4, &kptr))
    {
         mqtt.kp_u = (data_rx_mqtt[kptr + 4] - 48)*1.0 + (data_rx_mqtt[kptr + 6] - 48)*0.1 + (data_rx_mqtt[kptr + 7] - 48)*0.01;
       //  memcpy(&global_data_2.ipc_out[12], (uint16_t *)(&mqtt.kp_u), 2);
         memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    */
    else if(stringSeeker(s1_p, str_uh, 55, 4, &kptr))
    {
         //mqtt.uh = (data_rx_mqtt[kptr + 6] - 48)*0.1 + (data_rx_mqtt[kptr + 7] - 48)*0.01;
        mqtt.uh = (data_rx_mqtt[kptr + 6] - 48)*10.0 + (data_rx_mqtt[kptr + 7] - 48);//mqtt.uh = (data_rx_mqtt[kptr + 6] - 48)*0.1 + (data_rx_mqtt[kptr + 7] - 48)*0.01;
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    else if(stringSeeker(s1_p, str_ul, 55, 4, &kptr))
    {
        mqtt.ul = (data_rx_mqtt[kptr + 6] - 48)*10.0 + (data_rx_mqtt[kptr + 7] - 48);//mqtt.ul = (data_rx_mqtt[kptr + 6] - 48)*0.1 + (data_rx_mqtt[kptr + 7] - 48)*0.01;
        memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    /*
    else if(stringSeeker(s1, str_k3, 32, 4, &kptr))
    {
          mqtt.k3 = (data_rx_mqtt[kptr + 4] - 48)*1.0 + (data_rx_mqtt[kptr + 6] - 48)*0.1 + (data_rx_mqtt[kptr + 7] - 48)*0.01;
        //  memcpy(&global_data_2.ipc_out[18], (uint16_t *)(&mqtt.k3), 2);
          memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    else if(stringSeeker(s1, str_dD, 32, 4, &kptr))
    {
           mqtt.dD = (data_rx_mqtt[kptr + 4] - 48)*100 + (data_rx_mqtt[kptr + 5] - 48)*10 + (data_rx_mqtt[kptr + 6] - 48);
        //   global_data_2.ipc_out[20] = (uint16_t)mqtt.dD;
           memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    else if(stringSeeker(s1, str_af, 32, 4, &kptr))
    {
           mqtt.af = (data_rx_mqtt[kptr + 6] - 48)*0.1 + (data_rx_mqtt[kptr + 7] - 48)*0.01;
         //  memcpy(&global_data_2.ipc_out[23], (uint16_t *)(&mqtt.af), 2);
           memset (data_rx_mqtt, 0, sizeof(data_rx_mqtt));
    }
    */
    /*
    global_data_2.ipc_out[0] = mqtt.swicth;
    global_data_2.ipc_out[1] = mqtt.reset;
    global_data_2.ipc_out[2] = (uint16_t)mqtt.duty;
    global_data_2.ipc_out[3] = mqtt.hand_control;
    global_data_2.ipc_out[4] = (uint16_t)mqtt.freq;
    global_data_2.ipc_out[5] = (uint16_t)mqtt.freq_h;
    */

}
uint16_t waitATReceive_nonbloking(const char * at, uint16_t len, uint16_t timeout)
{
    uint16_t ptr = 1;
    static long wifi_timer = 0;
   // static uint16_t w = 0;
    const char *s1;
    uint16_t n = 0;
    s1 = (char*) (esp8266.AT_rx_buff);
    if(!stringSeeker(s1,at,SCI_AT_STRING_SIZE,len, &n))
    {
            ++wifi_timer;
            if(wifi_timer >= timeout)
            {
                wifi_timer = 0;
                ptr = 3;
            }
    }
    else
    {
           ptr = 2;
    }
    return ptr;
}
uint16_t waitATReceive_nonbloking_service(const char * at, uint16_t len, uint16_t timeout)
{
    uint16_t ptr = 1;
    static long wifi_timer = 0;
   // static uint16_t w = 0;
    const char *s1;
    uint16_t n = 0;
    s1 = (char*) (data_rx_service);
    if(!stringSeeker(s1,at,32,len, &n))
    {
            ++wifi_timer;
            if(wifi_timer >= timeout)
            {
                wifi_timer = 0;
                ptr = 3;
            }
    }
    else
    {
           ptr = 2;
    }
    return ptr;
}
void device_manager(void)
{
   ;
}
uint16_t IPD_parser(const char * str)
{
    uint16_t n = 0;
    const char *s_head = "@@"; //0x40 0x40
    const char *s1 = (char*) esp8266.AT_rx_buff;
        if(stringSeeker(s1,s_head, 100,2, &n))
        {
            esp8266.station[1].status = TCP_CLIENT_IS_CONNECTED;
            esp8266.station[1].lost_connection = 0;
            memcpy(&data_rx[0], &esp8266.AT_rx_buff[n + 2], 32);
        }
    return n;
}
uint16_t atStatusParser(const char * str)
{
    uint16_t n = 0;
    portSeeker(str, 128, &esp8266);
    return n;
}
uint16_t AT_parser(const char * str)
{
    uint16_t n = 0;
    const char *s_status = "+CIPSTATUS:";
    const char *s_serv = ">";
    const char *s_alr = "ALR";
    if (stringSeeker(str, s_status, 128 ,11, &n))
    {
        n =  atStatusParser(str);
    }
    else if (stringSeeker(str, s_serv, 64, 1, &n))
    {
        //memcpy(&data_rx_service[0], &esp8266.AT_rx_buff[n], 64);
        data_rx_service[0] = esp8266.AT_rx_buff[n];
    }
    else if (stringSeeker(str, s_alr, 64, 3, &n))
    {
        data_rx_service[0] = esp8266.AT_rx_buff[n];
    }
    return n;
}
void portSeeker(const char * str_src, uint16_t len_1,  ESP8266 * esp)
{
    uint16_t n = 0;
    uint16_t len_2 = 11;
    const char *str_tg;
    const char *cut_string;
    const char *c_status_0 = "CIPSTATUS:0";
    const char *c_status_1 = "CIPSTATUS:1";
    const char *c_status_2 = "CIPSTATUS:2";
    const char *c_status_3 = "CIPSTATUS:3";
    int i,j = 0;
    if(stringSeeker(str_src, c_status_0, len_1, len_2, &n))
    {
       n = 0;
       str_tg = c_status_0;
       for(i = 0; i < len_1; ++i )
                     {
                            cut_string = &str_src[i];
                            if(!strncmp(cut_string, str_tg, len_2))
                            {
                                esp->station[n].port_size = 0;
                                for(j = 0; j < 5; ++j)
                                {
                                    if(cut_string[j + len_2 + 1] != ',')
                                    {
                                        esp->station[n].port[j] = cut_string[j + len_2 + 1];
                                        ++(esp->station[n].port_size);
                                    }
                                }
                            }
                      }
                 esp8266.station[n].status = TCP_CLIENT_IS_CONNECTED;
    }
    if (stringSeeker(str_src, c_status_1, len_1, len_2, &n))
    {
       n = 1;
       str_tg = c_status_1;
       for(i = 0; i < len_1; ++i )
                     {
                            cut_string = &str_src[i];
                            if(!strncmp(cut_string, str_tg, len_2))
                            {
                                esp->station[n].port_size = 0;
                                for(j = 0; j < 5; ++j)
                                {
                                    if(cut_string[j + len_2 + 1] != ',')
                                    {
                                        esp->station[n].port[j] = cut_string[j + len_2 + 1];
                                        ++(esp->station[n].port_size);
                                    }
                                }
                            }
                      }
                 if(esp8266.station[n].status == TCP_CLIENT_IS_NOT_CONNECTED)
                 {
                     esp8266.station[n].status = TCP_CLIENT_IS_CONNECTED;
                    // atCommand_tx.tx_flag = 1;
                   //  esp8266.c_status = ESP8266_CIPSERVER_TX;
                   //  atCommandManager((int)AT_CIPSTART_UDP(NUM));
                 }
                 else if(esp->station[n].lost_connection < DEVICE_LOST_TIMEOUT)
                 {
                    ++(esp->station[n].lost_connection);
                 }
                 else
                 {
                     esp->station[n].lost_connection = 0;
                     esp8266.station[n].status = TCP_CLIENT_IS_DISCONNECTED;
                     resetLoops(&gen);
                 }
    }
    if(stringSeeker(str_src, c_status_2, len_1, len_2, &n))
    {
       n = 2;
       str_tg = c_status_2;
       for(i = 0; i < len_1; ++i )
                  {
                         cut_string = &str_src[i];
                         if(!strncmp(cut_string, str_tg, len_2))
                         {
                             esp->station[n].port_size = 0;
                             for(j = 0; j < 5; ++j)
                             {
                                 if(cut_string[j + len_2 + 1] != ',')
                                 {
                                     esp->station[n].port[j] = cut_string[j + len_2 + 1];
                                     ++(esp->station[n].port_size);
                                 }
                             }
                         }
                   }
              esp8266.station[n].status = TCP_CLIENT_IS_CONNECTED;
    }
    if(stringSeeker(str_src, c_status_3, len_1, len_2, &n))
    {
       n = 3;
       str_tg = c_status_3;
       for(i = 0; i < len_1; ++i )
                     {
                            cut_string = &str_src[i];
                            if(!strncmp(cut_string, str_tg, len_2))
                            {
                                esp->station[n].port_size = 0;
                                for(j = 0; j < 5; ++j)
                                {
                                    if(cut_string[j + len_2 + 1] != ',')
                                    {
                                        esp->station[n].port[j] = cut_string[j + len_2 + 1];
                                        ++(esp->station[n].port_size);
                                    }
                                }
                            }
                      }
                 esp8266.station[n].status = TCP_CLIENT_IS_CONNECTED;
    }
}
/*
int min_int(uint16_t * data, uint16_t len)
{
    int min = 0;
    int i = 0;
    min = data[0];
    for(i = 0; i < len; ++i)
    {
        if(min > data[i])
        {
            min = data[i];
        }
    }
    return min;
}
int mean_int(uint16_t * data, uint16_t len)
{
    uint32_t mean = 0;
    int i = 0;
    for(i = 0; i < len; ++i)
    {
        mean += data[i];
    }
    mean = mean / len;
    return mean;
}
*/
_iq loopProcessing(LOOP * loop, PI_REG * pi, LOOPS_ENABLE flag, _iq norm)
{
    PI_REG *pi_obj = (PI_REG*) pi;
    LOOP * loop_obj = (LOOP *) loop;
    _iq u;
    if(flag == LOOP_ENABLE)
    {
        (loop_obj->err) = (loop_obj->in) - _IQmpy((loop_obj->fbk),(loop_obj->k_err));
        pi_obj->in = (loop_obj->err);
        loop_obj->out = piProcessing(pi_obj);
        //u = loop_obj->out;
        u = _IQdiv((loop_obj->out),norm);
    }
    else
    {
        u = _IQdiv((loop_obj->in),norm);
    }

        if(u > AMP_LIMIT)
        {
            u = AMP_LIMIT;
        }
        else if (u < 0.0)
        {
            u = 0;
            pi_obj->accum = 0; // FOR TEST
            pi_obj->out = 0; //FOR TEST
        }

    return u;
}
inline _iq piProcessing(PI_REG * pi)
{
    PI_REG * obj = (PI_REG *) pi;
    _iq out, i_proc;

    obj->integrator.in_new = obj->in;
    i_proc = integratorProcessing(&(obj->integrator));
    obj->out = _IQmpy((obj->in),(obj->kp)) + _IQmpy(i_proc, (obj->ki));
    out = obj->out;
    if(out > _IQ(40.0))// if(out > (obj->integrator.limit_hi))
    {
     //  out = (obj->integrator.limit_hi); //SEEMS LIKE BUG
    }
    if(out < _IQ(0.0))// if(out < (obj->integrator.limit_low))
    {
       //out = (obj->integrator.limit_low);//SEEMS LIKE BUG
    }
    return out;
}
_iq integratorProcessing(INTEGRATOR_Q *i)
{
    _iq out, accum_new;
    INTEGRATOR_Q *obj = (INTEGRATOR_Q *) i;
    accum_new = _IQdiv2(_IQmpy((obj->T),(obj->in_new + obj->in_old)));///2;

    if((obj->out) > (obj->limit_hi)) // WAS >=// if((obj->out) > (obj->limit_hi)) // WAS >=
        {
            if(accum_new < 0)
            {
                (obj->accum) += accum_new;
            }
            else
            {
                obj->out = (obj->limit_hi);// _IQ(40.0);//obj->limit_hi;
            }
        }
     else if ((obj->out) < (obj->limit_low))//WAS <=
        {
            if(accum_new > 0)
            {
                (obj->accum) += accum_new;
            }
            else
            {
                obj->out = _IQ(0.0);//obj->limit_low;
            }
        }
        else
        {
            (obj->accum) += accum_new;
        }
        obj->in_old = obj->in_new;
        obj->out = obj->accum;
        out = obj->out;

        return out;
}
void resetLoops(GEN_STRUCT * d)
{
    // U_OUT_LOOP_INIT///
      d->u_out_loop.out = _IQ(0.0);
      d->u_out_loop.err = _IQ(0.0);
      d->u_out_loop.fbk = _IQ(0.0);
      d->u_out_loop.k_err = _IQ(1.0);
      d->u_out_loop.in = V_NOM - _IQ(0.5);//_IQ(38.0);

      d->pi_u.T = _IQ(0.1);
      d->pi_u.accum = _IQ(0.0);
      d->pi_u.in = _IQ(0.0);

      d->pi_u.integrator.T = _IQ(0.6);//d->pi_u.integrator.T = _IQ(0.1);
      d->pi_u.integrator.accum = _IQ(0.0);
      d->pi_u.integrator.in_new = _IQ(0.0);
      d->pi_u.integrator.in_old = _IQ(0.0);
      d->pi_u.integrator.limit_hi = _IQ(4000.0);
      d->pi_u.integrator.limit_low = _IQ(-60.0);
      d->pi_u.integrator.out = _IQ(0.0);


      d->pi_u.kd = _IQ(0.0);
      d->pi_u.ki = _IQ(0.1);//d->pi_u.ki = _IQ(0.3);
      d->pi_u.kp = _IQ(0.95);
      d->pi_u.out = _IQ(0.0);

      d->u_out_loop_en = LOOP_DISABLE;
}
