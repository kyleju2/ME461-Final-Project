#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"
#include "F28379dCAN.h"


#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define WHEEL_RADIUS_M 0.06
#define LAUNCHPAD_CPU_FREQUENCY 200
#define RX_MSG_DATA_LENGTH    8
#define RX_MSG_OBJ_ID_1       1  //measurement from sensor 1
#define RX_MSG_OBJ_ID_2       2  //measurement from sensor 2
#define RX_MSG_OBJ_ID_3       3  //quality from sensor 1
#define RX_MSG_OBJ_ID_4       4  //quality from sensor 2
#define WR                    0.173
#define RWH                   0.0593

// Interrupt function declarations
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void can_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void SPI_isr(void);
__interrupt void ADCA_ISR(void);


// Helper function declarations
void setupSpib(void);
void init_eQEPs(void);
float readEncRight(void);
float readEncLeft(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);


// Create struct to hold IMU data
typedef struct {
    int16_t accelx_raw, accely_raw, accelz_raw;
    int16_t gyrox_raw,  gyroy_raw,  gyroz_raw;
    float   accelx,     accely,     accelz;
    float   gyrox,      gyroy,      gyroz;
} IMUData;

// Struct to hold Wheel Kinematics and Control Params
typedef struct {
    float dist, dist_prev;
    float vel;
    float rotation_angle;
    //    float theta_avg = 0.0;
    //    float theta_avg_dot = 0.0;
    //    float rotation_angle_dot = 0.0;
    float e, e_prev;
    float I, I_prev;
    float u;
} wheelCtrl;


float rotation_angle_l = 0.0;
float rotation_angle_r = 0.0;
float theta_avg = 0.0;
float theta_avg_dot = 0.0;
float x_prev = 0.0;
float y_prev = 0.0;
float xr_dot = 0.0;
float xr_dot_prev = 0.0;
float yr_dot = 0.0;
float yr_dot_prev = 0.0;
float rotation_angle_r_dot = 0.0;
float rotation_angle_l_dot = 0.0;


// enum to hold hex values for each wii remote in the bitfield
typedef enum {
    BUTTON_Z          = 0x00020000, // nunchuk
    BUTTON_C          = 0x00010000, // nunchuk
    BUTTON_PLUS       = 0x00001000,
    BUTTON_UP         = 0x00000800, // vertical orientation
    BUTTON_DOWN       = 0x00000400,
    BUTTON_RIGHT      = 0x00000200,
    BUTTON_LEFT       = 0x00000100,
    BUTTON_HOME       = 0x00000080,
    BUTTON_MINUS      = 0x00000010,
    BUTTON_A          = 0x00000008,
    BUTTON_B          = 0x00000004,
    BUTTON_ONE        = 0x00000002,
    BUTTON_TWO        = 0x00000001,
    NO_BUTTON         = 0x00000000
} ButtonState;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// KZ GLOBAL VARIABLES FOR ADCA
uint16_t ADCINA0_result = 0;
uint16_t ADCINA1_result = 0;

float ADCINA0_float = 0.0;
float ADCINA1_float = 0.0;

float state_array_A0[22];
float state_array_A1[22];
float yk_A2 = 0.0;
float yk_A3 = 0.0;

float b_A[22] = {   -2.3890045153263611e-03,    -3.3150057635348224e-03,    -4.6136191242627002e-03,    -4.1659855521681268e-03,    1.4477422497795286e-03, 1.5489414225159667e-02, 3.9247886844071371e-02,
                    7.0723964095458614e-02, 1.0453473887246176e-01, 1.3325672639406205e-01, 1.4978314227429904e-01, 1.4978314227429904e-01, 1.3325672639406205e-01, 1.0453473887246176e-01,
                    7.0723964095458614e-02, 3.9247886844071371e-02, 1.5489414225159667e-02, 1.4477422497795286e-03, -4.1659855521681268e-03,    -4.6136191242627002e-03,    -3.3150057635348224e-03,    -2.3890045153263611e-03};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Counter Variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numTimer2calls = 0;
int32_t SpibNumCalls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// Global Variable Initializations
IMUData imu = {0}; // init imu data struct instance to zeroes
wheelCtrl leftWheelstruct  = {0};
wheelCtrl rightWheelstruct = {0};


// PID Params/ Gains
float Vref = 0;
float turn = 0;
//float Kp = 3.0f;
//float Ki = 25.0f;
//float Kd = 0.02f;
float KP_turn = 3.0;

float e_turn = 0.0;
float printLV3 = 0;
float printLV4 = 0;
float printLV5 = 0;
float printLV6 = 0;
float printLV7 = 0;
float printLV8 = 0;
float x = 0;
float y = 0;
float bearing = 0;
// KZ Variable for exercise 5

extern uint16_t NewCOMCdata; //KZ added for wii remote
extern uint32_t wii_button_state;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Exercise 2
// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.76;
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheel = 0;
float RightWheel = 0;
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t AverageIndex = -1;
float pred_P = 0;
float kalman_K = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

//Exercise 3

float LeftWheel_1 = 0.0;
float RightWheel_1 = 0.0;
float velRightdot = 0.0;
float velLeftdot = 0.0;
float velRightdot1 = 0.0;
float velLeftdot1 = 0.0;
float gyrorate = 0.0;
float gyrorate1 = 0.0;
float gyrorate_dot = 0.0;
float gyrorate_dot_1 = 0.0;
float tilt = 0.0;
float ubal = 0.0;
float K1 = -60.0;
float K2  = -4.5;
float K3  = -1.1;
float K4 = -0.1;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Exercise 4
float Kp = 3.0;
float Ki = 20.0;
float Kd = 0.08;
float WhlDiff;
float WhlDiff_1;
float vel_WhlDiff;
float vel_WhlDiff_1;

float turnref;
float errorDiff = 0;
float errorDiff_1;
float intDiff = 0;
float intDiff_1;

/////
float ForwardBackwardCommand = 0.0;
float avg_wheel_vel = 0.0;
float Segbot_refSpeed = 0.0;
float KpSpeed = 0.35;
float KiSpeed = 1.5;

float eSpeed = 0.0;
float eSpeed_1 = 0.0;

float intSpeed = 0.0;
float intSpeed_1 = 0.0;
/////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// ----- code for CAN start here -----

volatile uint32_t rxMsgCount_1 = 0;
volatile uint32_t rxMsgCount_3 = 0;
extern uint16_t rxMsgData[8];

uint32_t dis_raw_1[2];
uint32_t dis_raw_3[2];
uint32_t dis_1 = 0;
uint32_t dis_3 = 0;

uint32_t quality_raw_1[4];
uint32_t quality_raw_3[4];
float quality_1 = 0.0;
float quality_3 = 0.0;

uint32_t lightlevel_raw_1[4];
uint32_t lightlevel_raw_3[4];
float lightlevel_1 = 0.0;
float lightlevel_3 = 0.0;

uint32_t measure_status_1 = 0;
uint32_t measure_status_3 = 0;

volatile uint32_t errorFlag = 0;

// ----- code for CAN end here -----

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();
    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // ----- code for CAN start here -----
    //GPIO17 - CANRXB
    GPIO_SetupPinMux(17, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(17, GPIO_INPUT, GPIO_ASYNC);

    //GPIO12 - CANTXB
    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 2);
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);
    // ----- code for CAN end here -----

    // ----- code for CAN start here -----
    // Initialize the CAN controller
    InitCANB();

    // Set up the CAN bus bit rate to 1000 kbps
    setCANBitRate(200000000, 1000000);

    // Enables Interrupt line 0, Error & Status Change interrupts in CAN_CTL register.
    CanbRegs.CAN_CTL.bit.IE0 = 1;
    CanbRegs.CAN_CTL.bit.EIE = 1;
    // ----- code for CAN end here -----

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    PieVectTable.ADCA1_INT = &ADCA_ISR;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    // ----- code for CAN start here -----
    PieVectTable.CANB0_INT = &can_isr;
    // ----- code for CAN end here -----
    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 5000); // 5 ms for polling the ESP32-2 UART line
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA, 115200);
    init_eQEPs();
    setupSpib();

    // KZ Added EPWM2 inits from lab 3
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // KZ Set clock div to 0
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // KZ Set TBCTL to count-up mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 3; // KZ Set FREE_SOFT to free run so PWM continues when you set a break point in the
    EPwm2Regs.TBCTL.bit.PHSEN = 0; // KZ
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.CMPB.bit.CMPB = 0;
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;
    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    EALLOW;
    // KZ change the setting of PWM  to change the sampling frequency of the filter
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 0x2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 0x1; // Generate pulse on 1st event
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0x0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    // KZ change the setting for the ADCs to do different exercises
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x2; //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x3; //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT6;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA  CANB
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // ADCA1
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;  //SPiB
    // ----- code for CAN start here -----
    // Enable CANB in the PIE: Group 9 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx7 = 1;
    // ----- code for CAN end here -----

    // ----- code for CAN start here -----
    // Enable the CAN interrupt signal
    CanbRegs.CAN_GLB_INT_EN.bit.GLBINT0_EN = 1;
    // ----- code for CAN end here -----

    init_serialSCIC(&SerialC, 115200);
    init_serialSCID(&SerialD, 115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM
    // ----- code for CAN start here -----

    //    // Transmit Message
    //    // Initialize the transmit message object used for sending CAN messages.
    //    // Message Object Parameters:
    //    //      Message Object ID Number: 0
    //    //      Message Identifier: 0x1
    //    //      Message Frame: Standard
    //    //      Message Type: Transmit
    //    //      Message ID Mask: 0x0
    //    //      Message Object Flags: Transmit Interrupt
    //    //      Message Data Length: 4 Bytes
    //    //
    //    CANsetupMessageObject(CANB_BASE, TX_MSG_OBJ_ID, 0x1, CAN_MSG_FRAME_STD,
    //                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
    //                           TX_MSG_DATA_LENGTH);

    // Measured Distance from 1
    // Initialize the receive message object 1 used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 1
    //      Message Identifier: 0x060b0101
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes (Note that DLC field is a "don't care"
    //      for a Receive mailbox)
    //
    CANsetupMessageObject(CANB_BASE, RX_MSG_OBJ_ID_1, 0x060b0101,
                          CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0,
                          CAN_MSG_OBJ_RX_INT_ENABLE,
                          RX_MSG_DATA_LENGTH);

    // Measured Distance from 2
    // Initialize the receive message object 2 used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 2
    //      Message Identifier: 0x060b0102
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes (Note that DLC field is a "don't care"
    //      for a Receive mailbox)
    //

    CANsetupMessageObject(CANB_BASE, RX_MSG_OBJ_ID_2, 0x060b0103,
                          CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0,
                          CAN_MSG_OBJ_RX_INT_ENABLE,
                          RX_MSG_DATA_LENGTH);

    // Measurement Quality from 1
    // Initialize the receive message object 2 used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 3
    //      Message Identifier: 0x060b0201
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes (Note that DLC field is a "don't care"
    //      for a Receive mailbox)
    //

    CANsetupMessageObject(CANB_BASE, RX_MSG_OBJ_ID_3, 0x060b0201,
                          CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0,
                          CAN_MSG_OBJ_RX_INT_ENABLE,
                          RX_MSG_DATA_LENGTH);

    // Measurement Quality from 2
    // Initialize the receive message object 2 used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 4
    //      Message Identifier: 0x060b0202
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes (Note that DLC field is a "don't care"
    //      for a Receive mailbox)
    //

    CANsetupMessageObject(CANB_BASE, RX_MSG_OBJ_ID_4, 0x060b0203,
                          CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0,
                          CAN_MSG_OBJ_RX_INT_ENABLE,
                          RX_MSG_DATA_LENGTH);

    //
    // Start CAN module operations
    //
    CanbRegs.CAN_CTL.bit.Init = 0;
    CanbRegs.CAN_CTL.bit.CCE = 0;

    //    // Initialize the transmit message object data buffer to be sent
    //    txMsgData[0] = 0x12;
    //    txMsgData[1] = 0x34;
    //    txMsgData[2] = 0x56;
    //    txMsgData[3] = 0x78;

    //    // Loop Forever - A message will be sent once per second.
    //    for(;;)
    //    {
    //
    //        CANsendMessage(CANB_BASE, TX_MSG_OBJ_ID, TX_MSG_DATA_LENGTH, txMsgData);
    //        txMsgCount++;
    //        DEVICE_DELAY_US(1000000);
    //    }

    // ----- code for CAN end here -----

    // IDLE loop. Just sit and loop forever (optional):
    while (1)
    {
        if (UARTPrint == 1)
        {
            //            serial_printf(&SerialA,"Joystick: %.3f, %.3f \r\n", yk_A2, yk_A3);
            //            serial_printf(&SerialA,"Accel: %.3f, %.3f, %.3f \r\n", imu.accelx, imu.accely, imu.accelz);
            //            serial_printf(&SerialA,"Gyro: %.3f, %.3f, %.3f \r\n", imu.gyrox, imu.gyroy, imu.gyroz);
            //            serial_printf(&SerialA,"Angle: %.3f, %.3f \r\n", leftWheelstruct.rotation_angle, rightWheelstruct.rotation_angle);
            serial_printf(&SerialA,"Tilt: %.3f, Gyro: %.3f, LW: %.3f, RW: %.3f \r\n", tilt_value, gyro_value, LeftWheel, RightWheel);

            UARTPrint = 0;
        }
    }

}




// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void)
{

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");
    // Wait one cycle
    EINT;
    // Clear INTM to enable interrupts

    float turnRateCmd = 5.0f;
    float forwardSpeed = 6.0f;
    float backwardSpeed = -6.0f;


    Segbot_refSpeed = 0.0f; // init to stopped
    turnref = 0.0f; // init to no turning

    if (!(wii_button_state & (BUTTON_UP | BUTTON_DOWN))) {intDiff = 0.0f;}
    if (wii_button_state & BUTTON_ONE)  {Segbot_refSpeed = forwardSpeed;}
    if (wii_button_state & BUTTON_TWO)  {Segbot_refSpeed = backwardSpeed;}
    if (wii_button_state & BUTTON_UP)   {turnref = +turnRateCmd;}
    if (wii_button_state & BUTTON_DOWN) {turnref = -turnRateCmd;}
    if (wii_button_state & BUTTON_A)    {Segbot_refSpeed = 0; turnref = 0;} // E-STOP

    // Insert SWI ISR Code here.......
    gyrorate = gyro_value;
    tilt = tilt_value;
    velRightdot = 0.6 * velRightdot1 + 100 * RightWheel - 100 * RightWheel_1;
    velLeftdot = 0.6 * velLeftdot1 + 100 * LeftWheel - 100 * LeftWheel_1;
    gyrorate_dot = 0.6 * gyrorate_dot_1 + (gyrorate - gyrorate1) * 100;
    ubal = -K1 * tilt - K2 * gyrorate - K3 * (velLeftdot + velRightdot)/2.0 - K4 * gyrorate_dot;
    //////////////////////////////////////////////////

    WhlDiff =  LeftWheel - RightWheel;

    vel_WhlDiff = 166.667*(WhlDiff- WhlDiff_1) + 0.333*vel_WhlDiff_1; //KZ + EE Filtered value for vel_Whlk_diff using discrete form of 250s/(s+250)

//    errorDiff = turnref - WhlDiff;
    errorDiff = turnref - vel_WhlDiff; //  velocity control
    intDiff = intDiff_1 + 0.004/2*(errorDiff + errorDiff_1);

    turn = Kp*errorDiff + Ki*intDiff; // - Kd*vel_WhlDiff;

    if (fabs(turn) > 5.0)
    {
        intDiff = intDiff_1;
        turn = Kp*errorDiff + Ki*intDiff - Kd*vel_WhlDiff;
    }

    if (turn > 5.0)
    {
        turn = 5.0;
    }
    else if (turn < -5.0)
    {
        turn = -5.0;
    }


    avg_wheel_vel = (velRightdot + velLeftdot)/2.0;
    eSpeed = (Segbot_refSpeed - avg_wheel_vel);

    intSpeed = intSpeed_1 + (0.004/2.0) * (eSpeed + eSpeed_1); // Integral of eSpeed error

    ForwardBackwardCommand = KpSpeed*eSpeed + KiSpeed*intSpeed;
    if (fabs(ForwardBackwardCommand) > 6.0)
    {
        intSpeed = intSpeed_1;
    }

    if (ForwardBackwardCommand > 6.0)
        ForwardBackwardCommand = 6.0;

    else if (ForwardBackwardCommand < -6.0)
        ForwardBackwardCommand = -6.0;

    leftWheelstruct.u = ubal/2 + turn - ForwardBackwardCommand;
    rightWheelstruct.u = ubal/2 - turn - ForwardBackwardCommand;

    setEPWM2A(rightWheelstruct.u);
    setEPWM2B(-leftWheelstruct.u);


    theta_avg = 0.5 * (rotation_angle_r + rotation_angle_l);
    bearing = RWH / WR * (rotation_angle_r - rotation_angle_l);
    rotation_angle_r_dot = velRightdot / WHEEL_RADIUS_M;
    rotation_angle_l_dot = velLeftdot / WHEEL_RADIUS_M;
    theta_avg_dot = 0.5 * (rotation_angle_r_dot + rotation_angle_l_dot);

    xr_dot = RWH * theta_avg_dot * cos(bearing);
    yr_dot = RWH * theta_avg_dot * sin(bearing);

    x = x_prev + 0.5 * (xr_dot_prev + xr_dot) * 0.004;
    y = y_prev + 0.5 * (yr_dot_prev + yr_dot) * 0.004;

    // KZ save the previous values
    x_prev = x;
    y_prev = y;
    xr_dot_prev = xr_dot;
    yr_dot_prev = yr_dot;



    //Updates
    LeftWheel_1 = LeftWheel;
    RightWheel_1 = RightWheel;
    velRightdot1 = velRightdot;
    velLeftdot1 = velLeftdot;
    gyrorate1 = gyrorate;
    errorDiff_1 = errorDiff;
    vel_WhlDiff_1 = vel_WhlDiff;
    WhlDiff_1 = WhlDiff;
    gyrorate_dot_1 = gyrorate_dot;
    intSpeed_1 = intSpeed;
    eSpeed_1 = eSpeed;
    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls % 250) == 0)
    {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF)
        {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    //GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 2; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    //    SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 3; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0xDA;
    //    SpibRegs.SPITXBUF = 500; // PWM value
    //    SpibRegs.SPITXBUF = 2200; // PWM Value


    if ((numTimer0calls % 50) == 0)
    {
        //        // Blink LaunchPad Red LED
        //        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    numTimer1calls++;
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    //    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
    //
    //    if ((CpuTimer2.InterruptCount % 10) == 0)
    //    {
    //        UARTPrint = 1;
    //    }
}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the SPIB setup.
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason

    SpibRegs.SPIFFTX.bit.SPIRST = 1; // Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL

    //SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPIFFCT.bit.TXDLY = 0;

    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I dont think this is needed. Need to Test

    SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below

    //-----------------------------------------------------------------------------------------------------------------

    //Step 2.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16-bit transfers, so two registers at a time after the first 16-bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = (0x1300 | 0x0000);
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = (0x0000 | 0x0013);
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = (0x0200 | 0x0000);
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = (0x0800 | 0x0006);
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = (0x2300 | 0x0000);
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = (0x4000 | 0x008C);
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = (0x0200 | 0x0088);
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = (0x0C00 | 0x000A);

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081);
    // wait for one byte to be received
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00E9); // 0x7700
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x00B6); // 0x7800
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E3); // 0x7A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00C6); // 0x7B00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0026); // 0x7D00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x002C); // 0x7E00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
__interrupt void SPIB_isr(void)
{
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    spivalue1 = SpibRegs.SPIRXBUF;
    imu.accelx_raw = SpibRegs.SPIRXBUF;
    imu.accely_raw = SpibRegs.SPIRXBUF;
    imu.accelz_raw = SpibRegs.SPIRXBUF;

    spivalue2 = SpibRegs.SPIRXBUF;
    imu.gyrox_raw = SpibRegs.SPIRXBUF;
    imu.gyroy_raw = SpibRegs.SPIRXBUF;
    imu.gyroz_raw = SpibRegs.SPIRXBUF;

    // Scale values and fill struct
    imu.accelx = imu.accelx_raw * (4.0 / 32767.0);
    imu.accely = imu.accely_raw * (4.0 / 32767.0);
    imu.accelz = imu.accelz_raw * (4.0 / 32767.0);
    imu.gyrox = imu.gyrox_raw * (250.0 / 32767.0);
    imu.gyroy = imu.gyroy_raw * (250.0 / 32767.0);
    imu.gyroz = imu.gyroz_raw * (250.0 / 32767.0);

    //Code to be copied into SPIB_isr interrupt function after the IMU measurements have been collected.
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1) {
        accelx_offset+=imu.accelx;
        accely_offset+=imu.accely;
        accelz_offset+=imu.accelz;
        gyrox_offset+=imu.gyrox;
        gyroy_offset+=imu.gyroy;
        gyroz_offset+=imu.gyroz;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2) {
        imu.accelx -=(accelx_offset);
        imu.accely -=(accely_offset);
        imu.accelz -=(accelz_offset-accelzBalancePoint);
        imu.gyrox -= gyrox_offset;
        imu.gyroy -= gyroy_offset;
        imu.gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (imu.gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -imu.accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        AverageIndex++;
        // Kalman Filter used
        tilt_array[AverageIndex] = kalman_tilt;
        gyro_array[AverageIndex] = tiltrate;
        LeftWheelArray[AverageIndex] = readEncLeft();
        RightWheelArray[AverageIndex] = readEncRight();
        if (AverageIndex >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            AverageIndex = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }
    if((SpibNumCalls%200) == 0) {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
    }

    if ((SpibNumCalls % 200) == 0)
    {
        UARTPrint = 1;
    }
    SpibNumCalls++;

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

    //    setEPWM2A(rightWheelstruct.u);
    //    setEPWM2B(-leftWheelstruct.u);
}

// ----- code for CAN start here -----
__interrupt void can_isr(void)
{
    int i = 0;

    uint32_t status;

    GpioDataRegs.GPBSET.bit.GPIO52 = 1;
    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    status = CANgetInterruptCause(CANB_BASE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if (status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CANgetStatus(CANB_BASE);

    }

    //
    // Check if the cause is the transmit message object 1
    //
    //    else if(status == TX_MSG_OBJ_ID)
    //    {
    //        //
    //        // Getting to this point means that the TX interrupt occurred on
    //        // message object 1, and the message TX is complete.  Clear the
    //        // message object interrupt.
    //        //
    //        CANclearInterruptStatus(CANB_BASE, TX_MSG_OBJ_ID);
    //
    //        //
    //        // Since the message was sent, clear any error flags.
    //        //
    //        errorFlag = 0;
    //    }

    //
    // Check if the cause is the receive message object 2
    //
    else if (status == RX_MSG_OBJ_ID_1)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_1, rxMsgData);

        for (i = 0; i < 2; i++)
        {
            dis_raw_1[i] = rxMsgData[i];
        }

        dis_1 = 256 * dis_raw_1[1] + dis_raw_1[0];

        measure_status_1 = rxMsgData[2];

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_1);

        //
        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        rxMsgCount_1++;

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    }

    else if (status == RX_MSG_OBJ_ID_2)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_2, rxMsgData);

        for (i = 0; i < 2; i++)
        {
            dis_raw_3[i] = rxMsgData[i];
        }

        dis_3 = 256 * dis_raw_3[1] + dis_raw_3[0];

        measure_status_3 = rxMsgData[2];

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_2);

        //
        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        rxMsgCount_3++;

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    }

    else if (status == RX_MSG_OBJ_ID_3)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_3, rxMsgData);

        for (i = 0; i < 4; i++)
        {
            lightlevel_raw_1[i] = rxMsgData[i];
            quality_raw_1[i] = rxMsgData[i + 4];

        }

        lightlevel_1 = ((256.0 * 256.0 * 256.0) * lightlevel_raw_1[3]
                                                                   + (256.0 * 256.0) * lightlevel_raw_1[2]
                                                                                                        + 256.0 * lightlevel_raw_1[1] + lightlevel_raw_1[0]) / 65535;
        quality_1 = ((256.0 * 256.0 * 256.0) * quality_raw_1[3]
                                                             + (256.0 * 256.0) * quality_raw_1[2] + 256.0 * quality_raw_1[1]
                                                                                                                          + quality_raw_1[0]) / 65535;

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_3);

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    }

    else if (status == RX_MSG_OBJ_ID_4)
    {
        //
        // Get the received message
        //
        CANreadMessage(CANB_BASE, RX_MSG_OBJ_ID_4, rxMsgData);

        for (i = 0; i < 4; i++)
        {
            lightlevel_raw_3[i] = rxMsgData[i];
            quality_raw_3[i] = rxMsgData[i + 4];

        }

        lightlevel_3 = ((256.0 * 256.0 * 256.0) * lightlevel_raw_3[3]
                                                                   + (256.0 * 256.0) * lightlevel_raw_3[2]
                                                                                                        + 256.0 * lightlevel_raw_3[1] + lightlevel_raw_3[0]) / 65535;
        quality_3 = ((256.0 * 256.0 * 256.0) * quality_raw_3[3]
                                                             + (256.0 * 256.0) * quality_raw_3[2] + 256.0 * quality_raw_3[1]
                                                                                                                          + quality_raw_3[0]) / 65535;

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CANclearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID_4);

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
    }

    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CANclearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    InterruptclearACKGroup(INTERRUPT_ACK_GROUP9);
}

void init_eQEPs(void)
{
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}

float readEncLeft(void)
{
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U

    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue / 2)
        raw -= QEP_maxvalue; // I don't think this is needed and never true

    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (-raw * (2.0 * PI / (30.0 * 400.0)));
}

float readEncRight(void)
{

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue / 2)
        raw -= QEP_maxvalue; // I don't think this is needed and never true

    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw * (2.0 * PI / (30.0 * 400.0)));
}

void setEPWM2A(float controleffort)
{
    if (controleffort > 10.0)
    {
        controleffort = 10.0;
    }
    else if (controleffort < -10.0)
    {
        controleffort = -10.0;
    }
    EPwm2Regs.CMPA.bit.CMPA = EPwm2Regs.TBPRD * (controleffort + 10.0) / 20.0;
}
// KZ: Coded the function for setting PWM for the robot's second motor through CMPB
void setEPWM2B(float controleffort)
{
    if (controleffort > 10.0)
    {
        controleffort = 10.0;
    }
    else if (controleffort < -10.0)
    {
        controleffort = -10.0;
    }
    EPwm2Regs.CMPB.bit.CMPB = EPwm2Regs.TBPRD * (controleffort + 10.0) / 20.0;

}

__interrupt void ADCA_ISR(void) {
    ADCINA0_result = AdcaResultRegs.ADCRESULT0;
    ADCINA1_result  = AdcaResultRegs.ADCRESULT1;

    state_array_A0[0] = ADCINA0_result * (3.0/4096.0);
    state_array_A1[0] = ADCINA1_result * (3.0/4096.0);

    yk_A2 = 0.0;
    yk_A3 = 0.0;

    for (int i = 0; i < 21; i++) {
        yk_A2 += state_array_A0[i]*b_A[i];
        yk_A3 += state_array_A1[i]*b_A[i];
    }

    for (int i = 21; i > 0; i--) {
        state_array_A0[i] = state_array_A0[i-1];
        state_array_A1[i] = state_array_A1[i-1];
    }

    //    leftWheelstruct.rotation_angle = readEncRight();
    //    rightWheelstruct.rotation_angle = readEncLeft();


    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = 0xBA00;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;

    //    adc_count_A++;
    //    if ((adc_count_A % 100) == 0) {
    //        UARTPrint = 1;
    //    }
    // Print ADCIND0�s voltage value to TeraTerm every 100ms
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
// ----- code for CAN end here -----


