
//*************************************************************************
// Name of Group Members: Grace Pasternick, Varsha Borkar
// Creation Date: 4/16/2026
// Lab Section: Tuesday
// Lab this program is associated with: Project 2
// Lab due date: 5/2/2026
//
// Hardware Inputs used: PE3 (ADC), PF1 (IR), PF3 (Microphone)
//
// Hardware Outputs used: PB6 (PWM / Servo); PE1, PE2, PE4 (LEDs);
//
// Hardware used: breadboard, microphone, potentiometer, 3 LEDs, IR sensor, 3 resistors, jumper wires, Tiva C microcontroller
//
// Additional files needed: driverlib, inc
//
// Date of last modification: 4/30/2026
//
//*************************************************************************

//File include Statements
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "inc/hw_gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/watchdog.h"

//Global variables
unsigned int INPUT; //store sample for ADC
int current_state = 0; //initializing state, match type with struct defintion
unsigned long divider; // for PWM
unsigned long ulPeriod; //for PWM
float max_allowed_time; //define max time allowed to be in WARN, set by potentiometer
volatile int countdown_ticks = -1;  // set to -1 to prevent off counting
volatile int timer_reset = 0; //initialized to 0, resets timer set by potentiometer

//Variable Declarations
#define GPIO_PA0_U0RX           0x00000001
#define GPIO_PA1_U0TX           0x00000401

//Function Prototypes
void ADC_setup(void);
void UART_setup(void);
float calc(float ADC_val);
void print(float max_allowed_time);
void systick(int reload_val);
void pwm_setup(void);
void ISR(void);
int trigger(int ticks, int current_state);
void port_output_setup(int output_pin);
void portF_input_setup(int input_pin);
void servo_move(uint32_t angle);
void watchdog_setup(void);


//FSM Struct
struct fsm {
    unsigned char outLED; // toggles output pins to LEDs (red, yellow, green)
    unsigned long outSERVO; // toggles output pin to servo (off/"Quiet Please")
    unsigned long wait; // wait time in state
    char next[4]; //next state based on the 4 possible states
};

typedef struct fsm stype;

#define OFF   0
#define QUIET 1
#define WARN  2
#define LOUD  3

//fsm mapping
stype fsm[4] = {
    // outLED: port E, outSERVO: angle for servo, wait time, next states
    { 0x00, 80,    0, {OFF, QUIET, WARN, WARN} }, //off state - no LEDs (system off, no presence detected)
    { 0x02, 80,    0, {OFF, QUIET, WARN, WARN} }, //quiet - green LED on - system on, no noise
    { 0x10, 80,    0, {OFF, QUIET, WARN, LOUD} }, //warn - yellow LED on - system on, noise detected, count down start
    { 0x04, (210), 0, {OFF, QUIET, LOUD, LOUD} } //loud - red LED on - system on, noise detected, count down finished
};


int main()
{
    // SysCtlClockSet first before anything that calls SysCtlClockGet
    //SYSDIV_2_5 used to match 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //initialize input and outputs
    port_output_setup(0x16); //set up outputs for PE1, PE2, PE4
    portF_input_setup(0x0A); //set up inputs for PF1, PF3

    pwm_setup();  //sets up PWM channel 0 for port B pin 6
    ADC_setup(); //set up ADC0 for pot (PE3)
    UART_setup(); //initialize UART to print to terminal
    watchdog_setup(); //call watchdog setup

    systick(8000000); // config systick trigger to 80 MHz (needed for transitions AND servo )
    SysTickIntRegister(ISR); // set up systick handler  using ISR
    SysTickIntEnable(); //enable systick interrupt
    IntMasterEnable(); //enable global interrupts

    //while loop updates outputs, reads inputs, computes next FSM state, and signals IST for when timer should be reset
    while(1){
        servo_move(fsm[current_state].outSERVO); //move servo to angle determined in FSM

        GPIO_PORTE_DATA_R = fsm[current_state].outLED; //turn on LEDs as determined in FSM

        print(max_allowed_time); //print max allowed time to terminal through UART

        //read in ADC
        ADCProcessorTrigger(ADC0_BASE, 0); //cause processor trigger for sample sequence
        while(!ADCIntStatus(ADC0_BASE, 0, false)); //makes program wait until true returned
        ADCIntClear(ADC0_BASE, 0); //clears flag from ADCIntStatus
        ADCSequenceDataGet(ADC0_BASE, 0, &INPUT); //gets captured data for sample sequence

        max_allowed_time = calc(INPUT);//converts ADC value to time (seconds)

        int prev_state = current_state; //store current state

        int indx = trigger(countdown_ticks, current_state); //calls trigger function - sets index for next state

        current_state = fsm[current_state].next[indx]; //updates current state

        //signal ISR to reload timer when ENTERING warning state
        if (current_state == WARN && prev_state != WARN) {
            timer_reset = 1;
        }

        WatchdogIntClear(WATCHDOG0_BASE); //feed the dog
        SysCtlDelay(SysCtlClockGet() / 50); //delay printing to terminal from UART
    }

    return 0;
}


//UART Setup
void UART_setup(void)
{
    //enable clocks for UART0 and GPIOA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Wait for peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

    //map PA0 and PA1 pins to UART functions
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //set pin types to UART
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use 80000000 explicitly to match SYSCTL_SYSDIV_2_5 setting
    UARTConfigSetExpClk(UART0_BASE, 80000000, 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}


//ADC Setup
void ADC_setup(void)
{
    //enagle ADC0 and GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //configure PE3 as input
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    //set up sequence 0 triggered by processor
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

    //read channel 0
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);

    //enabled sequence
    ADCSequenceEnable(ADC0_BASE, 0);
}

//Function to calculate max allowed time from ADC value
float calc(float ADC_val)
{
    max_allowed_time = (ADC_val * 60.0) / 4095.0; //60 referenced for seconds, 4095 for 12-bit precision
    return max_allowed_time;
}

//UART to print time allowance and current state to terminal
void print(float max_allowed_time)
{
    //convert to ASCII
    int total_seconds = (int)max_allowed_time;
    int hundreds = (total_seconds / 100) % 10;
    int tens = (total_seconds / 10) % 10;
    int ones = total_seconds % 10;

    float remainder = max_allowed_time - (float)total_seconds;
    int fractional_part = (int)(remainder * 100.0 + 0.5);
    int tenths = (fractional_part / 10) % 10;
    int hundredths = fractional_part % 10;

    UARTCharPut(UART0_BASE, 'S');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, 'a');
    UARTCharPut(UART0_BASE, 't');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, ':');
    UARTCharPut(UART0_BASE, ' ');

    //printing states
    if(current_state == OFF)   { UARTCharPut(UART0_BASE, 'O'); }
    if(current_state == QUIET) { UARTCharPut(UART0_BASE, 'A'); }
    if(current_state == WARN)  { UARTCharPut(UART0_BASE, 'W'); }
    if(current_state == LOUD)  { UARTCharPut(UART0_BASE, 'U'); }

    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 'T');
    UARTCharPut(UART0_BASE, 'i');
    UARTCharPut(UART0_BASE, 'm');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 'A');
    UARTCharPut(UART0_BASE, 'l');
    UARTCharPut(UART0_BASE, 'l');
    UARTCharPut(UART0_BASE, 'o');
    UARTCharPut(UART0_BASE, 'w');
    UARTCharPut(UART0_BASE, 'a');
    UARTCharPut(UART0_BASE, 'n');
    UARTCharPut(UART0_BASE, 'c');
    UARTCharPut(UART0_BASE, 'e');
    UARTCharPut(UART0_BASE, ':');
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, (hundreds + 0x30));
    UARTCharPut(UART0_BASE, (tens     + 0x30));
    UARTCharPut(UART0_BASE, (ones     + 0x30));
    UARTCharPut(UART0_BASE, '.');
    UARTCharPut(UART0_BASE, (tenths    + 0x30));
    UARTCharPut(UART0_BASE, (hundredths+ 0x30));
    UARTCharPut(UART0_BASE, ' ');
    UARTCharPut(UART0_BASE, 's');
    UARTCharPut(UART0_BASE, '\r');
    UARTCharPut(UART0_BASE, '\n');
}

// ISR Function to handle count down timer
void ISR(void)
{
    if (timer_reset) {
        // Reload count down from current potentiometer value
        // *10 because ISR fires 10x per second (every 0.1s)
        countdown_ticks = (int)(max_allowed_time * 10);
        timer_reset = 0; //resets timer
    }
    else if (countdown_ticks > 0) {
        countdown_ticks--; //decrease timer
    }
    // if countdown_ticks is -1 do nothing - timer hasn't been started yet
}

// Output Setup Function - configure port E pins for LED outputs
void port_output_setup(int output_pin) {
    SYSCTL_RCGCGPIO_R |= 0x10; //enable clock for port E
    GPIO_PORTE_DIR_R  |= (output_pin); //set specified pins as output
    GPIO_PORTE_DEN_R  |= (output_pin); //data enable
}

//Input Setup Function - configure PF1 (IR) and PF3 (microphone)
void portF_input_setup(int input_pin) {
    SYSCTL_RCGCGPIO_R  |= 0x20; //setup clock port F
    GPIO_PORTF_LOCK_R  |= 0x4C4F434B; // unlock register
    GPIO_PORTF_CR_R    |= (input_pin); // allow changes to be made
    GPIO_PORTF_DIR_R   &= ~(input_pin); //set input direction
    GPIO_PORTF_PUR_R   |= (input_pin); //enable pull up resistors
    GPIO_PORTF_DEN_R   |= (input_pin); //data enable
}

//Systick Function
void systick(int reload_val) {
    NVIC_ST_CTRL_R    = 0;  //initialize control register to 0
    NVIC_ST_RELOAD_R  = (reload_val); //reload value established when function is called
    NVIC_ST_CURRENT_R = 0; //resetting current value used for counting starting at 0
    NVIC_ST_CTRL_R    = 7; //initialize the control register to 7 to allow counting and enable interrupts and clock
}

//PWN Setup Function
void pwm_setup(void)
{
    //PWN_DIV_64 - matches 80 MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    //enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));

    //calculate period for 80 MHz
    ulPeriod = (SysCtlClockGet()/64) / 50;

    //configure for PB6
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    //setup generator 0 in count down mode
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}

// Trigger function
int trigger(int ticks, int current_state)
{
    // buffer prevents flickering if noise briefly stops (due to mic sensitivity)
    static int silence_buffer = 0;

    bool person_present = (~GPIO_PORTF_DATA_R & 0x02); //read IR sensor input
    bool noise_detected = (~GPIO_PORTF_DATA_R & 0x08); //read microphone input

    // if IR sensor doesn't detect anyone, system goes to off state
    if (!person_present) {
        silence_buffer = 0;
        return 0; // OFF
    }

    // WARNING state
    if (current_state == WARN) {
        // if it then becomes quiet, wait for a little bit to go back to acceptable
        if (!noise_detected) {
            silence_buffer++;
            if (silence_buffer > 15) { // ~1 second of silence
                silence_buffer = 0;
                return 1; // back to QUIET
            }
        } else {
            silence_buffer = 0; // noise still happening, reset buffer
        }

        // if count down expires, move to LOUD
        if (ticks <= 0) {
            silence_buffer = 0;
            return 3; // go LOUD
        }
        return 2; // Stay in WARN
    }

    //for loud state
    if (current_state == LOUD) {

       // stay in LOUD if noise is still detected
       if (noise_detected) {
            return 3;
            //if count down runs out and noise is no longer detected, go to 1
            //this specific statement doesn't get reached, else if is what gets read
//            if (ticks <=0 && !noise_detected){
//                return 1;
//            }
        }

       //if count down runs out and noise is no longer detected, go to 1
       else if (ticks <=0 && !noise_detected){
                       return 1;
                   }


        else {
            // if it becomes quiet, go back to WARN first to "cool down"
            // prevents bouncing between yellow and green
            return 2;
        }

    }

    //logic for quiet state
    silence_buffer = 0;
    if (noise_detected) {
        return 2; // if noise detected, start the timer
    }

    return 1; // Stay in QUIET
}

//Servo function to update servo position
void servo_move(uint32_t angle)
{
    uint32_t min_pulse  = ulPeriod / 25; // 1ms pulse  = 0 degrees
    uint32_t max_pulse  = ulPeriod / 8; // 2ms pulse  = 180 degrees
    uint32_t pulse_width = min_pulse + ((angle * (max_pulse - min_pulse)) / 180);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulse_width);
}

//watchdog setup
void watchdog_setup(void) {
    // enable watchdog  0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0));

    // check if watchdog is locked and unlock it
    if(WatchdogLockState(WATCHDOG0_BASE) == true) {
        WatchdogUnlock(WATCHDOG0_BASE);
    }

    // set reload value - 2 ms time out
    WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet() * 2);

    // enable Reset (This ensures the MCU reboots if timer hits 0)
    WatchdogResetEnable(WATCHDOG0_BASE);

    // stall timer if code is paused
    WatchdogStallEnable(WATCHDOG0_BASE);

    // enable watchdog
    WatchdogEnable(WATCHDOG0_BASE);
}






