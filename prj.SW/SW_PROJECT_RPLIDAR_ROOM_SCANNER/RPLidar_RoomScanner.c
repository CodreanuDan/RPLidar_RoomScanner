/*********************************************************

               MSP430FR2355
          ^  -----------------
         /|\|                 |
          | |                 |
          --|RST              |
            |                 |
            |                 |
            |     P4.3/UCA1TXD|----> PC (echo)
            |     P4.2/UCA1RXD|<---- PC
            |                 |
            |     P1.7/UCA0TXD|----> LIDAR Rx
            |     P1.6/UCA0RXD|<---- LIDAR Tx
            |                 |
            |     P4.1/       |<---- Buton get_info, start
            |     P2.3/       |<---- Buton force_start
            |                 |
            |     P1.2/       |----> Motor Lidar
            |                 |
            |     P2.0/       |----> Servo Turret
            | TB.0 >> Timing  |
            | TB.1 >> Servo   |
            ------------------
 *** >>Baud Rate A1 PC    @ 115200bps
 *** >>Baud Rate A0 LIDAR @ 115200bps
 ***********************************************************/
#include <msp430.h>
#include <stdint.h>

/*
 * Settings
 */
#define NEW_UART1_TX 1
#define OLD_UART1_TX 0

#define TIMER_B_1_SEC 0
#define TIMER_B_2_SEC 1

/*
 * LIDAR COMMANDS AND RESPONSE FORMATS
 */

const unsigned char get_health_status[2]={0xA5,0x52};                                                       // get health request
const unsigned char start_scan[2]       ={0xA5,0x20};                                                       // start scan request
const unsigned char stop_scan[2]        ={0xA5,0x25};                                                       // stop scan request

const unsigned char get_health_status_resp[10] = {0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x0, 0x00, 0x00};   // get health status response
const unsigned char start_scan_resp[7]  = {0xA5,0x5A,0x05,0x00,0x00,0x40,0x81};                                 // start scan response

const unsigned char end_marker[4] = {0xFF,0xFF,0xFF,0xFF};

/*
 * SERVO TIMING VALUES AND COUNTER
 */

/* --- Servo timing (pentru SMCLK = 20MHz cu prescaler /8) --- */
#define SERVO_TIMER_PERIOD_COUNTS 50000u   // TB1CCR0 -> 20 ms @ (20MHz/8) = 2.5MHz => 2.5e6 * 0.02 = 50000
#define SERVO_MIN_PULSE_COUNTS    1250u    // 0.5 ms
#define SERVO_MAX_PULSE_COUNTS    6250u    // 2.5 ms


#define SG90_180DEG 2750u
#define SG90_0DEG   730u
#define SG90_1DEG_INC  11u
volatile uint8_t servo_pos;

/*
 * Global Varibles
 */
volatile uint8_t i;


/*
 * Lidar Ctrl control states
 */
typedef enum {
    LIDAR_STATE_STOP = 0u,
    LIDAR_STATE_IDLE,
    LIDAR_STATE_MEAS
} Lidar_States;

volatile Lidar_States prevLidarState;
volatile Lidar_States nextLidarState;

/*
 * Command messages from PC
 */
#define STOP_MEASUREMENT_CMD 0x00u
#define START_MEASUREMENT_CYCLE_CMD 0x01u
#define RESUME_MEASUREMENT_CMD 0x02u

/*
 * Config Function prototypes
 */
void Configuration_Callback(void(*fptr)(void));
void Configure_TimerB();
void Configure_WDT();
void Configure_CS();
void Configure_UART0_LIDAR_115200();
void Configure_UART1_PC_115200();
void Configure_GPIO();

/*
 * LIDAR Measurement Control Functions
 */
void LidarCtrl_StopMeasurement();
void LidarCtrl_StartMeasurement();
void LidarCtrl_MainFunction();

/*
 * Servo Motor Control Functions
 */
void ServoCtrl_setAngle(uint8_t degrees);
void ServoCtrl_testRange(int speed);
void delay_ms(uint16_t ms);

/**
 * main.c
 */
int main(void)
{
    /******************* Variabile *********************/
    i = 0;
    servo_pos = 0;
    prevLidarState = LIDAR_STATE_IDLE;
    nextLidarState = LIDAR_STATE_IDLE;

    /************ CLEAR UART TX BUFFERS ****************/

    /**************** Configurare WDT ******************/
    Configuration_Callback(Configure_WDT);

    /******** Configurare CS SMCLK=1MHz CS_09.c ********/
    Configuration_Callback(Configure_CS);

    /************* Configurare Power Mode **************/
    PM5CTL0 &= ~LOCKLPM5; // Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    /**************** Configurare Timere ***************/
    Configuration_Callback(Configure_TimerB);

    /**************** Configurare GPIO *****************/
    Configuration_Callback(Configure_GPIO);

    /********** Configurare UART UCA1 PC ***************/
    Configuration_Callback(Configure_UART1_PC_115200);

    /********** Configurare UART UCA0 LIDAR ************/
    Configuration_Callback(Configure_UART0_LIDAR_115200);

    /********** Curatare Intreruperi si IE *************/
    P4IFG &= ~BIT1;                             // P4.1 IFG cleared
    P2IFG &= ~BIT3;                             // P2.3 IFG cleared
    __enable_interrupt();                       // Interrupts enabled
    __no_operation();                           // For debugger


    /* Test the servo */
    ServoCtrl_testRange(25);

    while(1)
    {
        LidarCtrl_MainFunction();
    }
}

/*
 * HELPERS
 */
void Configuration_Callback(void(*fptr)(void))
{
    (*fptr)();
}

/*
 * **********************************************************************************************
 *                                     GPIO CONFIG & GPIO ISR
 * **********************************************************************************************
 */

void Configure_GPIO()
{
    /********* Configuram P3.0 ca MCLK si P3.4 ca SMCLK ********/
    P3DIR |= BIT0 | BIT4;                               // pini de iesire digitala
    P3SEL0 |= BIT0 | BIT4;                              // selectam primary function (P3.0 ca MCLK si P3.4 ca SMCLK)

    /********** Configure UART pins ****************************/
    // P1SEL0 |= BIT6 | BIT7;                           // set 2-UART pin as second function
    // P4.3 -> TxD
    // P4.2 ->  RxD
    P4SEL0 |= BIT2 | BIT3;                              // selectam functiile UCA1  TxD si RxD
    P1SEL0 |= BIT6 | BIT7;                              // selectam functiile UCA0  TxD si RxD

    /************ Configuram P4.1  ca buton *********************/
    P4OUT |= BIT1;                                      // Configure P1.3 as pulled-up
    P4REN |= BIT1;                                      // P1.3 pull-up register enable
    P4IES |= BIT1;                                      // P1.3 Hi/Low edge
    P4IE |= BIT1;                                       // P1.3 interrupt enabled
    /************* Configuram P2.3  ca buton ********************/
    P2OUT |= BIT3;                                      // Configure P1.3 as pulled-up
    P2REN |= BIT3;                                      // P1.3 pull-up register enable
    P2IES |= BIT3;                                      // P1.3 Hi/Low edge
    P2IE |= BIT3;                                       // P1.3 interrupt enabled

    /*********** Configuram P1.0 --> Signal light ****************/
//    P1DIR |= BIT0;
//    P1OUT &=~BIT0;

    /*********** Configuram P6.6 --> Signal light ****************/
//    P6DIR |= BIT6;
//    P6OUT &=~BIT6;

    /*********** Configuram P1.2 --> Pin Motor Lidar *************/
    P1DIR |= BIT2;                                      // P1.2 as output
    P1OUT &= ~BIT2;
    // P1.2 OFF
    /*********** Configuram P2.0 --> Pin Motor Servo *************/
    P2DIR |= BIT0;        // P2.0 as output
    P2SEL0 |= BIT0;       // P2.0 TB1.1 option select
    P2SEL1 &= ~BIT0;

}

/***  Port 2 interrupt service routine ----> STOP SCAN ***/
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    /* Clear P2.3 IFG */
    P2IFG &= ~BIT3;

    /* SET STOP SCAN SEQUENCE FLAG */
    nextLidarState = LIDAR_STATE_STOP;

}


/*** Port 4 interrupt service routine ----> START SCAN; Sequence: Get Health + Start scan ***/
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{
    /* Clear P4.1 IFG */
    P4IFG &= ~BIT1;

    /* SET START SCAN SEQUENCE FLAG */
    nextLidarState = LIDAR_STATE_MEAS;

}


/*
 * **********************************************************************************************
 *                                           CS CONFIG
 * **********************************************************************************************
 */

void Configure_CS()
{
    /******** Configurare CS SMCLK=1MHz CS_09.c ********/
    FRCTL0 = FRCTLPW | NWAITS_2;

    __bis_SR_register(SCG0);                            // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                          // Set REFO as FLL reference source
    CSCTL0 = 0;                                         // clear DCO and MOD registers
    CSCTL1 |= DCORSEL_6;                                // Set DCO = 20MHz
    CSCTL2 = FLLD_0 + 610;                              // DCOCLKDIV = 20MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                            // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));          // FLL locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;          // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
    CSCTL5 = DIVM_0 | DIVS_0;                           // default DCOCLKDIV as MCLK and SMCLK source


    PM5CTL0 &= ~LOCKLPM5;                               // Disable the GPIO power-on default high-impedance mode
                                                        // to activate 1previously configured port settings
}


/*
 * **********************************************************************************************
 *                                     UART CONFIG & UART ISR
 * **********************************************************************************************
 */


void Configure_UART0_LIDAR_115200()
{

    /******>>>> Baud Rate 115200bps <<<<*************************
     *
     *  UCOS16 -> 1
     *  UCBRx    -> 10
     *  UCBRFx   -> 13
     *  UCBRSx   ->0xAD
     *  UCAxMCTLW = UCBRSx + UCBFRx + UCOS16
     *  UCAxMCTLW = 0xADD1
     *  UCAxBRW = UCBRx
     *  UCAxBRW = 0x000A
     */

    /***************** Configure UART UCA0 LIDAR ****************/
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL_2;    // UCSSEL_2 set SMCLK

    /* Baud Rate A0 LIDAR @ 115200bps */
    UCA0BR0 = 0x0A;                             // @115200bps
    UCA0BR1 = 0x00;                             // @115200bps
    UCA0MCTLW = 0xADD1;                         // @115200bps

    UCA0CTLW0 &= ~UCSWRST;                      // Initialize eUSCI
    UCA0IE |= UCRXIE;                           // Enable USCI_A0 RX interrupt
}

void Configure_UART1_PC_115200()
{

    /******>>>> Baud Rate 115200bps <<<<*************************
     *
     *  UCOS16 -> 1
     *  UCBRx    -> 10
     *  UCBRFx   -> 13
     *  UCBRSx   ->0xAD
     *  UCAxMCTLW = UCBRSx + UCBFRx + UCOS16
     *  UCAxMCTLW = 0xADD1
     *  UCAxBRW = UCBRx
     *  UCAxBRW = 0x000A
     */

    /******************** Configure UART UCA1 PC  ***************/
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL_2;    // UCSSEL_2 set SMCLK

    /* Baud Rate A1 PC @ 115200bps */
    UCA1BR0 = 0x0A;                             // @115200bps
    UCA1BR1 = 0x00;                             // @115200bps
    UCA1MCTLW = 0xADD1;                         // @115200bps

    UCA1CTLW0 &= ~UCSWRST;                      // Initialize eUSCI
    UCA1IE |= UCRXIE;                           // Enable USCI_A1 RX interrupt

}


/********************* Rutina de tratare a intreruperilor UART A1 {{PC <--> MSP}} ************
 *
 *
 * [UART1_TX_buffer] --> <data> copy in --> [UART1_RX_buffer] --> <data> --> [COMx_TX_buffer_PC]
 *
 */
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
#if OLD_UART1_TX == 1
    switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            while(!(UCA1IFG&UCTXIFG));          // verifica daca poate transmite catre PC ( NU-s intreruperi pe UART TX adica )
            UCA1TXBUF = UCA1RXBUF;              // echo TX-RX
            __no_operation();
            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
#endif

#if NEW_UART1_TX == 1
    switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
        {
            uint8_t command_byte = UCA1RXBUF;

            if (command_byte == START_MEASUREMENT_CYCLE_CMD)
            {
                // Cmd 0x01: Incepe primul ciclu de masurare
                nextLidarState = LIDAR_STATE_MEAS;
            }
            else if (command_byte == RESUME_MEASUREMENT_CMD)
            {
                // Cmd 0x02: Reia masuratoarea (MSP430 iese din PAUSE)
                nextLidarState = LIDAR_STATE_MEAS;
            }
            else if (command_byte == STOP_MEASUREMENT_CMD)
            {
                // Cmd 0x00: LIDAR_STATE_STOP tot
                nextLidarState = LIDAR_STATE_STOP;
            }

            while(!(UCA1IFG&UCTXIFG));          // verifica daca poate transmite catre PC ( NU-s intreruperi pe UART TX adica )
            UCA1TXBUF = command_byte;           // Echo back the command

            __no_operation();
            break;
        }
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
#endif

}




/******************* Rutina de tratare a intreruperilor UART A0 {{LIDAR <--> MSP}} ***********
 *
 *   [LIDAR_TX] --> <data> --> [UART0_RX_buffer] --> <data> copy in --> [UART1_TX_buffer]
 *
 */
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
        {
            while(!(UCA1IFG&UCTXIFG));     // verifica daca poate transmite catre PC ( NU-s intreruperi pe UART RX adica )
            UCA1TXBUF = UCA0RXBUF;         // transmite catre UART 1 TX pachetul receptionat de la LIDAR prin UART0
            __no_operation();
            break;
        }
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}


/*
 * **********************************************************************************************
 *                                     TIMER CONFIG & TIMER ISR
 * **********************************************************************************************
 */


/*
 * Init_TimerB
 */
void Configure_TimerB()
{

    /****** TB.0 -> Flow control/ Meaurement cycle *******/
#if TIMER_B_1_SEC == 1
    // Configure Timer_B0 in Stop Mode until measurement is triggered
    TB0CTL = TBSSEL_1 | MC_0 | TBCLR; // ACLK, Stop mode: Timer is halted, clear timer

    /* Set CCR0 for desired period (e.g., 1 Hz: 32768 / 1 = 32768) */
    TB0CCR0 = 32768; // Period = 1 second (adjust as needed)
#endif

#if TIMER_B_2_SEC == 1
    // Configure Timer_B0 in Stop Mode until measurement is triggered
    TB0CTL = TBSSEL_1 | ID__2 | MC_0 | TBCLR; // ACLK, Stop mode: Timer is halted, clear timer, ID_2 for 2 second cycle

    /* Set CCR0 for desired period (e.g., 0.5 Hz: 32768 / 1 = 32768) */
    TB0CCR0 = 32768; // Period = 2 second (adjust as needed)
#endif

    // Enable CCR0 interrupt
    TB0CCTL0 = CCIE;

    /****** TB.1 -> Servo Control *******/
    TB1CCR0 = SERVO_TIMER_PERIOD_COUNTS;   // PWM period = 20ms (50Hz) at SMCLK/8
    TB1CCTL1 = OUTMOD_7;                   // Reset/set mode
    TB1CCR1 = SERVO_MIN_PULSE_COUNTS + ((SERVO_MIN_PULSE_COUNTS + SERVO_MAX_PULSE_COUNTS)/2); // center-ish (option)
    // SMCLK, up mode, divide by 8
    TB1CTL = TBSSEL_2 | ID_3 | MC_1 | TBCLR;   // TBSSEL_2 = SMCLK, ID_3 => /8, MC_1 => up

}


/*
 * TIMER0_B0_VECTOR_ISR
 */
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer_B (void)
{

    /* SET STOP SCAN SEQUENCE FLAG */
    nextLidarState = LIDAR_STATE_IDLE;

    /***********************************************************
     * Stop Timer B from counting after the scan is stopped
     ***********************************************************/
    TB0CTL &= ~ (MC_1 | MC_2 | MC_3);           // Clear old MC bits
    TB0CTL |=  MC_0;                            // Stop mode: Timer is halted
}


/*
 * **********************************************************************************************
 *                                     WDT CONFIG & WDT ISR
 * **********************************************************************************************
 */

void Configure_WDT()
{
    WDTCTL = WDTPW | WDTHOLD;                 // Stop watchdog timer
}



/*
 * **********************************************************************************************
 *                                     LIDAR CTRL FUNCTIONS
 * **********************************************************************************************
 */

/*
 * Function name: LidarCtrl_StopMeasurement
 * Stop scan sequence:
 * 1. Send STOP CMD 0xA5 0x25
 * 2. Stop Lidar Motor
 */
void LidarCtrl_StopMeasurement()
{
    /*** STOP SCAN COMMAND : [0xA5 0x25] --> UART1: PC; UART0: LIDAR ***/
    for(i = 0; i < 2; i++)
    {
        while(!(UCA1IFG&UCTXIFG));              // verifica daca nu se transmite ceva ( NU-s intreruperi pe UART adica )
        UCA1TXBUF = stop_scan[i];               // trimite catre PC
    }

    for(i = 0; i < 2; i++)
    {
        while(!(UCA0IFG&UCTXIFG));              // verifica daca nu se transmite ceva ( NU-s intreruperi pe UART adica )
        UCA0TXBUF = stop_scan[i];               // trimite catre LIDAR
    }

    /* Send end_marker to PC */
    for (i = 0; i < 4; i++)
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = end_marker[i];
    }

    servo_pos = 0;

    P1OUT &= ~BIT2;                             // STOP LIDAR MOTOR
}

/*
 * Function name: LidarCtrl_StartMeasurement
 * Start scan sequence:
 * 1. Start Lidar Motor
 * 2. Send GET HEALTH STATUS CMD 0xA5 0x52
 * 3. Send START SCAN CMD 0xA5 0x20
 * 4. Strat Timer B counter mode
 */
void LidarCtrl_StartMeasurement()
{
    P1OUT |= BIT2;                              // Start LIDAR Motor

    /*** GET HEALTH STATUS COMMAND : [0xA5 0x52] --> UART1: PC; UART0: LIDAR ***/
    for(i = 0; i <2 ; i++)
    {
        while(!(UCA1IFG&UCTXIFG));              // verifica daca nu se transmite ceva ( NU-s intreruperi pe UART adica )
        UCA1TXBUF = get_health_status[i];       // trimite catre PC
    }

    for(i = 0; i < 2; i++)
    {
        while(!(UCA0IFG&UCTXIFG));              // verifica daca nu se transmite ceva ( NU-s intreruperi pe UART adica )
        UCA0TXBUF = get_health_status[i];       // trimite catre LIDAR
    }

    /*** START SCAN COMMAND : [0xA5 0x20] --> UART1: PC; UART0: LIDAR **********/
    for(i = 0; i < 2; i++)
    {
        while(!(UCA1IFG&UCTXIFG));              // verifica daca nu se transmite ceva ( NU-s intreruperi pe UART adica )
        UCA1TXBUF = start_scan[i];              // trimite catre PC
    }

    for(i = 0; i < 2; i++)
    {
        while(!(UCA0IFG&UCTXIFG));              // verifica daca nu se transmite ceva ( NU-s intreruperi pe UART adica )
        UCA0TXBUF = start_scan[i];              // trimite catre LIDAR
    }

    /**********************************************************
     * Start Timer B in UP mode to run for 1 second.
     **********************************************************/
     TB0CTL |= MC_1;                             // Set to Up mode (Timer starts counting)
}


/*
 * Function name: LidarCtrl_MainFunction
 *
 */
void LidarCtrl_MainFunction()
{
    /* Check if a state transition occurs */
    if (prevLidarState != nextLidarState)
    {
        prevLidarState = nextLidarState;

        switch(prevLidarState)
        {
            case LIDAR_STATE_IDLE:
            {
                ServoCtrl_setAngle(0);
                LidarCtrl_StopMeasurement();
                break;
            }
            case LIDAR_STATE_MEAS:
            {
                LidarCtrl_StartMeasurement();
                break;
            }
            case LIDAR_STATE_STOP:
                ServoCtrl_setAngle(0);
                LidarCtrl_StopMeasurement();
                break;
            default:
                break;
        }
    }
    else
    {
        __no_operation();
    }
}

/*
 * **********************************************************************************************
 *                                     SERVO CTRL FUNCTIONS
 * **********************************************************************************************
 */

/*
 * Function name: ServoCtrl_setAngle
 *
 */
void ServoCtrl_setAngle(uint8_t degrees)
{
    /* degrees: 0 .. 180
     * Map to pulse between SERVO_MIN_PULSE_COUNTS (1ms) and SERVO_MAX_PULSE_COUNTS (2ms)
     * Using 32-bit intermediate to avoid overflow in multiplication.
     */
    if (degrees > 180) degrees = 180;

    uint32_t range = (uint32_t)SERVO_MAX_PULSE_COUNTS - (uint32_t)SERVO_MIN_PULSE_COUNTS;
    uint32_t ccr = (uint32_t)SERVO_MIN_PULSE_COUNTS + ((uint32_t)degrees * range) / 180u;
    TB1CCR1 = (uint16_t) ccr;
}

/* Simple test  */
void ServoCtrl_testRange(int speed)
{
    for (i = 0; i < 180; i++)
    {
        ServoCtrl_setAngle(i);
        delay_ms(speed);
    }

    for (i = 180; i > 0 ; i--)
    {
        ServoCtrl_setAngle(i);
        delay_ms(speed);
    }
}

/* Simple delay in ms */
void delay_ms(uint16_t ms)
{
    while(ms--)
        __delay_cycles(20000);   // 20,000 cycles = 1 ms @ 20 MHz
}
