t/*********************************************************

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
 *** >>Baud Rate A1 PC    @ 115200bps
 *** >>Baud Rate A0 LIDAR @ 115200bps
 ***********************************************************/
#include <msp430.h>
#include <stdint.h>

/*
 * LIDAR COMMANDS AND RESPONSE FORMATS
 */

const unsigned char get_health_status[2]={0xA5,0x52};                                                       // get health request
const unsigned char start_scan[2]       ={0xA5,0x20};                                                       // start scan request
const unsigned char stop_scan[2]        ={0xA5,0x25};                                                       // stop scan request

const unsigned char get_health_status_resp[10] = {0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, 0x0, 0x00, 0x00};   // get health status response
const unsigned char start_scan_resp[7]  = {0xA5,0x5A,0x05,0x00,0x00,0x40,0x81};                                 // start scan response

/*
 * Varibles
 */
volatile uint8_t i = 0;

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

/**
 * main.c
 */
int main(void)
{
    /******************* Variabile *********************/


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
    P1DIR |= BIT0;
    P1OUT &=~BIT0;
}

/***  Port 2 interrupt service routine ----> STOP SCAN ***/
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    P2IFG &= ~BIT3;                             // Clear P2.3 IFG

    /*** GET HEALTH STATUS COMMAND : [0xA5 0x52] --> UART1: PC; UART0: LIDAR ***/
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
}



/*** Port 4 interrupt service routine ----> START SCAN; Sequence: Get Health + Start scan ***/
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{
    P4IFG &= ~BIT1;                             // Clear P4.1 IFG

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
switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
{
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
        while(!(UCA1IFG&UCTXIFG));          // verifica daca poate transmite catre PC ( NU-s intreruperi pe UART TX adica )
        UCA1TXBUF = UCA1RXBUF;              // transmite catre PC pachetul receptionat de la UART0 care a receptionat de la LIDAR
        __no_operation();
        break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
    }
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
            while(!(UCA1IFG&UCTXIFG));     // verifica daca poate transmite catre PC ( NU-s intreruperi pe UART RX adica )
            UCA1TXBUF = UCA0RXBUF;         // transmite catre UART 1 TX pachetul receptionat de la LIDAR prin UART0
            __no_operation();
            break;
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
    // Configure Timer_B0 in up mode
    TB0CTL = TBSSEL_1 | MC_1 | TBCLR; // ACLK, up mode, clear timer

    // Set CCR0 for desired period (e.g., 1 Hz: 32768 / 1 = 32768)
    TB0CCR0 = 32768; // Period = 1 second (adjust as needed)

    // Enable CCR0 interrupt
    TB0CCTL0 = CCIE;

}


/*
 * TIMER0_B0_VECTOR_ISR
 */
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer_B (void)
{
    P1OUT ^= BIT0;
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
