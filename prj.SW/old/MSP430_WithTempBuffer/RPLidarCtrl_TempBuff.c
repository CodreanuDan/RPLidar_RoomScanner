/*********************************************************

               MSP430FR2355
            -----------------
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

 *** >>Baud Rate A1 PC    @ 115200bps
 *** >>Baud Rate A0 LIDAR @ 115200bps
 ***********************************************************/


#include <msp430.h>

#define SIZE_BUFF 5

volatile unsigned char get_health_status[2]={0xA5,0x52};                        // get health request
volatile unsigned char stop_scan[2]={0xA5,0x25};                                // stop scan request
volatile unsigned char reset_scan[2]={0xA5,0x40};                               // reset request
volatile unsigned char start_scan[2]={0xA5,0x20};                               // start scan request
volatile unsigned char start_scan_resp[7]={0xA5,0x5A,0x05,0x00,0x00,0x40,0x81}; // start scan respons
volatile unsigned char force_scan[2]={0xA5,0x21};                               // force scan request
volatile unsigned char express_scan[2]={0xA5,0x82};                             // express scan request

volatile unsigned char get_info[2]={0xA5,0x50};                                 // get info request
volatile unsigned char get_sample_rate[2]={0xA5,0x59};                          // get sample rate request
volatile unsigned char get_lidar_conf[2]={0xA5,0x84};                           // get lidar conf request


volatile unsigned char express_scan_legacy[9]={0xA5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22};   // express scan legacy request
volatile unsigned char express_scan_legacy_resp[7]={0xA5,0x5A,0x54,0x00,0x00,0x40,0x82};        // express scan legacy response

volatile unsigned char express_scan_extended[9]={0xA5,0x82,0x05,'M',0x00,0x00,0x00,0x00,'C'};   // express scan legacy request
                                                                                                // M modurile disponibile
                                                                                                // C checksum ?
volatile unsigned char express_scan_extended_resp[7]={0xA5,0x5A,0x84,0x00,0x00,0x40,0x84};      // express scan legacy response

volatile unsigned char express_scan_dens[9]={0xA5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22};     // express scan legacy request
volatile unsigned char express_scan_dens_resp[7]={0xA5,0x5A,0x54,0x00,0x00,0x40,0x85};          // express scan legacy response

volatile unsigned char tempBuff[5]={0x00,0x00,0x00,0x00,0x00};

// dense_data response:
// Sync + StartAngle
// byte 0: sync1[7:4] <-> ChkSum[3:0]
// byte 1: sync2[7:4] <-> ChkSum[7:4]
// byte 2: start_angle_q6[7:0]
// byte 3: S[7] <-> start_angle_q6[14:8]

// Distance Data
// bytes 4 + 5: cabin[0] = distance_msb[15:8]<->[7:0]lsb_distance = distance[15:0]
// bytes 6 + 7: cabin[1] = distance_msb[15:8]<->[7:0]lsb_distance = distance[15:0]
//..
// bytes 82+83: cabin[40] = distance_msb[15:8]<->[7:0]lsb_distance = distance[15:0]

volatile unsigned char frame_count;
volatile unsigned char cabin_count;
volatile unsigned char rec_byte_count;
char sizebuf = 0;
int j;

volatile unsigned i;
void Init_GPIO();

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                 // Stop watchdog timer

    /******* Variabile **********/
    i=0;  frame_count=0;  cabin_count=0;  rec_byte_count=0;

    /******* Configure GPIO *****/
    //Init_GPIO();

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
    P4OUT |= BIT1;                          // Configure P1.3 as pulled-up
    P4REN |= BIT1;                          // P1.3 pull-up register enable
    P4IES |= BIT1;                          // P1.3 Hi/Low edge
    P4IE |= BIT1;                           // P1.3 interrupt enabled
    /************* Configuram P2.3  ca buton ********************/
    P2OUT |= BIT3;                          // Configure P1.3 as pulled-up
    P2REN |= BIT3;                          // P1.3 pull-up register enable
    P2IES |= BIT3;                          // P1.3 Hi/Low edge
    P2IE |= BIT3;                           // P1.3 interrupt enabled


    /******************** Configure UART UCA1 PC  ***************/
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL_2;    // UCSSEL_1 set ACLK as BRCLK=32768Hz

    /***************** Configure UART UCA0 LIDAR ****************/
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL_2;

    // UCSSEL_2 set SMCLK
    // Baud Rate calculation. Referred to UG 17.3.10
    // (1) N=32768/4800=6.827
    // (2) OS16=0, UCBRx=INT(N)=6
    // (4) Fractional portion = 0.827. Refered to UG Table 17-4, UCBRSx=0xEE.
    // 16 biti UCA1BRW = 8 biti UCA1BR1  8 biti UCA1BR0

    /*
        // 4800bps @ ACLK = 32768 Hz
        UCA1BR0 = 0x06;                              // INT(32768/4800)
        UCA1BR1 = 0x00;
        UCA1MCTLW = 0xEE00;
    */

    /*
        // 9600bps @ 1MHz
        UCA1BR0 = 0x06;                              // INT(32768/4800)
        UCA1BR1 = 0x00;
        UCA1MCTLW = 0x1181; // din calcule 0x1181 -> real 0x2081
    */

    /******>>>> Baud Rate 115200bps <<<<**********************************/

    /*
    * UCOS16 -> 1
    UCBRx -> 10
    UCBRFx -> 13
    UCBRSx ->0xAD

    UCAxMCTLW = UCBRSx + UCBFRx + UCOS16
    UCAxMCTLW = 0xADD1

    UCAxBRW = UCBRx
    UCAxBRW = 0x000A
    */

    // Baud Rate A1 PC @ 115200bps
    UCA1BR0 = 0x0A;// @115200bps
    UCA1BR1 = 0x00;// @115200bps
    UCA1MCTLW = 0xADD1; // @115200bps

    // Baud Rate A0 LIDAR @ 115200bps
    UCA0BR0 = 0x0A;// @115200bps
    UCA0BR1 = 0x00;// @115200bps
    UCA0MCTLW = 0xADD1; // @115200bps
//
    UCA1CTLW0 &= ~UCSWRST;                      // Initialize eUSCI
    UCA1IE |= UCRXIE;                           // Enable USCI_A1 RX interrupt

    UCA0CTLW0 &= ~UCSWRST;                      // Initialize eUSCI
    UCA0IE |= UCRXIE;                           // Enable USCI_A0 RX interrupt

    P4IFG &= ~BIT1;                             // P4.1 IFG cleared
    P2IFG &= ~BIT3;                             // P2.3 IFG cleared
    __bis_SR_register(GIE);                     // Enter LPM3, interrupts enabled
    __no_operation();                           // For debugger
}

/***  Port 4 interrupt service routine ----> STOP SCAN ***/

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    P2IFG &= ~BIT3;                         // Clear P2.3 IFG

    for(i=0;i<2;i++)
     {
         while(!(UCA1IFG&UCTXIFG));         // trimite catre PC
         UCA1TXBUF = stop_scan[i];
     }

    for(i=0;i<2;i++)
     {
         while(!(UCA0IFG&UCTXIFG));          // verifica daca nu se transmite ceva
         UCA0TXBUF = stop_scan[i];
     }
}

/*** Port 4 interrupt service routine ----> START SCAN; Sequence: Get Health + Start scan ***/

#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{
    P4IFG &= ~BIT1;                         // Clear P4.1 IFG
/*
    for(i=0;i<2;i++)
     {
         while(!(UCA1IFG&UCTXIFG));// trimite catre PC
         UCA1TXBUF = get_health_status[i];
     }
*/
    for(i=0;i<2;i++)
    {
        while(!(UCA0IFG&UCTXIFG));          // verifica daca nu se transmite ceva
        UCA0TXBUF = get_health_status[i];
    }

   for(i=0;i<2;i++)
    {
    while(!(UCA0IFG&UCTXIFG));              // verifica daca nu se transmite ceva
        UCA0TXBUF = start_scan[i];
    }

}

/********************* Rutina de tratare a intreruperilor UART A1 PC **********************/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)

{
switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
{
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
            if (sizebuf == 5)
            {
                while(!(UCA1IFG&UCTXIFG))
                {
                    for (j = 0; j < SIZE_BUFF; j++)
                    {
                        UCA1TXBUF = tempBuff[j];
                    }
                }
                sizebuf = 0;
            }
            __no_operation();
        break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
    }
}

/******************* Rutina de tratare a intreruperilor UART A0 LIDAR *********************/
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)

{
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
         while(!(UCA1IFG&UCTXIFG))
         {
             UCA1TXBUF = UCA0RXBUF;
             tempBuff[sizebuf] = UCA1TXBUF;km '/ '
             sizebuf++;
         }
         __no_operation();
          break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
