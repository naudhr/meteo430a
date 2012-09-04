//#include <msp430g2553.h>
#include <msp430.h>

#define FLUEGER_FOTO_1 BIT4
#define FLUEGER_FOTO_2 BIT5
#define ANEM_FOTO BIT3
#define Seconds60 60

unsigned char fl_reg;
unsigned char flueger_state;
unsigned char flueger_average;

unsigned int anem_pin_counter;
unsigned int anem_pin_second;
unsigned int anem_average;
unsigned char uart_byte;

void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;

    //    BCSCTL1 = CALBC1_1MHZ;
    //    DCOCTL = CALDCO_1MHZ;

    P1OUT = 0;
    P1DIR = 0xFF; // (BIT0 | BIT6); // according to the user guide p340
    P1DIR &= ~(FLUEGER_FOTO_1 | FLUEGER_FOTO_2);
    //    P1REN |= BIT3;
    /*
      while(1) // pin state echo monitor - delete it later
       {
       if(P1IN & FLUEGER_FOTO_1)
       P1OUT |= BIT0;
       else
       P1OUT &= ~BIT0;
       if(P1IN & FLUEGER_FOTO_2)
       P1OUT |= BIT6;
       else
       P1OUT &= ~BIT6;
       }
    */
    P1IES &= ~(FLUEGER_FOTO_1 | FLUEGER_FOTO_2 | ANEM_FOTO); // low -> high is selected with IES.x = 0
    P1IFG &= ~(FLUEGER_FOTO_1 | FLUEGER_FOTO_2 | ANEM_FOTO); // To prevent an immediate interrupt, clear the flag before enabling the interrupt
    // P1IE  |=  (FLUEGER_FOTO_1 | FLUEGER_FOTO_2 | ANEM_FOTO); // Enable interrupts for the selected pins

    TA0CCR0 = 512;
    TA0CCTL0 = CCIE;
    TA0CTL = TASSEL_1 | MC_1 | ID_3 | TACLR;

    BCSCTL3 = LFXT1S_0 | XCAP_3;
    BCSCTL1 |= DIVA_3;

    P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2;
    UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
    UCA0BR0 = 0x03;                           // 32kHz/9600 = 3.41
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS1 + UCBRS0;               // Modulation UCBRSx = 3
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    // IE2 |= UCA0TXIE;               // Enable USCI_A0 TX interrupt

    flueger_state = 0;
    anem_pin_counter = 0;
    anem_pin_second = 0;

    fl_reg = P1IN & (FLUEGER_FOTO_1 | FLUEGER_FOTO_2);

    _BIS_SR(GIE + LPM3_bits);
}

__attribute__((interrupt (PORT1_VECTOR)))
void P1_isr(void)
{
    // FLUEGER_* handling assumes the detector following the switching 1/3 mode
    if(P1IFG & FLUEGER_FOTO_1)
    {
        if(P1IES & FLUEGER_FOTO_1) // was catching the falling edge
        {
            fl_reg &= ~FLUEGER_FOTO_1;
            if(fl_reg & FLUEGER_FOTO_2)
                flueger_state += 3;
            else
                flueger_state -= 3;
        }
        else
        {
            fl_reg |= FLUEGER_FOTO_1;
            if(fl_reg & FLUEGER_FOTO_2)
                flueger_state -= 3;
            else
                flueger_state += 3;
        }
        P1OUT ^= BIT0;
        P1IES ^= FLUEGER_FOTO_1; // swith the edge detection
        P1IFG &= ~FLUEGER_FOTO_1;
    }
    if(P1IFG & FLUEGER_FOTO_2)
    {
        if(P1IES & FLUEGER_FOTO_2) // was catching the falling edge
        {
            fl_reg &= ~FLUEGER_FOTO_2;
            if(fl_reg & FLUEGER_FOTO_1)
                flueger_state -= 3;
            else
                flueger_state += 3;
        }
        else
        {
            fl_reg |= FLUEGER_FOTO_2;
            if(fl_reg & FLUEGER_FOTO_1)
                flueger_state += 3;
            else
                flueger_state -= 3;
        }
        P1OUT ^= BIT6;
        P1IES ^= FLUEGER_FOTO_2; // swith the edge detection
        P1IFG &= ~FLUEGER_FOTO_2;
    }
    if(flueger_state > 360)
        flueger_state -= 360;
    if(flueger_state < 0)
        flueger_state += 360;

    if(P1IFG & ANEM_FOTO)
    {
        anem_pin_counter ++;
        if(anem_pin_counter == 0) // overflown
        {
            anem_pin_counter = 0xFFFF;
            P1IE &= ~ANEM_FOTO;
        }
        P1IFG &= ~ANEM_FOTO;
    }
    P1IFG = 0;
}


__attribute__((interrupt (TIMER0_A0_VECTOR)))
void CCR0_isr(void)
{
    anem_pin_second++;
    if(anem_pin_second >= Seconds60)
    {
        flueger_average = flueger_state;
        anem_average = anem_pin_counter / Seconds60;

        anem_pin_counter = 0;
        anem_pin_second = 0;

        uart_byte = 0;

        unsigned char one_char = '0';
        while(anem_average > 10000)
        {
            one_char ++;
            anem_average -= 10000;
        }
        UCA0TXBUF = one_char;
        IE2 |= UCA0TXIE;  // Enable USCI_A0 TX interrupt

        P1IE  |= (FLUEGER_FOTO_1 | FLUEGER_FOTO_2 | ANEM_FOTO);
    }
}

// USCI A0/B0 Transmit ISR
__attribute__ ((interrupt (USCIAB0TX_VECTOR)))
void USCI0TX_isr(void)
{
    uart_byte ++;
    unsigned char one_char = ' ';
    if(uart_byte > 17)
        IE2 &= ~UCA0TXIE;
    else if(uart_byte == 17)
        one_char = '\n';
    else if(uart_byte == 16)
        one_char = '\r';
    else if(uart_byte < 5)
    {
        const unsigned int degree = uart_byte == 4 ? 1 : uart_byte == 3 ? 10 : uart_byte == 2 ? 100 : 1000;
        one_char = '0';
        while(anem_average > degree)
        {
            one_char ++;
            anem_average -= degree;
        }
    }
    else if(uart_byte == 5)
        one_char = ' ';
    else if(uart_byte < 9)
    {
        const unsigned char degree = uart_byte == 8 ? 1 : uart_byte == 7 ? 10 : 100;
        one_char = '0';
        while(flueger_average > degree)
        {
            one_char ++;
            flueger_average -= degree;
        }
    }
    UCA0TXBUF = one_char;
}

