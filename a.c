//#include <msp430g2553.h>
#include <msp430.h>

#include "pff2a/pff.h"

#define FLUEGER_FOTO_1 BIT6
#define FLUEGER_FOTO_2 BIT7
#define ANEM_FOTO BIT3
#define HEATER BIT0
#define Seconds60 60
#define DataBufLen 30

//unsigned char arch_write_to, arch_uart_from;
//struct {  unsigned int flueger, anem;  } arch[60];

unsigned char fl_reg;
unsigned char anem_pin_second;

struct DumpData {
    unsigned int anem;
    unsigned char flug;
    char tC;
} data_buf[DataBufLen];
unsigned char data_index;

//-----------------------------------------------------------------------

static void dump_to_sd(void);

//-----------------------------------------------------------------------

void main(void)
{
    WDTCTL = WDTPW + WDTHOLD;

    P1OUT = 0;
    P1DIR = 0xFF ^ (FLUEGER_FOTO_1 | FLUEGER_FOTO_2 | ANEM_FOTO); // according to the user guide p340
    P1REN = (FLUEGER_FOTO_1 | FLUEGER_FOTO_2 | ANEM_FOTO); // as phototransistors will set pins high
    fl_reg = P1IN & (FLUEGER_FOTO_1 | FLUEGER_FOTO_2);
    P1IES &= ~(fl_reg | ANEM_FOTO); // low -> high is selected with IES.x = 0
    P1IFG &= ~(FLUEGER_FOTO_1 | FLUEGER_FOTO_2 | ANEM_FOTO); // To prevent an immediate interrupt, clear the flag before enabling the interrupt

    TA0CCR0 = 4096;
    //TA0CCR1 = 200;
    TA0CCTL0 = CCIE | OUTMOD_3; // making a set/reset sequence to feed adc10 trigger
    TA0CTL = TASSEL_1 | MC_1 | ID_3 | TACLR;

    BCSCTL3 = LFXT1S_0 | XCAP_3;
    //BCSCTL1 |= DIVA_3;

    ADC10CTL1 = INCH_10 + ADC10DIV_7 + CONSEQ_2 + SHS_2 + ADC10SSEL_3; // Temp Sensor ADC10CLK/4 ACLK Repeat-single-channel Sampling on signal from TA.0
    ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ENC;

    P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
    P1SEL2 = BIT1 + BIT2;
    UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
    UCA0BR0 = 0x03;                           // 32kHz/9600 = 3.41
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS1 + UCBRS0;               // Modulation UCBRSx = 3 ( == (3.41-3)*8  p432 )
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

    while(1)
    {
        data_index = 0;
        for(unsigned char i=0; i<DataBufLen; ++i)
            data_buf[i].anem = data_buf[i].flug = data_buf[i].tC = 0;
        anem_pin_second = 0;

        _BIS_SR(GIE + LPM3_bits);
        dump_to_sd();
    }
}

//-----------------------------------------------------------------------

__attribute__((interrupt (PORT1_VECTOR)))
void P1_isr(void)
{
    struct DumpData* d = data_buf + data_index;
    // FLUEGER_* handling assumes the detector following the switching 1/3 mode
    if(P1IFG & FLUEGER_FOTO_1)
    {
        if(P1IES & FLUEGER_FOTO_1) // was catching the falling edge
        {
            fl_reg &= ~FLUEGER_FOTO_1;
            if(fl_reg & FLUEGER_FOTO_2)
                d->flug ++;
            else
                d->flug --;
        }
        else
        {
            fl_reg |= FLUEGER_FOTO_1;
            if(fl_reg & FLUEGER_FOTO_2)
                d->flug --;
            else
                d->flug ++;
        }
        P1IES ^= FLUEGER_FOTO_1; // swith the edge detection
        P1IFG &= ~FLUEGER_FOTO_1;
    }
    if(P1IFG & FLUEGER_FOTO_2)
    {
        if(P1IES & FLUEGER_FOTO_2) // was catching the falling edge
        {
            fl_reg &= ~FLUEGER_FOTO_2;
            if(fl_reg & FLUEGER_FOTO_1)
                d->flug --;
            else
                d->flug ++;
        }
        else
        {
            fl_reg |= FLUEGER_FOTO_2;
            if(fl_reg & FLUEGER_FOTO_1)
                d->flug ++;
            else
                d->flug --;
        }
        P1IES ^= FLUEGER_FOTO_2; // swith the edge detection
        P1IFG &= ~FLUEGER_FOTO_2;
    }
    if(d->flug == 0xFF) // overflown
        d->flug = 119;
    else if(d->flug > 120)
        d->flug -= 120;

    if(P1IFG & ANEM_FOTO)
    {
        if(d->anem == 0xFFFF) // overflown
            P1IE &= ~ANEM_FOTO;
        else
            d->anem ++;
        P1IFG &= ~ANEM_FOTO;
    }
    P1IFG = 0;
}

//-----------------------------------------------------------------------

__attribute__((interrupt (TIMER0_A0_VECTOR)))
void CCR0_isr(void)
{
    anem_pin_second++;
    if(anem_pin_second >= Seconds60)
    {
        anem_pin_second = 0;

        ADC10CTL0 |= ADC10IE;
        P1IE  |= (FLUEGER_FOTO_1 | FLUEGER_FOTO_2 | ANEM_FOTO);

        data_index ++;
        if(data_index >= DataBufLen)
        {
            data_index = 0;
            _BIC_SR_IRQ(LPM3_bits);
        }
    }
}

//-----------------------------------------------------------------------

__attribute__((interrupt (ADC10_VECTOR)))
void ADC10_isr(void)
{
    // oC = ((A10/1024)*1500mV)-986mV)*1/3.55mV = A10*423/1024 - 278
    const long temp = ADC10MEM;
    char oC_degree = ((temp - 673) * 423) / 1024;
    if(oC_degree < 15)
        P1OUT |= HEATER;
    else
        P1OUT &= ~HEATER;
}

//-----------------------------------------------------------------------


static void data_to_str(const struct DumpData* d, char* s, unsigned char len)
{
    for(unsigned char byte = 0; byte < len; ++byte)
    {
        char* c = s + byte;
        if(byte < 5)
        {
            unsigned int anem_average = d->anem;
            const unsigned int degree = byte == 4 ? 1 : byte == 3 ? 10 : byte == 2 ? 100 : byte == 1 ? 1000 : 10000;
            *c = '0';
            while(anem_average > degree)
            {
                *c ++;
                anem_average -= degree;
            }
        }
        else if(byte == 5)
            *c = ' ';
        else if(byte < 9)
        {
            unsigned char flueger_average = d->flug;
            const unsigned char degree = byte == 8 ? 1 : byte == 7 ? 10 : 100;
            *c = '0';
            while(flueger_average > degree)
            {
                *c ++;
                flueger_average -= degree;
            }
        }
        else if(byte == 9)
            *c = ' ';
        else if(byte == 10)
            *c = d->tC < 0 ? '-' : '+';
        else if(byte < 14)
        {
            unsigned char oC_degree = d->tC < 0 ? -d->tC : d->tC;
            const unsigned char degree = byte == 13 ? 1 : byte == 12 ? 10 : 100;
            *c = '0';
            while(oC_degree > degree)
            {
                *c ++;
                oC_degree -= degree;
            }
        }
        else if(byte == 14)
            *c = '\r';
        else if(byte == 15)
            *c = '\n';
        else
            *c = ' ';
    }
}


static void dump_to_sd(void)
{
    char dump[DataBufLen * 16];
    for(unsigned char i=0; i<DataBufLen; i++)
        data_to_str(data_buf + i, dump + 16*i, 16);
    if(pf_open("kek") != FR_OK)
        return;
    pf_lseek(0xFF);
    WORD bytes_written = 0;
    while(sizeof(dump) > bytes_written)
    {
        WORD bw;
        if(pf_write(dump+bytes_written, sizeof(dump)-bytes_written, &bw) != FR_OK)
            break;
        bytes_written += bw;
    }
}

