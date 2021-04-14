#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>

#define SLAVE_ADDR  0x38
#define MAX_BUFFER_SIZE     20
#define TYPE_1_LENGTH   2
#define dataCommand               0xAC

float Temp;
int X0, X1;
int X_out;
float Xtemp, Yhum ;

uint16_t temp_and_hum[6] = {0};
uint32_t AHT10_ADC_Raw;
//uint8_t AHT10_RX_Data[6];


//uint8_t MasterType2 [TYPE_1_LENGTH] = {0xe1, 0xac};
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;


void mcu_init();
void init_i2c(void);
void i2c_write(const uint8_t addr);
uint8_t i2c_read(void);
void calcTempHum(void);
//void humCalc(void);
void i2c_reset(void);

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    mcu_init();
    init_i2c();
    PM5CTL0 &= ~LOCKLPM5;

    while(1) {
        calcTempHum();
        _delay_cycles(700);
      /*  temp_and_hum[0] = Xtemp; // get the integer part of the temperature value
        Xtemp = Xtemp - (float)temp_and_hum[0];
        temp_and_hum[1] = (Xtemp * 100);  // get the floating point of the temperature value
*/
        /* humidity calculation */
      /*  humCalc();*/
        _delay_cycles(700);
  /*      temp_and_hum[2]  = Yhum;
        Yhum = Yhum - (float)temp_and_hum[2];
        temp_and_hum[3]  = (Yhum * 100);

        temp_and_hum[4] = 2;
        temp_and_hum[5] = 0;
        temp_and_hum[6] = 2;
        temp_and_hum[7] = 0;*/

        _delay_cycles(100);
    }

    return 0;
}

void init_i2c(void)
{
    P5OUT = 0x00;
    P5DIR = 0xFF;
    P5SEL0 |= BIT2 | BIT3;

    UCB0CTLW0 = UCSWRST;                                    // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB0BRW = 160;                                           // fSCL = SMCLK/80 = ~100kHz
    UCB0TBCNT = 0x0002;
    UCB0I2CSA = 0x38;                                       // Slave Address
    UCB0CTLW0 |= UCTXSTT;                                   // Transmit START
    UCB0CTLW0 &= ~UCSWRST;                                  // Clear SW reset, resume operation
    UCB0CTLW1 |= UCASTP_2;   //// Automatic stop generated
    //UCB0IE |= UCRXIE | UCNACKIE | UCBCNTIE ;
}

void i2c_write(const uint8_t addr)
{
    while(UCB0IV & UCTXIFG0);
    //for(int i = 0; i++; i<2)
    UCB0TXBUF = addr;
//    UCB0CTLW0 |= UCTXSTP;           // I2C stop condition
//    UCB0IFG   &=~UCTXIFG;            // Clear USCI_B0 TX int flag
}

uint8_t i2c_read(void)
{
    while(UCB0IV & UCRXIFG0);
    return UCB0RXBUF;
}

void calcTempHum(void)
{
    while (UCB0CTLW0 & UCTXSTP);
    UCB0CTL1 |= UCTXSTT;
    while(UCB0IV & USCI_I2C_UCNACKIFG);
    UCB0CTLW0 |= UCTR | UCTXSTT;

    UCB0TBCNT = 0x0001;  // we want to 2 byte data to send
    UCB0I2CSA = 0x38;    // the slave address

// Temperature calculation is started
 //   i2c_write(0xe1);  // start temperature measurement
    __delay_cycles(30000);

    i2c_write(dataCommand);  // start temperature measurement
    __delay_cycles(30000);
    /*int i = 0;
    for(i = 0; i++; i<6) {
        temp_and_hum[i++] = i2c_read();
    }*/
    temp_and_hum[0] = i2c_read();
    temp_and_hum[1] = i2c_read();
    temp_and_hum[2] = i2c_read();
    /*temp_and_hum[3] = i2c_read();
    temp_and_hum[4] = i2c_read();
    temp_and_hum[5] = i2c_read();*/

    /* Convert to Temperature in °C */
    AHT10_ADC_Raw = (((uint32_t)temp_and_hum[0] & 15) << 16) | ((uint32_t)temp_and_hum[1] << 8) | temp_and_hum[2];
    /* X0 = X0 << 8;
    X_out = X0+X1;
    Xtemp=(175.72*X_out)/65536.0;
    Xtemp=Xtemp-46.85;*/

// temperature calculation is finished

    UCB0CTLW0 &= ~UCTR;

    while (UCB0CTL1 & UCTXSTP);                 // Ensure stop condition got sent
    UCB0CTL1 |= UCTXSTT;

}


void mcu_init() {

    // Configure clock
    __bis_SR_register(SCG0);                        // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                      // Set REFO as FLL reference source
    CSCTL0 = 0;                                     // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                         // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                            // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;                           // DCODIV = 16MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                        // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));      // Poll until FLL is locked

 //   CSCTL4 |= SELMS__DCOCLKDIV | SELA__REFOCLK;     // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                                    // default DCODIV as MCLK and SMCLK source
//    CSCTL5 |= DIVM__1 | DIVS__2;                    // SMCLK = 1MHz, MCLK = 2MHz
}


/* void humCalc(void)
{
    int Y0, Y1, Y2;
    float Y_out1, Y_out2;
    while (UCB0CTLW0 & UCTXSTP);
    UCB0CTL1 |= UCTXSTT;
    while(UCB0IV & USCI_I2C_UCNACKIFG);
    UCB0CTLW0 |= UCTR | UCTXSTT;
    UCB0I2CSA = 0x40;
    UCB0TBCNT = 0x0002;

//  humidity calculation is started

 //   i2c_write(HUMID_HOLD_MASTER);

    Y0 = i2c_read();
    Y2=Y0 / 100;
    Y0=Y0 % 100;
    Y1 = i2c_read();

    Y_out1 = Y2*25600.0;
    Y_out2 = Y0*256.0+Y1;

    Y_out1 = Y_out1;  // (125.0*Y_out1)/65536.0;
    Y_out2 = Y_out2;  // (125.0*Y_out2)/65536.0;
    Yhum = 125.0*(Y_out1+Y_out2)/65536.0;
    Yhum = Ynem-6;

//  humidity calculation is finished


    UCB0CTLW0 &= ~UCTR;

    while (UCB0CTL1 & UCTXSTP);                 // Ensure stop condition got sent
    UCB0CTL1 |= UCTXSTT;


}
*/

/*void i2c_reset(void)
{
    i2c_write(RESET);
}*/

