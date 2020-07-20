/*
 * File:   newmain.c
 * Author: jake
 *
 * Created on July 29, 2018, 12:16 PM
 */

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = OFF      // Brown-out reset enable bits (Brown-out reset disabled)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = OFF     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block not write protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration Register not write protected)
#pragma config WRTSAF = OFF     // Storage Area Flash Write Protection bit (SAF not write protected)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/Vpp must be used for programming)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (UserNVM code protection disabled)

#include <xc.h>
#include <stdint.h>


#define SCK_tris TRISCbits.TRISC0
#define SDA_tris TRISCbits.TRISC1

#define LED_DRIVER_ADDRESS 0b11000000

#define _XTAL_FREQ 1000000

// delay for spiral sequence
#define SPIRAL_DELAY 25
// delay for sparkle sequence
#define SPARKLE_DELAY 50
#define FADE_DELAY 100
#define RANDOM_DELAY 1000
#define INOUT_DELAY 1000

#define SEQUENCE_COUNT 5
enum{SEQ_STANDBY = 0, SEQ_STARTUP, SEQ_SPARKLE, SEQ_FADE, SEQ_INOUT, SEQ_RANDOM, SEQ_SPIRAL, };

void sys_initialize(void);
void i2c_initialize(void);

_Bool I2C_Transmit(uint8_t data);
_Bool I2C_Startup(void);
_Bool I2C_Stop(void);

uint8_t data;
uint8_t driver_address;
uint8_t control_word;
uint8_t sequence_state = 0;

void write_map(uint16_t leds);
void sequence_standby(void);
void sequence_startup(void);
void sequence_random(void);
void sequence_inOut(void);
void sequence_spiral(void);
void sequence_sparkle(void);
void sequence_fade(void);
void check_switch(void);
void switch_sequence(void);

void Initialize_Driver(void);

enum{SUCCESS,FAILURE};


typedef struct I2C_Flags_t{    
    _Bool START;
    _Bool SEND;
    _Bool STOP;
} I2C_Flags;


I2C_Flags I2C_FLAGS;

void main(void) {
        
    sys_initialize();
    i2c_initialize();
    Initialize_Driver();
    
    while(1){ 
                
        switch (sequence_state)
        {
        case SEQ_RANDOM:
            sequence_random();
            break;
        case SEQ_INOUT:
            sequence_inOut();
            break;
        case SEQ_SPIRAL:
            sequence_spiral();
            break;
        case SEQ_SPARKLE:
            sequence_sparkle();
            break;
        case SEQ_FADE:
            sequence_fade();
            break;
        case SEQ_STARTUP:
            sequence_startup();
            break;
        case SEQ_STANDBY:
            sequence_standby();
            break;
        default:
            break;
        }
    }
    
    return;
}

void __interrupt() int_handler(void){
    
    // switch flag
    if(IOCCFbits.IOCCF3 == 1){
        if(PORTCbits.RC3 == 1){
        // blocking debounce
            __delay_ms(50);
            if(PORTCbits.RC3 == 1){
                IOCCFbits.IOCCF3 = 0;
                // wait for switch to be depressed
                while(PORTCbits.RC3);
                switch_sequence();
            }
        }
        // clear flags
        IOCCFbits.IOCCF3 = 0;
        
    }
    
}

void sys_initialize(void){
    
        
    // CLKRMD CLKR enabled; SYSCMD SYSCLK enabled; FVRMD FVR enabled; IOCMD IOC enabled; NVMMD NVM enabled; 
    PMD0 = 0x00;
    // TMR0MD TMR0 enabled; TMR1MD TMR1 enabled; TMR2MD TMR2 enabled; NCOMD DDS(NCO) enabled; 
    PMD1 = 0x00;
    // ZCDMD ZCD disabled; CMP1MD CMP1 disabled; ADCMD ADC disabled; CMP2MD CMP2 disabled; DAC1MD DAC1 disabled; 
    PMD2 = 0x67;
    // CCP2MD CCP2 disabled; CCP1MD CCP1 disabled; PWM4MD PWM4 disabled; PWM3MD PWM3 disabled; PWM6MD PWM6 disabled; PWM5MD PWM5 disabled; 
    PMD3 = 0x3F;
    // CWG1MD CWG1 disabled; UART2MD EUSART2 disabled; MSSP1MD MSSP1 enabled; UART1MD EUSART disabled; 
    PMD4 = 0xC1;
    // CLC3MD CLC3 disabled; CLC4MD CLC4 disabled; CLC1MD CLC1 disabled; CLC2MD CLC2 disabled; 
    PMD5 = 0x1E;
    SSP1CLKPPS = 0x10;
    SSP1DATPPS = 0x11;
    RC0PPS = 0x15;
    RC1PPS = 0x16;
    
    //TMR0 source = LFINTOSC
    T0CON1bits.T0CS = 0b100; 
    //TMR0 postscaler = 0000;
    T0CON0bits.T0OUTPS = 0;
    //TMR0 is 16 bit
    T0CON0bits.T016BIT = 0;
    //Enable TMR0
    T0CON0bits.T0EN = 1;
    
    // tristate switch port
    TRISCbits.TRISC3 = 1;
    
    // enable global interrupts
    INTCON = 0xF1;
    // enable interrupt-on-change interrupt
    PIE0bits.IOCIE = 1;
    // enable RC3 ioc on positive edge
    IOCCPbits.IOCCP3 = 1;
    // clear flag
    IOCCFbits.IOCCF3 = 0;
    
    I2C_FLAGS.START = SUCCESS;
    I2C_FLAGS.SEND = SUCCESS;
    I2C_FLAGS.STOP = SUCCESS;
    
    
}

void i2c_initialize(void){
    // setup SCK and SDA
    
    // All port C pins digital
    ANSELC = 0x00;
    
    //Set MSSP pins to inputs
    SCK_tris = 1;
    SDA_tris = 1;
    
    TRISCbits.TRISC4 = 0;
    LATCbits.LATC4 = 1;

    
    // SMP Standard Speed; CKE disabled; 
    SSP1STAT = 0x80;
    // SSPEN enabled; I2C master mode
    SSP1CON1 = 0x28;
    // SBCDE disabled; BOEN disabled; SCIE disabled; PCIE disabled; DHEN disabled; SDAHT 100ns; AHEN disabled; 
    SSP1CON3 = 0b01110000;
    // SSPADD 3; 
    SSP1ADD = 0x04; //50kHz 4f
  
}



_Bool I2C_Startup(void){
    
    SSP1CON2bits.SEN = 1; //Initial start condition
    while(SSP1CON2bits.SEN != 0);
    
    
    //Check for bus collision
    if (PIR3bits.BCL1IF == 1){
        PIR3bits.BCL1IF = 0;
        return FAILURE;
    }
    //Otherwise clear interrupt
    else{
        PIR3bits.SSP1IF = 0;
        __delay_ms(1);
        return SUCCESS;

    }   
}

_Bool I2C_Transmit(uint8_t data){
    
    SSP1BUF = data;
    
    if(SSP1CON2bits.ACKSTAT == 1){
        return FAILURE;
    }
    else{
        if (PIR3bits.BCL1IF == 1){
            PIR3bits.BCL1IF = 0;
            return FAILURE;
        } 
        else{
            PIR3bits.SSP1IF = 0;
            __delay_ms(1);
            return SUCCESS;

        }
    }
    
}

_Bool I2C_Stop(void){
    PIR3bits.SSP1IF = 0;
    SSP1CON2bits.PEN = 1;
    while(SSP1CON2bits.PEN != 0);

    if (PIR3bits.BCL1IF == 1){
//            PIR3bits.BCL1IF = 0;
            return FAILURE;
        } 
        else{
            PIR3bits.SSP1IF = 0;
            __delay_ms(1);
            return SUCCESS;
            ;
    }
       
}

void Initialize_Driver(){
    
    // startup delay
    __delay_ms(5);
    I2C_FLAGS.STOP = I2C_Stop();
    
    //Turn the damn thing on
    //Send start condition
    I2C_FLAGS.START = I2C_Startup();
    //Slave Address
    I2C_FLAGS.SEND = I2C_Transmit(LED_DRIVER_ADDRESS);
    //Select MODE1 register 0x00
    I2C_FLAGS.SEND = I2C_Transmit(0x00);
    //Enable main oscillator
    I2C_FLAGS.SEND = I2C_Transmit(0x00);
    //Done with this block
    I2C_FLAGS.STOP = I2C_Stop();
    
    //Enable output drivers
    //Send start condition
    I2C_FLAGS.START = I2C_Startup();
    //Slave address
    I2C_FLAGS.SEND = I2C_Transmit(LED_DRIVER_ADDRESS);
    //Select LEDOUT0 register 0x14 WITH AUTO INCREMENT
    I2C_FLAGS.SEND = I2C_Transmit(0b10000000 | 0x14);
    //Enable LEDOUT0
    I2C_FLAGS.SEND = I2C_Transmit(0xFF);
    //Enable LEDOUT1
    I2C_FLAGS.SEND = I2C_Transmit(0xFF);
    //Enable LEDOUT2
    I2C_FLAGS.SEND = I2C_Transmit(0xFF);
    //Enable LEDOUT3
    I2C_FLAGS.SEND = I2C_Transmit(0xFF);
    //Done with this block
    I2C_FLAGS.STOP = I2C_Stop();
    
    
}

void sequence_random(){
    
    // create seed
    srand(TMR0L);
    // create map
    volatile uint16_t map = (uint16_t)rand();

    write_map(map);

    __delay_ms(RANDOM_DELAY);

}

void sequence_inOut(){

    // write inner leds
    // 0011 0010 0100 1001
    write_map(0x3249);
    __delay_ms(INOUT_DELAY);
    
    // write outer leds
    // 1100 1101 1011 0110
    write_map(0xCDB6);
    __delay_ms(INOUT_DELAY);

}


void sequence_spiral(){

    // LED 1
    write_map(0x01);
    __delay_ms(SPIRAL_DELAY);
    // LED 4
    write_map(0x08);
    __delay_ms(SPIRAL_DELAY);
    // LED 7
    write_map(0x40);
    __delay_ms(SPIRAL_DELAY);
    // LED 10
    write_map(0x200);
    __delay_ms(SPIRAL_DELAY);
    // LED 13
    write_map(0x1000);
    __delay_ms(SPIRAL_DELAY);
    // LED 14
    write_map(0x2000);
    __delay_ms(SPIRAL_DELAY);
    // LED 1
    write_map(0x01);
    __delay_ms(SPIRAL_DELAY);
    // LED 2
    write_map(0x02);
    __delay_ms(SPIRAL_DELAY);
    // LED 3
    write_map(0x04);
    __delay_ms(SPIRAL_DELAY);
    // LED 5
    write_map(0x10);
    __delay_ms(SPIRAL_DELAY);
    // LED 6
    write_map(0x20);
    __delay_ms(SPIRAL_DELAY);
    // LED 8
    write_map(0x80);
    __delay_ms(SPIRAL_DELAY);
    // LED 9
    write_map(0x0100);
    __delay_ms(SPIRAL_DELAY);
    // LED 11
    write_map(0x400);
    __delay_ms(SPIRAL_DELAY);
    // LED 12
    write_map(0x800);
    __delay_ms(SPIRAL_DELAY);
    // LED 15
    write_map(0x4000);
    __delay_ms(SPIRAL_DELAY);
    // LED 16
    write_map(0x8000);
    __delay_ms(SPIRAL_DELAY);


}



void sequence_sparkle(){

    // LEDs 1/10
    write_map(0x0201);
    __delay_ms(SPARKLE_DELAY);
    // LEDs 2/6/11
    write_map(0x0422);
    __delay_ms(SPARKLE_DELAY);
    // LEDs 3/7/12/14
    write_map(0x2844);
    __delay_ms(SPARKLE_DELAY);
    // LEDs 4/8/13/15
    write_map(0x5088);
    __delay_ms(SPARKLE_DELAY);
    // LEDs 5/9/16
    write_map(0x8110);
    __delay_ms(SPARKLE_DELAY);

}


void sequence_fade(void){
    // LEDs 2/3
    write_map(0x0006);
    __delay_ms(FADE_DELAY);
    // LEDs 1/4/5/16
    write_map(0x8019);
    __delay_ms(FADE_DELAY);
    // LEDs 6/15
    write_map(0x4020);
    __delay_ms(FADE_DELAY);
    // LEDs 7/14
    write_map(0x2040);
    __delay_ms(FADE_DELAY);
    // LEDs 8
    write_map(0x0080);
    __delay_ms(FADE_DELAY);
    // LEDs 9/10/13
    write_map(0x1300);
    __delay_ms(FADE_DELAY);
    // LEDs 11/12
    write_map(0x0C00);
    __delay_ms(FADE_DELAY);
}

void write_map(uint16_t leds){

    //Send start condition
    I2C_FLAGS.START = I2C_Startup();
    //Slave address
    I2C_FLAGS.SEND = I2C_Transmit(LED_DRIVER_ADDRESS);
    //Select PWM0 register 0x02 WITH AUTO INCREMENT
    I2C_FLAGS.SEND = I2C_Transmit(0b10100000 | 0x02);


    // shift and masks to check each led
    for(int i = 0; i < 16; i++){

        // turn on led i if LSB is 1, off if bit is 0
        if( (leds>>i) & 0x01 ){
            I2C_FLAGS.SEND = I2C_Transmit(0x0F);
        } else {
            I2C_FLAGS.SEND = I2C_Transmit(0x00);
        }

    }

    I2C_FLAGS.STOP = I2C_Stop(); 

}


void switch_sequence(void){
        
    if(sequence_state < SEQUENCE_COUNT){
        sequence_state ++;
    } else{
        sequence_state = 0;
    }
    
}

void sequence_startup(void){
    // turn on voltage regulator
    LATCbits.LATC4 = 1;
    i2c_initialize();
    Initialize_Driver();
    sequence_state ++;
}

void sequence_standby(void){
    
    // turn off voltage regulator
    LATCbits.LATC4 = 0;
    //disable i2c
    SSP1CON1bits.SSPEN = 0;
    // ground i2c pins
    SCK_tris = 0;
    SDA_tris = 0;
    LATCbits.LATC0 = 0;
    LATCbits.LATC1 = 0;
}