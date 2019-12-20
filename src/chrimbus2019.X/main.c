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

#define SPIRAL_DELAY 200

void sys_initialize(void);
void i2c_initialize(void);



_Bool I2C_Transmit(uint8_t data);
_Bool I2C_Startup(void);
_Bool I2C_Stop(void);

uint8_t data;
uint8_t driver_address;
uint8_t control_word;
uint8_t sequence;

typedef struct LED_t {
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint8_t reg;
    uint8_t num; 
} LED;

typedef struct LEDmap_t{
    uint8_t LED00 : 1;
    uint8_t LED01 : 1;
    uint8_t LED02 : 1;
    uint8_t LED03 : 1;
    uint8_t LED04 : 1;
    uint8_t LED05 : 1;
    uint8_t LED06 : 1;
    uint8_t LED07 : 1;
    uint8_t LED08 : 1;
    uint8_t LED09 : 1;
    uint8_t LED10 : 1;
    uint8_t LED11 : 1;
    uint8_t LED12 : 1;
    uint8_t LED13 : 1;
    uint8_t LED14 : 1;
} LEDmap;


void Write_LED(LED *led);
void Initialize_Driver(void);

enum{SUCCESS,FAILURE};
enum{STANDBY = 0, RANDOM, INOUT, SPIRAL};

typedef struct I2C_Flags_t{    
    _Bool START;
    _Bool SEND;
    _Bool STOP;
} I2C_Flags;

LED LED1 = {0, 0, 0, 0x02, 1};
LED LED2 = {0, 0, 0, 0x05, 2};
LED LED3 = {0, 0, 0, 0x08, 3};
LED LED4 = {0, 0, 0, 0x0B, 4};
LED LED5 = {0, 0, 0, 0x0E, 5};

I2C_Flags I2C_FLAGS;

void main(void) {
        
    sequence = 1;
    sys_initialize();
    i2c_initialize();
    Initialize_Driver();
    
    while(1){ 

        switch (sequence)
        {
        case RANDOM:
            sequence_random();
            break;
        case INOUT:
            sequence_inOut();
        default:
            break;
        }
    }
    
    return;
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
    
    //Turn on star LED
    //Send start condition
    I2C_FLAGS.START = I2C_Startup();
    //Slave address
    I2C_FLAGS.SEND = I2C_Transmit(LED_DRIVER_ADDRESS);
    //Select PWM15 register 0x11
    I2C_FLAGS.SEND = I2C_Transmit(0x11);
    //Light em up bb
    I2C_FLAGS.SEND = I2C_Transmit(0xFF);
    //Done with this block
    I2C_FLAGS.STOP = I2C_Stop();
    
}

void sequence_random(){
    
    // create seed
    srand(TMR0L);
    // create map
    volatile LEDmap map = rand();

    write_map(map);

    __delay_ms(2000);

}

void sequence_inOut(){

    // write inner leds
    // 0011 0010 0100 1001
    write_map(0x3249);
    __delay_ms(1000);
    
    // write outer leds
    // 1100 1101 1011 0110
    write_map(0xCDB6);
    __delay_ms(1000);

}


void sequence_spiral(){

    // LED 1
    write_map(0x01);
    __delay_ms(SPIRAL_DELAY);
    // LED 4
    write_map(0x04);
    __delay_ms(SPIRAL_DELAY);
    // LED 7
    write_map(0x30);
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
    write_map(0x03);
    __delay_ms(SPIRAL_DELAY);
    // LED 5
    write_map(0x10);
    __delay_ms(SPIRAL_DELAY);
    // LED 6
    write_map(0x20);
    __delay_ms(SPIRAL_DELAY);
    // LED 8
    write_map(040);
    __delay_ms(SPIRAL_DELAY);
    // LED 9
    write_map(0x0100);
    __delay_ms(SPIRAL_DELAY);
    // LED 11
    write_map(0x300);
    __delay_ms(SPIRAL_DELAY);
    // LED 12
    write_map(0x400);
    __delay_ms(SPIRAL_DELAY);
    // LED 15
    write_map(0x3000);
    __delay_ms(SPIRAL_DELAY);
    // LED 16
    write_map(0x4000);
    __delay_ms(SPIRAL_DELAY);


}

void write_map(LEDmap leds){

    //Send start condition
    I2C_FLAGS.START = I2C_Startup();
    //Slave address
    I2C_FLAGS.SEND = I2C_Transmit(LED_DRIVER_ADDRESS);
    //Select PWM0 register 0x02 WITH AUTO INCREMENT
    I2C_FLAGS.SEND = I2C_Transmit(0b10100000 | 0x02);

    for(int i = 0; i < 16; i++){

        // turn on if bit is 1, off if bit is 0
        if( (leds>>i) & 0x01 ){
            I2C_FLAGS.SEND = I2C_Transmit(0xFF);
        } else {
            I2C_FLAGS.SEND = I2C_Transmit(0x00);
        }

    }

    //Send R value
    I2C_FLAGS.SEND = I2C_Transmit(led->R);
    //Send G value
    I2C_FLAGS.SEND = I2C_Transmit(led->G);
    //Send B value
    I2C_FLAGS.SEND = I2C_Transmit(led->B);
    //Hopefully things worked?
    I2C_FLAGS.STOP = I2C_Stop();

}
