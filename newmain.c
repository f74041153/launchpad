/*
 * File:   newmain.c
 * Author: USER
 *
 * Created on 2017?12?12?, ?? 7:18
 */
// CONFIG1H
#pragma config OSC  = INTIO67      // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = ON       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:1)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)


#include <xc.h>
#include <plib/i2c.h>

unsigned char I2C_Send[21]="MICROCHIP:I2C_MASTER";
unsigned char I2C_Recv[21];
void main(void) {
    unsigned char sync_mode=0, slew=0,add1,i,data,status;
   /* PIR1=0;
    PIR2=0;
    INTCON=0;
    INTCON2=0;
    INTCON3=0;
    PIE1=0;
    PIE2=0;
    RCONbits.IPEN=1;
    INTCONbits.GIEL=0;
    INTCONbits.GIEH=1;*/
    for(i=0;i<21;i++)
        I2C_Recv[i]=0;
    add1 = 0xA2;
    CloseI2C();
    sync_mode = MASTER;
    slew = SLEW_OFF;
    OpenI2C(sync_mode,slew);
    SSPADD=0x0A;
    IdleI2C();
    StartI2C();
    data = SSPBUF;  //read previous content in buffer to clear buffer
    
    do{
        status=WriteI2C(add1 | 0x00);   //write address of slave
        if(status == -1){   //collision happen
            data=SSPBUF;
            SSPCON1bits.WCOL = 0;
        }
    }
    while(status!=0);   //wait until success communicate
    while(putsI2C(I2C_Send)!=0);    //write data to be sent to slave
    IdleI2C();  
    RestartI2C();
    IdleI2C();
    data=SSPBUF;
    add1=0xA2;
    do{
        status=WriteI2C(add1 | 0x01);
        if(status == -1){
            data = SSPBUF;
            SSPCON1bits.WCOL=0;
        }
    }
    while(status!=0);
    while(getsI2C(I2C_Recv,20));
    I2C_Recv[20]='\0';
    NotAckI2C();
    CloseI2C();
    while(1);   //end of program
    if(SSPSTAT & 0x04)
        while(putsI2C(I2C_Send));
    CloseI2C();
    
    while(1);    
    
    /*ADCON1=0b00001111;
    TRISC=0;
    OpenI2C(MASTER,SLEW_ON);
    SSPADD=19;
    while(1)
    {
        IdleI2C();
        StartI2C();
        
        WriteI2C(0b00010010);
        while(SSPCON2bits.ACKSTAT==1){};
        WriteI2C(0b01101110);
        while(SSPCON2bits.ACKSTAT==1){};
        StopI2C();
    }*/
    CloseI2C();
    return;
}
