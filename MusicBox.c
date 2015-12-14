/****** ASEN 4/5519 Lab 6 ******************************************************
 * Author: Maurice Woods
 * Date  : November 2015
 *
 * Description
 * 
 * 
 * Hardware Notes
 *  RB0/INT0    :: External interrupt for button 1
 *  RB1/INT1    :: External interrupt for button 2
 *  RB5-RB7     :: LED2-LED4 (used to designate Mode)
 *  RE1-RE5     :: Photogate set (of 5) for note 1
 *  RF1-RF5     :: Photogate set (of 5) for note 2
 *  RE6&RE7     :: Photogates for grey code
 *  
 *
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Setup
 *   BootSequence
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *
 ******************************************************************************/

 
#include <p18cxxx.h>
#include "LCDroutines.h"
#include "delays.h"
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
const rom far char LCDBootRow1[] = {0x80,'M','o',' ','W','o','o','d','s',0x00};
const rom far char LCDBootRow2[] = {0xc0,'A','S','E','N','5','5','1','9',0x00};
unsigned int LED_count = 9760; //9760 * 16-bit Timer = 1s
const rom far char MusicBox1[] = {0x80,'D','i','g','i','t','a','l',' ',0x00};
const rom far char MusicBox2[] = {0xc0,'M','u','s','i','c','B','o','x',0x00};
const rom far char NewSong1[] = {0x80,'N','e','w',' ',' ',' ',' ',' ',0x00};
const rom far char NewSong2[] = {0xc0,'S','o','n','g','?',' ',' ',' ',0x00};
const rom far char MoreCards1[] = {0x80,'M','o','r','e',' ',' ',' ','Y',0x00};
const rom far char MoreCards2[] = {0xc0,'C','a','r','d','s','?',' ','N',0x00};
const rom far char ErasePlay1[] = {0x80,'N','e','w',' ','S','o','n','g',0x00};
const rom far char ErasePlay2[] = {0xc0,'P','l','a','y','S','o','n','g',0x00};
const rom far char Playing1[] = {0x80,'P','l','a','y','i','n','g',' ',0x00};
const rom far char Playing2[] = {0xc0,' ',' ',' ',' ','S','o','n','g',0x00};
const char Blank1[] = {0x80,' ',' ',' ',' ',' ',' ',' ',' ',0x00};
const char Blank2[] = {0xc0,' ',' ',' ',' ',' ',' ',' ',' ',0x00};
const rom far char Tempo1[] = {0x80,'1','/','4','N','o','t','e','=',0x00};
char Tempo2[] = {0xc0,' ',' ',' ',' ','B','P','M',' ',0x00};
unsigned char Line1 = 0;
unsigned char Line2 = 0;
volatile unsigned char Servo_On=3;
volatile unsigned int Servo_Off=37;
volatile unsigned int Servo_Duty=3;
unsigned int Servo_Counter = 0;
unsigned int Length16th=10000;
//unsigned int Note[32]={0,11364,10726,10124,9555,9019,8513,8035,7584,7159,6757,6378,6020,5682,5363,5062,4778,4510,4257,4018,3792,3579,3378,3189,3010,2841,2681,2531,2389,2255,2128,2009};
unsigned int Note[81]={0,64267,60680,57261,54066,51020,48151,45455,42896,40492,38226,36075,34051,32142,30340,28637,27027,25510,24080,22727,21452,20246,19110,18038,17025,16071,15168,14317,13514,12755,12039,11364,10726,10124,9556,9019,8513,8035,7584,7159,6757,6378,6020,5682,5363,5062,4778,4510,4257,4018,3792,3579,3378,3189,3010,2841,2681,2531,2389,2255,2128,2009,1896,1790,1689,1594,1505,1420,1341,1265,1194,1127,1064,1004,948,895,845,797,752,710,670};
unsigned int TempoCounter=0;
unsigned int Pot = 0;
unsigned int DisplayTempo;
unsigned char TempoOnes;
unsigned char TempoTens;
unsigned char TempoHundreds;
volatile unsigned char ModeSelect;
//unsigned char *stkptr1,*stktop1,Stack1[928]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29};
//unsigned char *stkptr2,*stktop2,Stack2[928]={81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52};
//unsigned char *stkptr3,*stktop3,Stack3[928]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29};
unsigned char *stkptr1,*stktop1,Stack1[928]={4,4,6,6,8,8,8,8,11,11,11,0,11,11,11,11,11,11,13,13,11,11,11,11,8,8,8,8,4,4,4,4,4,4,6,6,8,8,8,0,8,8,8,8,6,6,6,0,6,6,6,6,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,23,22,21,20,23,18,19,18,23,17,18,17,23,16,17,16,23,15,16,15,23,14,15,14,23,13,14,13,23,12,13,12,25,25,0,0,0,0,6,6,0,0,0,0,7,7,0,0,4,4,6,6,0,0,7,7,0,0,0,0,2,2,0,0,25,25,0,0,0,0,6,6,0,0,0,0,7,7,0,0,4,4,6,6,0,0,7,7,0,0,0,0,14,14,15,15,4,4,0,0,0,0,6,6,0,0,0,0,7,7,0,0,13,13,15,15,0,0,16,16,0,0,0,0,15,15,15,15,25,25,0,0,0,0,18,18,0,0,0,0,19,19,0,0,16,16,18,18,0,0,19,19,0,0,0,0,14,14,15,15,13,13,13,13,13,13,7,7,7,7,7,7,7,7,7,7,7,7,7,7,13,13,13,13,7,7,7,7,13,13,13,13,14,14,14,14,14,14,14,14,2,3,4,5,6,5,4,3,2,3,4,5,6,7,8,9,10,11,10,9,8,7,6,4};
unsigned char *stkptr2,*stktop2,Stack2[928]={0,0,0,0,0,0,0,0,0,0,0,0,19,19,19,19,0,0,0,0,0,0,0,0,0,0,0,0,19,19,19,19,0,0,0,0,0,0,0,0,0,0,0,0,19,19,19,19,0,0,0,0,15,15,15,15,15,15,15,15,0,0,0,0,0,0,0,0,0,0,0,0,0,24,23,22,21,23,22,21,20,22,21,20,19,21,20,19,18,20,19,18,17,19,18,17,16,18,17,16,15,17,16,15,14,12,0,12,12,3,3,5,5,12,12,6,6,5,5,3,3,12,0,12,12,3,3,5,5,12,12,3,3,10,10,1,1,12,0,12,12,3,3,5,5,12,12,6,6,5,5,3,3,12,0,12,12,3,3,5,5,12,12,3,3,10,10,1,1,12,0,12,12,3,3,5,5,12,12,6,6,5,5,3,3,12,0,12,12,3,3,5,5,12,12,3,3,10,10,1,1,12,0,12,12,3,3,5,5,12,12,6,6,5,5,3,3,12,0,12,12,3,3,5,5,12,12,3,3,10,10,1,1,12,12,7,7,12,12,7,7,12,12,7,7,12,12,7,7,12,12,7,7,12,12,7,7,12,12,7,7,12,12,7,7,1,1,8,8,1,1,8,8,1,1,8,8,1,1,8,8,1,1,8,8,10,10,8,8,7,7,5,5,3,3,1,1};
unsigned char *stkptr3,*stktop3,Stack3[928]={0,0,0,0,25,25,25,25,0,0,0,0,0,0,0,0,0,0,0,0,25,25,25,25,0,0,0,0,0,0,0,0,0,0,0,0,25,25,25,25,0,0,0,0,0,0,0,0,0,0,0,0,25,25,25,25,25,25,25,25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile unsigned char Confirm;
unsigned char TempoFlag;
unsigned char Demo=0;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void HiPriISR(void);
void LoPriISR(void);
void Setup(void);           // Function to initialize hardware and interrupts
void BootSequence(void);    //Function to blink D2,D3,and D4 LEDs in sequence at boot
void MODE1(void);    //
void MODE2(void);    //
void MODE3(void);    //
void MODE4(void);    //
void TMR0handler(void);     // Interrupt handler for TMR1
void CCP1NoteTone(void);
void CCP2NoteTone(void);
void CCP3NoteTone(void);
void CCP4Tempo(void);
void CCP5ServoFeed(void);
void UpdateLCD(void);
void EraseSong(void);
void MeasurePot(void);
void CaptureLine(void);     //Capture line of data punched into card
void ReceiveCard(void);     //
void FeedCard(void);
void Playback(void);

#pragma code highVector=0x08
void atHighVector(void)
{
 _asm GOTO HiPriISR _endasm
}
#pragma code

#pragma code lowVector=0x18
void atLowVector(void)
{
 _asm GOTO LoPriISR _endasm
}
#pragma code

/******************************************************************************
 * The "meat and potatoes", as it were
 ******************************************************************************/
void main() {
     Setup();                 // Initialize everything
     BootSequence();
      while(1) {
          if(ModeSelect==1){MODE1();} // MODE 1: Standby (Includes memory purge)
          else if(ModeSelect==2){MODE2();} // MODE 2: Card Feed 
          else if(ModeSelect==3){MODE3();} // MODE 3: Tempo Select
          else if(ModeSelect==4){MODE4();} // MODE 4: Playback
          else{MODE2();}
     }
}

/******************************************************************************
 * Setup subroutine
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR1 and sets CCP1 for compare, and enables LoPri interrupts for
 * both.
 ******************************************************************************/
void Setup() {
    // Configure the IO ports
    TRISA = 0b10000011;
    LATA = 0b00000000;
    TRISB  = 0b00001110;    //<0-3>Unused,<4-7>D2,D3,D4,Alive(off until after BootSequence)
    LATB = 0b00010000;               //Turn LED's off
    TRISC  = 0b10010011;
    LATC = 0b00000000;
    TRISGbits.TRISG3=0;
    TRISE=0b11111111;
    TRISF=0b11111111;
    
    stktop1=&Stack1[60];
    stktop2=&Stack2[0];
    stkptr1=&Stack1[0];
    stkptr2=&Stack2[0];
    stkptr3=&Stack3[0];
    
    // Configure the LCD pins for output
    LCD_RS_TRIS   = 0;
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b00001111;
    TRISE=0b11111111;

    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(MusicBox1);
    DisplayC(MusicBox2);
    
    // Initialize ADC module (see p362 of manual) (MeasureTemp(AN0)/MeasurePot(AN1) will change ADCON0<2-6> to select source)
    ANCON0 = 0b00000011;    //<0-7>AN0 and AN1 as analog ports, rest as digital ports
    ANCON1 = 0b00000000;    //<0-7>
    ANCON2 = 0b00000000;    //<0-7>
    ADCON0 = 0b00000001;    //<0>1=On, <1>0=DONE(idle), <2-6>00000=AN0, <7>0=Unused
    ADCON1 = 0b00000000;    //<0-2>000=AVss_NegCh, <3>0=AVss_ExtRef, <4-5>00=AVdd, <6-7>00=(Don't care) (Bit Resolution = 500uV/bit)
    ADCON2 = 0b10100100;    //<0-2>000=2*Tosc_ClockScale, <3-5>101=Fosc/16_AcquisitionTime, <6>0=(Don't care), <7>1=RightJust.

    // Initializing TMR0
    T0CON = 0b01001000;     // 8-bit, Fosc / 4, no pre/post scale timer
    TMR0L = 0;              // Clearing TMR0 registers
    TMR0H = 0;
    T0CONbits.TMR0ON = 1;   // Turning on TMR0
    T1CON = 0b00000000;     // <0>0=Off, <1>0=2x8bit_R/W, <2>0=(Don't care),<3>1=SOSC/SCLKI_enabled, <4-5>00=1:1_Prescale, <6-7>01=Fosc
    TMR1H = 0;              // Clearing TMR0 registers
    TMR1L = 0;

    CCP1CONbits.CCP1M = 0; // 8 = On, 0 = Off
    CCPTMRS0bits.C1TSEL = 0;
    CCP2CONbits.CCP2M = 0;
    CCPTMRS0bits.C2TSEL = 0;
    CCP3CONbits.CCP3M = 0;
    CCPTMRS0bits.C3TSEL = 0;
    CCP4CONbits.CCP4M = 0;
    CCPTMRS1bits.C4TSEL = 0;
    CCP5CONbits.CCP5M = 0;
    CCPTMRS1bits.C5TSEL0 = 0;
    
    // Configuring Interrupts
    RCONbits.IPEN = 1;              //Enable priority levels
    
    INTCONbits.TMR0IE=0;            //Disable TMRx Interrupts
    INTCON2bits.TMR0IP = 0;         //Assign low priority to TMRx
    INTCONbits.TMR0IF = 0;          // Clear flag

    PIE1bits.TMR1IE = 0;            // Disable TMRx Interrupts
    IPR1bits.TMR1IP = 0;            // Assign low priority to TMRx
    PIR1bits.TMR1IF = 0;            // Clear flag
    
    PIE3bits.CCP1IE = 1;            //Enable CCPx Interrupts
    IPR3bits.CCP1IP = 0;            // Assign low priority to CCPx
    PIR3bits.CCP1IF = 0;            // Clear flag
    
    PIE3bits.CCP2IE = 1;            // Enable CCPx Interrupts
    IPR3bits.CCP2IP = 0;            // Assign low priority to CCPx
    PIR3bits.CCP2IF = 0;            // Clear flag
    
    PIE4bits.CCP3IE = 1;            // Enable CCPx Interrupts    
    IPR4bits.CCP3IP = 0;            // Assign low priority to CCPx
    PIR4bits.CCP3IF = 0;            // Clear flag
    
    PIE4bits.CCP4IE = 1;            // Enable CCPx Interrupts    
    IPR4bits.CCP4IP = 0;            // Assign low priority to CCPx
    PIR4bits.CCP4IF = 0;            // Clear flag
    
    PIE4bits.CCP5IE = 1;            // Enable CCPx Interrupts    
    IPR4bits.CCP5IP = 0;            // Assign low priority to CCPx
    PIR4bits.CCP5IF = 0;            // Clear flag
    
    INTCONbits.INT0IE = 1;          // External interrupt (RB0)
    //...................           // p.163 "INT0 is always a High Priority Interrupt"
    INTCONbits.INT0IF = 0;          // Clear flag
    
    INTCON3bits.INT1IE=1;           // External interrupt (RB1)
    INTCON3bits.INT1IP=1;           // Assign high priority to INT1
    INTCON3bits.INT1IF=0;           // Clear flag
    
    INTCONbits.GIEL = 1;            //Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            //Enable all interrupts        
    
    T0CONbits.TMR0ON = 1;           //Turning on TMR0 
    T1CONbits.TMR1ON = 1;           //Turning on TMR1
    
}

/******************************************************************************
 * MODE 0: BootSequence
 *
 * Play start melody
 * 
 ******************************************************************************/
void BootSequence(){
    Playback();
    ModeSelect=2;
}

/******************************************************************************
 * MODE 1: NewSong (RB5,RB6, and RB7 LEDs on)
 *
 * 1. Prompt new song (erase stored song?) 1[NewSong?] 2[ ERASE? ]
 * 2a. If select "ERASE", set top-of-stack pointer to bottom of stack and display 1[        ] 2[ ERASED ]
 * 2b. Otherwise, do nothing
 * 
 ******************************************************************************/
void MODE1(){
    //Confirm=0;
    while(ModeSelect==1){
        DisplayC(ErasePlay1);
        DisplayC(ErasePlay2);
        Confirm=0;
        while(Confirm==0){}                 //Wait for user response
        if(Confirm==1){
            ModeSelect==2;
            EraseSong();
        }                //If more cards are to be read, reset card line counter and read card
        else if(Confirm==2){
            ModeSelect==3;
        } //If finished, change to Mode3
        Confirm=0;
    }
}
void EraseSong(){
    stktop1=&Stack1[0];
    stktop2=&Stack2[0];
}

/******************************************************************************
 * MODE 2: CardRead (RB5 LED on, RB6 and RB7 LEDs off)
 *
 * 1. Start feed of punch card by slowly turning servo
 * 2. When 'grey-ish code' detects a change in pattern, stop feed and capture the activated gates
 * 3. Store the captured notes (bit combinations) in the stack, display note number and tones 1[ #____  ] 2[___::___]  (ex. 1[ #125  ] 2[Ab4::F 5])
 * 4. Repeat steps 1. trough 3. until 80(?) lines have been captured
 * 5. After 80 lines are captured, prompt acknowledgement for DONE or MORE CARDS 1[  More  ] 2[ Cards? ]
 * 6a. If MORE CARDS is selected, go back to step 1.
 * 6b. If DONE is selected, prompt change to MODE 3
 * 
 ******************************************************************************/
void MODE2(){
    unsigned char i=1;
    DisplayC(NewSong1);
    DisplayC(NewSong2);
    Confirm=0;
    ModeSelect=2;
    if(Demo==1){
        while(ModeSelect==2){
            CCP5CONbits.CCP5M = 8;
            CCP1CONbits.CCP1M = 8;
            CCP2CONbits.CCP2M = 8;
            //CCP3CONbits.CCP3M = 8;
            //CCP4CONbits.CCP4M = 8;
            *stkptr1=PORTF;
            *stkptr2=PORTE;
            Delay10KTCYx(1);
            if(stkptr1==&Stack1[900]){            }
            else if(*stkptr1==0 && *stkptr2==0){}
            else{
                    stkptr1++;                  //COMMENT OUT FOR DEMO
                    stkptr2++;                  //COMMENT OUT FOR DEMO
            }
            //DisplayC(MoreCards1);               //Prompt "More Cards? Y/N"
            //DisplayC(MoreCards2);
            //Confirm=0;
            //while(Confirm==0){}                 //Wait for user response
            //if(Confirm==1){i=1;}                //If more cards are to be read, reset card line counter and read card
            //else if(Confirm==2){ModeSelect==3;} //If finished, change to Mode3

        }
        stktop1=stkptr1;                    //COMMENT OUT FOR DEMO
        stkptr1=&Stack1[60];
        stkptr2=&Stack2[60];
        stkptr3=&Stack3[60];
    }
    else
    {
        while(ModeSelect==2){
            CCP5CONbits.CCP5M = 8;
            CCP1CONbits.CCP1M = 8;
            CCP2CONbits.CCP2M = 8;
            //CCP3CONbits.CCP3M = 8;
            //CCP4CONbits.CCP4M = 8;
            *stkptr1=PORTF;
            *stkptr2=PORTE;
            Delay10KTCYx(1);
        }
    }
    CCP5CONbits.CCP5M = 0;

}
void CaptureLine() {
    Line1 = PORTE;                      //Capture first 5 bits of line (RE1-RE5)
    Line1>>1;
    Line1 & 0b00011111;
    Line1=~Line1;
    *stktop1 = Line1;                   //Store captured bits in Stack1
    //Display note name on LCD
    stktop1++;
    
    Line2 = PORTF;                      //Capture last 5 bits of line (RF1-RF5)
    Line2>>1;
    Line2 & 0b00011111;
    Line2=~Line2;
    *stktop2 = Line2;                   //Store captured bits in Stack2
    //Display note name on LCD
    stktop2++;
}

/******************************************************************************
 * MODE 3: TempoSelect (RB6 LED on, RB5 and RB7 LEDs off)
 *
 * 1. Read potentiometer and convert to tempo
 * 2. Store converted value as tempo for a quarter note (individual punch card lines represent 16th notes)
 * 3. Display 1[1/4 Note] 2[ ___bpm ] converted AND scaled value of potentiometer on LCD (DisplayTempo = Tempo*8 = (Pot*XXX)*8)
 * 
 ******************************************************************************/
void MODE3(){
    Confirm=0;
    CCP5CONbits.CCP5M = 0;
    CCP1CONbits.CCP1M = 0;
    CCP2CONbits.CCP2M = 0;
    DisplayC(Tempo1);
    while(ModeSelect==3){
        MeasurePot();
    }
}
void MeasurePot(){
    ADCON0 = 0b00000111;                                    //<0>1=On, <1>1=GO, <2-6>00001=AN1, <7>0=Unused
    while(ADCON0bits.GO==1){
        DisplayTempo=(Pot*0.0443)+59;                        //Converting Potentiometer's 0-3.3V range to 60-240bpm range
        Length16th=(60.0*250000.0)/(4.0*DisplayTempo);                            //Timing for a 16th note 
        TempoHundreds = DisplayTempo/100;                   //Isolate 100s, 10s, and 1s place of tempo to display
        DisplayTempo = DisplayTempo-(100*TempoHundreds);    
        TempoTens = DisplayTempo/10;
        DisplayTempo = DisplayTempo-(10*TempoTens);
        TempoOnes = DisplayTempo;
        if(TempoHundreds==0){TempoHundreds = ' ';}
        else{TempoHundreds = TempoHundreds|0x30;}                 //Convert display values to ASCII
        TempoTens = TempoTens|0x30; 
        TempoOnes = TempoOnes|0x30;
        Tempo2[1] = TempoHundreds;                        //Place tempo values into display string
        Tempo2[2] = TempoTens;
        Tempo2[3] = TempoOnes;
        DisplayV(Tempo2);                                   //Update the display
    }
    Pot=ADRES;    
}

/******************************************************************************
 * Mode 4: Playback (RB7 LED on, RB5 and RB6 LEDs off)
 *
 * 1. Point to bottom of stack
 * 2. Set PWM half-period value to first value in stack for each piezo speaker
 * 3. Wait appropriate amount of time (delay = DisplayTempo/8 = Tempo)
 * 4. Set PWM half-period value to next value in stack for each piezo speaker
 * 5. Repeat steps 3. and 4. until top of stack is reached
 * 6. When top of stack is reached, prompt replay
 * 
 ******************************************************************************/
void MODE4(){
    //Confirm=0;
    //while(Confirm==0){}                 //Wait for user response
    //if(Confirm==1){Playback();}                //If more cards are to be read, reset card line counter and read card
    while(1){
    DisplayC(Playing1);
    DisplayC(Playing2);
    Playback();
    while(ModeSelect==4){};
    }
}

void Playback(){
    unsigned char i=0;
    CCP1CONbits.CCP1M = 8;
    CCP2CONbits.CCP2M = 8;
    CCP3CONbits.CCP3M = 8;
    CCP4CONbits.CCP4M = 8;
    
    //stkptr1=&Stack1[0];
    //stkptr2=&Stack2[0];
    //stkptr3=&Stack3[0];
    while(stkptr1!=stktop1){
        while(TempoFlag==0){}
        TempoFlag=0;
        stkptr1++;
        stkptr2++;
        stkptr3++;
        i++;
    }
    CCP1CONbits.CCP1M = 0;
    CCP2CONbits.CCP2M = 0;
    CCP3CONbits.CCP3M = 0;
    CCP4CONbits.CCP4M = 0;            
    stkptr1=&Stack1[60];
    stkptr2=&Stack2[60];
    stkptr3=&Stack3[60];
    stktop1=&Stack1[920];
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/
#pragma interrupt HiPriISR
void HiPriISR() {
    LATBbits.LATB4=0;
    if(PORTBbits.RB0==1)
    {
        while(PORTBbits.RB0==1){}
        //Confirm=1;
    }
    else if(PORTBbits.RB1==1){
        while(PORTBbits.RB1==1){}
        //Confirm=2;
    }
    if(ModeSelect==4){;}
    else{ModeSelect++;}
    INTCONbits.INT0IF=0;
    INTCON3bits.INT1IF=0;
    Delay10KTCYx(150);
    INTCONbits.INT0IF=0;
    INTCON3bits.INT1IF=0;
    LATBbits.LATB4=1;

}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR1IF and CCP1IF are clear.
 ******************************************************************************/
#pragma interruptlow LoPriISR nosave=TBLPTR, TBLPTRU, TABLAT, PCLATH, PCLATU, PROD, section("MATH_DATA")//, section(".tmpdata")
void LoPriISR() {
    while(PIR3bits.CCP1IF!=0 || PIR3bits.CCP2IF!=0 || PIR4bits.CCP3IF!=0 || PIR4bits.CCP4IF!=0){              //Exit only once all interrupt flags are cleared
        if(PIR3bits.CCP1IF){
                CCP1NoteTone();
                PIR3bits.CCP1IF = 0;      //Clear flag and return to polling routine
        }
        else if(PIR3bits.CCP2IF){
            CCP2NoteTone();
            PIR3bits.CCP2IF = 0;      //Clear flag and return to polling routine
        }
        else if(PIR4bits.CCP3IF){
            CCP3NoteTone();
            PIR4bits.CCP3IF = 0;      //Clear flag and return to polling routine
        }
        else if(PIR4bits.CCP4IF){
            CCP4Tempo();
            PIR4bits.CCP4IF = 0;      //Clear flag and return to polling routine
        }
    }
    while(PIR4bits.CCP5IF!=0){ 
            CCP5ServoFeed();
            PIR4bits.CCP5IF = 0;      //Clear flag and return to polling routine
    }
}

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * Change the temporary data section for all code following this #pragma
 * statement. Normally the compiler/linker uses .tmpdata section for all non
 * ISR code and a separate tempdata section for each ISR (Hi and Lo). However,
 * when your primary ISR calls other functions the called functions share the
 * .tmpdata section with the main code. This can cause issues, just like not
 * saving WREG, STATUS and BSR. There are two options:
 *
 *   1. have the ISR save the .tmpdata section. This may have large effects on
 *      latency
 *   2. Force the functions called by the ISR to have their own temp data
 *      section.
 *
 * We are using option 2. However, this means that no main code functions can
 * appear after the following pragma, unless you change the temp data section
 * back.
 *
 * The temp data section is used for complex math operations such as 
 * a = b*c + d*e so you may not need a temp data section depending on what
 * you are doing.
 *
 * Keep in mind you may have the same issue with the MATH_DATA section.
 * MATH_DATA is used for arguments, return values and temporary locations
 * for math library functions.
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#pragma tmpdata handler_temp

/******************************************************************************
 * TMR1handler interrupt service routine
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/

void CCP1NoteTone() {
    if(*stkptr1==0){}
    else{LATAbits.LATA5=~LATAbits.LATA5;}
    CCPR1=CCPR1+(Note[(*stkptr1+45)]/2);
}

void CCP2NoteTone() {
    if(*stkptr2==0){}
    else{LATAbits.LATA4=~LATAbits.LATA4;}
    CCPR2=CCPR2+(Note[(*stkptr2+22)]/2);
}

void CCP3NoteTone() {
    if(*stkptr3==0){}
    else{LATCbits.LATC5=~LATCbits.LATC5;}
    CCPR3=CCPR3+(Note[(*stkptr3)]/2);
}

void CCP4Tempo() {
    if(TempoCounter==10){
        TempoCounter=0;
        TempoFlag=1;
    }
    else{
        TempoCounter++;
    }
    CCPR4=CCPR4+Length16th;
    PIR4bits.CCP4IF = 0;      //Clear flag and return to polling routine
}

void CCP5ServoFeed() {
    CCPR5=CCPR5+1250;
    if(Servo_Counter < Servo_Duty){
        Servo_Counter++;
    }
    else{
        //CCP5CONbits.CCP5M0 = ~CCP5CONbits.CCP5M0;
        LATCbits.LATC3 = ~LATCbits.LATC3;
        Servo_Counter=0;
        if(LATCbits.LATC3 == 1){Servo_Duty = Servo_On;}
        else{Servo_Duty = Servo_Off;}
    }
    PIR4bits.CCP5IF = 0;      //Clear flag and return to polling routine
}

void Delay10KTCYx( unsigned char unit );