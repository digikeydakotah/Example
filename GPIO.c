// PIC32MX360F512L Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (PLL Divide by 4)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = ON            // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>


main()
{
    /*Configure tri-state registers*/
    TRISDbits.TRISD6 = 1;   //SW1 as input
    TRISDbits.TRISD7 = 1;   //SW2 as input
    TRISDbits.TRISD13 = 1;  //SW3 as input
    
    TRISDbits.TRISD0 = 0;   //LED1 as output
    TRISDbits.TRISD1 = 0;   //LED2 as output
    TRISDbits.TRISD2 = 0;   //LED3 as output
    
    while(1)
    {
        LATDbits.LATD0 = !PORTDbits.RD6;    //LED1 turns on when SW1 is pressed
        LATDbits.LATD1 = !PORTDbits.RD7;    //LED2 turns on when SW2 is pressed
        LATDbits.LATD2 = !PORTDbits.RD13;   //LED3 turns on when SW3 is pressed
    }
}