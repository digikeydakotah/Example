#include "project.h"

void INITIALIZEEVERYTHINGPLEASE(void)
{
    initGPIO();
    initSPI();
    initAccel();
    initDriver();
    initADC();
    //initStallGuard(THRESHOLD,SENSITIVITY);
    initUART();
    initTimer(64);
    LED1=LED2=LED3=LED4=LED5=LED6=0;
}



void initGPIO(void)
{
    TRISEbits.TRISE4 = 0;  //CS1
    TRISEbits.TRISE3 = 0;  //CS2
    TRISDbits.TRISD1 = 0;  //CS3
    TRISDbits.TRISD7 = 0;  //LED1
    TRISDbits.TRISD6 = 0;  //LED2
    TRISDbits.TRISD5 = 0;  //LED3
    TRISBbits.TRISB2 = 0;  //LED5
    TRISBbits.TRISB3 = 0;  //LED6
    TRISDbits.TRISD13 = 0; //LED4
    TRISAbits.TRISA2 = 1;  //MODE
    TRISGbits.TRISG2 = 1;  //DOWN
    TRISAbits.TRISA3 = 1;  //UP
    TRISGbits.TRISG3 = 1;  //SELECT
}



void initSPI(void)
{
    CS1 = CS2 = CS3 = 1;              // Set CS high
    
    IEC0bits.SPI1EIE = 0;       // SPI interrupts disabled
    IEC0bits.SPI1RXIE = 0;
    IEC0bits.SPI1TXIE = 0;
    
    SPI1CONbits.ON = 0;         // Turn off SPI module
    
    SPI1BUF = 0;                // Clear the receive buffer
    
    SPI1BRG = 3;                // FSCK = 2.5MHz
    
    SPI1STATbits.SPIROV = 0;    // Clear overflow flag
    
    
    /* SPI1CON settings */
    SPI1CONbits.FRMEN = 0;      // Framed SPI support is disabled
    SPI1CONbits.SIDL = 0;       // Continue operation in IDLE mode
    SPI1CONbits.DISSDO = 0;     // SDO1 pin is controlled by the module
    SPI1CONbits.MODE16 = 0;     // 8 bit mode
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.CKP = 1;        // Idle state for clock is high, active state is low
    SPI1CONbits.CKE = 0;        // Output data changes on transition from idle to active
    SPI1CONbits.SSEN = 0;       // Not in slave mode
    SPI1CONbits.MSTEN = 1;      // Master mode
    SPI1CONbits.SMP = 1;        // Input data sampled at the end of data output time
    
    SPI1CONbits.ON = 1;         // Turn module on
}



void initDriver(void)
{
    unsigned char address;
    unsigned long data;
    int i = 1;

    address = 0x90;
    data = 0x00071808;              // register 0x10, IHOLD_IRUN
    ReadWrite(1, &address, &data);
    
    address = 0x90;
    data = 0x00071500;              // register 0x10, IHOLD_IRUN
    ReadWrite(2, &address, &data);
    
    for(; i<3 ; i++)
    {

        address = 0x91;
        data = 0x000000FF;              // register 0x11, TPOWERDOWN
        ReadWrite(i, &address, &data);

        address = 0xE0;
        data = 0xAAAAB554;              // register 0x60, MSLUT[0]
        ReadWrite(i, &address, &data);

        address = 0xE1;
        data = 0x4A9554AA;              // register 0x61, MSLUT[1]
        ReadWrite(i, &address, &data);

        address = 0xE2;
        data = 0x24492929;              // register 0x62, MSLUT[2]
        ReadWrite(i, &address, &data);

        address = 0xE3;
        data = 0x10104222;              // register 0x63, MSLUT[3]
        ReadWrite(i, &address, &data);

        address = 0xE4;
        data = 0xFBFFFFFF;              // register 0x64, MSLUT[4]
        ReadWrite(i, &address, &data);

        address = 0xE5;
        data = 0xB5BB777D;              // register 0x65, MSLUT[5]
        ReadWrite(i, &address, &data);

        address = 0xE6;
        data = 0x49295556;              // register 0x66, MSLUT[6]
        ReadWrite(i, &address, &data);

        address = 0xE7;
        data = 0x00404222;              // register 0x67, MSLUT[7]
        ReadWrite(i, &address, &data);

        address = 0xE8; 
        data = 0xFFFF8056;              // register 0x68, MSLUTSEL
        ReadWrite(i, &address, &data);

        address = 0xE9;
        data = 0x00F70000;              // register 0x69, MSLUTSTART
        ReadWrite(i, &address, &data);

        address = 0xEC;
        data = 0x000101D5;              // register 0x6C, CHOPCONF
        ReadWrite(i, &address, &data);

        address = 0xA0;
        data = 0x00000000;              // register 0x20, RAMPMODE
        ReadWrite(i, &address, &data);

        address = 0xA3; 
        data = 0x00000003;              // register 0x23, VSTART
        ReadWrite(i, &address, &data);

        address = 0xA4; 
        data = 0x000003E8;              // register 0x24, A1
        ReadWrite(i, &address, &data);

        address = 0xA5;
        data = 0x00000000;              // register 0x25, V1
        ReadWrite(i, &address, &data);

        address = 0xA6;
        data = 2000000;                 // register 0x26, AMAX
        ReadWrite(i, &address, &data);

        address = 0xA7;
        data = 0x0007A120;              // register 0x27, VMAX
        ReadWrite(i, &address, &data);

        address = 0xA8;
        data = 2000000;                 // register 0x28, DMAX
        ReadWrite(i, &address, &data);

        address = 0xAA;
        data = 0x00000578;              // register 0x2A, D1
        ReadWrite(i, &address, &data);

        address = 0xAB;
        data = 0x0000000A;              // register 0x2B, VSTOP
        ReadWrite(i, &address, &data);
        
    }
    
    address = 0x80;
    data = 4;                       // register 0x00, GCONF
    ReadWrite(2, &address, &data);

    address = 0x93;
    data = 5000;                    // register 0x13, TPWMTHRS
    ReadWrite(2, &address, &data);

    address = 0xF0;
    data = 0x60100;                 // register 0x70, PWMCONF
    ReadWrite(2, &address, &data);
}



void initADC(void)
{
    AD1PCFGbits.PCFG0 = 0;          // Analog input in Analog mode
    AD1PCFGbits.PCFG1 = 0;
    TRISBbits.TRISB0 = 1;           // Pin set as input
    TRISBbits.TRISB1 = 1;
    
    AD1CHSbits.CH0NA = 0;           // Channel 0 negative input is VR-
    AD1CHSbits.CH0SA = 0b0000;      // Channel 0 positive input is AN0
    
    AD1CON1bits.FORM = 0b000;       // Integer 16-bit output
    
    AD1CON1bits.SSRC = 0b111;       // Internal counter ends sampling and starts conversion
    
    AD1CSSL = 0;                    // No scanning required
    
    AD1CON2bits.VCFG = 0b000;       // Internal voltage references
    
    AD1CON2bits.CSCNA = 0;          // Do not scan inputs
    
    AD1CON2bits.BUFM = 0;           // Buffer configured as one 16-word buffer
    
    AD1CON2bits.ALTS = 0;           // Always use MUX A input multiplexer settings
    
    AD1CON3bits.ADRC = 0;           // Clock derived from PBclock
    AD1CON3bits.ADCS = 0b00111111;  // TAD = 2*TPB
    
    AD1CON3bits.SAMC = 0b11111;     // 31 TAD auto-sample time
    
    AD1CON1bits.ON = 1;             // A/D converter module is operating
}



void initStallGuard(unsigned long threshold, unsigned long sensitivity)
{
    unsigned char address;
    unsigned long data;
    int i = 1;
    
    for(; i<3; i++)
    {
        address = 0x94;
        data = threshold;
        ReadWrite(i, &address, &data);     // TCOOLTHRS (Velocity threshold)

        address = 0xED;                    // SGT (Sensitivity)
        data = sensitivity<<16;
        ReadWrite(i, &address, &data);

        address = 0xB4;
        data = 1<<10;                     // sg_stop (Enable)
        ReadWrite(i, &address, &data);
    }
    
}



void initUART(void)
{
    U1MODEbits.BRGH = 0;                // Baud Rate = 9600
    U1BRG = 129;
    
    U1MODEbits.SIDL = 0;                // Continue operation in SLEEP mode
    
    U1MODEbits.IREN = 0;                // IrDA is disabled
    
    U1MODEbits.RTSMD = 0;               // U1RTS pin is in Flow Control mode
    
    U1MODEbits.UEN = 0b00;              // U1TX, U1RX are enabled
    
    U1MODEbits.WAKE = 1;                // Wake-up enabled
    
    U1MODEbits.LPBACK = 0;              // Loopback mode is disabled
    
    U1MODEbits.RXINV = 0;               // U1RX IDLE state is '1'
    
    U1MODEbits.PDSEL = 0b00;            // 8-bit data, no parity
    
    U1MODEbits.STSEL = 0;               // 1 stop bit
    
    U1STAbits.UTXINV = 0;               // U1TX IDLE state is '1'
    
    U1MODEbits.ON = 1;                  // UART1 is enabled
    
    U1STAbits.URXEN = 1;                // UART1 receiver is enabled
    
    U1STAbits.UTXEN = 1;                // UART1 transmitter is enabled
    
    PORTDbits.RD14 = 0;
    PORTDbits.RD15 = 0;
}



void initAccel(void)
{
    WriteReadAccel(0b0010000001100111); // CTRL_REG1_XM
    WriteReadAccel(0b0010010010010100); // CTRL_REG5_XM
    WriteReadAccel(0b0010010100000000); // CTRL_REG6_XM
    WriteReadAccel(0b0010011000000000); // CTRL_REG7_XM
}



void initTimer(int prescale)
{
    T2CON = 0x8030;                     // Enable Timer2 (prescaler = 8)
    
    if(prescale == 256)                 // Adjust prescaler if wanted
        T2CONbits.TCKPS = 7;
    else if(prescale == 64)
        T2CONbits.TCKPS = 6;
    else if(prescale == 32)
        T2CONbits.TCKPS = 5;
    else if(prescale == 16)
        T2CONbits.TCKPS = 4;
    else if(prescale == 8)
        T2CONbits.TCKPS = 3;
    else if(prescale == 4)
        T2CONbits.TCKPS = 2;
    else if(prescale == 2)
        T2CONbits.TCKPS = 1;
    else if(prescale == 1)
        T2CONbits.TCKPS = 0;
}



/*void homing(void)
{
    ResetSG();                          // Make sure StallGuard is set
    
    MoveAt(1,"cw",40000);              // Horizontal motor
    MoveAt(2,"ccw",40000);             // Vertical motor
    
    Delay(8000);
    
    SetPos(1,0);                        // Horizontal set to 0
    //SetPos(2,147000);                   // Vertical set to 180
    SetPos(2,129600);
    ResetSG();
    
    //MoveTo(2, 73500, 40000);
    MoveTo(2,64800,40000);
    MoveTo(1,64800,40000);
    Delay(3000);
}



void Homing(void)
{
    ResetSG();                          // Make sure StallGuard is set
    
    MoveAt(2,"ccw",40000);             // Vertical motor
    
    Delay(6000);
    
    SetPos(1,0);                        // Horizontal set to 0
    MoveTo(1,0,40000);
    SetPos(2,129600);                   // Vertical set to 180
    ResetSG();
    
    MoveTo(2,64800,40000);
    Delay(3000);
}*/



void Homing(void)
{
    while(!SELECT)
        Joystick();
    while(SELECT);
    Delay(50);
    SetPos(1,64800);
}



void Delay(int delay)
{
    int i = 0;
    
    for(; i<delay; i++)
    {
        TMR2 = 0;
        while(TMR2 < DELAY);
    }
}



void ReadWrite(int CS, unsigned char *address, unsigned long *data)
{
    SPI1CONbits.MODE16 = 0;             // 8 bit mode for address
    SPI1CONbits.MODE32 = 0;
    
    if(CS==1)                           // Chip select
        CS1 = 0;
    
    else if(CS==2)
        CS2 = 0;
   
    SPI1BUF = *address;                 // Send address
    while (!SPI1STATbits.SPIRBF);
    *address = SPI1BUF;
        
    SPI1CONbits.MODE16 = 1;             // 32 bit mode for data
    SPI1CONbits.MODE32 = 1;
    
    SPI1BUF = *data;                    // Send data
    while(!SPI1STATbits.SPIRBF);
    *data = SPI1BUF;
    
    if(CS==1)                           // Chip select
        CS1 = 1;
    
    else if(CS==2)
        CS2 = 1;
}



short WriteReadAccel(unsigned short i)
{
    SPI1CONbits.MODE16 = 1; // 16 bit mode
    SPI1CONbits.MODE32 = 0;
    CS3 = 0;
    SPI1BUF = i;                    // Write to buffer for transmission
    while (!SPI1STATbits.SPIRBF);   // Wait for transfer complete
    CS3 = 1;
    return SPI1BUF;                 // Read the received value
}



float ReadAccelX(void)
{
   short X_H = WriteReadAccel(0b1010100100000000);
   short X_L = WriteReadAccel(0b1010100000000000);
   X_L = X_L & 0b0000000011111111;
   X_H = X_H << 8;
   X_H = X_H & 0b1111111100000000;
   signed short X = X_H | X_L;
   float value = (X * 0.000061 * 0.9958)-0.0269;
   return value;
}

float ReadAccelY(void)
{
   short Y_H = WriteReadAccel(0b1010101100000000);
   short Y_L = WriteReadAccel(0b1010101000000000);
   Y_L = Y_L & 0b0000000011111111;
   Y_H = Y_H << 8;
   Y_H = Y_H & 0b1111111100000000;
   signed short Y = Y_H | Y_L;
   float value = (Y * 0.000061*0.955)+0.066;
   return value;
}

float ReadAccelZ(void)
{
   short Z_H = WriteReadAccel(0b1010110100000000);
   short Z_L = WriteReadAccel(0b1010110000000000);
   Z_L = Z_L & 0b0000000011111111;
   Z_H = Z_H << 8;
   Z_H = Z_H & 0b1111111100000000;
   signed short Z = Z_H | Z_L;
   float value = Z * 0.000061;
   return value;
}

float ReadMagX(void)
{
   short X_H = WriteReadAccel(0b1000100100000000);
   short X_L = WriteReadAccel(0b1000100000000000);
   X_L = X_L & 0b0000000011111111;
   X_H = X_H << 8;
   X_H = X_H & 0b1111111100000000;
   signed short X = X_H | X_L;
   float value = (X * 0.00008) + 0.18;
   return value;
}

float ReadMagY(void)
{
   short Y_H = WriteReadAccel(0b1000101100000000);
   short Y_L = WriteReadAccel(0b1000101000000000);
   Y_L = Y_L & 0b0000000011111111;
   Y_H = Y_H << 8;
   Y_H = Y_H & 0b1111111100000000;
   signed short Y = Y_H | Y_L;
   float value = (Y * 0.00008) - 0.098;
   return value;
}

float ReadMagZ(void)
{
   short Z_H = WriteReadAccel(0b1000110100000000);
   short Z_L = WriteReadAccel(0b1000110000000000);
   Z_L = Z_L & 0b0000000011111111;
   Z_H = Z_H << 8;
   Z_H = Z_H & 0b1111111100000000;
   signed short Z = Z_H | Z_L;
   float value = (Z * 0.00008) - 0.256;
   return value;
}



float AccelAngle(void)
{
    MoveAt(1,"cw",0);
    MoveAt(2,"cw",0);
    float angle,x,y;
    float i;
    char values[40];
    
    angle = 0;
    for(i=0; i<200; i++)
    {
        x = ReadAccelX();
        y = ReadAccelY();
        
        angle = angle + atan2f(y,x);
        Delay(10);
    }
    //sprintf(values,"x: %f y: %f angle: ",x,y);
    //SendString(values);
    angle = (angle*180)/(M_PI*200);
    return angle;
}



int ReadADC(int ch)
{
    AD1CHSbits.CH0SA = ch;          // Select input channel
    AD1CON1bits.SAMP = 1;           // Start sampling
    while(!AD1CON1bits.DONE);       // Wait for conversion to complete
    return ADC1BUF0;                // Read conversion result
}



void MoveTo(int motor, unsigned long position, unsigned long speed)
{
    unsigned char address;
    unsigned long data;
    
    address = 0xA0;                     // RAMPMODE set to position mode
    data = 0;
    ReadWrite(motor, &address, &data);
    
    address = 0xA7;                     // Set speed
    data = speed;
    ReadWrite(motor, &address, &data);
    
    address = 0xAD;                     // XTARGET set to given position
    data = position;
    ReadWrite(motor, &address, &data);   
}



void MoveAt(int motor, char direction[], unsigned long speed)
{
    unsigned char address = 0xA0;       // RAMPMODE set to velocity mode
    unsigned long data;
    
    if( !(strcmp(direction,"CCW")) || !(strcmp(direction,"ccw")) )      // Direction selection
        data = 1;
        
    else if( !(strcmp(direction,"CW")) || !(strcmp(direction,"cw")) )
        data = 2;
    
    ReadWrite(motor, &address, &data);
    
    address = 0xA7;                     // VMAX set to given velocity
    data = speed;
    ReadWrite(motor, &address, &data);
}



void ResetSG(void)
{
    unsigned char address;
    unsigned long data;
    
    address = 0x35;                     // Read RAMP_STAT
    ReadWrite(1, &address, &data);
    address = 0x35;
    ReadWrite(2, &address, &data);
}



unsigned long ReadPos(int motor)
{
    unsigned char address;
    unsigned long data;
    
    address = 0x21;                     // X_ACTUAL
    data = 0;
    ReadWrite(motor, &address, &data);
    
    address = 0x21;
    data = 0;
    ReadWrite(motor, &address, &data);
    
    return data;
}



void SetPos(int motor, unsigned long position)
{
    unsigned char address;
    unsigned long data;
    
    address = 0xA0;                     // RAMPMODE, position mode
    data = 0;
    ReadWrite(motor, &address, &data);
    
    address = 0xA1;
    data = position;                    // Set XACTUAL
    ReadWrite(motor, &address, &data);
    
    address = 0xAD;
    data = position;                    // Set XTARGET to stop movement
    ReadWrite(motor, &address, &data);
}



void SendString(char *string)
{
    
   int i = 0;
    
    U1STAbits.UTXEN = 1;                    // Make sure transmitter is enabled
    
    while(*string)
    {
        while(U1STAbits.UTXBF);             // Wait while buffer is full
        U1TXREG = *string;
        string++;
    }
}



void SendChar(char c)
{
    U1STAbits.UTXEN = 1;                    // Transmitter is enabled
    while(U1STAbits.UTXBF);                 // wait while buffer is full
    U1TXREG = c;
}



void ReadString(char *string, int length)
{   
    int count = length;
    
    do
    {
        *string = ReadChar();               // Read in character
        //SendChar(*string);                  // Echo character
        if(*string == 'x')
            break;
        
        if(*string == 0x7F && count>length) // Backspace conditional
        {
            length++;
            string--;
            continue;
        }
        
        if(*string == '\r')                 // End reading if enter is pressed
            break;
        
        string++;
        length--;
        
    }while(length>1);
    
    *string = '\0';                         // Add null terminator
}



char ReadChar(void) 
{
    while(!U1STAbits.URXDA)                // Wait for received information
    {
        if(RESET)
        {
            while(RESET);
            Reset();
            return 'x';
        }
    }
    return U1RXREG;
}



void FanOn(void)            // Turn fan on
{
    SendString("fan1");
}

void FanOff(void)           // Turn fan off
{
    SendString("fan0");
}

void SemiAuto(void)         // Semi-automatic mode
{
    SendString("semi");
}

void FullAuto(void)         // Full-automatic mode
{
    SendString("full");
}

void Fire(void)             // Fire gun
{
    SendString("fire");
}

void Burst2(void)           // 2-round burst
{
    SendString("2bur");
}

void Burst3(void)           // 3-round burst
{
    SendString("3bur");
}

void Manual(void)
{
    SendString("manu");
}

void Smart(void)
{
    SendString("smrt");
}

void Xvelocity(signed int velocity)
{
    char vel[10];
    SendString("xvel");
    Delay(20);
    sprintf(vel,"%07i",velocity);
    SendString(vel);
}

void Yvelocity(signed int velocity)
{
    char vel[10];
    SendString("yvel");
    Delay(20);
    sprintf(vel,"%07i",velocity);
    SendString(vel);
}



void FindAngles(float *theta, float *phi)
{
    unsigned char address;
    unsigned long data;
    
    address = 0x21;                     // Theta (horizontal) angle     
    data = 0;
    ReadWrite(1, &address, &data);
    
    address = 0x21;
    data = 0;
    ReadWrite(1, &address, &data);
    
    *theta = data/720.0;
    
    
    /*address = 0x21;                     // Phi (vertical) angle
    data = 0;
    ReadWrite(2, &address, &data);
    
    address = 0x21;
    data = 0;
    ReadWrite(2, &address, &data);
    
    *phi = 180.0 - (data/720.0);*/
    *phi = AccelAngle();
}



float FindDistance(float phi)
{
    float d;
    phi = phi * (M_PI/180);
    
    d  = HEIGHT * tanf(phi);
    return d;
}



float PtoRx(float angle, float distance)
{
    float x;
    while(angle<0)
        angle = angle+360;
    while(angle>360)
        angle = angle-360;
    angle = angle * (M_PI/180.0);
    
    x = distance * cosf(angle);
    return x;
}

float PtoRy(float angle, float distance)
{
    float y;
    while(angle<0)
        angle = angle+360;
    while(angle>360)
        angle = angle-360;
    angle = angle * (M_PI/180.0);
    
    y = distance * sinf(angle);
    return y;
}

float RtoPr(float x, float y)
{
    float r;
    
    r = sqrt((x*x) + (y*y));
    return r;
}

float RtoPa(float x, float y)
{
    float a;
    
    a = atan2f(y,x);
    a = a * 180.0/M_PI;
    while (a<0)
        a = a+360;
    while(a>360)
        a = a-360;
    return a;
}



void FindCoords(float *angle, float *distance, float theta1, float distance1, float offsetangle)
{
    float D,L,dx,dy,Dx,Dy,Lx,Ly,beta;
    float phi2,theta2;
    
    FindAngles(&theta2,&phi2);
    
    D = FindDistance(phi2);
    
    dx = PtoRx(theta1,distance1);
    dy = PtoRy(theta1,distance1);
    
    Dx = PtoRx(theta2,D);
    Dy = PtoRy(theta2,D);
    
    Lx = Dx-dx;
    Ly = Dy-dy;
    *distance = RtoPr(Lx,Ly);
    *angle = RtoPa(Lx,Ly) + offsetangle;
}



void FindGun(float *angle, float *distance, float *offsetangle)
{
    float phi1,theta,distance1,theta1,theta2,d1,d2,centerangle,centerlength;

    while(!SELECT)
    {
        Joystick();
        LED5 = 1;
        LED6 = 0;
    }
    MoveAt(1,"cw",0);
    MoveAt(2,"cw",0);

    while(SELECT);
    Delay(500);
    FindAngles(&theta1,&phi1);
    d1 = FindDistance(phi1);

    while(!SELECT)
    {
        Joystick();
        LED5 = 1;
        LED6 = 1;
    }
    MoveAt(1,"cw",0);
    MoveAt(2,"cw",0);

    while(SELECT);
    Delay(500);
    FindCoords(&theta2,&d2,theta1,d1,0);

    if(theta1>90)
        centerangle = theta2 + 45.0;
    else if(theta1<90)
        centerangle = theta2 - 45.0;
    centerlength = (LENGTH/2)*sqrt(2);


    AddVectors(&theta, &distance1, theta1, d1, centerangle, centerlength);

    LED5 = 0;
    LED6 = 0;
    
    *angle = theta;
    *distance = distance1;
    *offsetangle = 90 - theta2;
}



void AddVectors(float *totalangle, float *totaldistance, float angle1, float distance1, float angle2, float distance2)
{
    float x1,x2,y1,y2,xtot,ytot;
    
    x1 = PtoRx(angle1,distance1);
    y1 = PtoRy(angle1,distance1);
    
    x2 = PtoRx(angle2,distance2);
    y2 = PtoRy(angle2,distance2);
    
    xtot = x1+x2;
    ytot = y1+y2;
    
    *totaldistance = RtoPr(xtot,ytot);
    *totalangle = RtoPa(xtot,ytot);
}


void Joystick(void)
{
    int speed, x, y;
    
    x = ReadADC(1);
        
    if(x>550)
    {
        speed =(x-550)*SPEED;
        MoveAt(1,"ccw",speed);
    }
    else if(x<500)
    {
        speed = (500-x)*SPEED;
        MoveAt(1,"cw",speed);
    }
    else
        MoveAt(1,"cw",0);

    y = ReadADC(0);

    if(y>530)
    {
        speed = (y-530)*SPEED;
        MoveAt(2,"cw",speed);
    }
    else if(y<480)
    {
        speed = (480-y)*SPEED;
        MoveAt(2,"ccw",speed);
    }
    else
        MoveAt(2,"cw",0);
}



void JoystickMan(void)
{
    int speed, x, y;
    
    x = ReadADC(1);
        
    if(x>550)
    {
        speed =(x-550)*SPEED*1.3;
        Xvelocity(speed);
    }
    else if(x<500)
    {
        speed = (x-500)*SPEED*1.3;
        Xvelocity(speed);
    }
    else
        Xvelocity(0);

    Delay(10);
    y = ReadADC(0);

    if(y>530)
    {
        speed = (y-530)*SPEED;
        Yvelocity(speed);
    }
    else if(y<480)
    {
        speed = (y-480)*SPEED;
        Yvelocity(speed);
    }
    else
        Yvelocity(0);
}



void Reset(void)
{
    char ok[5];
    
    if(U1STAbits.URXDA)
    {
        U1STAbits.URXDA = 0;
    }
    
    while(!U1STAbits.URXDA)
    {
        SendChar('r');
    }
    
    ReadString(ok,3);
}