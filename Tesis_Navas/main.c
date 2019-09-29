#include <msp430.h> 
#include <stdbool.h>
#include <stdint.h>

#define BUTTON  BIT1
#define LED1    BIT0
#define LED2    BIT7
#define UART0_RX BIT4
#define UART0_TX BIT3
#define UART1_RX BIT5
#define UART1_TX BIT4
#define I2C1_SCL BIT2
#define I2C1_SDA BIT1
#define UART0_BUFF_SIZE 40

/*Registros del RTC usado en la prueba*/
#define RTC_slave_dir       0x68
#define RTC_seconds         0x00

/*Registros del Acelerometro ADXL355*/

#define ADXL355_dir     0x53
#define Devid_AD        0x00
#define Devid_MST       0x01
#define PARTID          0x02
#define REVID           0x03
#define Status          0x04
#define FiFo_Entries    0x05
#define TEMP2           0x06
#define TEMP1           0x07
#define XDATA3          0x08
#define XDATA2          0x09
#define XDATA1          0x0A
#define YDATA3          0x0B
#define YDATA2          0x0C
#define YDATA1          0x0D
#define ZDATA3          0x0E
#define ZDATA2          0x0F
#define ZDATA1          0x10
#define FiFo_Data       0x11
#define Offset_X_H      0x1E
#define Offset_X_L      0x1F
#define Offset_Y_H      0x20
#define Offset_Y_L      0x21
#define Offset_Z_H      0x22
#define Offset_Z_L      0x23
#define ACT_EN          0x24
#define ACT_THRESH_H    0x25
#define ACT_THRESH_L    0x26
#define ACT_COUNT       0x27
#define Filter          0x28
#define FiFo_Samples    0x29
#define INT_MAP         0x2A
#define Sync            0x2B
#define RANGE           0x2C
#define POWER_CTL       0x2D
#define SELF_TEST       0x2E
#define Reset           0x2F

char buffer[3];
char buffer_1byte[3];

uint32_t Resultado_20_bits_X;
unsigned char XData3 = 0;
unsigned char XData2 = 0;
unsigned char XData1 = 0;


/*Prototipado de funciones*/
void Disable_Watchdog(void);
void Config_Register(void);
void Enable_Interrupts(void);

void UART0_init(void);
void UART0_send(char data);
void UART0_putstring(char *Stringptr);
void Enable_UART0(void);

void UART1_init(void);
void UART1_send(char data);
void UART1_putstring(char *Stringptr);
void Enable_UART1(void);


void I2C_init(void);
void I2C_transmit(unsigned char slave_address, unsigned char slave_register, unsigned char data);
unsigned char I2C_receive(unsigned char slave_address, unsigned char slave_register);
void Enable_I2C(void);


unsigned char decimaltobcd(unsigned char value);
unsigned char bcdtodecimal(unsigned char value);
void RTC_I2C_set_seconds(unsigned char slave_address, unsigned char slave_register, unsigned char data);
unsigned char RTC_I2C_get_seconds(unsigned char slave_address, unsigned char slave_register);

void Acelerometer_I2C_set(unsigned char slave_address, unsigned char slave_register, unsigned char data);
unsigned char Acelerometer_I2C_get_Axis(unsigned char slave_address, unsigned char register);
void Read_from_Acelerometer_I2C(unsigned char slave_address, unsigned char slave_register,unsigned char cuantity_registers);

void itoa(long unsigned int inteiro, char* string, int base);

int main(void)
{
    Disable_Watchdog();
    Config_Register();
    UART0_init();
    UART1_init();
    Enable_I2C();
    I2C_init();
    Enable_UART0();
    Enable_UART1();

    UART0_putstring("Megaproyecto: Sismografo UMG");
    UART1_putstring("Megaproyecto: Sismografo UMG");
    Acelerometer_I2C_set(ADXL355_dir,RANGE,0x01);
    Acelerometer_I2C_set(ADXL355_dir,POWER_CTL,0x06);
    //RTC_I2C_set_seconds(RTC_slave_dir,RTC_seconds,0);
    Enable_Interrupts();


    while(1)
    {
        XData3 = Acelerometer_I2C_get_Axis(ADXL355_dir,XDATA3);
        XData2 = Acelerometer_I2C_get_Axis(ADXL355_dir,XDATA2);
        XData1 = Acelerometer_I2C_get_Axis(ADXL355_dir,XDATA1);
        UART0_putstring("El valor del XDATA3: ");
        itoa(XData3,buffer_1byte,10);
        UART0_putstring(buffer_1byte);
        UART0_putstring("El valor del XDATA2: ");
        itoa(XData2,buffer_1byte,10);
        UART0_putstring(buffer_1byte);
        UART0_putstring("El valor del XDATA3: ");
        itoa(XData1,buffer_1byte,10);
        UART0_putstring(buffer_1byte);

        Resultado_20_bits_X = (XData3<<12|XData2<<4|XData1>>4);
        _delay_cycles(500000);


        //P1OUT^=(BIT1);
        //_delay_cycles(500000);
       //itoa(RTC_I2C_get_seconds(RTC_slave_dir,RTC_seconds),buffer,10);
       //UART1_putstring(buffer);
       //_delay_cycles(500000);


    }
}




/*Declaracion de Funciones*/

void Disable_Watchdog(void)
{
    WDTCTL = WDTPW | WDTHOLD;
}

void Config_Register(void)
{
        P2DIR &=~(BUTTON);
        P2REN |=(BUTTON);
        P2OUT |=(BUTTON);

        P1DIR |=LED1;
        P1OUT &=~(LED1);

       // P4SEL |=(I2C1_SCL + I2C1_SDA);
}

void UART0_init(void)
{
    P3SEL |=(UART0_TX + UART0_RX);
    UCA0CTL1 |= UCSWRST;    //
    UCA0CTL1 |= UCSSEL_1;   //Auxiliary Clock ~32Khz
    UCA0BR0 = 3;
    UCA0BR1 = 0;
    UCA0MCTL |= (UCBRS_3 + UCBRF_0 );

}

void UART0_send(char data)
{
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = data;
}

void UART0_putstring(char *Stringptr)
{
    while(*Stringptr !=0x00)
    {
        UART0_send(*Stringptr);
        Stringptr++;
    }
    UART0_send(0x0D);
    UART0_send(0x0A);
}

void Enable_UART0(void)
{
    UCA0CTL1 &= ~UCSWRST;          //Ya puedo usar la maquina del uart
}

void UART1_init(void)
{

    P4SEL |=(UART0_TX + UART0_RX);
    UCA1CTL1 |= UCSWRST;
    UCA1CTL1 |= UCSSEL_1;   //Auxiliary Clock ~32Khz
    UCA1BR0 = 3;
    UCA1BR1 = 0;
    UCA1MCTL |= (UCBRS_3 + UCBRF_0 );

}


void UART1_send(char data)
{
    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = data;
}

void UART1_putstring(char *Stringptr)
{
    while(*Stringptr !=0x00)
    {
        UART1_send(*Stringptr);
        Stringptr++;
    }
    UART1_send(0x0D);
    UART1_send(0x0A);
}

void Enable_UART1(void)
{
    UCA1CTL1 &= ~UCSWRST;          //Ya puedo usar la maquina del uart
}

void Enable_Interrupts(void)
{
    UCA0IE   |= UCRXIE; //Interrupcion por recepcion de uart0
    _bis_SR_register(GIE);
}

void I2C_init(void)
{
    UCB1CTL1 |= UCSWRST;
    UCB1CTL1 |=UCSSEL_2 + UCSWRST;
    UCB1CTL0 |=UCSYNC + UCMODE_3 + UCMST;
    UCB1BR0 = 12; //400Khz I2C Fast Mode
    UCB1BR1 = 0;
    P4DIR &=(I2C1_SCL + I2C1_SDA); //Como entrada
    P4REN |=(I2C1_SCL + I2C1_SDA);
    P4OUT |=(I2C1_SCL + I2C1_SDA);
    P4SEL |=(I2C1_SCL + I2C1_SDA);
}

void Enable_I2C(void)
{
    UCB1CTL1 &=~(UCSWRST);
}

void I2C_transmit(unsigned char slave_address, unsigned char slave_register, unsigned char data)
{
    UCB1I2CSA = slave_address;
    UCB1CTL1 |=UCTR + UCTXSTT;
    UCB1TXBUF = slave_register;
   // while(!(UCB1IFG & UCTXIFG));
    //while(UCB1CTL1 & UCTXSTT);
    UCB1TXBUF = data;
    //while(!(UCB1IFG & UCTXIFG));
    UCB1CTL1 |= UCTXSTP;
   // while(UCB1CTL1 & UCTXSTP);
}

unsigned char I2C_receive(unsigned char slave_address, unsigned char slave_register)
{
   unsigned char data;

   UCB1I2CSA = slave_address;
   UCB1CTL1 |= UCTXSTT + UCTR;
   UCB1TXBUF = slave_register;
  // while(!(UCB1IFG & UCTXIFG));
//   while(UCB1CTL1 & UCTXSTT);
   UCB1CTL1 |= UCTXSTT;
   UCB1I2CSA = slave_address;
   UCB1CTL1 &=~(UCTR);
   //while(UCB1CTL1 & UCTXSTT);
   data = UCB1RXBUF;
   UCB1CTL1 |= UCTXSTP;
//   while(UCB1CTL1 & UCTXSTP);
   return data;

}


unsigned char decimaltobcd(unsigned char value)
{
    return((value/10*16)+(value%10));
}

unsigned char bcdtodecimal(unsigned char value)
{
    return((value/16*10)+(value%16));
}

void RTC_I2C_set_seconds(unsigned char slave_address, unsigned char slave_register, unsigned char data)
{
    I2C_transmit(slave_address,slave_register,decimaltobcd(data));
}

unsigned char RTC_I2C_get_seconds(unsigned char slave_address, unsigned char slave_register)
{
    unsigned char data;
    data=I2C_receive(slave_address,slave_register);
    return (bcdtodecimal(data) & 0x7F);
}

void Acelerometer_I2C_set(unsigned char slave_address, unsigned char slave_register, unsigned char data)
{
    I2C_transmit(slave_address,slave_register,data);
}

unsigned char Acelerometer_I2C_get_Axis(unsigned char slave_address, unsigned char slave_register)
{
    unsigned char data;
    data = I2C_receive(slave_address,slave_register);
    return data;
}
void Read_from_Acelerometer_I2C(unsigned char slave_address, unsigned char slave_register,unsigned char cuantity_registers)
{

    unsigned char i = 0;
    UCB1I2CSA = slave_address;
    while(!(UCB1IFG & UCTXIFG));
    UCB1CTL1 |= UCTXSTT + UCTR;
    UCB1TXBUF = slave_register;
    while(!(UCB1IFG & UCTXIFG));
    while(UCB1CTL1 & UCTXSTT);
    UCB1CTL1 |= UCTXSTT;
    UCB1I2CSA = slave_address;
    UCB1CTL1 &=~(UCTR);
    while(UCB1CTL1 & UCTXSTT);
    buffer[0] = UCB1RXBUF;
    while(!(UCB1IFG & UCRXIFG));
    for(i=0;i<cuantity_registers-1;i++)
    {
        buffer[i]=UCB1RXBUF;
        while(!(UCB1IFG & UCRXIFG));
    }
    buffer[cuantity_registers-1]=UCB1RXBUF;
    while(!(UCB1IFG & UCRXIFG));
    UCB1CTL1 |= UCTXSTP;
    while(UCB1CTL1 & UCTXSTP);
}
void itoa(long unsigned int inteiro, char* string, int base){
    // por http://www.strudel.org.uk/itoa/

    // checa se a base é válida
    if (base < 2 || base > 36) {
        *string = '\0';
    }

    char *ptr = string, *ptr1 = string, tmp_char;
    int tmp_inteiro;

    do {
        tmp_inteiro = inteiro;
        inteiro /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_inteiro - inteiro * base)];
    } while ( inteiro );

    // Aplica sinal negativo
    if (tmp_inteiro < 0) *ptr++ = '-';

    *ptr-- = '\0';

    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
}
