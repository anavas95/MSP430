#include <msp430.h> 
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define UART0_RX BIT4
#define UART0_TX BIT3
#define UART1_RX BIT5
#define UART1_TX BIT4
#define I2C1_SCL BIT2
#define I2C1_SDA BIT1
#define UART0_BUFF_SIZE 40

enum State
{
    nitido,
    malo
};

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

union Data_32bits_X
{
    uint8_t data[3];
    uint32_t var_32bits;
};

union Data_32bits_Y
{
    uint8_t data[3];
    uint32_t var_32bits;
};

union Data_32bits_Z
{
    uint8_t data[3];
    uint32_t var_32bits;
};

char buffer_1byte[3];
char axis_buffer[9];
char long_buffer[50];

volatile  uint32_t Resultado_20_bits_X = 0;
//volatile  uint16_t Resultado_16_bits_X = 0; No utilizada

volatile  uint32_t Resultado_20_bits_Y = 0;
//volatile  uint16_t Resultado_16_bits_Y = 0; No utilizada

volatile  uint32_t  Resultado_20_bits_Z = 0;
//volatile  uint16_t Resultado_16_bits_Z = 0; No utilizada

unsigned char XData3 = 0;
unsigned char XData2 = 0;
unsigned char XData1 = 0;

unsigned char YData3 = 0;
unsigned char YData2 = 0;
unsigned char YData1 = 0;

unsigned char ZData3 = 0;
unsigned char ZData2 = 0;
unsigned char ZData1 = 0;

unsigned int fsm_states = 0;


//variables de pruebas

/*Prototipado de funciones*/
void Disable_Watchdog(void);
void Config_Register(void);
void Enable_Interrupts(void);
void binario(int num);

void UART0_init(void);
void UART0_send(char data);
void UART0_putstring(char *Stringptr);
void UART0_putstringE(char *Stringptr);
void Enable_UART0(void);

void UART1_init(void);
void UART1_send(char data);
void UART1_putstring(char *Stringptr);
void UART1_putstringE(char *Stringptr);
void Enable_UART1(void);


void I2C_init(void);
void I2C_transmit(unsigned char slave_address, unsigned char slave_register, unsigned char data);
unsigned char I2C_receive(unsigned char slave_address, unsigned char slave_register);
void Enable_I2C(void);

void Acelerometer_I2C_set(unsigned char slave_address, unsigned char slave_register, unsigned char data);
unsigned char Acelerometer_I2C_get_Axis(unsigned char slave_address, unsigned char register);
void Read_from_Acelerometer_I2C(unsigned char slave_address, unsigned char start_register_slave,char *buffer,unsigned char cuantity_registers);

void itoa(long unsigned int inteiro, char* string, int base);

int main(void)
{

    union Data_32bits_X Resolucion_32_bits_X;
    union Data_32bits_Y Resolucion_32_bits_Y;
    union Data_32bits_Z Resolucion_32_bits_Z;
    Disable_Watchdog();
    Config_Register();
    UART0_init();
    UART1_init();
    I2C_init();
    Enable_UART0();
    Enable_UART1();
    Enable_I2C();
    //UART0_putstring("Megaproyecto: Sismografo UMG");
    UART1_putstring("Megaproyecto: Sismografo UMG");
    Acelerometer_I2C_set(ADXL355_dir,RANGE,1);
    Acelerometer_I2C_set(ADXL355_dir,POWER_CTL,6);
    //Acelerometer_I2C_set(ADXL355_dir,Sync,0);
    ////RTC_I2C_set_seconds(RTC_slave_dir,RTC_seconds,0);
    Enable_Interrupts();

    fsm_states = nitido;


    while(1)
    {
                    UART0_putstring("Pruebas de primera lectura \r\n");
                    Read_from_Acelerometer_I2C(ADXL355_dir,XDATA3,axis_buffer,9);
                    UART0_putstring("El valor del primer byte de X (0x08) es: ");
                    XData3 = axis_buffer[0];
                    Resolucion_32_bits_X.data[2]=XData3;
                    itoa(XData3,buffer_1byte,16);
                    UART0_putstring(buffer_1byte);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);

                    UART0_putstring("El valor del segundo byte de X (0x09) es: ");
                    XData2 = axis_buffer[1];
                    Resolucion_32_bits_X.data[1]=XData2;
                    itoa(XData2,buffer_1byte,16);
                    UART0_putstring(buffer_1byte);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);

                    UART0_putstring("El valor del tercer byte de X (0x0A) es: ");
                    XData1 = axis_buffer[2];
                    Resolucion_32_bits_X.data[0]=XData1;
                    itoa(XData1,buffer_1byte,16);
                    UART0_putstring(buffer_1byte);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);

                    UART0_putstring("La aceleracion en el eje X es de: ");
                    Resultado_20_bits_X = (Resolucion_32_bits_X.var_32bits>>4);
                    ltoa(Resultado_20_bits_X,long_buffer);
                    UART0_putstring(long_buffer);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);



                    UART0_putstring("El valor del primer byte de Y (0x0B) es: ");
                    YData3 = axis_buffer[3];
                    Resolucion_32_bits_Y.data[2]=YData3;
                    itoa(YData3,buffer_1byte,16);
                    UART0_putstring(buffer_1byte);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);

                    UART0_putstring("El valor del segundo byte de Y (0x0C) es: ");
                    YData2 = axis_buffer[4];
                    Resolucion_32_bits_Y.data[1]=YData2;
                    itoa(YData2,buffer_1byte,16);
                    UART0_putstring(buffer_1byte);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);

                    UART0_putstring("El valor del tercer byte de Y (0x0D) es: ");
                    YData1 = axis_buffer[5];
                    Resolucion_32_bits_Y.data[0]=YData1;
                    itoa(YData1, buffer_1byte,16);
                    UART0_putstring(buffer_1byte);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);

                    UART0_putstring("La aceleracion en el eje Y es de: ");
                    Resultado_20_bits_Y = (Resolucion_32_bits_Y.var_32bits>>4);
                    ltoa(Resultado_20_bits_Y,long_buffer);
                    UART0_putstring(long_buffer);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);


                    UART0_putstring("El valor del primer byte de Z (0x0E) es: ");
                    ZData3 = axis_buffer[6];
                    Resolucion_32_bits_Z.data[2]=ZData3;
                    itoa(ZData3, buffer_1byte,16);
                    UART0_putstring(buffer_1byte);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);

                    UART0_putstring("El valor del segundo byte de Z (0x0F) es: ");
                    ZData2 = axis_buffer[7];
                    Resolucion_32_bits_Z.data[1]=ZData2;
                    itoa(ZData2, buffer_1byte, 16);
                    UART0_putstring(buffer_1byte);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);

                    UART0_putstring("El valor del primer byte de Z (0x10) es: ");
                    ZData1 = axis_buffer[8];
                    Resolucion_32_bits_Z.data[0]=ZData1;
                    itoa(ZData1, buffer_1byte,16);
                    UART0_putstring(buffer_1byte);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);

                    UART0_putstring("La aceleracion en el eje Z es de: ");
                    Resultado_20_bits_Z = (Resolucion_32_bits_Z.var_32bits>>4);
                    ltoa(Resultado_20_bits_Z,long_buffer);
                    UART0_putstring(long_buffer);
                    UART0_putstring("\r\n");
                    _delay_cycles(500000);


    }//end while(1)
}//END int main()




/*Declaracion de Funciones*/

void Disable_Watchdog(void)
{
    WDTCTL = WDTPW | WDTHOLD;
}

void Config_Register(void)
{

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
}

void UART0_putstringE(char *Stringptr)
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
}

void UART1_putstringE(char *Stringptr)
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
    P4SEL |=(I2C1_SCL + I2C1_SDA);
    UCB1CTL1 &=~(UCSWRST);
}

void Enable_I2C(void)
{
    UCB1CTL1 &=~(UCSWRST);
}

void I2C_transmit(unsigned char slave_address, unsigned char slave_register, unsigned char data)
{
    UCB1I2CSA = slave_address;
    UCB1CTL1 |= UCTR + UCTXSTT;
    UCB1TXBUF = slave_register;
    while(!(UCB1IFG & UCTXIFG));
    while(UCB1CTL1 & UCTXSTT);
    UCB1TXBUF = data;
    while(!(UCB1IFG & UCTXIFG));
    UCB1CTL1 |= UCTXSTP;
    while(UCB1CTL1 & UCTXSTP);
}

unsigned char I2C_receive(unsigned char slave_address, unsigned char slave_register)
{
   unsigned char data;

   UCB1I2CSA = slave_address;
   UCB1CTL1 |= UCTXSTT + UCTR;
   UCB1TXBUF = slave_register;
   while(!(UCB1IFG & UCTXIFG));
   while(UCB1CTL1 & UCTXSTT);
   UCB1CTL1 |= UCTXSTT;
   UCB1I2CSA = slave_address;
   UCB1CTL1 &=~(UCTR);
   while(UCB1CTL1 & UCTXSTT);
   data = UCB1RXBUF;
   UCB1CTL1 |= UCTXSTP;
   while(UCB1CTL1 & UCTXSTP);
   return data;

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
void Read_from_Acelerometer_I2C(unsigned char slave_address, unsigned char start_register_slave,char *buffer,unsigned char cuantity_registers)
{

    unsigned char i = 0;
    UCB1I2CSA = slave_address;
    while(!(UCB1IFG & UCTXIFG));
    UCB1CTL1 |= UCTXSTT + UCTR;
    UCB1TXBUF = start_register_slave;
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

void binario(int num)
{
   int aux;

if(num==0)//<-La funcion recursiva tiene como condicion de salida,
         //cuando el numero es cero
   return;
           //si no
   aux=num%2;//<-"guardamos" el residuo de la divicion
   num=num/2;//<-actualizamos el valor del numero, para la proxima llamada
             //recursiva
   binario(num);//<-hacemos la sig llamada recursiva con nuestro valor actualizado
   printf("%d",aux);//<-Al poner la llamada de impresion despues de la llamada
                    //recursiva sig, hacemos que imprima primero las llamadas
                    //que se realizaron al final, haciendo el efecto de imprimir
                //inversamente
}
