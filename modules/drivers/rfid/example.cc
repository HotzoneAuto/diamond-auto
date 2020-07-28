#include<iostream>
using namespace std;
int main()
{
 char Rx_Buff[20]; //接收缓冲
 volatile unsigned char Rx_Length; //接收数据长度计数 
 #pragma vector = USART_RXC_vect // 串口中断 
 __interrupt void Usart_RFID_Rx_ISR(void);
 {
  char UDR;
  unsigned char i = 0;
  char Rx_Char;
  Rx_Char = UDR; //接收到一位数据
  Rx_Buff[Rx_Length] = Rx_Char;
  if (Rx_Char == 0x02) //如果收到的数据是 02，证明接收到的是数据头
  {
   Rx_Length = 0; //已经接收了 1 位数据了
  }
  Rx_Length++;
   if ((Rx_Char == 0x03) && (Rx_Length == 13)) // 收 13 个数据
   {
    bool Read_Card_Flag = true; // 接收完成，卡号已经放 Rx_Buff[]里了
   }
 }
}
void Init_Uart(void) // 初始化串口为 9600,8N1
{
 int UCSRB = 0x00; //disable while setting baud rate
 int UCSRA = 0x00;
 int UCSRC = 0x0E;
 int UBRRL = 0x2F; //set baud rate lo
 int UBRRH = 0x00; //set baud rate hi
  UCSRB = 0xD8;
 int Rx_Length = 0;
}