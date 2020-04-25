#ifndef __DHT11_H_
#define __DHT11_H_
#include "main.h" 

//	DHT11_SDA : PB11 
  
#include "Sys.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

//IO方向设置
#define DHT11_IO_IN()  {DHT11_SDA_GPIO_Port->CRH &= 0xFFFF0FFF;	DHT11_SDA_GPIO_Port->CRH |= 8 << 12;}	// PB11 IN  MODE
#define DHT11_IO_OUT() {DHT11_SDA_GPIO_Port->CRH &= 0xFFFF0FFF;	DHT11_SDA_GPIO_Port->CRH |= 3 << 12;}	// PB11 OUT MODE

////IO操作函数
#define	DHT11_DQ_OUT PBout(11) //数据端口	PB11
#define	DHT11_DQ_IN  PBin(11)  //数据端口	PB11

u8 DHT11_Init(void);//初始化DHT11
u8 DHT11_Read_Data(u8 *temp, u8 *humi); //读取温湿度
u8 DHT11_Read_Data_Float(float *temp,float *humi); 
/**************供上层调用****************/
u8 DHT11_Read_Byte(void);//读出一个字节
u8 DHT11_Read_Bit(void);//读出一个位
u8 DHT11_Check(void);//检测是否存在DHT11
void DHT11_Rst(void);//复位DHT11

#endif


