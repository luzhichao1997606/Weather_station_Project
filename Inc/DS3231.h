#ifndef __DS3231_H_
#define __DS3231_H_
#include "main.h" 

//RTC_SDA   :   PB12  
//RTC_SCL   :   PB13
//RTC_SQW   :   PB14
//RTC_32K   :   PB15
#define SDA_GPIO_Port GPIOB
#define SDA_Pin	RTC_SDA_Pin
#define SCL_Pin	RTC_SCL_Pin

typedef struct DS3231_Date{ 

	char Seconds;
	char Minutes;
	char Hour;
	char Day;   // Range 1-7
	char Date;  // Range 1-31
	char Week;
	char Month;
	char Year;  // Range 0-99 
	
}DS3231_DateTime;

extern  DS3231_DateTime DS3231_ReadDate;// structure for read date
 
//DS3231初始宏设置
#define DS3231_WriteAddress 0xD0    //器件写地址 
#define DS3231_ReadAddress  0xD1    //器件读地址
#define DS3231_SECOND       0x00    //秒
#define DS3231_MINUTE       0x01    //分
#define DS3231_HOUR         0x02    //时
#define DS3231_WEEK         0x03    //星期
#define DS3231_DAY          0x04    //日
#define DS3231_MONTH        0x05    //月
#define DS3231_YEAR         0x06    //年
//闹铃1            
#define DS3231_SALARM1ECOND 0x07    //秒
#define DS3231_ALARM1MINUTE 0x08    //分
#define DS3231_ALARM1HOUR   0x09    //时
#define DS3231_ALARM1WEEK   0x0A    //星期/日
//闹铃2
#define DS3231_ALARM2MINUTE 0x0b    //分
#define DS3231_ALARM2HOUR   0x0c    //时
#define DS3231_ALARM2WEEK   0x0d    //星期/日
#define DS3231_CONTROL      0x0e    //控制寄存器
#define DS3231_STATUS       0x0f    //状态寄存器
#define BSY                 2       //忙
#define OSF                 7       //振荡器停止标志
#define DS3231_XTAL         0x10    //晶体老化寄存器
#define DS3231_TEMPERATUREH 0x11    //温度寄存器高字节(8位)
#define DS3231_TEMPERATUREL 0x12    //温度寄存器低字节(高2位) 
 

uint8_t DS3231_ReadByte(uint8_t ReadAddr);
uint8_t BCD2_Hex(uint8_t val);
void 	Read_RTC(void);
void ModifyTime(uint8_t yea,uint8_t mon,uint8_t da,uint8_t hou,uint8_t min,uint8_t sec);
#endif


