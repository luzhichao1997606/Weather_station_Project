#include "DS3231.h"
#include "main.h"  

//RTC_SDA   :   PB12  
//RTC_SCL   :   PB13
//RTC_SQW   :   PB14
//RTC_32K   :   PB15 
uint8_t	Week;

// sec_temp,day_temp,tem_temp;
 
 
DS3231_DateTime DS3231_ReadDate;// structure for read date
/*******************************************
函数名称：IIC_SCL
功    能：IIC输出高低电平
参    数：State = 'H' OR 'L'
返回值  ：无
********************************************/
//State = 'H' OR 'L'
//IIC输出高低电平
void IIC_SCL(unsigned char State)
{ 
    // 设置引脚状态
    if(State == 0x48)
    {
        HAL_GPIO_WritePin(SDA_GPIO_Port,SCL_Pin,GPIO_PIN_SET);
    }
    else if(State == 0x4c)
    {
        HAL_GPIO_WritePin(SDA_GPIO_Port,SCL_Pin,GPIO_PIN_RESET);
    }
} 
/*******************************************
函数名称：IIC_SDA
功    能：IIC输出高低电平
参    数：State = 'H' OR 'L'
返回值  ：无
********************************************/
//State = 'H' OR 'L'
//IIC输出高低电平
void IIC_SDA(unsigned char State)
{
    GPIO_InitTypeDef GPIO_InitStruct ;
    //使能时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    //初始化引脚
    GPIO_InitStruct.Pin = SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // 设置引脚状态
    if(State == 0x48)
    {
        HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_SET);
    }
    else if(State == 0x4c)
    {
        HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_RESET);
    }
} 
/*******************************************
函数名称：IIC_READ_SDA
功    能：读取IIC-SDA引脚的电平
参    数：无
返回值  ：引脚的高低电平
********************************************/
//读取IIC-SDA引脚的电平
uint8_t IIC_READ_SDA(void)
{
    uint8_t Pin_State;
    GPIO_InitTypeDef GPIO_InitStruct  ;
    //使能时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    //初始化引脚-上拉输入
    GPIO_InitStruct.Pin = SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    //读取返回电平
    Pin_State = HAL_GPIO_ReadPin(SDA_GPIO_Port,SDA_Pin);
    return Pin_State;
}
/*******************************************
函数名称：BCD2_Hex
功    能：BCD转换为Byte
参    数：无
返回值  ：无
********************************************/
//BCD转换为Byte
uint8_t BCD2_Hex(uint8_t val)
{
        uint8_t temp;
        temp = val & 0x0f;
        val>>= 4;
        val &= 0x0f;
        val *= 10;
        temp+= val;  
        return temp;
}

/*******************************************
函数名称：HEX2_Bcd
功    能：B转换为BCD
参    数：无
返回值  ：无
********************************************/
//B转换为BCD
uint8_t HEX2_Bcd(uint8_t val)
{
        uint8_t i,j,k;
        i = val/10;
        j = val%10;
        k = j+(i<<4);
        return k;
}

/*******************************************
函数名称：RTC_IIC_Start
功    能：I2C起始数据
参    数：无
返回值  ：无
从时序图可以看出来，开始信号为SCL为高时，SDA由高变低
********************************************/
void RTC_IIC_Start()
{
    IIC_SDA('H'); 
    IIC_SCL('H');
    HAL_Delay(5);

    //START:when CLK is high,DATA change form high to low 
    IIC_SDA('L');
    HAL_Delay(5);

    //Ready for data
    IIC_SCL('L');
}

/*******************************************
函数名称：RTC_IIC_Stop
功    能：I2C停止数据
参    数：无
返回值  ：无
********************************************/
void RTC_IIC_Stop(void)
{
 
    IIC_SCL('L');
    HAL_Delay(5);
    //STOP:when CLK is high DATA change form low to high
    IIC_SDA('L');
    HAL_Delay(5);

    IIC_SCL('H'); 
    HAL_Delay(5);

    IIC_SDA('H');//发送I2C总线结束信号
    HAL_Delay(5);                                                                   
}
/*******************************************
函数名称：RTC_IIC_Ack
功    能：I2C的Ack信号的产生
参    数：无
返回值  ：无
********************************************/
void RTC_IIC_Ack(void)
{
    IIC_SCL('L');
    IIC_SDA('L');
    HAL_Delay(5);
    
    IIC_SCL('H');
    HAL_Delay(5);

    IIC_SCL('L'); 

}
/*******************************************
函数名称：RTC_IIC_Nack
功    能：I2C的Nack信号的产生
参    数：无
返回值  ：无
********************************************/
void RTC_IIC_Nack(void)
{
    IIC_SCL('L');
    IIC_SDA('H');
    HAL_Delay(5);
    
    IIC_SCL('H');
    HAL_Delay(5);

    IIC_SCL('L'); 

}
/*******************************************
函数名称：RTC_IIC_Wait_Ack
功    能：等待IIC的Ack信号
参    数：无
返回值  ：1，接收应答失败
          0，接收应答成功
********************************************/ 
uint8_t Pin_State;
uint8_t RTC_IIC_Wait_Ack(void)
{
        uint8_t ucAck=0 ; 
        IIC_READ_SDA();  
        IIC_SCL('H');   
        Pin_State = IIC_READ_SDA();
        if (Pin_State)
        {   
            ucAck = 1; 
        }
        else
        {
            ucAck = 0;
        }
        
        IIC_SCL('L');//时钟输出0            
        return ucAck;  
}

/*******************************************
函数名称：RTC_IIC_Send_Byte
功    能：I2C发送数据
参    数：data
返回值  ：None
********************************************/
void RTC_IIC_Send_Byte(uint8_t data)
{
       uint8_t Count = 0;
       IIC_SCL('L');

       for (Count = 0; Count < 8; Count++)
       {
            //如果是1，SDA输出高电平。
            if((data & 0x80)>>7)
            {
                IIC_SDA('H');
            }
            else
            {
                 IIC_SDA('L');
            }

            data <<= 1;

            HAL_Delay(5);

            IIC_SCL('H');
            HAL_Delay(5);

            IIC_SCL('L');
            HAL_Delay(5); 
       } 
}

/*******************************************
函数名称：RTC_IIC_Read_Byte
功    能：I2C读取数据
参    数：ack    
          ack=0时，发送nACK
          ack=1时，发送ACK，
          ack=2时，什么都不发送   
返回值  ：Receive
********************************************/
uint8_t RTC_IIC_Read_Byte(uint8_t ack)
{
    uint8_t i,Receive = 0;
 
    IIC_READ_SDA();

    for(i = 0;i < 8; i++ )
    {
        IIC_SCL('L');
        HAL_Delay(5);

        IIC_SCL('H'); 
        Receive <<= 1;

        if( IIC_READ_SDA() )

            Receive++;   

 /*    
        if( IIC_READ_SDA() ) 
        {    
            Receive |=  (temp >> i);           //高电平保留   
        } 
        else
        {
            Receive &= ~(temp >> i);
        }

        IIC_SCL('L');
*/            

        // HAL_Delay(5); 
    }            

    if (ack == 0)
    {
        RTC_IIC_Nack();
    }
    else if (ack == 1)
    {
        RTC_IIC_Ack();
    }
    
          
    return Receive;
}
/*******************************************
函数名称：DS3231_WriteByte
功    能：I2C总线给DS3231发送单字节
参    数：WriteAddr DataToWrite
返回值  ：无
********************************************/
void DS3231_WriteByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
    RTC_IIC_Start();

    RTC_IIC_Send_Byte(0xD0);       //发送器件地址    
    RTC_IIC_Wait_Ack();

    RTC_IIC_Send_Byte(WriteAddr);  //发送首地址
    RTC_IIC_Wait_Ack();

    RTC_IIC_Send_Byte(DataToWrite);//发送数据
    RTC_IIC_Wait_Ack();

    RTC_IIC_Stop(); 
}
/*******************************************
函数名称：DS3231_ReadByte
功    能：I2C总线从DS3231接收单字节
参    数：ReadAddr DataToRead
返回值  ：Data
********************************************/
uint8_t DS3231_ReadByte(uint8_t ReadAddr)
{
  uint8_t R_Data=0;

  RTC_IIC_Start();

  RTC_IIC_Send_Byte(0XD0);//读地址 
  RTC_IIC_Ack();

  RTC_IIC_Send_Byte(ReadAddr);
  RTC_IIC_Ack();

  RTC_IIC_Start();
  RTC_IIC_Send_Byte(0XD1); 

  R_Data = RTC_IIC_Read_Byte(2); 
  HAL_Delay(5);
  RTC_IIC_Nack();

  RTC_IIC_Stop();

  return R_Data;
}

/*******************************************
函数名称：Readtime
功    能：读取DS3231时间
参    数：R_tmpdate
返回值  ：无
********************************************/
void Read_RTC()
{    
	unsigned char rtc_address[6]={0x00,0x01,0x02,0x04,0x05,0x06};
    unsigned char R_tmpdate[6];
    unsigned char i,*p;
    p = rtc_address;             //地址传递
    for(i=0;i<6;i++)             //分6次读取 秒分时日月年
    {
      R_tmpdate[i] = DS3231_ReadByte (*p);  
      p++;
    }

   DS3231_ReadDate.Seconds  = R_tmpdate[0] ;
   DS3231_ReadDate.Minutes  = R_tmpdate[1] ;
   DS3231_ReadDate.Hour     = R_tmpdate[2] ;
   DS3231_ReadDate.Day      = R_tmpdate[3] ;
   DS3231_ReadDate.Month    = R_tmpdate[4] ;
   DS3231_ReadDate.Year     = R_tmpdate[5] ; 
 
}
/*******************************************
函数名称：ModifyTime
功    能：改变DS3231时间
参    数：R_tmpdate
返回值  ：无
********************************************/
void ModifyTime(uint8_t yea,uint8_t mon,uint8_t day,uint8_t hou,uint8_t min,uint8_t sec)
{
    uint8_t temp=0;

    temp=HEX2_Bcd(yea);
    DS3231_WriteByte(DS3231_YEAR,temp);         //修改年
                
    temp=HEX2_Bcd(mon);
    DS3231_WriteByte(DS3231_MONTH,temp);        //修改月
                
    temp=HEX2_Bcd(day);
    DS3231_WriteByte(DS3231_DAY,temp);          //修改日
                
    temp=HEX2_Bcd(hou);
    DS3231_WriteByte(DS3231_HOUR,temp);          //修改时
                
    temp=HEX2_Bcd(min);
    DS3231_WriteByte(DS3231_MINUTE,temp);        //修改分        
                
    temp=HEX2_Bcd(sec);
    DS3231_WriteByte(DS3231_SECOND,temp);        //修改秒 
}