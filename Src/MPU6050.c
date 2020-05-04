#include "MPU6050.h"
#include "main.h"  

//MPU6050_SCL : PB4
//MPU6050_SDA : PB5
//MPU6050_XDA : PB6
//MPU6050_XCL : PB7
//MPU6050_ADO : PB8
//MPU6050_INT : PB9 
#include "sys.h" 

/*******************************************
函数名称：MPU_IIC_SCL
功    能：IIC输出高低电平
参    数：State = 'H' OR 'L'
返回值  ：无
********************************************/
//State = 'H' OR 'L'
//IIC输出高低电平
void MPU_IIC_SCL(unsigned char State)
{ 
    // 设置引脚状态
    if(State == 0x48)
    {
        HAL_GPIO_WritePin(MPU6050_SCL_GPIO_Port,MPU6050_SCL_Pin,GPIO_PIN_SET);
    }
    else if(State == 0x4c)
    {
        HAL_GPIO_WritePin(MPU6050_SCL_GPIO_Port,MPU6050_SCL_Pin,GPIO_PIN_RESET);
    }
} 
/*******************************************
函数名称：MPU_IIC_SDA
功    能：IIC输出高低电平
参    数：State = 'H' OR 'L'
返回值  ：无
********************************************/
//State = 'H' OR 'L'
//IIC输出高低电平
void MPU_IIC_SDA(unsigned char State)
{
    GPIO_InitTypeDef GPIO_InitStruct ;
    //使能时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    //初始化引脚
    GPIO_InitStruct.Pin = MPU6050_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // 设置引脚状态
    if(State == 0x48)
    {
        HAL_GPIO_WritePin(MPU6050_SDA_GPIO_Port,MPU6050_SDA_Pin,GPIO_PIN_SET);
    }
    else if(State == 0x4c)
    {
        HAL_GPIO_WritePin(MPU6050_SDA_GPIO_Port,MPU6050_SDA_Pin,GPIO_PIN_RESET);
    }
} 
/*******************************************
函数名称：MPU_IIC_READ_SDA
功    能：读取IIC-SDA引脚的电平
参    数：无
返回值  ：引脚的高低电平
********************************************/
//读取IIC-SDA引脚的电平
uint8_t MPU_IIC_READ_SDA(void)
{
    uint8_t MPU_Pin_State;
    GPIO_InitTypeDef GPIO_InitStruct  ;
    //使能时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    //初始化引脚-上拉输入
    GPIO_InitStruct.Pin = MPU6050_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    //读取返回电平
    MPU_Pin_State = HAL_GPIO_ReadPin(MPU6050_SDA_GPIO_Port,MPU6050_SDA_Pin);
    return MPU_Pin_State;
} 

/*******************************************
函数名称：MPU_IIC_Start
功    能：I2C起始数据
参    数：无
返回值  ：无
从时序图可以看出来，开始信号为SCL为高时，SDA由高变低
********************************************/
void MPU_IIC_Start()
{
    MPU_IIC_SDA('H'); 
    MPU_IIC_SCL('H');
    HAL_Delay(1);

    //START:when CLK is high,DATA change form high to low 
    MPU_IIC_SDA('L');
    HAL_Delay(1);

    //Ready for data
    MPU_IIC_SCL('L');
}

/*******************************************
函数名称：MPU_IIC_Stop
功    能：I2C停止数据
参    数：无
返回值  ：无
********************************************/
void MPU_IIC_Stop(void)
{
 
    MPU_IIC_SCL('L');
    HAL_Delay(1);
    //STOP:when CLK is high DATA change form low to high
    MPU_IIC_SDA('L');
    HAL_Delay(1);

    MPU_IIC_SCL('H'); 
    HAL_Delay(1);

    MPU_IIC_SDA('H');//发送I2C总线结束信号
    HAL_Delay(1);                                                                   
}
/*******************************************
函数名称：MPU_IIC_Ack
功    能：I2C的Ack信号的产生
参    数：无
返回值  ：无
********************************************/
void MPU_IIC_Ack(void)
{
    MPU_IIC_SCL('L');
    MPU_IIC_SDA('L');
    HAL_Delay(1);
    
    MPU_IIC_SCL('H');
    HAL_Delay(1);

    MPU_IIC_SCL('L'); 

}
/*******************************************
函数名称：MPU_IIC_Nack
功    能：I2C的Nack信号的产生
参    数：无
返回值  ：无
********************************************/
void MPU_IIC_Nack(void)
{
    MPU_IIC_SCL('L');
    MPU_IIC_SDA('H');
    HAL_Delay(1);
    
    MPU_IIC_SCL('H');
    HAL_Delay(1);

    MPU_IIC_SCL('L'); 

}
/*******************************************
函数名称：MPU_IIC_Wait_Ack
功    能：等待IIC的Ack信号
参    数：无
返回值  ：1，接收应答失败
          0，接收应答成功
********************************************/ 
uint8_t MPU_Pin_State;
uint8_t MPU_IIC_Wait_Ack(void)
{
        uint8_t ucAck=0 ; 
        MPU_IIC_READ_SDA();  
        MPU_IIC_SCL('H');   
        MPU_Pin_State = MPU_IIC_READ_SDA();
        if (MPU_Pin_State)
        {   
            ucAck = 1; 
        }
        else
        {
            ucAck = 0;
        }
        
        MPU_IIC_SCL('L');//时钟输出0            
        return ucAck;  
}

/*******************************************
函数名称：MPU_IIC_Send_Byte
功    能：I2C发送数据
参    数：data
返回值  ：None
********************************************/
void MPU_IIC_Send_Byte(uint8_t data)
{
       uint8_t Count = 0;
       MPU_IIC_SCL('L');

       for (Count = 0; Count < 8; Count++)
       {
            //如果是1，SDA输出高电平。
            if((data & 0x80)>>7)
            {
                MPU_IIC_SDA('H');
            }
            else
            {
                 MPU_IIC_SDA('L');
            }

            data <<= 1;

            HAL_Delay(1);

            MPU_IIC_SCL('H');
            HAL_Delay(1);

            MPU_IIC_SCL('L');
            HAL_Delay(1); 
       } 
}


/*******************************************
函数名称：MPU_IIC_Read_Byte
功    能：I2C读取数据
参    数：ack    
          ack=0时，发送nACK
          ack=1时，发送ACK，
          ack=2时，什么都不发送   
返回值  ：Receive
********************************************/
uint8_t MPU_IIC_Read_Byte(uint8_t ack)
{
    uint8_t i,Receive = 0;
 
    MPU_IIC_READ_SDA();

    for(i = 0;i < 8; i++ )
    {
        MPU_IIC_SCL('L');
        HAL_Delay(1);

        MPU_IIC_SCL('H'); 
        Receive <<= 1;

        if( MPU_IIC_READ_SDA() )

            Receive++;    
    }            

    if (ack == 0)
    {
        MPU_IIC_Nack();
    }
    else if (ack == 1)
    {
        MPU_IIC_Ack();
    }
    
          
    return Receive;
}

/*******************************************
函数名称：MPU_WriteByte
功    能：I2C总线给MPU发送单字节
参    数：WriteAddr DataToWrite
返回值  ：无
********************************************/
void MPU_WriteByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
    MPU_IIC_Start();

    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0X00);//发送器件地址+写命令	
    MPU_IIC_Wait_Ack();

    MPU_IIC_Send_Byte(WriteAddr);  //发送首地址
    MPU_IIC_Wait_Ack();

    MPU_IIC_Send_Byte(DataToWrite);//发送数据
    MPU_IIC_Wait_Ack();

    MPU_IIC_Stop(); 
}
/*******************************************
函数名称：MPU_ReadByte
功    能：I2C总线从MPU6050接收单字节
参    数：ReadAddr DataToRead
返回值  ：Data
********************************************/
uint8_t MPU_ReadByte(uint8_t ReadAddr)
{
  uint8_t R_Data=0;

  MPU_IIC_Start();

  MPU_IIC_Send_Byte((MPU_ADDR<<1)|0X00);//发送器件地址+写命令	
  MPU_IIC_Wait_Ack();

  MPU_IIC_Send_Byte(ReadAddr);
  MPU_IIC_Wait_Ack();

  MPU_IIC_Start();
  MPU_IIC_Send_Byte((MPU_ADDR<<1)|0x01);//发送器件地址+读命令	
  MPU_IIC_Wait_Ack();

  R_Data = MPU_IIC_Read_Byte(1);  

  MPU_IIC_Stop();

  return R_Data;
} 
/*********************************************
函数名：MPU_Read_Len
功  能：IIC连续读
形  参：addr:器件地址
        reg:要读取的寄存器地址
        len:要读取的长度
        buf:读取到的数据存储区
返回值： 0,正常
**********************************************/
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    MPU_IIC_Start(); 
    MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
    MPU_IIC_Wait_Ack(); 	//等待应答
 
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答

    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 

	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//产生一个停止条件 
	return 0;	
}  
/*********************************************
函数名：MPU_Write_Len
功  能：IIC连续写
形  参：addr:器件地址 
        reg:寄存器地址
        len:写入长度
        buf:数据区
返回值： 0,正常
        其他,错误代码
**********************************************/
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
    MPU_IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令	
    MPU_IIC_Wait_Ack();             //等待应答
 
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答

    for(i=0;i<len;i++)
    {
        MPU_IIC_Send_Byte(buf[i]);	//发送数据
        MPU_IIC_Wait_Ack();		    //等待ACK
         		
    }    
    MPU_IIC_Stop();	 
    return 0;	
} 
//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{ 
	u8 res;
    GPIO_InitTypeDef  GPIO_InitStructure;
	
    //使能时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
  
    GPIO_InitStructure.Pin = MPU6050_SDA_Pin;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;  
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	MPU_AD0_CTRL=0;                         //控制MPU6050的AD0脚为低电平,从机地址为:0X68
	
	 
    MPU_WriteByte(MPU_PWR_MGMT1_REG,0X80);  //复位MPU6050
    HAL_Delay(100);
    MPU_WriteByte(MPU_PWR_MGMT1_REG,0X00);  //唤醒MPU6050 
    MPU_Set_Gyro_Fsr(3);                    //陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);                   //加速度传感器,±2g
    MPU_Set_Rate(50);                       //设置采样率50Hz
    MPU_WriteByte(MPU_INT_EN_REG,0X00);     //关闭所有中断
    MPU_WriteByte(MPU_USER_CTRL_REG,0X00);  //I2C主模式关闭
    MPU_WriteByte(MPU_FIFO_EN_REG,0X00);    //关闭FIFO
    MPU_WriteByte(MPU_INTBP_CFG_REG,0X80);  //INT引脚低电平有效
    res = MPU_ReadByte(MPU_DEVICE_ID_REG);
    if(res==MPU_ADDR)//器件ID正确
    {
    	MPU_WriteByte(MPU_PWR_MGMT1_REG,0X01);  //设置CLKSEL,PLL X轴为参考
    	MPU_WriteByte(MPU_PWR_MGMT2_REG,0X00);  //加速度与陀螺仪都工作
    	MPU_Set_Rate(50);                       //设置采样率为50Hz
    }else return 1;
    return 0;
}
////设置MPU6050陀螺仪传感器满量程范围
////fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
////返回值:0,设置成功
////    其他,设置失败 
 u8 MPU_Set_Gyro_Fsr(u8 fsr)
 {
    MPU_WriteByte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
    return 0 ;
 }
////设置MPU6050加速度传感器满量程范围
////fsr:0,±2g;1,±4g;2,±8g;3,±16g
////返回值:0,设置成功
////    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    MPU_WriteByte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
    return 0;
}
////设置MPU6050的数字低通滤波器
////lpf:数字低通滤波频率(Hz)
////返回值:0,设置成功
////    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
    MPU_WriteByte(MPU_CFG_REG,data);//设置数字低通滤波器  
    return 0;
}
////设置MPU6050的采样率(假定Fs=1KHz)
////rate:4~1000(Hz)
////返回值:0,设置成功
////    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	MPU_WriteByte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}
//
////得到温度值
////返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
////得到陀螺仪值(原始值)
////gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
////返回值:0,成功
////    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
////得到加速度值(原始值)
////gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
////返回值:0,成功
////    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}


////IIC写一个字节 
////reg:寄存器地址
////data:数据
////返回值:0,正常
////    其他,错误代码
//u8 MPU_WriteByte(u8 reg,u8 data) 				 
//{ 
//    MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
//	if(MPU_IIC_Wait_Ack())	//等待应答
//	{
//		MPU_IIC_Stop();		 
//		return 1;		
//	}
//    MPU_IIC_Send_Byte(reg);	//写寄存器地址
//    MPU_IIC_Wait_Ack();		//等待应答 
//	MPU_IIC_Send_Byte(data);//发送数据
//	if(MPU_IIC_Wait_Ack())	//等待ACK
//	{
//		MPU_IIC_Stop();	 
//		return 1;		 
//	}		 
//    MPU_IIC_Stop();	 
//	return 0;
//}
////IIC读一个字节 
////reg:寄存器地址 
////返回值:读到的数据
//u8 MPU_ReadByte(u8 reg)
//{
//	u8 res;
//    MPU_IIC_Start(); 
//	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
//	MPU_IIC_Wait_Ack();		//等待应答 
//    MPU_IIC_Send_Byte(reg);	//写寄存器地址
//    MPU_IIC_Wait_Ack();		//等待应答
//    MPU_IIC_Start();
//	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
//    MPU_IIC_Wait_Ack();		//等待应答 
//	res=MPU_IIC_Read_Byte(0);//读取数据,发送nACK 
//    MPU_IIC_Stop();			//产生一个停止条件 
//	return res;		
//}
// 
////传送数据给匿名四轴上位机软件(V2.6版本)
////fun:功能字. 0XA0~0XAF
////data:数据缓存区,最多28字节!!
////len:data区有效数据个数
//void usart1_niming_report(u8 fun,u8*data,u8 len)
//{
//	u8 send_buf[32];
//	u8 i;
//	if(len>28)return;	//最多28字节数据 
//	send_buf[len+3]=0;	//校验数置零
//	send_buf[0]=0X88;	//帧头
//	send_buf[1]=fun;	//功能字
//	send_buf[2]=len;	//数据长度
//	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
//	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	 
//}
////发送加速度传感器数据和陀螺仪数据
////aacx,aacy,aacz:x,y,z三个方向上面的加速度值
////gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
//{
//	u8 tbuf[12]; 
//	tbuf[0]=(aacx>>8)&0XFF;
//	tbuf[1]=aacx&0XFF;
//	tbuf[2]=(aacy>>8)&0XFF;
//	tbuf[3]=aacy&0XFF;
//	tbuf[4]=(aacz>>8)&0XFF;
//	tbuf[5]=aacz&0XFF; 
//	tbuf[6]=(gyrox>>8)&0XFF;
//	tbuf[7]=gyrox&0XFF;
//	tbuf[8]=(gyroy>>8)&0XFF;
//	tbuf[9]=gyroy&0XFF;
//	tbuf[10]=(gyroz>>8)&0XFF;
//	tbuf[11]=gyroz&0XFF; 
//}	
////通过串口1上报结算后的姿态数据给电脑
////aacx,aacy,aacz:x,y,z三个方向上面的加速度值
////gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
////roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
////pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
////yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
//void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
//{
//	u8 tbuf[28]; 
//	u8 i;
//	for(i=0;i<28;i++)tbuf[i]=0;//清0
//	tbuf[0]=(aacx>>8)&0XFF;
//	tbuf[1]=aacx&0XFF;
//	tbuf[2]=(aacy>>8)&0XFF;
//	tbuf[3]=aacy&0XFF;
//	tbuf[4]=(aacz>>8)&0XFF;
//	tbuf[5]=aacz&0XFF; 
//	tbuf[6]=(gyrox>>8)&0XFF;
//	tbuf[7]=gyrox&0XFF;
//	tbuf[8]=(gyroy>>8)&0XFF;
//	tbuf[9]=gyroy&0XFF;
//	tbuf[10]=(gyroz>>8)&0XFF;
//	tbuf[11]=gyroz&0XFF;	
//	tbuf[18]=(roll>>8)&0XFF;
//	tbuf[19]=roll&0XFF;
//	tbuf[20]=(pitch>>8)&0XFF;
//	tbuf[21]=pitch&0XFF;
//	tbuf[22]=(yaw>>8)&0XFF;
//	tbuf[23]=yaw&0XFF; 
//} 
//
