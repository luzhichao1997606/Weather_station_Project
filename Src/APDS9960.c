#include "APDS9960.h"
#include "main.h"  

//APDS_INT			:    PA15
//APDS_SDA			:    PA12
//APDS_SCL		    :    PA11
gesture_data_type gesture_data = {0};
uint8_t fifo_data[128] = {0};
/* -----------------------------------------全局变量定义-----------------------------------*/
int gesture_ud_delta;
int gesture_lr_delta;
int gesture_ud_count;
int gesture_lr_count;
int gesture_near_count;
int gesture_far_count;
int gesture_state;
int gesture_motion; 

/*******************************************
函数名称：APDS_IIC_SCL
功    能：IIC输出高低电平
参    数：State = 'H' OR 'L'
返回值  ：无
********************************************/
//State = 'H' OR 'L'
//IIC输出高低电平
void APDS_IIC_SCL(unsigned char State)
{ 
    // 设置引脚状态
    if(State == 0x48)
    {
        HAL_GPIO_WritePin(APDS_SCL_GPIO_Port,APDS_SCL_Pin,GPIO_PIN_SET);
    }
    else if(State == 0x4c)
    {
        HAL_GPIO_WritePin(APDS_SCL_GPIO_Port,APDS_SCL_Pin,GPIO_PIN_RESET);
    }
} 
/*******************************************
函数名称：APDS_IIC_SDA
功    能：IIC输出高低电平
参    数：State = 'H' OR 'L'
返回值  ：无
********************************************/
//State = 'H' OR 'L'
//IIC输出高低电平
void APDS_IIC_SDA(unsigned char State)
{
    GPIO_InitTypeDef GPIO_InitStruct ;
    //使能时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    //初始化引脚
    GPIO_InitStruct.Pin = APDS_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // 设置引脚状态
    if(State == 0x48)
    {
        HAL_GPIO_WritePin(APDS_SDA_GPIO_Port,APDS_SDA_Pin,GPIO_PIN_SET);
    }
    else if(State == 0x4c)
    {
        HAL_GPIO_WritePin(APDS_SDA_GPIO_Port,APDS_SDA_Pin,GPIO_PIN_RESET);
    }
} 
/*******************************************
函数名称：APDS_IIC_READ_SDA
功    能：读取IIC-SDA引脚的电平
参    数：无
返回值  ：引脚的高低电平
********************************************/
//读取IIC-SDA引脚的电平
uint8_t APDS_IIC_READ_SDA(void)
{
    uint8_t APDS_Pin_State;
    GPIO_InitTypeDef GPIO_InitStruct  ;
    //使能时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    //初始化引脚-上拉输入
    GPIO_InitStruct.Pin = APDS_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //读取返回电平
    APDS_Pin_State = HAL_GPIO_ReadPin(APDS_SDA_GPIO_Port,APDS_SDA_Pin);
    return APDS_Pin_State;
} 

/*******************************************
函数名称：APDS_IIC_Start
功    能：I2C起始数据
参    数：无
返回值  ：无
从时序图可以看出来，开始信号为SCL为高时，SDA由高变低
********************************************/
void APDS_IIC_Start()
{
    APDS_IIC_SDA('H'); 
    APDS_IIC_SCL('H');
    HAL_Delay(5);

    //START:when CLK is high,DATA change form high to low 
    APDS_IIC_SDA('L');
    HAL_Delay(5);

    //Ready for data
    APDS_IIC_SCL('L');
}

/*******************************************
函数名称：APDS_IIC_Stop
功    能：I2C停止数据
参    数：无
返回值  ：无
********************************************/
void APDS_IIC_Stop(void)
{
 
    APDS_IIC_SCL('L');
    HAL_Delay(5);
    //STOP:when CLK is high DATA change form low to high
    APDS_IIC_SDA('L');
    HAL_Delay(5);

    APDS_IIC_SCL('H'); 
    HAL_Delay(5);

    APDS_IIC_SDA('H');//发送I2C总线结束信号
    HAL_Delay(5);                                                                   
}
/*******************************************
函数名称：APDS_IIC_Ack
功    能：I2C的Ack信号的产生
参    数：无
返回值  ：无
********************************************/
void APDS_IIC_Ack(void)
{
    APDS_IIC_SCL('L');
    APDS_IIC_SDA('L');
    HAL_Delay(5);
    
    APDS_IIC_SCL('H');
    HAL_Delay(5);

    APDS_IIC_SCL('L'); 

}
/*******************************************
函数名称：APDS_IIC_Nack
功    能：I2C的Nack信号的产生
参    数：无
返回值  ：无
********************************************/
void APDS_IIC_Nack(void)
{
    APDS_IIC_SCL('L');
    APDS_IIC_SDA('H');
    HAL_Delay(5);
    
    APDS_IIC_SCL('H');
    HAL_Delay(5);

    APDS_IIC_SCL('L'); 

}
/*******************************************
函数名称：APDS_IIC_Wait_Ack
功    能：等待IIC的Ack信号
参    数：无
返回值  ：1，接收应答失败
          0，接收应答成功
********************************************/ 
uint8_t APDS_Pin_State;
uint8_t APDS_IIC_Wait_Ack(void)
{
        uint8_t ucAck=0 ; 
        APDS_IIC_READ_SDA();  
        APDS_IIC_SCL('H');   
        APDS_Pin_State = APDS_IIC_READ_SDA();
        if (APDS_Pin_State)
        {   
            ucAck = 1; 
        }
        else
        {
            ucAck = 0;
        }
        
        APDS_IIC_SCL('L');//时钟输出0            
        return ucAck;  
}

/*******************************************
函数名称：APDS_IIC_Send_Byte
功    能：I2C发送数据
参    数：data
返回值  ：None
********************************************/
void APDS_IIC_Send_Byte(uint8_t data)
{
       uint8_t Count = 0;
       APDS_IIC_SCL('L');

       for (Count = 0; Count < 8; Count++)
       {
            //如果是1，SDA输出高电平。
            if((data & 0x80)>>7)
            {
                APDS_IIC_SDA('H');
            }
            else
            {
                 APDS_IIC_SDA('L');
            }

            data <<= 1;

            HAL_Delay(5);

            APDS_IIC_SCL('H');
            HAL_Delay(5);

            APDS_IIC_SCL('L');
            HAL_Delay(5); 
       } 
}


/*******************************************
函数名称：APDS_IIC_Read_Byte
功    能：I2C读取数据
参    数：ack    
          ack=0时，发送nACK
          ack=1时，发送ACK，
          ack=2时，什么都不发送   
返回值  ：Receive
********************************************/
uint8_t APDS_IIC_Read_Byte(uint8_t ack)
{
    uint8_t i,Receive = 0;
 
    APDS_IIC_READ_SDA();

    for(i = 0;i < 8; i++ )
    {
        APDS_IIC_SCL('L');
        HAL_Delay(5);

        APDS_IIC_SCL('H'); 
        Receive <<= 1;

        if( APDS_IIC_READ_SDA() )

            Receive++;    
    }            

    if (ack == 0)
    {
        APDS_IIC_Nack();
    }
    else if (ack == 1)
    {
        APDS_IIC_Ack();
    }
    
          
    return Receive;
}

/*******************************************
函数名称：APDS_WriteByte
功    能：I2C总线给APDS发送单字节
参    数：WriteAddr DataToWrite
返回值  ：无
********************************************/
void APDS_WriteByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
    APDS_IIC_Start();

    APDS_IIC_Send_Byte((APDS9960_I2C_ADDR<<1)|0X00);//发送器件地址+写命令	
    APDS_IIC_Wait_Ack();

    APDS_IIC_Send_Byte(WriteAddr);  //发送首地址
    APDS_IIC_Wait_Ack();

    APDS_IIC_Send_Byte(DataToWrite);//发送数据
    APDS_IIC_Wait_Ack();

    APDS_IIC_Stop(); 
}
/*******************************************
函数名称：APDS_ReadByte
功    能：I2C总线从APDS9960接收单字节
参    数：ReadAddr DataToRead
返回值  ：Data
********************************************/
uint8_t APDS_ReadByte(uint8_t ReadAddr)
{
  uint8_t R_Data=0;

  APDS_IIC_Start();

  APDS_IIC_Send_Byte((APDS9960_I2C_ADDR<<1)|0X00);//发送器件地址+写命令	
  APDS_IIC_Wait_Ack();

  APDS_IIC_Send_Byte(ReadAddr);
  APDS_IIC_Wait_Ack();

  APDS_IIC_Start();
  APDS_IIC_Send_Byte((APDS9960_I2C_ADDR<<1)|0x01);//发送器件地址+读命令	
  APDS_IIC_Wait_Ack();

  R_Data = APDS_IIC_Read_Byte(1);  

  APDS_IIC_Stop();

  return R_Data;
}

/*********************************************
函数名：APDS9960_get_data
功  能：读取一次寄存器值
形  参：
返回值： 
**********************************************/
uint16_t APDS9960_get_data(const uint8_t addr, uint8_t *val, uint16_t len)
{
    uint16_t i = 0;
    APDS_IIC_Start();                                   // 启动
    APDS_IIC_Send_Byte((APDS9960_I2C_ADDR<<1)|0X00);    // 发送地址
    APDS_IIC_Wait_Ack();

    APDS_IIC_Send_Byte(addr);
    APDS_IIC_Wait_Ack();

    APDS_IIC_Start();
    APDS_IIC_Send_Byte((APDS9960_I2C_ADDR<<1)|0x01);//发送器件地址+读命令	
    APDS_IIC_Wait_Ack();
 
    while (len)
    {
        if (len == 1)
        {
            val[i] =  APDS_IIC_Read_Byte(0);
        }
        else
        {
            val[i] =  APDS_IIC_Read_Byte(1);
        }
        i++;
        len--;
    }
    APDS_IIC_Stop();
    return i;
} 
/*********************************************
函数名：resetGestureParameters
功  能：设置变量
形  参：
返回值：  
**********************************************/
void resetGestureParameters(void)
{
    gesture_data.index = 0;
    gesture_data.total_gestures = 0;

    gesture_ud_delta = 0;
    gesture_lr_delta = 0;

    gesture_ud_count = 0;
    gesture_lr_count = 0;

    gesture_near_count = 0;
    gesture_far_count = 0;

    gesture_state = 0;
    gesture_motion = DIR_NONE;
} 
/*********************************************
函数名：APDS9960_Set_GestureGain
功  能：设置手势增益
形  参：
返回值：  
**********************************************/
void APDS9960_Set_GestureGain(uint8_t gain)
{

    uint8_t val;
    val = APDS_ReadByte(APDS9960_GCONF2);
    /* 将寄存器中的位设置为给定值 */
    gain &= 00000011;//取gain最低两位
    gain = gain << 5;//移动到6:5
    val &= 10011111;//将6:5位清零
    val |= gain;//将gain的6:5位赋值给val
    /* 将值回写入使能寄存器中 */
    APDS_WriteByte(APDS9960_GCONF2, val);

} 
/*********************************************
函数名：APDS9960_Set_Gesture_LEDDrive
功  能：在手势模式下设置LED驱动电流
形  参：
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA 
备  注：驱动LED驱动电流的值 
**********************************************/
void APDS9960_Set_Gesture_LEDDrive(uint8_t drive)
{
    uint8_t val;
    val = APDS_ReadByte(APDS9960_GCONF2);
    /* 将寄存器中的位设置为给定值 */
    drive &= 00000011;
    drive = drive << 3;//bit 4:3
    val &= 11100111;
    val |= drive;
    /* 将值回写入使能寄存器中 */
    APDS_WriteByte(APDS9960_GCONF2, val);
} 
/*********************************************
函数名：APDS9960_Set_Gesture_WaitTime
功  能：设置手势检测之间的低功耗模式时间
形  参：
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
返回值：如果操作成功，则为True。否则就错了。
备  注：等待时间的值 
**********************************************/
void APDS9960_Set_Gesture_WaitTime(uint8_t time)
{
    uint8_t val;
    val = APDS_ReadByte(APDS9960_GCONF2);
    /* 将寄存器中的位设置为给定值 */
    time &= 00000111;
    val &= 11111000;
    val |= time;
    /* 将值回写入使能寄存器中 */
    APDS_WriteByte(APDS9960_GCONF2, val);
} 
/*********************************************
函数名：APDS9960_Set_Gesture_IntEnable
功  能：打开或关闭与手势相关的中断
形  参：1--启用中断，0--禁用中断

返回值：如果操作成功，则为True。否则就错了。 
**********************************************/
void APDS9960_Set_Gesture_IntEnable(uint8_t enable)
{
    uint8_t val;
    val = APDS_ReadByte(APDS9960_GCONF4);
    /* 将寄存器中的位设置为给定值 */
    enable &= 00000001;
    enable = enable << 1;
    val &= 11111101;
    val |= enable;
    /* 将值回写入使能寄存器中 */
    APDS_WriteByte(APDS9960_GCONF4, val);
} 

/*********************************************
函数名称：APDS9960_Get_Mode
功    能：I2C总线从APDS9960接收单字节 
参    数：ReadAddr DataToRead
返回值  ： 
**********************************************/
uint8_t APDS9960_Get_Mode(void)
{
    /* 读取当前ENABLE寄存器 */ 
    return APDS_ReadByte(APDS9960_ENABLE);
} 
/*********************************************
函数名称：APDS9960_SetMode
功    能：I2C总线从APDS9960进行设置模式
参    数：ReadAddr DataToRead
返回值  ：pdTRUE或者pdFALSE
**********************************************/
BaseType_t APDS9960_SetMode(int8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = APDS9960_Get_Mode();
	//printf("First_setMode_regval = %.2x\n",reg_val);//打印读取到的使能寄存器的值 0x80 = 0x4d
    if( reg_val == ERROR ) {//如果读取到的值为0xFF，则错误
        return pdFALSE;
    } 
    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if((mode >= 0) && (mode <= 6)) //使能或失能某个位
	{
		if(enable) //使能
		{
            reg_val |= (1 << mode);
        } 
		else      //失能
		{
            reg_val &= ~(1 << mode);
        }
    } 
	else if( mode == ALL ) //使能全部
	{
        if (enable) 
		{
            reg_val = 0x7F;//0x80=0x7F   全部使能
        } 
		else //全部失能
		{
            reg_val = 0x00;//0x80=0x00 
        }
    }
       
    /* Write value back to ENABLE register */
    APDS_WriteByte(APDS9960_ENABLE, reg_val); 
    return pdTRUE;
} 

/*********************************************
函数名称：APDS9960_Init
功    能：I2C总线对APDS9960进行初始化 
参    数：ReadAddr DataToRead
返回值  ：Data 
**********************************************/
BaseType_t APDS9960_Init()
{
    //先对读取ID进行监测，如果ID都不正常那么IIC出现问题
    if ( APDS9960_Get_ID() != 0XAB )
    {
        return pdFALSE;
    }
    /* 失能失能寄存器0x80 = 0x00 */
    if (!APDS9960_SetMode(ALL,OFF))
    {
       return pdFALSE;
    } 
    //设置手势接近进入阀值  
	//手势接近阀值会与接近数据PDATA进行比较并决定是否进入手势状态机
    APDS_WriteByte(APDS9960_GPENTH,  DEFAULT_GPENTH) ;

    //设置手势接近退出(手势状态机)阀值为0xA1 = 30
    APDS_WriteByte(APDS9960_GEXTH, DEFAULT_GEXTH); 

    //设置配置寄存器1 0xA2 = 0x40
    //1.在4个数据集被添加到FIFO里后产生中断
    //2.All UDLR 探测数据被包含到集合中
    //3.手势退出持久性.当连续的手势结束发生称为比GEXPERS大于或等于的值时，
    //  手势状态机退出(第1个手势结束发生导致手势状态机退出)
    APDS_WriteByte(APDS9960_GCONF1, DEFAULT_GCONF1); 

    //设置手势增益 设置配置寄存器2 0xA3 的 bit 6:5 = 10  8x增益
    APDS9960_Set_GestureGain(DEFAULT_GGAIN);

    //设置配置寄存器2 0xA3 的 bit 4:3 = 00  100ma
    APDS9960_Set_Gesture_LEDDrive(DEFAULT_GLDRIVE);

    //设定配置寄存器2 0xA3 的 bit 2:0=001   39.2ms
    APDS9960_Set_Gesture_WaitTime(DEFAULT_GWTIME);

    //设置手势UP偏移寄存器 0xA4 = 0 没有偏移
    APDS_WriteByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET);

    //设置手势DOWN偏移寄存器 0xA5 = 0 没有偏移
    APDS_WriteByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET);

    //设置手势LEFT偏移寄存器 0xA7 = 0 没有偏移
    APDS_WriteByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET);

    //设置手势RIGHT偏移寄存器 0xA9 = 0 没有偏移
    APDS_WriteByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET);

    //设置收势脉冲数和脉宽寄存器0xA6 = 0xC9   32us, 10 pulses
    APDS_WriteByte(APDS9960_GPULSE, DEFAULT_GPULSE);

    //设置配置寄存器3  0xAA 的bit 1:0 = 00  所有光电二极管在手势期间均有效
    APDS_WriteByte(APDS9960_GCONF3, DEFAULT_GCONF3);


    //设置配置寄存器4 0xAB 的bit1 = 0 关闭手势中断 GIEN=0
    APDS9960_Set_Gesture_IntEnable(DEFAULT_GIEN); 

    return pdTRUE;
}
/*********************************************
函数名称：APDS9960_Get_ID
功    能：I2C总线从APDS9960接收单字节-APDS9960的ID
参    数：ReadAddr DataToRead
返回值  ：Data
            如果ID = 0xAB则IIC通讯正常
            如果ID = 0xFF则IIC通讯不正常 
**********************************************/
uint8_t APDS9960_Get_ID(void)
{
    /* 读取当前ID寄存器 */ 
    return APDS_ReadByte(APDS9960_ID);
}  
/*********************************************
函数名：APDS9960_Set_LED_Boost
功  能：设置LED当前的升压值
形  参：
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
返回值：如果操作成功，则为True。否则就错了。
备  注：驱动电流提升值（0-3）（100-300%） 
**********************************************/
void APDS9960_Set_LED_Boost(uint8_t boost)
{
    uint8_t val;
    val = APDS_ReadByte(APDS9960_CONFIG2);
    /* 将寄存器中的位设置为给定值 */
    boost &= 00000011;
    boost = boost << 4;
    val &= 11001111;
    val |= boost;
    /* 将值回写入使能寄存器中 */
    APDS_WriteByte(APDS9960_CONFIG2, val);
}
/*********************************************
函数名：APDS9960_Set_Gesture_mode
功  能：告诉状态机进入或退出手势状态机
形  参：0--关闭  1--开启
返回值：如果操作成功，则为True。否则就错了。 
**********************************************/
void APDS9960_Set_Gesture_mode(uint8_t mode)
{
    uint8_t val;
    val = APDS_ReadByte(APDS9960_GCONF4);
    /* 将寄存器中的位设置为给定值 */
    mode &= 00000001;
    val &= 11111110;
    val |= mode;
    /* 将值回写入使能寄存器中 */
    APDS_WriteByte(APDS9960_GCONF4, val);
}
 
/*********************************************
函数名：APDS9960_Gesture_EN
功  能：使能手势状态机， 启用手势模式
形  参：0--关闭  1--开启
返回值：如果操作成功，则为True。否则就错了。 
**********************************************/
void APDS9960_Gesture_EN(uint8_t interrupts)
{
    /* 启用手势模式
       将“启用”设置为0（关闭电源）
       将WTIME设置为0xFF
       将AUX设置为LED增强
       启用PON、WEN、PEN、GEN-in启用
    */
    resetGestureParameters();//复位手势变量=0

    //设置等待时间寄存器0x83 = 0xFF (WLONG=1  0.03s)   (WLONG=0   2.78ms)
    APDS_WriteByte(APDS9960_WTIME, 0xFF);

    //设置接近脉冲计数寄存器 0x8E = 0x89 16us, 10 pulses
    APDS_WriteByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE);

    //设置接近脉冲计数寄存器 0x8E = 0x89 16us, 10 pulses
    APDS_WriteByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE);

    //设置配置寄存器2 0x90的bit5:4=11  %300   LED驱动电流
    APDS9960_Set_LED_Boost(LED_BOOST_300);

    //是否开启手势中断配置寄存器4  0xAB
    if (interrupts)
    {
        APDS9960_Set_Gesture_IntEnable(1);
    }
    else
    {
        APDS9960_Set_Gesture_IntEnable(0);
    }

    //设置手势模式GMODE = 1
    APDS9960_Set_Gesture_mode(1);

    //打开APDS-9960  PON = 1  0x80 的 bit0 = 1
    APDS9960_SetMode(POWER, 1);

    //WEN = 1   0x80 的 bit3 = 1
    APDS9960_SetMode(WAIT, 1);

    //PEN=1   0x80 的 bit2 = 1
    APDS9960_SetMode(PROXIMITY, 1);

    //PIEN=1   0x80 的 bit6 = 1
    APDS9960_SetMode(GESTURE, 1);

    //设置ADC 积分时间寄存器  0x81 有效距离  [0,255]   0-->15cm   255-->5cm
    APDS_WriteByte(APDS9960_ATIME, 0x0);
    //等待时间寄存器  0x83 
    APDS_WriteByte(APDS9960_WTIME, 0xF0); 
    APDS_WriteByte(APDS9960_CONTROL, 0x0F);
}
/*********************************************
函数名：APDS9960_Check_Gesture_State
功  能：检测手势数据是否有效
形  参：
返回值：1--有效  0--无效 
**********************************************/
uint8_t APDS9960_Check_Gesture_State(void)
{
    uint8_t val;
    val = APDS_ReadByte(APDS9960_GSTATUS);
    val &= APDS9960_GVALID; //判断0xAF最低位GVALID是否为1
    if (val == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * 处理原始手势数据确定滑动方向
 *
 * @return True if near or far state seen. False otherwise.
 */
uint8_t processGestureData(void)
{
    uint8_t u_first = 0;
    uint8_t d_first = 0;
    uint8_t l_first = 0;
    uint8_t r_first = 0;
    uint8_t u_last = 0;
    uint8_t d_last = 0;
    uint8_t l_last = 0;
    uint8_t r_last = 0;
    int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;

    /* If we have less than 4 total gestures, that's not enough */
    if (gesture_data.total_gestures <= 4)
    {
        return 0;
    }

    /* Check to make sure our data isn't out of bounds */
    if ((gesture_data.total_gestures <= 32) && \
        (gesture_data.total_gestures > 0))
    {

        /* Find the first value in U/D/L/R above the threshold */
        for (i = 0; i < gesture_data.total_gestures; i++)
        {
            if ((gesture_data.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data.r_data[i] > GESTURE_THRESHOLD_OUT))
            {
                u_first = gesture_data.u_data[i];
                d_first = gesture_data.d_data[i];
                l_first = gesture_data.l_data[i];
                r_first = gesture_data.r_data[i];
                break;
            }
        }
        /* If one of the _first values is 0, then there is no good data */
        if ((u_first == 0) || (d_first == 0) || \
            (l_first == 0) || (r_first == 0))
        {
            return 0;
        }
        /* Find the last value in U/D/L/R above the threshold */
        for (i = gesture_data.total_gestures - 1; i >= 0; i--)
        {
            if ((gesture_data.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data.r_data[i] > GESTURE_THRESHOLD_OUT))
            {

                u_last = gesture_data.u_data[i];
                d_last = gesture_data.d_data[i];
                l_last = gesture_data.l_data[i];
                r_last = gesture_data.r_data[i];

                break;
            }
        }
    }

    /* Calculate the first vs. last ratio of up/down and left/right */
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);


    /* Determine the difference between the first and last ratios */
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;


    /* Accumulate the UD and LR delta values */
    gesture_ud_delta += ud_delta;
    gesture_lr_delta += lr_delta;

    /* Determine U/D gesture */
    if (gesture_ud_delta >= GESTURE_SENSITIVITY_1)  //50
    {
        gesture_ud_count = 1;//U-->D
    }
    else if (gesture_ud_delta <= -GESTURE_SENSITIVITY_1)
    {
        gesture_ud_count = -1;//D--->U
    }
    else
    {
        gesture_ud_count = 0;
    }

    /* Determine L/R gesture */
    if (gesture_lr_delta >= GESTURE_SENSITIVITY_1)
    {
        gesture_lr_count = 1;//L--->R
    }
    else if (gesture_lr_delta <= -GESTURE_SENSITIVITY_1)
    {
        gesture_lr_count = -1;//R--->L
    }
    else
    {
        gesture_lr_count = 0;
    }
    /* Determine Near/Far gesture */
    if ((gesture_ud_count == 0) && (gesture_lr_count == 0))
    {
        if ((abs(ud_delta) < GESTURE_SENSITIVITY_2) && (abs(lr_delta) < GESTURE_SENSITIVITY_2))  //20
        {
            if ((ud_delta == 0) && (lr_delta == 0))
            {
                gesture_near_count++;
            }
            else if ((ud_delta != 0) || (lr_delta != 0))
            {
                gesture_far_count++;
            }

            if ((gesture_near_count >= 10) && (gesture_far_count >= 2))
            {
                if ((ud_delta == 0) && (lr_delta == 0))
                {
                    gesture_state = NEAR_STATE;
                }
                else if ((ud_delta != 0) && (lr_delta != 0))
                {
                    gesture_state = FAR_STATE;
                }
                return 1;
            }
        }
    }
    else
    {
        if ((abs((int)ud_delta) < GESTURE_SENSITIVITY_2) && (abs((int)lr_delta) < GESTURE_SENSITIVITY_2))
        {

            if ((ud_delta == 0) && (lr_delta == 0))
            {
                gesture_near_count++;
            }

            if (gesture_near_count >= 10)
            {
                gesture_ud_count = 0;
                gesture_lr_count = 0;
                gesture_ud_delta = 0;
                gesture_lr_delta = 0;
            }
        }
    }
    return 0;
}
/**
 * 确定滑动方向、远近状态
 *
 * @return True if near/far event. False otherwise.
 */
uint8_t decodeGesture(void)
{
    /* Return if near or far event is detected */
    if (gesture_state == NEAR_STATE)  //手势状态 = 进距离
    {
        gesture_motion = DIR_NEAR;
        return 1;
    }
    else if (gesture_state == FAR_STATE)   //手势状态 = 远距离
    {
        gesture_motion = DIR_FAR;
        return 1;
    }

    /* Determine swipe direction 确定滑动方向 */
    if ((gesture_ud_count == -1) && (gesture_lr_count == 0))
    {
        gesture_motion = DIR_UP;
    }
    else if ((gesture_ud_count == 1) && (gesture_lr_count == 0))
    {
        gesture_motion = DIR_DOWN;
    }
    else if ((gesture_ud_count == 0) && (gesture_lr_count == 1))
    {
        gesture_motion = DIR_RIGHT;
    }
    else if ((gesture_ud_count == 0) && (gesture_lr_count == -1))
    {
        gesture_motion = DIR_LEFT;
    }
    else if ((gesture_ud_count == -1) && (gesture_lr_count == 1))
    {
        if (abs(gesture_ud_delta) > abs(gesture_lr_delta))
        {
            gesture_motion = DIR_UP;
        }
        else
        {
            gesture_motion = DIR_RIGHT;
        }
    }
    else if ((gesture_ud_count == 1) && (gesture_lr_count == -1))
    {
        if (abs(gesture_ud_delta) > abs(gesture_lr_delta))
        {
            gesture_motion = DIR_DOWN;
        }
        else
        {
            gesture_motion = DIR_LEFT;
        }
    }
    else if ((gesture_ud_count == -1) && (gesture_lr_count == -1))
    {
        if (abs(gesture_ud_delta) > abs(gesture_lr_delta))
        {
            gesture_motion = DIR_UP;
        }
        else
        {
            gesture_motion = DIR_LEFT;
        }
    }
    else if ((gesture_ud_count == 1) && (gesture_lr_count == 1))
    {
        if (abs(gesture_ud_delta) > abs(gesture_lr_delta))
        {
            gesture_motion = DIR_DOWN;
        }
        else
        {
            gesture_motion = DIR_RIGHT;
        }
    }
    else
    {
        return 0;
    }
    return 1;
}

 
/*********************************************
函数名：APDS9960_Gesture_Get_State
功  能：得到手势状态
形  参：
返回值： 
**********************************************/
int APDS9960_Gesture_Get_State(void)
{
    uint8_t fifo_level = 0;
    char bytes_read = 0;

    uint8_t gstatus;
    int i;


    /* 确保电源和手势打开且数据有效 */
    if (!APDS9960_Check_Gesture_State() || !(APDS9960_Get_Mode() & 01000001))
    {
        return DIR_NONE;
    }

    gstatus = APDS_ReadByte(APDS9960_GSTATUS);//手势状态寄存器 3--溢出  1--有效
    if ((gstatus & APDS9960_GVALID) == APDS9960_GVALID)//手势数据有效
    {
        fifo_level = APDS_ReadByte(APDS9960_GFLVL);//fifo中数据的数量
        if (fifo_level > 0) //如果FIFO里有东西，把它读到我们的数据块里
        {
            bytes_read = APDS9960_get_data(APDS9960_GFIFO_U, (uint8_t *)fifo_data, (fifo_level * 4));
        }
        /* 如果至少有一组数据，则将数据分类为U/D/L/R */
        if (bytes_read >= 4)
        {
            for (i = 0; i < bytes_read; i += 4)
            {
                gesture_data.u_data[gesture_data.index] = fifo_data[i + 0];
                gesture_data.d_data[gesture_data.index] = fifo_data[i + 1];
                gesture_data.l_data[gesture_data.index] = fifo_data[i + 2];
                gesture_data.r_data[gesture_data.index] = fifo_data[i + 3];
                gesture_data.index++;
                gesture_data.total_gestures++;
            }
            /* 过滤和处理手势数据。解码近/远状态 */
            if (processGestureData())
            {
                if (decodeGesture())
                {
                    //***TODO: U-Turn Gestures
                }
            }
            /* Reset data */
            gesture_data.index = 0;
            gesture_data.total_gestures = 0;
        }
    }
    return 0;
} 
