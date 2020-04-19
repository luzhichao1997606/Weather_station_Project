#ifndef __APDS9960_H_
#define __APDS9960_H_
#include "cmsis_os.h" 
#include "main.h"  
#include <stdlib.h>
//APDS_INT			:    PA15
//APDS_SDA			:    PA12
//APDS_SCL		    :    PA11 
/* -----------------------------------------宏定义-----------------------------------------*/
/* APDS-9960 I2C address */
#define APDS9960_I2C_ADDR       0x39	//I2C地址

/* Gesture parameters */
#define GESTURE_THRESHOLD_OUT   10	 //输出阀值
#define GESTURE_SENSITIVITY_1   50   //灵敏度1
#define GESTURE_SENSITIVITY_2   20	 //灵敏度2   远近敏感度  越大越敏感

/* Error code for returned values */
#define ERROR                   0xFF 

/* Acceptable device IDs */
#define APDS9960_ID_1           0xAB
#define APDS9960_ID_2           0x9C 

/* Misc parameters */
#define FIFO_PAUSE_TIME         30      // Wait period (ms) between FIFO reads

/* APDS-9960 register addresses */
/*0x80	使能寄存器
7 保留
6 GEN：手势使能
5 PIEN：接近中断使能
4 AIEN：环境光感中断使能
3 WEN：等待使能
2 PEN：接近检测使能
1 AEN：环境光检测使能
0 PON：上电使能
*/
#define APDS9960_ENABLE         0x80	
 

/*0x81  ADC积分时间寄存器
7:0	  ATIME 
字段值			  周期		       时间		      最大值	
0                 256              712ms           65535
182                72              200ms           65535
256-TIME/2.78      ...             ...             ...
219                37              103ms           37889
246                10              27.8ms          10241
255                1               2.78ms           1025
*/
#define APDS9960_ATIME          0x81




/*0x83  等待时间寄存器
7:0  WTIME
字段值			  等待时间		   时间(WLONG=0)	   时间(WLONG=1)	  	
0                 256              712ms               8.54s             
256-TIME/2.78      ...             ...                 ...
171                85              236ms               2.84s
255                1               2.78ms              0.03s
*/
#define APDS9960_WTIME          0x83




/*0x84 -- 0x87  环境光中断阀值寄存器
0x84  AILTL     low byte low interrupt threshold
0x85  AILTH	  	high byte low interrupt threshold
0x86  AIHTL		low byte high  interrupt threshold  
0x87  AIHTH		high byte high  interrupt threshold
*/
#define APDS9960_AILTL          0x84
#define APDS9960_AILTH          0x85
#define APDS9960_AIHTL          0x86
#define APDS9960_AIHTH          0x87




/*0x89、0x8B	接近中断阀值寄存器
0x89	PILT	低中断阀值
0x8B	PIHT	高中断阀值
*/
#define APDS9960_PILT           0x89
#define APDS9960_PIHT           0x8B



/*0x8C	持续寄存器
PPERS   7:4   	 接近中断持续，控制主进程接近中断的速率
		字段值   当下列情况发生时接近中断产生
		0        每个接近周期
		1		 任何在阀值范围外的接近值
		2		 2个连续的在阀值范围外的接近值
		...		 ...
APERS   3:0		 环境光感应中断持续，控制主进程环境光感应中断的速率
		字段值	 当下列情况发生时环境光中断产生
		0        每个接近周期
		1		 任何在阀值范围外的环境光感应值
		2		 2个连续的在阀值范围外的环境光感应值
		3        3...
		4        5...
		5        10...
		6        15...
		7        20...
		8        25...
		9        30...
		10       35...
		11       40...
		...      ...
		15       60
*/
#define APDS9960_PERS           0x8C




/*0x8D  配置寄存器1  上电时被设为0x40
7    保留   写0
6    保留   写1
5	 保留   写1
4-2	 保留	写0
1    WLONG	 WaitLong 长时间等待   12x 设置值
0	 保留
*/
#define APDS9960_CONFIG1        0x8D




/*0x8E  接近脉冲计数寄存器    
PPLEN  7:6   接近脉宽
		0      	4us
		1		8us(default)
		2		16us
		3		32us
PPULSE  5:0   接近脉冲计数
		0		1
		1		2
		2		3
		...		...
		63		64
*/
#define APDS9960_PPULSE         0x8E





/*0x8F	控制寄存器1
LDRIVE	7:6	  LED驱动强度
		0		100ma
		1		50ma
		2		25ma
		3		12.5ma
保留	5:4		写0
PGAIN	3:2		接近增益控制
		0		1x
		1		2x
		2		4x
		3		8x
AGAIN	1:0		环境光感应及颜色增益
		0		1x
		1		4x
		2		16x
		3		64x
*/
#define APDS9960_CONTROL        0x8F




/*0x90   配置寄存器2
PSIEN		7	  接近饱和中断使能
				  0 接近饱和中断失能
				  1	接近饱和中断使能
CPSIEN		6	  清除光电二极管饱和中断(ALS饱和中断)使能
				  0 环境光感饱和中断失能
				  1 环境光感饱和中断使能
LED_BOOST	5:4   增加LDR当前接近和手势LED脉冲，当前值由LDRIVE设置，增长由LDE_BOOST
				  0  100%
				  1	 150%
				  2  200%
				  3  300%
保留		3:1   写0
保留		0     写1，在POR时默认被设为高
*/			
#define APDS9960_CONFIG2        0x90





/*0x92   ID寄存器（只读）
ID			7:0		唯一标识的器件ID
					0xAB = APDS-9960
*/
#define APDS9960_ID             0x92




/*0x93    状态寄存器（只读）在上电时被设为0x04
CPSAT		7	    清除光电二极管饱和，这个位会被还原当发送清除通道中断命令(0xE6 CICLEAR)
					或者使能ADC位(AEN=0),这个位会出发中断如果CPSIEN被设置
PGSAT		6		显示模拟饱和事件在前面的手势或接近周期，一旦被设置，这个位将一直被保持
                    直到被接近中断特殊的功能命令(0xE5 PICLEAR)清除或者使能接近(PEN=0)这个位
					引发一次中断如果PSIEN被设置
PINT		5		接近中断，这个位触发一次中断如果PIEN在使能中被设置
AINT		4		环境光感中断，这个位触发一次中断如果AIEN在使能中被设置
保留		3		不作操作
GINT		2		手势中断，GFVLV变成比GFIFOTH大或GVALID被置位(GMODE发送0时)被设置,这个位
                    被复位当FIFO完全为空时(read)
PVALID		1		有效接近值.完整显示接近周期自从PEN被设置或PDATA被最后读,读一次PDATA将自动清除PVALID
AVALID		0		有效环境光感.完整显示环境光感周期当AEN被设置或在ALS/Color数据寄存器其中任一被读出
*/
#define APDS9960_STATUS         0x93




/*0x94--0x9B   RGBC数据寄存器
CDATAL	0x94	7:0		clear通道低字节
CDATAH	0x95	7:0		clear通道高字节
RDATAL	0x96	7:0		red通道低字节
RDATAH	0x97	7:0		red通道高字节
GDATAL	0x98	7:0		green通道低字节
GDATAH	0x99	7:0		green通道高字节
BDATAL 	0x9A	7:0		blue通道低字节
BDATAH	0x9B	7:0		blue通道高字节
*/
#define APDS9960_CDATAL         0x94
#define APDS9960_CDATAH         0x95
#define APDS9960_RDATAL         0x96
#define APDS9960_RDATAH         0x97
#define APDS9960_GDATAL         0x98
#define APDS9960_GDATAH         0x99
#define APDS9960_BDATAL         0x9A
#define APDS9960_BDATAH         0x9B




/*0x9C	接近数据寄存器
PDATA	 7:0	接近数据
*/
#define APDS9960_PDATA          0x9C




/*0x9D	接近偏移 UP和RIGHT
POFFSET_UR	7:0		字段值			偏移校正因子
					01111111			127
					...					...
					00000001			1
					00000000			0
					10000001			-1
					...					...
					11111111			-127
*/
#define APDS9960_POFFSET_UR     0x9D





/*0x9E	接近偏移 DOWN和LEFT
POFFSET_DL	7:0		字段值			偏移校正因子
					01111111			127
					...					...
					00000001			1
					00000000			0
					10000001			-1
					...					...
					11111111			-127
*/
#define APDS9960_POFFSET_DL     0x9E




/*0x9F		配置寄存器3 
保留		7:6		写0
PCMP		5		接近增益补偿使能.这个位提供增益，当接近光电二极管信号由于遮挡二减少时.如果
					只有一个二极管(在二极管对中)有效,那么仅仅只有一般的信号在ADC中是有效的，这个
					结果ADC最大值是127.使PCMP额外的增益2X,将会使最大的ADC值为255
					PLMASK_X(U.D.L.R)		PCMP
					0,1,1,1					1
					1,0,1,1					1
					1,1,0,1					1
					1,1,1,0					1
					0,1,0,1					1
					1,0,1,0					1
					All Other				0
SAI			4		中断后进入睡眠.被使能后当发生中断时自动进入低功耗模式，并且器件状态会转换到SAI决策块
					正常的操作是当中断引脚被I2C清零时被恢复
PMASK_U		3		接近遮挡UP使能	写1失能此光电二极管
PMASK_D		2		接近遮挡DOWN使能	写1失能此光电二极管
PMASK_L		1		接近遮挡LEFT使能	写1失能此光电二极管
PMASK_R		0		接近遮挡RIGHT使能 写1失能此光电二极管
*/
#define APDS9960_CONFIG3        0x9F




/*0xA0	手势接近进入阀值寄存器 
GPENTH		7:0		接近手势输入阀值.这个寄存器设定接近阀值用作决定手势开始，并且同时进入
					手势状态机(bit4必须设为0)
*/
#define APDS9960_GPENTH         0xA0




/*0xA1   手势退出阀值寄存器
GEXTH		7:0		手势退出阀值.此寄存器设置阀值决定手势结束,同时退出手势状态机.
					设置GTHR_OUT为0x00会防止手势退出直到GMODE被设为0
*/
#define APDS9960_GEXTH          0xA1




/*0xA2	手势配置寄存器1
GFIFOTH		7:6		手势FIFO阀值.这个值与FIFO  level比较产生一个中断
					0		在1个数据集被添加到FIFO里后产生中断
					1		4
					2		8
					3		16
GEXMSK		5:2	    手势退出遮挡.控制哪个手势探测光电二极管被包含到决定手势结束并且同时退出手势状态机
			0000	All UDLR 探测数据被包含到集合中
			0001	R不被包含到集合中
			0010	L
			0100	D
			1000	U
			0101	...
			0110	L D
			1111	UDLR
GEXPERS		1:0		手势退出持久性.当连续的手势结束发生称为比GEXPERS大于或等于的值时，手势状态机退出
			0		第1个手势结束发生导致手势状态机退出
			1		2
			2		4
			3		7
*/
#define APDS9960_GCONF1         0xA2




/*0xA3   手势配置寄存器2
保留		7		写0
GGAIN		6:5		手势增益控制，设定一个增益手势接收在手势模式下
			0		1x
			1		2x
			2		4x
			3		8x
GLDRIVE		4:3		手势LED驱动强度
			0		100ma
			1		50ma
			2		25ma
			3		12.5ma
GWTIME		2:0		手势等待时间	
			0		0ms
			1		2.8ms
			2		5.6ms	
			3		8.4ms
			4		14.0ms
			5		22.4ms
			6		30.8ms
			7		39.2ms
*/
#define APDS9960_GCONF2         0xA3




/*0xA4    手势UP偏移寄存器
GOFFSET_U	7:0		字段值				偏移校正因子
					01111111			127
					...					...
					00000001			1
					00000000			0
					10000001			-1
					...					...
					11111111			-127
*/
#define APDS9960_GOFFSET_U      0xA4





/*0xA5    手势DOWN偏移寄存器
GOFFSET_D	7:0		字段值				偏移校正因子
					01111111			127
					...					...
					00000001			1
					00000000			0
					10000001			-1
					...					...
					11111111			-127
*/
#define APDS9960_GOFFSET_D      0xA5





/*0xA7    手势LEFT偏移寄存器
GOFFSET_L	7:0		字段值				偏移校正因子
					01111111			127
					...					...
					00000001			1
					00000000			0
					10000001			-1
					...					...
					11111111			-127
*/
#define APDS9960_GOFFSET_L      0xA7






/*0xA9    手势RIGHT偏移寄存器
GOFFSET_R	7:0		字段值				偏移校正因子
					01111111			127
					...					...
					00000001			1
					00000000			0
					10000001			-1
					...					...
					11111111			-127
*/
#define APDS9960_GOFFSET_R      0xA9




/*0xA6		手势脉冲数和脉宽寄存器
GPLEN		7:6		手势脉宽.设定LED_ON脉宽在手势LDR脉冲期间
			0		4us
			1		8us(default)
			2		16us
			3		32us
GPULSE		5:0		手势脉冲数.特定的脉冲数由LDR产生
			0		1
			1		2
			2		3
			...		...
			63		64
*/
#define APDS9960_GPULSE         0xA6




/*0xAA	手势配置寄存器3
保留		7:2		写0
GDIMS		1:0		选择哪种手势光电二极管在对手势时能够手机结果
			0		两对都有效.UP-DOWN  LEFT-RIGHT FIFO数据有效
			1		UP-DOWN有效		LEFT-RIGHT不用管
			2		LEFT-RIGHT		UP-DOWN
			3		同0
*/
#define APDS9960_GCONF3         0xAA





/*0xAB	手势配置寄存器4
保留		7:3		写0
GFIFO_CLR	2		设1清除GFIFO,GINT,GVALID,GFIFO_OV,GFIFO_LVL
GIEN		1		手势中断使能	
GMODE		0		手势模式	
*/
#define APDS9960_GCONF4         0xAB





/*0xAE	手势FIFO Level寄存器
GFLVL		7:0		一个四字节数据等同于一个GFLVL数
*/
#define APDS9960_GFLVL          0xAE




/*0xAF	手势状态寄存器
保留		7:2		不作操作
GFOV		1		手势FIFO溢出
GVALID		0		手势FIFO数据
*/
#define APDS9960_GSTATUS        0xAF





/*0xE4--0xE7	清除中断寄存器
IFORCE		0xE4	7:0		强制中断
PICLEAR		0xE5	7:0		接近中断清除
CICLEAR		0xE6	7:0		环境光感中断清除
AICLEAR		0xE7	7:0		清除非手势中断
*/
#define APDS9960_IFORCE         0xE4
#define APDS9960_PICLEAR        0xE5
#define APDS9960_CICLEAR        0xE6
#define APDS9960_AICLEAR        0xE7




/*0xFC--0xFF	手势FIFO寄存器
GFIFO_U		0xFC	7:0		手势FIFO-UP 值
GFIFO_D		0xFD	7:0		手势FIFO-DOWN值
GFIFO_L		0xFE	7：0	手势FIFO-LEFT值
GFIFO_R		0xFF	7:0		手势FIFO-RIGHT值
*/
#define APDS9960_GFIFO_U        0xFC
#define APDS9960_GFIFO_D        0xFD
#define APDS9960_GFIFO_L        0xFE
#define APDS9960_GFIFO_R        0xFF



/* Bit fields */
#define APDS9960_PON            00000001//Power on enable
#define APDS9960_AEN            00000010//ALS enable
#define APDS9960_PEN            00000100//Prox enable
#define APDS9960_WEN            00001000//Wait enable
#define APSD9960_AIEN           00010000//ALS interrupt enable
#define APDS9960_PIEN           00100000//Peox interrupt enable
#define APDS9960_GEN            01000000//Gesture enable
#define APDS9960_GVALID         00000001//Gesture FIFO data

/* On/Off definitions */
#define OFF                     0
#define ON                      1

/* Acceptable parameters for setMode */
#define POWER                   0
#define AMBIENT_LIGHT           1
#define PROXIMITY               2
#define WAIT                    3
#define AMBIENT_LIGHT_INT       4
#define PROXIMITY_INT           5
#define GESTURE                 6
#define ALL                     7 

/* LED Drive values */
#define LED_DRIVE_100MA         0
#define LED_DRIVE_50MA          1
#define LED_DRIVE_25MA          2
#define LED_DRIVE_12_5MA        3

/* Proximity Gain (PGAIN) values */
#define PGAIN_1X                0
#define PGAIN_2X                1
#define PGAIN_4X                2
#define PGAIN_8X                3

/* ALS Gain (AGAIN) values */
#define AGAIN_1X                0
#define AGAIN_4X                1
#define AGAIN_16X               2
#define AGAIN_64X               3

/* Gesture Gain (GGAIN) values */
#define GGAIN_1X                0
#define GGAIN_2X                1
#define GGAIN_4X                2
#define GGAIN_8X                3

/* LED Boost values */
#define LED_BOOST_100           0
#define LED_BOOST_150           1
#define LED_BOOST_200           2
#define LED_BOOST_300           3    

/* Gesture wait time values */
#define GWTIME_0MS              0
#define GWTIME_2_8MS            1
#define GWTIME_5_6MS            2
#define GWTIME_8_4MS            3
#define GWTIME_14_0MS           4
#define GWTIME_22_4MS           5
#define GWTIME_30_8MS           6
#define GWTIME_39_2MS           7

/* Default values */
#define DEFAULT_ATIME           219     // 103ms
#define DEFAULT_WTIME           246     // 27ms
#define DEFAULT_PROX_PPULSE     0x87    // 16us, 8 pulses
#define DEFAULT_GESTURE_PPULSE  0x89    // 16us, 10 pulses
#define DEFAULT_POFFSET_UR      0       // 0 offset
#define DEFAULT_POFFSET_DL      0       // 0 offset      
#define DEFAULT_CONFIG1         0x60    // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE          LED_DRIVE_100MA
#define DEFAULT_PGAIN           PGAIN_4X
#define DEFAULT_AGAIN           AGAIN_4X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost  
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30      // Threshold for exiting gesture mode    
#define DEFAULT_GCONF1          0x40    // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN           GGAIN_4X
#define DEFAULT_GLDRIVE         LED_DRIVE_100MA
#define DEFAULT_GWTIME          GWTIME_2_8MS
#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define DEFAULT_GPULSE          0xC9    // 32us, 10 pulses
#define DEFAULT_GCONF3          0       // All photodiodes active during gesture
#define DEFAULT_GIEN            0       // Disable gesture interrupts

/* Direction definitions */
enum {
  DIR_NONE,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_UP,
  DIR_DOWN,
  DIR_NEAR,
  DIR_FAR,
  DIR_ALL
};

/* State definitions */
enum {
  NA_STATE,
  NEAR_STATE,
  FAR_STATE,
  ALL_STATE
};
/* -----------------------------------------头文件-----------------------------------------*/
#include "main.h"

/* -----------------------------------------结构体定义-------------------------------------*/
/* Container for gesture data */
typedef struct
{
    uint8_t u_data[32];
    uint8_t d_data[32];
    uint8_t l_data[32];
    uint8_t r_data[32];
    uint8_t index;
    uint8_t total_gestures;
    uint8_t in_threshold;
    uint8_t out_threshold;
} gesture_data_type;

extern gesture_data_type gesture_data;
/* -----------------------------------------全局变量定义-----------------------------------*/
extern int gesture_motion; 
 
/* -----------------------------------------应用程序---------------------------------------*/
 
//获取ADPS9960的ID=0XAB
uint8_t APDS9960_Get_ID(void);

//设置模式 0x80寄存器
BaseType_t APDS9960_SetMode(int8_t mode, uint8_t enable);

//初始化APDS9960
BaseType_t APDS9960_Init();

// 启用手势模式
void APDS9960_Gesture_EN(uint8_t interrupts);

//检测手势数据是否有效
uint8_t APDS9960_Check_Gesture_State(void);

//得到手势状态
int APDS9960_Gesture_Get_State(void);

//确定滑动方向、远近状态
uint8_t decodeGesture(void);

//设置变量
void resetGestureParameters(void);
#endif

