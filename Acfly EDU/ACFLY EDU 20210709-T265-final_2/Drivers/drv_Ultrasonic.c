


#ifdef ULTRASONIC
#include "Basic.h"
#include "STS.h"
#include "drv_Ultrasonic.h"
#include "Sensors_Backend.h"
#include "MeasurementSystem.h"

#include "TM4C123GH6PM.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "pin_map.h"
#include "pwm.h"
#include "rom.h"
#include "sysctl.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_gpio.h"

static void Ultrasonic_Handler();
static void Ultrasonic_Server( unsigned int Task_ID );
/*状态机*/
	//超声波测高状态
	//255：未开始
	//0：已发送开始信号
	//1：已记录第一个边沿
	static uint8_t Ultrasoinc_counter = 255;
/*状态机*/

void init_drv_Ultrasonic()
{
	//打开GPIOF电源（PF0：Echo PF1：Trig）
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	//PF0解锁
	GPIOF->LOCK = GPIO_LOCK_KEY;
	GPIOF->CR |= 0x1;
	GPIOF->LOCK = 0;
	
	//配置PF1为输出模式
	GPIOPinTypeGPIOOutput( GPIOF_BASE , GPIO_PIN_1 );
	GPIOPinWrite( GPIOF_BASE , GPIO_PIN_1 , 0 );
	
	//配置PF0为输入捕获引脚
	GPIOPinTypeTimer(GPIOF_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PF0_T0CCP0);
	
	//开启定时器0(T0CCP0)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	
	//配置定时器0A为双边沿捕获
	TimerConfigure( TIMER0_BASE ,TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP ); 
	TimerControlEvent(TIMER0_BASE,TIMER_A,TIMER_EVENT_BOTH_EDGES);	
	TimerLoadSet( TIMER0_BASE , TIMER_A , 0xffff );
	TimerPrescaleSet( TIMER0_BASE , TIMER_A , 0xff );
	
	//开启定时器中断
	TimerIntRegister(TIMER0_BASE,  TIMER_A , Ultrasonic_Handler);	
	IntPrioritySet( INT_TIMER0A , INT_PRIO_7);
	TimerIntEnable( TIMER0_BASE , TIMER_CAPA_EVENT);
	TimerEnable( TIMER0_BASE, TIMER_A );
	IntEnable( INT_TIMER0A );
	
	//注册传感器
	PositionSensorRegister( default_ultrasonic_sensor_index , \
													Position_Sensor_Type_RangePositioning , \
													Position_Sensor_DataType_s_z , \
													Position_Sensor_frame_ENU , \
													0.05f , \
													false );
	
	//添加超声波发送任务
	STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 1.0f/10 , 0 , Ultrasonic_Server );
}

static void Ultrasonic_Server( unsigned int Task_ID )
{
	if( Ultrasoinc_counter != 255 )
		PositionSensorSetInavailable( default_ultrasonic_sensor_index );
	
	GPIOPinWrite( GPIOF_BASE , GPIO_PIN_1 , GPIO_PIN_1 );
	delay( 10e-6f );
	Ultrasoinc_counter = 0;
	GPIOPinWrite( GPIOF_BASE , GPIO_PIN_1 , 0 );
}

static void Ultrasonic_Handler()
{
	TimerIntClear( TIMER0_BASE , TIMER_CAPA_EVENT );
	
	static uint32_t last_value = 0;

	
	switch( Ultrasoinc_counter )
	{
		case 0:
			//已发送开始信号
			last_value = TimerValueGet( TIMER0_BASE , TIMER_A );
			++Ultrasoinc_counter;
			break;
		
		case 1:
		{
			//已记录第一个边沿
			uint32_t trig_value = TimerValueGet( TIMER0_BASE , TIMER_A );
			float t;
			if( trig_value > last_value )
				t = ( trig_value - last_value ) * ( 1e-6f / 80 );
			else
				t = ( trig_value + 0xffffff - last_value ) * ( 1e-6f / 80 );
			
			vector3_float position;
			position.z = t * 17000.0f;
			if( position.z > 1 && position.z < 600 )
			{
				float lean_cosin = get_lean_angle_cosin();
				position.z *= lean_cosin;
				PositionSensorUpdatePosition( default_ultrasonic_sensor_index , position , true , -1 );
			}
			else
				PositionSensorSetInavailable( default_ultrasonic_sensor_index );
			
			Ultrasoinc_counter = 255;
			
			break;
		}
	}
}


#else

#include "Basic.h"
#include "drv_Uart2.h"

#include "Quaternion.h"
#include "MeasurementSystem.h"
#include "drv_SDI.h"
#include "Commulink.h"
#include "STS.h"
#include "Sensors_Backend.h"
#include "RingBuf.h"

#include "TM4C123GH6PM.h"
#include "uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "Timer.h"
#include "udma.h"

float TFMiniHigh = -1;

/*接收缓冲区*/
#define RX_BUFFER_SIZE 24
static uint8_t R7_rx_buffer[RX_BUFFER_SIZE];
static uint8_t R7_Rx_RingBuf;
static float HIGH_EIGHT;
static float LOW_EIGHT;
static float ks109_HIGH;
static void UART7_Handler(void);
static void UART7_receive(void);
static void ks109_send(unsigned int Task_ID);
void init_drv_Ultrasonic(void)
{
   
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	 GPIOPinConfigure(GPIO_PE0_U7RX);
	 GPIOPinConfigure(GPIO_PE1_U7TX);
	 
	 GPIOPinTypeUART(GPIOE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	 UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	 UARTFIFOEnable( UART7_BASE );	
	
	//配置串口接收中断
	UARTIntEnable( UART7_BASE , UART_INT_RX | UART_INT_RT );
	UARTIntRegister( UART7_BASE , UART7_Handler );	 
	
	UARTFIFOEnable( UART7_BASE );
	UARTIntEnable(UART7_BASE,UART_INT_RX | UART_INT_RT);//使能UART0发送接收中断		
  UARTIntRegister(UART7_BASE,UART7_Handler);//UART中断地址注册	
	IntPrioritySet(INT_UART7, INT_PRIO_7);
	//STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 1.0f/10 , 0 , send );
//	PositionSensorRegister( default_ultrasonic_sensor_index , \
//													Position_Sensor_Type_RangePositioning , \
//													Position_Sensor_DataType_s_z , \
//													Position_Sensor_frame_ENU , \
//													0.05f , \
//													0, 0 );
	PositionSensorRegister( default_ultrasonic_sensor_index , \
													Position_Sensor_Type_RangePositioning , \
													Position_Sensor_DataType_s_z , \
													Position_Sensor_frame_ENU , \
													0.05f , \
													false );
}

uint16_t hign_z;
static uint8_t hbyte[9];

static void UART7_Handler()
{
	int i = 0;
	UARTIntClear( UART7_BASE , UART_INT_OE | UART_INT_RT );
	UARTRxErrorClear( UART7_BASE );
	while( ( UART7->FR & (1<<4) ) == false	)
	{
		//接收
		
    uint8_t rdata = UART7->DR & 0xff;
		static unsigned char rc_counter = 0;
		static unsigned char receive = 0;
		
		static uint16_t check_sum;
		static signed char sum = 0;
		
	  if( rc_counter < 2 )
		{
			//接收包头
			if( rdata != 0x59)
			{
				rc_counter = 0;
			}else
			{
				hbyte[rc_counter] = rdata;
				++rc_counter;
			}
		}else if( rc_counter < 9 )
		{
			hbyte[rc_counter] = rdata;
			++rc_counter;
		}else if(rc_counter == 9)
		{
			check_sum = hbyte[0]+hbyte[1]+hbyte[2]+hbyte[3]+hbyte[4]+hbyte[5]+hbyte[6]+hbyte[7];
		if((check_sum&0xFF) != hbyte[8])
			{
				rc_counter = 0;
			}
			else{
				++rc_counter;
			}
		}else
		{
			vector3_float position;
			position.z = ((float)(signed short)(((((unsigned short)hbyte[3])<<8)|(uint8_t)hbyte[2])));
			TFMiniHigh = position.z;
				hign_z = position.z;
			if( position.z > 1 && position.z < 5000 )
			{
				// PositionSensorUpdatePosition( default_ultrasonic_sensor_index , position , true , -1, 0 , 0 );
				PositionSensorUpdatePosition( default_ultrasonic_sensor_index , position , true , -1);
			}
			else
			{
				PositionSensorSetInavailable( default_ultrasonic_sensor_index );
			}
			rc_counter = 0;
		}
		
			
				
		}
		
}
	

	uint16_t get_position_z()
	{
		return hign_z;
	}


#endif
