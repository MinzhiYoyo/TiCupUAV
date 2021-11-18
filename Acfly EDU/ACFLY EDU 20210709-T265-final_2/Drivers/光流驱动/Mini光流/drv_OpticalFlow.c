#include "Basic.h"
#include "drv_OpticalFlow.h"
#include "Sensors_Backend.h"
#include "MeasurementSystem.h"

#include "TM4C123GH6PM.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "hw_types.h"
#include "hw_ints.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "pin_map.h"
#include "sysctl.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_gpio.h"

static void OpticalFlow_Handler();

void init_drv_OpticalFlow()
{
	//使能Uart1引脚（Rx:PB0 Tx:PB1）
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//使能UART1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	
	//配置GPIO
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIOB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
	
	//配置Uart
	//UART协议配置 波特率19200 8位 1停止位  无校验位	
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet() , 19200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	
	UARTFIFOEnable( UART1_BASE );
	UARTIntEnable(UART1_BASE,UART_INT_RX|UART_INT_RT);//使能UART0接收中断		
	UARTIntRegister(UART1_BASE,OpticalFlow_Handler);//UART中断地址注册	
	IntPrioritySet(INT_UART1, INT_PRIO_7);
	
	//注册传感器
	PositionSensorRegister( default_optical_flow_index , \
													Position_Sensor_Type_RelativePositioning , \
													Position_Sensor_DataType_v_xy , \
													Position_Sensor_frame_BodyHeading , \
													0.1f , \
													false );
}

typedef struct
{   //迷你光流模块数据40ms更新一次，光流数据有更新时，数据从UART-TX发出
	int16_t flow_x_integral;	// X轴从最后一次数据更新到当前更新的位移
	int16_t flow_y_integral;	// Y轴从最后一次数据更新到当前更新的位移
	uint8_t flow_sum;	// 光流数据累加和取低8位
	uint8_t quality;	//图像质量0到100，30以下数据较差建议舍弃
}__PACKED _Flow;

static const unsigned char packet_ID[2] = { 0xfe , 0x04 };
static void OpticalFlow_Handler()
{
	uint32_t uart_err = UARTRxErrorGet( UART1_BASE );
	UARTIntClear( UART1_BASE , UART_INT_OE | UART_INT_RT );
	UARTRxErrorClear( UART1_BASE );
	
	//状态机接收数据
	while( ( UART1->FR & (1<<4) ) == false	)
	{
		//接收
		uint8_t rdata = UART1->DR & 0xff;
		
		/*状态机*/
			static _Flow  Flow;
			static unsigned char rc_counter = 0;
			static signed char sum = 0;
		/*状态机*/
		
		if( rc_counter < 2 )
		{
			//接收包头
			if( rdata != packet_ID[ rc_counter ] )
				rc_counter = 0;
			else
			{
				++rc_counter;
				sum = 0;
			}
		}
		else if( rc_counter < 8 )
		{	//接收数据
			( (unsigned char*)&Flow )[ rc_counter - 2 ] = rdata;
			sum += (signed char)rdata;
			++rc_counter;
		}
		else
		{	//校验
			if( rdata == 0xAA && Flow.quality > 30 )
			{
				vector3_float vel;
				float ultra_height;
				bool ultra_height_available = get_Estimated_Sensor_Position_z( &ultra_height , default_ultrasonic_sensor_index );
				if( ultra_height_available )
				{
					float ultra_deadband = ultra_height - 5;
          if( ultra_deadband < 5 )
						ultra_deadband = 0;
					#define OKp 400
					float rotation_compensation_x = -constrain_float( get_AngularRateCtrl().y * OKp , 4500000000 );
					float rotation_compensation_y = constrain_float(  get_AngularRateCtrl().x * OKp , 4500000000 );
					static TIME last_update_TIME = {0};					
					float integral_time = get_pass_time_st(&last_update_TIME);
					if( integral_time > 1e-3f )
					{
						float freq = 1.0f / integral_time;
						vel.x = ( -(float)Flow.flow_x_integral*freq - rotation_compensation_x ) * (1.0/OKp) * ( 1 + ultra_deadband );
						vel.y = ( (float)Flow.flow_y_integral*freq - rotation_compensation_y ) * (1.0/OKp) * ( 1 + ultra_deadband );
						PositionSensorUpdateVel( default_optical_flow_index , vel , true , -1 );
					}
					else
						PositionSensorSetInavailable( default_optical_flow_index );
				}
				else
					PositionSensorSetInavailable( default_optical_flow_index );
			}
			rc_counter = 0;
		}
	}
}