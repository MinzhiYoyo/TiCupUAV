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

#define SENSOR_IIC_ADDR 0xdc
const static uint8_t tab_focus[4] = {0x96,0x26,0xbc,0x50};
const static uint8_t Sensor_cfg[]={
//地址, 数据
0x12, 0x80, 
0x11, 0x30, 
0x1b, 0x06, 
0x6b, 0x43, 
0x12, 0x20, 
0x3a, 0x00, 
0x15, 0x02, 
0x62, 0x81, 
0x08, 0xa0, 
0x06, 0x68, 
0x2b, 0x20, 
0x92, 0x25, 
0x27, 0x97, 
0x17, 0x01, 
0x18, 0x79, 
0x19, 0x00, 
0x1a, 0xa0, 
0x03, 0x00, 
0x13, 0x00, 
0x01, 0x13, 
0x02, 0x20, 
0x87, 0x16, 
0x8c, 0x01, 
0x8d, 0xcc, 
0x13, 0x07, 
0x33, 0x10, 
0x34, 0x1d, 
0x35, 0x46, 
0x36, 0x40, 
0x37, 0xa4, 
0x38, 0x7c, 
0x65, 0x46, 
0x66, 0x46, 
0x6e, 0x20, 
0x9b, 0xa4, 
0x9c, 0x7c, 
0xbc, 0x0c, 
0xbd, 0xa4, 
0xbe, 0x7c, 
0x20, 0x09, 
0x09, 0x03, 
0x72, 0x2f, 
0x73, 0x2f, 
0x74, 0xa7, 
0x75, 0x12, 
0x79, 0x8d, 
0x7a, 0x00, 
0x7e, 0xfa, 
0x70, 0x0f, 
0x7c, 0x84, 
0x7d, 0xba, 
0x5b, 0xc2, 
0x76, 0x90, 
0x7b, 0x55, 
0x71, 0x46, 
0x77, 0xdd, 
0x13, 0x0f, 
0x8a, 0x10, 
0x8b, 0x20, 
0x8e, 0x21, 
0x8f, 0x40, 
0x94, 0x41, 
0x95, 0x7e, 
0x96, 0x7f, 
0x97, 0xf3, 
0x13, 0x07, 
0x24, 0x58, 
0x97, 0x48, 
0x25, 0x08, 
0x94, 0xb5, 
0x95, 0xc0, 
0x80, 0xf4, 
0x81, 0xe0, 
0x82, 0x1b, 
0x83, 0x37, 
0x84, 0x39, 
0x85, 0x58, 
0x86, 0xff, 
0x89, 0x15, 
0x8a, 0xb8, 
0x8b, 0x99, 
0x39, 0x98, 
0x3f, 0x98, 
0x90, 0xa0, 
0x91, 0xe0, 
0x40, 0x20, 
0x41, 0x28, 
0x42, 0x26, 
0x43, 0x25, 
0x44, 0x1f, 
0x45, 0x1a, 
0x46, 0x16, 
0x47, 0x12, 
0x48, 0x0f, 
0x49, 0x0d, 
0x4b, 0x0b, 
0x4c, 0x0a, 
0x4e, 0x08, 
0x4f, 0x06, 
0x50, 0x06, 
0x5a, 0x56, 
0x51, 0x1b, 
0x52, 0x04, 
0x53, 0x4a, 
0x54, 0x26, 
0x57, 0x75, 
0x58, 0x2b, 
0x5a, 0xd6, 
0x51, 0x28, 
0x52, 0x1e, 
0x53, 0x9e, 
0x54, 0x70, 
0x57, 0x50, 
0x58, 0x07, 
0x5c, 0x28, 
0xb0, 0xe0, 
0xb1, 0xc0, 
0xb2, 0xb0, 
0xb3, 0x4f, 
0xb4, 0x63, 
0xb4, 0xe3, 
0xb1, 0xf0, 
0xb2, 0xa0, 
0x55, 0x00, 
0x56, 0x40, 
0x96, 0x50, 
0x9a, 0x30, 
0x6a, 0x81, 
0x23, 0x33, 
0xa0, 0xd0, 
0xa1, 0x31, 
0xa6, 0x04, 
0xa2, 0x0f, 
0xa3, 0x2b, 
0xa4, 0x0f, 
0xa5, 0x2b, 
0xa7, 0x9a, 
0xa8, 0x1c, 
0xa9, 0x11, 
0xaa, 0x16, 
0xab, 0x16, 
0xac, 0x3c, 
0xad, 0xf0, 
0xae, 0x57, 
0xc6, 0xaa, 
0xd2, 0x78, 
0xd0, 0xb4, 
0xd1, 0x00, 
0xc8, 0x10, 
0xc9, 0x12, 
0xd3, 0x09, 
0xd4, 0x2a, 
0xee, 0x4c, 
0x7e, 0xfa, 
0x74, 0xa7, 
0x78, 0x4e, 
0x60, 0xe7, 
0x61, 0xc8, 
0x6d, 0x70, 
0x1e, 0x39, 
0x98, 0x1a
};

static void static_write_uart1( const uint8_t* buf, uint16_t length )
{
	for( uint8_t i = 0; i < length; ++i )
	{
		while( ( UART1->FR & (1<<5) ) );
		UART1->DR = buf[i];
	}
}

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
	
	
	/*发送初始化*/
		uint8_t buf[30];
		uint16_t len = sizeof(Sensor_cfg);
		
		//0xAA指令
		buf[0] = 0xAA;
		static_write_uart1( buf, 1 );

		//0xAB指令				
		buf[0] = 0xAB;
		static_write_uart1( buf, 1 );	
		static_write_uart1( tab_focus, 4 );
		buf[0] = tab_focus[0]^tab_focus[1]^tab_focus[2]^tab_focus[3];
		static_write_uart1( buf, 1 );			 
		delay(0.01);
				
		//0xBB指令							
		for(uint16_t i=0; i<len;i+=2 )
		{
			buf[0] = 0xBB;
			buf[1] = SENSOR_IIC_ADDR;
			buf[2] = Sensor_cfg[i];
			buf[3] = Sensor_cfg[i+1];
			buf[4] = SENSOR_IIC_ADDR^Sensor_cfg[i]^Sensor_cfg[i+1];
			static_write_uart1( buf, 5 );								 
		}
		delay(0.01);
		 
		//0xDD			 
		buf[0] = 0xDD;
		static_write_uart1( buf, 1 );	
	/*发送初始化*/	
	
	
	//注册传感器
	PositionSensorRegister( default_optical_flow_index , \
													Position_Sensor_Type_RelativePositioning , \
													Position_Sensor_DataType_v_xy , \
													Position_Sensor_frame_BodyHeading , \
													0.1f , \
													false );
}

typedef struct
{
	int16_t flow_x_integral;	// X 像素点累计时间内的累加位移(radians*10000)
														// [除以 10000 乘以高度(mm)后为实际位移(mm)]
	int16_t flow_y_integral;	// Y 像素点累计时间内的累加位移(radians*10000)
														// [除以 10000 乘以高度(mm)后为实际位移(mm)]
	uint16_t integration_timespan;	// 上一次发送光流数据到本次发送光流数据的累计时间（us）
	uint16_t ground_distance; // 预留。 默认为 999（0x03E7）
	uint8_t valid;	// 状态值:0(0x00)为光流数据不可用
									//245(0xF5)为光流数据可用
	uint8_t version; //版本号
}__PACKED _Flow;
static const unsigned char packet_ID[2] = { 0xfe , 0x0a };
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
		else if( rc_counter < 12 )
		{	//接收数据
			( (unsigned char*)&Flow )[ rc_counter - 2 ] = rdata;
			sum ^= (signed char)rdata;
			++rc_counter;
		}
		else if( rc_counter == 12 )
		{	//校验
			if( sum != rdata )
				rc_counter = 0;
			else
				++rc_counter;
		}
		else
		{	//接收包尾
			if( rdata == 0x55 )
			{
				vector3_float vel;
				float ultra_height;
				bool ultra_height_available = get_Estimated_Sensor_Position_z( &ultra_height , default_ultrasonic_sensor_index );
				if( ultra_height_available && Flow.valid )
				{
					float ultra_deadband = ultra_height - 5;
          if( ultra_deadband < 5 )
						ultra_deadband = 0;
					float rotation_compensation_x = -constrain_float( get_AngularRateCtrl().y * 10000 , 45000000000 );
					float rotation_compensation_y = constrain_float(  get_AngularRateCtrl().x * 10000 , 45000000000 );
					float integral_time = (Flow.integration_timespan * 1e-6f);
					vel.x = ( (float)Flow.flow_x_integral/integral_time - rotation_compensation_x ) * 1e-4f * ( 1 + ultra_deadband );
					vel.y = ( -(float)Flow.flow_y_integral/integral_time - rotation_compensation_y ) * 1e-4f * ( 1 + ultra_deadband ) ;
					PositionSensorUpdateVel( default_optical_flow_index , vel , true , -1 );
				}
				else
					PositionSensorSetInavailable( default_optical_flow_index );
			}
			rc_counter = 0;
		}
	}
}