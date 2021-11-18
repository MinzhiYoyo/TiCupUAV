#include "Basic.h"
#include "drv_SDI.h"

#include "Quaternion.h"
#include "MeasurementSystem.h"

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

#define BUFFER_SIZE 64

//串口中断
static void UART3_Handler();

/*发送缓冲区*/
	static uint8_t tx_buffer[BUFFER_SIZE];
	static RingBuf_uint8_t Tx_RingBuf;
/*发送缓冲区*/

/*接收缓冲区*/
	static uint8_t rx_buffer[BUFFER_SIZE];
	static RingBuf_uint8_t Rx_RingBuf;
/*接收缓冲区*/

static bool SDI_RCTrigger( unsigned int Task_ID );
static void SDI_Server( unsigned int Task_ID );

void init_drv_SDI()
{
	//使能Uart3引脚（Rx:PC6 Tx:PC7）
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	//使能UART3
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	
	//配置GPIO
	GPIOPinConfigure(GPIO_PC6_U3RX);
	GPIOPinConfigure(GPIO_PC7_U3TX);
	GPIOPinTypeUART(GPIOC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
		
	//配置Uart
	UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet() , 115200,			
	                   (UART_CONFIG_WLEN_8 
										| UART_CONFIG_STOP_ONE 
										|	UART_CONFIG_PAR_NONE));	
	
	//初始化缓冲区
	RingBuf_uint8_t_init( &Tx_RingBuf , tx_buffer , BUFFER_SIZE );
	RingBuf_uint8_t_init( &Rx_RingBuf , rx_buffer , BUFFER_SIZE );
	
	//配置串口接收中断
	UARTIntEnable( UART3_BASE , UART_INT_RX | UART_INT_RT );
	UARTIntRegister( UART3_BASE , UART3_Handler );
	
	//配置DMA发送
	uDMAChannelControlSet( UDMA_PRI_SELECT | UDMA_CH17_UART3TX , \
		UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1 );
	UARTDMAEnable( UART3_BASE , UART_DMA_TX );
	UARTIntRegister( UART3_BASE , UART3_Handler );	
	uDMAChannelAssign(UDMA_CH17_UART3TX  );	
	
	//打开中断
	IntPrioritySet( INT_UART3 , INT_PRIO_5 );
	IntEnable( INT_UART3 );
	
	
//	//注册传感器
//	PositionSensorRegister( 3 , Position_Sensor_Type_RelativePositioning , Position_Sensor_DataType_s_xy , Position_Sensor_frame_ENU , 0.1f , false );
	//添加简单二次开发协议解析任务
	STS_Add_Task( STS_Task_Trigger_Mode_Custom , 0 , SDI_RCTrigger , SDI_Server );
}

static bool SDI_RCTrigger( unsigned int Task_ID )
{
	if( Uart3_DataAvailable() )
		return true;
	return false;
}
float SDI_Point[6];
TIME SDI_Time = {0};
static void SDI_Server( unsigned int Task_ID )
{
	//简单二次开发协议解析
	
	/*状态机变量*/
		static uint8_t rc_step1 = 0;	//0：接收包头'A' 'C'
																	//1：接收1字节消息类别
																	//2：接收1字节消息长度
																	//3：接收数据包内容
																	//4：接收2字节校验
		static uint8_t rc_step2 = 0;
	
		#define MAX_SDI_PACKET_SIZE 4*6
		static uint8_t msg_type;
		static uint8_t msg_length;
		ALIGN4 static uint8_t msg_pack[MAX_SDI_PACKET_SIZE];
		static uint8_t sumA;
		static uint8_t sumB;
		
		#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )
	/*状态机变量*/
	
	uint8_t rc_buf[20];
	uint8_t length = read_Uart3( rc_buf , 20 );
	for( uint8_t i = 0 ; i < length ; ++i )
	{
		uint8_t r_data = rc_buf[i];
		
		switch( rc_step1 )
		{
			case 0 :
				//接收包头'A''C'
				if( rc_step2 == 0 )
				{
					if( r_data == 'A' )
						rc_step2 = 1;
				}
				else
				{
					if( r_data == 'C' )
					{
						rc_step1 = 1;
						rc_step2 = 0;
						sumA = sumB = 0;
					}
					else
						rc_step2 = 0;
				}
				break;
				
			case 1:
				//接收消息类别
				msg_type = r_data;
				sumB += r_data;
				sumA += sumB;
				rc_step1 = 2;
				rc_step2 = 0;
				break;
			
			case 2:
				//接收消息长度
				if( r_data > MAX_SDI_PACKET_SIZE )
				{
					reset_SDI_RC;
					break;
				}
				msg_length = r_data;
				sumB += r_data;
				sumA += sumB;
				if( msg_length == 0 )
					rc_step1 = 4;
				else
					rc_step1 = 3;
				rc_step2 = 0;
				break;
				
			case 3:
				//接收数据包
				msg_pack[ rc_step2 ] = r_data;
				sumB += r_data;
				sumA += sumB;
				++rc_step2;
				if( rc_step2 >= msg_length )
				{
					rc_step1 = 4;
					rc_step2 = 0;
				}
				break;
				
			case 4:
				//接收校验位
				if( rc_step2 == 0 )
				{
					if( sumB != r_data )
					{
						reset_SDI_RC;
						break;
					}
					rc_step2 = 1;
				}
				else
				{
					if( sumA == r_data )
					{
						if( msg_type == 1 )
						{
							//PositionSensorSetInavailable( 3 );
							Time_set_inValid( &SDI_Time );
						}
						if( msg_type == 2 )
						{
							SDI_Point[0] = *(float*)&msg_pack[0];    //存储x方向速度
							SDI_Point[1] = *(float*)&msg_pack[4];    //存储y方向速度
							SDI_Point[2] = *(float*)&msg_pack[8];
							SDI_Point[3] = *(float*)&msg_pack[12];
							SDI_Time = get_TIME_now();
//							vector3_float position;
//							float yaw = Quaternion_getYaw( get_history_attitude( 0.1f ) );
//							float sin_Yaw , cos_Yaw;
//							arm_sin_cos_f32( rad2degree(yaw) , &sin_Yaw , &cos_Yaw );
//							position.x = map_BodyHeading2ENU_x( *(float*)&msg_pack[0] , *(float*)&msg_pack[4] , sin_Yaw , cos_Yaw );
//							position.y = map_BodyHeading2ENU_y( *(float*)&msg_pack[4] , *(float*)&msg_pack[4] , sin_Yaw , cos_Yaw );
//							PositionSensorUpdatePosition( 3 , position , true , -1 );
						}
					}
					reset_SDI_RC;
				}			
					
				break;
		}
	}
}





void Uart3_Send( const uint8_t* data , uint16_t length )
{

	IntDisable( INT_UART3 );
	
	//获取剩余的缓冲区空间
	int16_t buffer_space = RingBuf_uint8_t_get_Freesize( &Tx_RingBuf );
	//获取DMA中待发送的字节数
	int16_t DMA_Remain = uDMAChannelSizeGet( UDMA_CH17_UART3TX );
	
	//计算要发送的字节数
	int16_t max_send_count = buffer_space - DMA_Remain;
	if( max_send_count < 0 )
		max_send_count = 0;
	uint16_t send_count = ( length < max_send_count ) ? length : max_send_count;
	
	//将待发送字节压入缓冲区
	RingBuf_uint8_t_push_length( &Tx_RingBuf , data , send_count );
//	for( uint8_t i = 0 ; i < send_count ; ++i )
//		RingBuf_uint8_t_push( &Tx_RingBuf , data[i] );
	
	//获取DMA发送是否完成
	if( uDMAChannelIsEnabled( UDMA_CH17_UART3TX ) == false )
	{
		//DMA已完成
		//可以继续发送
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH17_UART3TX , \
				UDMA_MODE_BASIC , p , (void*)&UART3->DR , length );
			uDMAChannelEnable( UDMA_CH17_UART3TX );
		}
	}
	IntEnable( INT_UART3 );
}
static void UART3_Handler()
{
	UARTIntClear( UART3_BASE , UART_INT_OE );
	UARTRxErrorClear( UART3_BASE );
	while( ( UART3->FR & (1<<4) ) == false	)
	{
		//接收
		uint8_t rdata = UART3->DR;
		RingBuf_uint8_t_push( &Rx_RingBuf , rdata );
	}
	
	if( uDMAChannelIsEnabled( UDMA_CH17_UART3TX ) == false )
	{
		uint16_t length;
		uint8_t* p = RingBuf_uint8_t_pop_DMABuf( &Tx_RingBuf , &length );
		if( length )
		{
			uDMAChannelTransferSet( UDMA_PRI_SELECT | UDMA_CH17_UART3TX , \
				UDMA_MODE_BASIC , p , (void*)&UART3->DR , length );
			uDMAChannelEnable( UDMA_CH17_UART3TX );
		}
	}
}
uint16_t read_Uart3( uint8_t* data , uint16_t length )
{
	IntDisable( INT_UART3 );
	uint8_t read_bytes = RingBuf_uint8_t_pop_length( &Rx_RingBuf , data , length );
	IntEnable( INT_UART3 );
	return read_bytes;
}
uint16_t Uart3_DataAvailable()
{
	IntDisable( INT_UART3 );
	uint16_t bytes2read = RingBuf_uint8_t_get_Bytes2read( &Rx_RingBuf );
	IntEnable( INT_UART3 );
	return bytes2read;
}