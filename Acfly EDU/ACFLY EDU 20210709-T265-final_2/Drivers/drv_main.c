#include "Basic.h"
#include "drv_main.h"

#include "drv_uDMA.h"
#include "drv_LED.h"
#include "drv_Sensors.h"
#include "drv_USB.h"
#include "drv_OLED.h"
#include "drv_Receiver.h"
#include "drv_PWMOut.h"
#include "drv_Ultrasonic.h"
#include "drv_OpticalFlow.h"
#include "drv_GPS.h"
#include "drv_Uart2.h"
#include "drv_SDI.h"
#include "drv_SDI2.h"
#include "drv_adc.h"


void init_Drivers()
{
	init_drv_uDMA();
	init_drv_LED();
	init_drv_OLED();
	init_drv_PWMOut();

	
// 不注释就是校准电调
// 注释就是正常飞行
//	PWM_PullUpAll();
//	delay( 4.0f );
//	PWM_PullDownAll();
	
	init_drv_Sensors();
	init_drv_USB();	
	init_drv_Receiver();	
	init_drv_Ultrasonic();
	init_drv_OpticalFlow();
	init_drv_GPS();
	init_drv_Uart2();
	init_drv_SDI();
	init_drv_SDI2();
	init_drv_ADC();
}