/*------定义由树莓派发送的消息中，控制量的类型----------*/
#ifndef _CONTROL_MODE_
#define _CONTROL_MODE_

#define KEEP								'a'		//保持原位悬空
#define TAKE_OFF         	  'b'		//CON1为起飞高度，其余两个弃用
#define GROUND             	'c'		//CON1为降落速度，其余两个弃用
#define VELOCITY_CONTROL  	'd'		//CON1=Vx,CON2=Vy,CON3=Vz
#define POSITION_CONTROL		'e'		//CON1=Sx,CON2=Sy,CON3=Sz
#define POSTURE_CONTROL			'f'		//CON1=row,CON2=pitch,CON3=yaw（三角的角度数据）
#define CIRCLE							'g'		//CON1=Vx,CON2=Vy,CON3=yaw（偏航角旋转速度）
#define PLANE_FLIGHT        'h'   //CON1=Vx,CON2=Vy,锁住Z轴
#define LOCK								'i'		//锁定无人机，进入M01
#define UNLOCK							'j'		//解锁无人机，进入M35
#define DEBUG               'k'   //调试无人机
#define XYLOCK_VZ						'l'		//CON1 CON2无效，CON3为z速度
#define VZ_TAKEOFF					'm'		// CON1 是z的速度，CON2 是高度, CON3 是误差允许,只要 当前高度 > CON2 - CON3 就行
#define Get_Z       				'n'   //发送当前高度给树莓派
#define EMPTY								'o'		// 空状态  什么都不执行的

#endif
