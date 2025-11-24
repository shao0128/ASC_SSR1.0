#include "stm32f10x.h"                  // Device header
#include "Motor.h"

//电机目标速度
extern int16_t Speed_Target;  //30,60,90

//电机实际速度
extern int16_t Speed1;
extern int16_t Speed2;

//红外传感器判断小车应走直线
void PID_Straight(void)
{
	//给定电机1一个速度。通过PID算法实现电机1稳定在此速度
	static int16_t Location1_Target;
	
	//pid控制定义参数变量kp，ki，kd（增量式pid控制速度）
  static float kp=0.3,ki=0.6,kd=0.02;

  static float error0=0,error1=0,error2=0,errorInt=0;

  static float Out1;
	
	Location1_Target=Speed_Target;
	
	error2 = error1;
  error1 = error0;
  error0 = Location1_Target-Speed1;
            
  // 增量式PID
  Out1+=kp*(error0 - error1)+ki*error0+ kd*(error0-2*error1+error2);
            
  // 输出限幅
  if (Out1 > 100) Out1 = 100;
  if (Out1 < 0) Out1 = 0;
				
	// 执行控制
  Moter_SetPower1(-Out1);
	
  
	//给定电机1一个速度，通过PID算法实现电机2与电机1速度相同
	
	//速度式PID控制电机2
	static float kp_loc=0.3,ki_loc=0.6,kd_loc=0.02;
  
  static float error0_loc=0,error1_loc=0,error2_loc=0,errorInt_loc=0;
 
  static float Out2;

  static int16_t Location2_Actually;
	Location2_Actually=Location1_Target;
	

  //利用PID实现电机1的跟随转动
	error2_loc= error1_loc;
  error1_loc= error0_loc;
  error0_loc= Location2_Actually-Speed2;
            
  // 增量式PID
  Out2+=kp_loc*(error0_loc- error1_loc)+ki_loc*error0_loc+kd_loc*(error0_loc-2*error1_loc+error2_loc);
            
  // 输出限幅
  if (Out2>100) Out2=100;
  if (Out2<0) Out2=0;
            
  // 执行控制
  Moter_SetPower2(-Out2);
}


/**********************************************************************************************************************************/

//红外传感器判断小车应向右轻微拐弯
void PID_RightSlight(void)
{
	//轻微拐弯，左轮（电机1）比右轮（电机2）的速度快30
	
	
	
	//给定电机1一个速度。通过PID算法实现电机1稳定在此速度
	static int16_t Location1_Target;
	
	//pid控制定义参数变量kp，ki，kd（增量式pid控制速度）
  static float kp=0.3,ki=0.6,kd=0.02;

  static float error0=0,error1=0,error2=0,errorInt=0;

  static float Out1;
	
	Location1_Target=Speed_Target;
	
	error2 = error1;
  error1 = error0;
  error0 = Location1_Target-Speed1;
            
  // 增量式PID
  Out1+=kp*(error0 - error1)+ki*error0+ kd*(error0-2*error1+error2);
            
  // 输出限幅
  if (Out1 > 100) Out1 = 100;
  if (Out1 < 0) Out1 = 0;
				
	// 执行控制
  Moter_SetPower1(-Out1);
	
	//速度式PID控制电机2
	static float kp_loc=0.3,ki_loc=0.6,kd_loc=0.02;
  
  static float error0_loc=0,error1_loc=0,error2_loc=0,errorInt_loc=0;
 
  static float Out2;

  //电机2比电机1速度小30
  static int16_t Location2_Actually;
	Location2_Actually=Location1_Target-30;
	

  //利用PID实现电机1的跟随转动
	error2_loc= error1_loc;
  error1_loc= error0_loc;
  error0_loc= Location2_Actually-Speed2;
            
  // 增量式PID
  Out2+=kp_loc*(error0_loc- error1_loc)+ki_loc*error0_loc+kd_loc*(error0_loc-2*error1_loc+error2_loc);
            
  // 输出限幅
  if (Out2>100) Out2=100;
  if (Out2<0) Out2=0;
            
  // 执行控制
  Moter_SetPower2(-Out2);
}

/***********************************************************************************************************************************/

//红外传感器判断小车应向左轻微拐弯
void PID_LeftSlight(void)
{
	//轻微拐弯，左轮（电机1）比右轮（电机2）的速度慢30
	
	
	//速度式PID控制电机2
	static float kp_loc=0.3,ki_loc=0.6,kd_loc=0.02;
  
  static float error0_loc=0,error1_loc=0,error2_loc=0,errorInt_loc=0;
 
  static float Out2;

  //电机2比电机1速度小30
  static int16_t Location2_Actually;
	Location2_Actually=Speed_Target;
	

  //利用PID实现电机1的跟随转动
	error2_loc= error1_loc;
  error1_loc= error0_loc;
  error0_loc= Location2_Actually-Speed2;
            
  // 增量式PID
  Out2+=kp_loc*(error0_loc- error1_loc)+ki_loc*error0_loc+kd_loc*(error0_loc-2*error1_loc+error2_loc);
            
  // 输出限幅
  if (Out2>100) Out2=100;
  if (Out2<0) Out2=0;
            
  // 执行控制
  Moter_SetPower1(-Out2);
	
	//给定电机1一个速度。通过PID算法实现电机1稳定在此速度
	static int16_t Location1_Target;
	
	//pid控制定义参数变量kp，ki，kd（增量式pid控制速度）
  static float kp=0.3,ki=0.6,kd=0.02;

  static float error0=0,error1=0,error2=0,errorInt=0;

  static float Out1;
	
	Location1_Target=Location2_Actually-30;
	
	error2 = error1;
  error1 = error0;
  error0 = Location1_Target-Speed1;
            
  // 增量式PID
  Out1+=kp*(error0 - error1)+ki*error0+ kd*(error0-2*error1+error2);
            
  // 输出限幅
  if (Out1 > 100) Out1 = 100;
  if (Out1 < 0) Out1 = 0;
				
	// 执行控制
  Moter_SetPower2(-Out1);
}

/*********************************************************************************************************************************/

//红外传感器判断小车应向右直角拐弯
void PID_RightStraight(void)
{
	//直角拐弯，左轮（电机1）比右轮（电机2）的速度快60
	
	
	
	//给定电机1一个速度。通过PID算法实现电机1稳定在此速度
	static int16_t Location1_Target;
	
	//pid控制定义参数变量kp，ki，kd（增量式pid控制速度）
  static float kp=0.3,ki=0.6,kd=0.02;

  static float error0=0,error1=0,error2=0,errorInt=0;

  static float Out1;
	
	Location1_Target=Speed_Target;
	
	error2 = error1;
  error1 = error0;
  error0 = Location1_Target-Speed1;
            
  // 增量式PID
  Out1+=kp*(error0 - error1)+ki*error0+ kd*(error0-2*error1+error2);
            
  // 输出限幅
  if (Out1 > 100) Out1 = 100;
  if (Out1 < 0) Out1 = 0;
				
	// 执行控制
  Moter_SetPower1(-Out1);
	
	//速度式PID控制电机2
	static float kp_loc=0.3,ki_loc=0.6,kd_loc=0.02;
  
  static float error0_loc=0,error1_loc=0,error2_loc=0,errorInt_loc=0;
 
  static float Out2;

  //电机2比电机1速度小30
  static int16_t Location2_Actually;
	Location2_Actually=Location1_Target-60;
	

  //利用PID实现电机1的跟随转动
	error2_loc= error1_loc;
  error1_loc= error0_loc;
  error0_loc= Location2_Actually-Speed2;
            
  // 增量式PID
  Out2+=kp_loc*(error0_loc- error1_loc)+ki_loc*error0_loc+kd_loc*(error0_loc-2*error1_loc+error2_loc);
            
  // 输出限幅
  if (Out2>100) Out2=100;
  if (Out2<0) Out2=0;
            
  // 执行控制
  Moter_SetPower2(-Out2);
}

/*******************************************************************************************************************************/

//红外传感器判断小车应向左直角拐弯
void PID_LeftStraight(void)
{
	//直角拐弯，左轮（电机1）比右轮（电机2）的速度慢60
	
	
	//速度式PID控制电机2
	static float kp_loc=0.3,ki_loc=0.6,kd_loc=0.02;
  
  static float error0_loc=0,error1_loc=0,error2_loc=0,errorInt_loc=0;
 
  static float Out2;

  //电机2比电机1速度小30
  static int16_t Location2_Actually;
	Location2_Actually=Speed_Target;
	

  //利用PID实现电机1的跟随转动
	error2_loc= error1_loc;
  error1_loc= error0_loc;
  error0_loc= Location2_Actually-Speed2;
            
  // 增量式PID
  Out2+=kp_loc*(error0_loc- error1_loc)+ki_loc*error0_loc+kd_loc*(error0_loc-2*error1_loc+error2_loc);
            
  // 输出限幅
  if (Out2>100) Out2=100;
  if (Out2<0) Out2=0;
            
  // 执行控制
  Moter_SetPower1(-Out2);
	
	//给定电机1一个速度。通过PID算法实现电机1稳定在此速度
	static int16_t Location1_Target;
	
	//pid控制定义参数变量kp，ki，kd（增量式pid控制速度）
  static float kp=0.3,ki=0.6,kd=0.02;

  static float error0=0,error1=0,error2=0,errorInt=0;

  static float Out1;
	
	Location1_Target=Location2_Actually-60;
	
	error2 = error1;
  error1 = error0;
  error0 = Location1_Target-Speed1;
            
  // 增量式PID
  Out1+=kp*(error0 - error1)+ki*error0+ kd*(error0-2*error1+error2);
            
  // 输出限幅
  if (Out1 > 100) Out1 = 100;
  if (Out1 < 0) Out1 = 0;
				
	// 执行控制
  Moter_SetPower2(-Out1);
}


void PID_Still(void)
{
	Moter_SetPower1(0);
	Moter_SetPower2(0);
}