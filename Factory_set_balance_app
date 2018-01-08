/*
 * main.c
 */
#include "DSP28x_Project.h"

/***************************************************
系统时钟原本为为90MHz，现在example.h中改为80M

用户函数声明都在project.h中

中文编码（Encoding） 采用 UTF-8格式，如出现乱码请修改edit-set encoding为该编码格式

TI V4示例程序——基础平衡实验与APP控制
***************************************************/


/*============================User DATA=======================*/


Uint8 Flag_Target;//启动停止标志位
Uint8 Flag_Stop=1;

Uint8 Flag_blink=0;
Uint16 led_cnt=0;

Uint8 data_mode[8];
Uint8 data_stop[3]={0x59,0x59,0x59};//停止数据样本
Uint8 data_start[3]={0x58,0x58,0x58};//启动数据样本
Uint8 Flag_qian=0,Flag_hou=0,Flag_left=0,Flag_right=0,Flag_sudu=2;//蓝牙遥控相关变量

float Angle_Balance,Gyro_Balance,Gyro_Turn;  //平衡倾角、平衡陀螺仪
int Balance_Pwm=0,Velocity_Pwm=0,Turn_Pwm=0;
int Moto1,Moto2;
int Encoder_Left,Encoder_Right;//LEFT RIGHT


Uint16 battery_adc=1200;//电压测量
int Volt=0;
/****************************User FUNC***************************************/
int myabs(int a)
{
	  int temp;
		if(a<0)  temp=-a;
	  else temp=a;
	  return temp;
}

int balance(float Angle,float Gyro)
{
   float Bias,kp=70,kd=0.4;
	 int balance;

	 Bias=Angle-ZHONGZHI;       //===求出平衡的角度中值 和机械相关
	 balance=(int)(kp*Bias+Gyro*kd);   //===求出平衡控制的电机PWM PD控制
	 return balance;
}
Uint8 Turn_Off(float angle);

int velocity(int encoder_left,int encoder_right)
{
    static float Velocity=0,Encoder_Least=0,Encoder=0,Movement=0;
	static float Encoder_Integral=0,Target_Velocity=0;
	float kp=46,ki=0.23;//46 0.23
	//==============================================//
	if(Flag_sudu==1)		Target_Velocity=28;
	else 				Target_Velocity=56;
	if(1==Flag_qian)   	Movement=-Target_Velocity/Flag_sudu;
	else if(1==Flag_hou) Movement=Target_Velocity/Flag_sudu;
	else Movement=0;

	//==============================================//
	Encoder_Least =(encoder_left+encoder_right)-0;                    //获取最新速度偏差=测量速度（左右编码器之和）-目标速度（此处为0）
	Encoder *= 0.8;		                                              //一阶低通滤波器
	Encoder += Encoder_Least*0.2;	                                  //一阶低通滤波器
	Encoder_Integral +=Encoder;                                       //积分出位移，积分时间：10ms
	Encoder_Integral=Encoder_Integral-Movement;                       //接收遥控器数据，控制前进后退
	if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //积分限幅
	if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //
	Velocity=(int)(Encoder*kp+Encoder_Integral*ki);                          //速度控制
	if(Turn_Off(Angle_Balance)==1)   Encoder_Integral=0;    //电机关闭后清除积分
	return Velocity;
}

int turn(int encoder_left,int encoder_right,float gyro)
{
	static float Turn_Target=0,Turn,Encoder_Temp=0,Turn_Convert=0.9,Turn_Count=0;
	float Turn_Amplitude=44/Flag_sudu,Kp=42,Kd=0;
	//===============遥控左右旋转部分========================//
	if(1==Flag_left||1==Flag_right)				//根据旋转前的速度调整速度的起始速度
	{
		if(++Turn_Count==1)
		Encoder_Temp=myabs(encoder_left+encoder_right);
		Turn_Convert=50/Encoder_Temp;
		if(Turn_Convert<0.6) Turn_Convert=0.6;
		if(Turn_Convert>3)	Turn_Convert=3;
	}
	else
	{
		Turn_Convert=0.9;
		Turn_Count=0;
		Encoder_Temp=0;
	}
	if(1==Flag_left)			Turn_Target-=Turn_Convert;
	else if(1==Flag_right) 	Turn_Target+=Turn_Convert;
	else Turn_Target=0;

	if(Turn_Target>Turn_Amplitude)	Turn_Target=Turn_Amplitude;//转向速度限幅
	if(Turn_Target<-Turn_Amplitude)	Turn_Target=-Turn_Amplitude;
	if(Flag_qian==1||Flag_hou==1)  Kd=0.5;
	else Kd=0;		//转向的时候取消陀螺仪的纠正
	//==================转向PD控制器==========================//
	Turn=-Turn_Target*Kp-gyro*Kd;
	return Turn;
}


void Xianfu_Pwm(void)
{
	int Amplitude=2000;    //===PWM满幅是7200 限制在6900,满幅是2000时取1900
//	if(Flag_Qian==1)  Moto1+=DIFFERENCE;  //DIFFERENCE是衡量平衡小车电机和机械安装差异的一个变量，直接作用与输出，保持良好一致性
//	if(Flag_Hou==1)   Moto2-=DIFFERENCE;
    if(Moto1<-Amplitude) Moto1=-Amplitude;
    if(Moto1>Amplitude)  Moto1=Amplitude;
	if(Moto2<-Amplitude) Moto2=-Amplitude;
	if(Moto2>Amplitude)  Moto2=Amplitude;
}


void Set_Pwm(int moto1,int moto2)
{
	if(moto1>0)  PWM1A=2000,PWM1B=2000-myabs(moto1);
	else 	     PWM1A=2000-myabs(moto1),PWM1B=2000;

	if(moto2>0)  PWM2A=2000-myabs(moto2),PWM2B=2000;
	else 	     PWM2A=2000,PWM2B=2000-myabs(moto2);
}


void Key(void)
{
	Uint8 val;
	val=click();
	if(val==1)  Flag_Stop=!Flag_Stop;
}

Uint8 Turn_Off(float angle)
{
	    Uint8 temp;
			if(angle<-50||angle>50||1==Flag_Stop)//倾角大于40度关闭电机
			{
      temp=1;
			PWM1A=0;
			PWM1B=0;
			PWM2A=0;
			PWM2B=0;
      }
			else
      temp=0;
      return temp;
}
/**************************************************************************/
//----------------------------I2C-----------------------------------------------

//---------------------------中断服务声明------------------------------------------

interrupt void cpu_timer0_isr(void);
interrupt void i2c_int1a_isr(void);
//interrupt void scia_int_isr(void);
interrupt void scia_rx_isr(void);

Uint16 cnt=0;

int main(void)
 {
 	InitSysCtrl();
	DINT;//关全局
	InitPieCtrl();//关PIE全局和PIE子中断和标志
	IER = 0x0000;//关CPU中断
	IFR = 0x0000;
	InitPieVectTable();//开PIE全局（允许the PIE Vector Table？）
	//以上为初始化结束
/*****************************************************************************/
	// Section secureRamFuncs contains user defined code that runs from CSM secured RAM
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
	//--- Initialize the Flash and OTP
	InitFlash();
/*****************************************************************************/
	InitLEDGPIO();
	Init_KEY_GPIO();

	/***********************关闭蜂鸣器******************************************************/
	GpioDataRegs.GPBSET.bit.GPIO56=1;

	/*********定时器配置***********/
	EALLOW;
	PieVectTable.TINT0=&cpu_timer0_isr;
	EDIS;

	InitCpuTimers();
	ConfigCpuTimer(&CpuTimer0,80,5000);//配置完要启动timer，设置为5ms做控制周期
	/*********moto配置***********/
	InitPWM1(2000);	//PWM1A & PWM1B
	InitPWM2(2000);	//PWM2A & PWM2B
	/*********qep配置***********/
	InitEQep1Gpio();
	InitEQep2Gpio();
	POSSPEED_Init();
	/*********SCI配置***********/
	//Init_SCIA();
	EALLOW;
	PieVectTable.SCIRXINTA = &scia_rx_isr;
	EDIS;
	Init_SCIA();
	/*********ADC配置**********/
	Init_Adc_battery();
	/*********LCD配置***********/
	initial_lcdgpio();
	initial_lcd();
	DELAY_US(20000);
	clear_screen();
	DELAY_US(20000);
	/********IIC配置***********/
	Init_I2CA();
	MPU6050_Init();
	DELAY_US(10000);
	/********************/
	IER |=M_INT1;//开CPU级中断
//	IER |=M_INT8;
	IER |=M_INT9;
	PieCtrlRegs.PIEIER1.bit.INTx7  =1;//开Timer0的PIE中断
//	PieCtrlRegs.PIEIER8.bit.INTx1 = 1;//
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;//RXA
	EINT;//开全局
	ERTM;

	for(;;)
	{

		LEDTurnOver();

		Volitage_Measure();
/***********************蜂鸣器对应的io为56*************/
//		if(Volt<1050)
//		{
//			GpioDataRegs.GPBCLEAR.bit.GPIO56=1;
//		}
//		else
//		{
//			GpioDataRegs.GPBSET.bit.GPIO56=1;
//		}
/***********************蜂鸣器对应的io为56*************/

		display_string_8x16(1,1,"Left:");
		if(Encoder_Left<0)
					{
					show_number(1,58,(int)(-Encoder_Left),3);
					display_string_8x16(1,50,"-");
					}
				else {
					 show_number(1,58,(int)Encoder_Left,3);
					 display_string_8x16(1,50,"+");
				}


		display_string_8x16(3,1,"Right:");
		if(Encoder_Right<0)
							{
							show_number(3,58,(int)(-Encoder_Right),3);
							display_string_8x16(3,50,"-");
							}
						else {
							 show_number(3,58,(int)Encoder_Right,3);
							 display_string_8x16(3 ,50,"+");
						}

		display_string_8x16(5,1,"Volt:");
		display_string_8x16(5,58,".");
		display_string_8x16(5,90,"V");
		show_number(5,45,Volt/100,2);
		show_number(5,68,Volt%100,2);

		display_string_8x16(7,1,"Angle:");
		if(Angle_Balance<0)
			{
			show_number(7,58,(int)(-(Angle_Balance)),3);
			display_string_8x16(7,50,"-");
			}
		else {
			 show_number(7,58,(int)Angle_Balance,3);
			 display_string_8x16(7,50,"+");
		}

		DELAY_US(500000);
		//asm("NOP");
	}
}



//*-------------------------------中断处理函数--------------------------------*/
interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;
	   Encoder_Left=Read_Encoder(1);
	   Encoder_Right=Read_Encoder(2);
	   MPU6050_Read();
	   Get_Angle();
	   Key();
	   Balance_Pwm =balance(Angle_Balance,Gyro_Balance);
	   Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);
	   Turn_Pwm=turn(Encoder_Left,Encoder_Right,Gyro_Turn);
	   Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;//2000/7200
	   Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;
	   Xianfu_Pwm();
	   if(Turn_Off(Angle_Balance)==0)
		Set_Pwm(Moto1,Moto2);
//		fastdecay_Set_Pwm(Moto1,Moto2);

	   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//	   return 0;
}


interrupt void scia_int_isr(void)
{
//	Uint16 i;

	CpuTimer0.InterruptCount++;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8 | PIEACK_GROUP9;  //娴ｈ儻鍏樻径鏍啎娑擃厽鏌囩粭顒�鍙撶紒鍕倱閺冩湹濞囬懗鐣岊儑娑旀繄绮嶆稉顓熸焽

}

interrupt void scia_rx_isr(void)
{
	Uint16 ReceivedChar;
	ReceivedChar = SciaRegs.SCIRXBUF.all;

	if(data_mode[0]==0x58 && data_mode[1]==0x58 && data_mode[2]==0x58)
	{
		Flag_Stop=0;			//三击高速，启动电机
		data_mode[0]=0;data_mode[1]=0;data_mode[2]=0;
	}
	if(data_mode[0]==0x59 && data_mode[1]==0x59 && data_mode[2]==0x59)
	{
		Flag_Stop=1;			//三击低速，关闭电机
		data_mode[0]=0;data_mode[1]=0;data_mode[2]=0;
	}
	if(ReceivedChar==0x59)  Flag_sudu=2;//低速档
	if(ReceivedChar==0x58)	Flag_sudu=1;//高速

	if(ReceivedChar>10)
	{
		if(ReceivedChar==0x5A)		Flag_qian=0,Flag_hou=0,Flag_left=0,Flag_right=0; //break
		else if(ReceivedChar==0x41)	Flag_qian=1,Flag_hou=0,Flag_left=0,Flag_right=0; //forward
		else if(ReceivedChar==0x45)	Flag_qian=0,Flag_hou=1,Flag_left=0,Flag_right=0; //backward
		else if(ReceivedChar==0x42||ReceivedChar==0x43||ReceivedChar==0x44)
									Flag_qian=0,Flag_hou=0,Flag_left=0,Flag_right=1; //right
		else if(ReceivedChar==0x46||ReceivedChar==0x47||ReceivedChar==0x48)
									Flag_qian=0,Flag_hou=0,Flag_left=1,Flag_right=0;	//left
		else 						Flag_qian=0,Flag_hou=0,Flag_left=0,Flag_right=0; //break
	}
	data_mode[2]=data_mode[1];
	data_mode[1]=data_mode[0];

	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}
