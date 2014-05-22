#include <hidef.h>      /* common defines and macros */
#include <mc9s12dg128.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12dg128b"
#include <math.h>
                                       
#define MID 4500;                      //舵机中间值
#define MAXTURN 450;
#define MIDL 4050;         ////左右极限值
#define MIDR 4950;

int speed=100;
int ruwantimer=0;
int stop=0;

int zhidao_speed=210;           // 弯道 直到速度设定
int wandao_speed=190;
int chongchu_speed=170;
int ruwan_speed=0;

//int tdeg_min= 35;
//int tdeg_max=57;
float tdeg=13;                ////比例系数 
//float tdeg_k=10;

int station=0;
int stat=0;

uint ad_value[4];
float sen1=0;
float sen2=0;
float sen3=0;
int senor=MID;
float zhuanjiao=0;
int lastturn=0;

char boma=0;

//unsigned int chesu=0;
int Spd_Count=0;
int Spd_Now=0;
int Spd_Avg=0;

void SET_PLL(void) 
{
    CLKSEL=0x00;
    PLLCTL=0xe1;
    SYNR=2;                   //设置晶振参数
    REFDV=1;
    PLLCTL=0x60;
    asm NOP;
    asm NOP;
    asm NOP;
    while((CRGFLG&0x08)==0);
    CLKSEL=0x80;
    
    
} 
/**************************实时中断初始化************************************************************/ 
void RTI_Init(void)
{                 
   //  setup of the RTI interrupt frequency 
    // adjusted to get 1 millisecond (1.024 ms) with 16 MHz oscillator 
         RTICTL = 0b11001111;//RTICTL=0b10001111;//RTICTL = 0b10001111;  //5*2^16:48.8hz //0x1f // set RTI prescaler ::晶振16384分频;(低四位+1)*(2^(高三位+9))
         CRGINT = 0x80;      // enable RTI interrupts;   //低四位:1-7,不可为零,否则分频器不工作
         PACTL=0b01000000;
} 
   
                                  /*锁相环程序*/

/******************************ECT_Init()************************************************************/                              
                               /*电机初始化程序*/
void ECT_Init(void)        //ECT初始化，使用输入捕捉功能
{  
     TSCR1_TFFCA=1;    //automatically clear flag                
     TSCR2 = 0x87;  //Timer Overflow interrupt open,TCNT prescaler:24MHZ/128=187.5kHz计数频率 
     TIOS_IOS0=0;      //The corresponding channel acts as an input capture 
                       // ON CH0 PT0;
     TCTL4_EDG0A=1;    //  capture on rising edge only on channel 0
     TCTL4_EDG0B=0;
     ICSYS=0x02;
     TIE_C0I=1;         // Enable Ch1 Interrupts
     PACTL_PAMOD=0;
     PACTL_PEDGE=1;
     PACTL_PAEN=1;
     TSCR1_TEN=1;      //ENABLE TIMER
}

void PA_Init(void)
{
    PACTL_PAMOD=0;
    PACTL_PEDGE=1;
    PACTL_PAEN=1;
}

void ATD_Init(void)
{   
    ATD0CTL2=0xc0;//不开中断2;  ////上电，标志位快速清零	...	 ATD队列转换完成中断使能
    ATD0CTL3=0x20;//sequence length:4   0;	 //转换队列长度4，非FIFO存储模式，
    ATD0CTL4=0x85;  //8位精度，AD时钟为bus_clk/(2*(5+1))
    ATD0CTL5=0x90; //右对齐，无符号数，单次转换队?
}

void Servo_Init(void)      //舵机 16位PWM控制
{
     PWMCTL_CON01= 1;//将0，1级联成16位
     PWME_PWME1=0;     //disable PWM1  servo 
     PWMPRCLK|= 0x03;  //CLKA:16分频
     PWMCLK=0x00 ;    //这就是默认值//通道1用clock A时钟源
                       //24MHZ/8=3M                         
     PWMPOL_PPOL1 = 1; //先高电平，占空比计数完之后拉低
     
     PWMCAE=0X00;			//对齐方式默认 左对齐
     
     PWMCTL_CON01 = 1; //通道1:16BIT 连接
     PWMPER01  = 60000;//60000:20ms;;;20000//舵机的频率是50Hz	   1/3M * x=1/50Hz
                       //1ms 脉冲 到2ms 脉冲
                       //1.5ms脉冲对应0度
     PWMDTY01 =4500;   //4500:1.5ms;	 //控制3500左极限 5500右极限  4500正中央
     PWMCNT01  = 0;		 //写计数寄存器,会使PWMDTYx,PWMPERx进入锁存器,同时使能通道时会从此值
     PWME_PWME1   = 1; //PWM通道1输出
}                                           /*舵机初始化程序*/

void Forward_Init(void) // 假定前进方向，初始化函数
{
     PWME_PWME3= 0;
     PWME_PWME5= 0;  
     PWMPRCLK|= 0x20;  //clock b总线频率的4分频			
                       //通道3用clock B 时钟源
                       // 24MHZ/4=6M                           
     PWMPOL_PPOL3 = 0; //low electrical level first,,high  electrical level first ,but high level makes mc33886 disabled
     PWMPOL_PPOL5 = 0;			             //对齐方式默认 左对齐PWMCAE
     PWMCTL_CON23 = 1; //16BIT 连接
     PWMCTL_CON45 = 1;
     PWMPER23  = 600;  //电机频率10k Hz	   	
     PWMPER45  = 600;	
     PWMDTY23  = 0;	 // 占空比精度 1/750 
     PWMDTY45 = 0; 
     PWMCNT23  = 0;		 //启动PWM
     PWMCNT45  = 0;		 //启动PWM
     PWME_PWME3 =1;    //PWM通道3输出  
     PWME_PWME5 =1;    //PWM通道3输出
}       /*电机初始化程序*/

void Stop_Init(void) 
{
     DDRH_DDRH0 = 0;  //停车用PH0口中断
     PPSH_PPSH0 = 0;  //下降沿触发中断
     PIEH_PIEH0 = 1;  //开中断
} 

int Spd_P(int DesSpd,int NowSpd)
{
     int a,b;
     a=(int)(DesSpd-5*NowSpd);
     if(a<2&&a>-2) b=DesSpd;
     else 
     {
         b=(int)2*a+DesSpd;
         if(b>600) b=600;
         if(b<0) b=0;
     }
     return b;
}
         

void main(void)
{ 
   dword i,j;

      	   
	SET_PLL();
	Servo_Init();
	Forward_Init();
	ATD_Init();
	RTI_Init();
	Stop_Init();
  //ECT_Init();
  PA_Init();                  //测速
	


/***************拨码开关***********************/
//*擦轮 每次选择完拨码要重启电源！*//	
	  DDRK=0XF0;
	  boma=PORTK;//读入拨码开关；
	  
    boma=~boma;
    boma&=0x0f;
  
    switch(boma)
    {
     case 0x00:{              //基速
       zhidao_speed =180;
       wandao_speed =150;
       chongchu_speed=140;// 
       ruwan_speed=100;
       break; 
     }
     case 0x01: {              
       zhidao_speed =210;
       wandao_speed =170;
       chongchu_speed=150;// 
       ruwan_speed=120;
       break; 
     }
     case 0x02: {
       zhidao_speed =220;
       wandao_speed =180;
       chongchu_speed=168;// 
       ruwan_speed=100;
       break;  
     }
     case 0x03: {
       zhidao_speed =230;
       wandao_speed =190;
       chongchu_speed=175;// 
       ruwan_speed=80;
       break;
     }
     
     case 0x04: {
       zhidao_speed =240;
       wandao_speed =190;
       chongchu_speed=170;// 
       ruwan_speed=40;
       break; 
     }
     case 0x05: {
       zhidao_speed =240;		   
       wandao_speed =195;
       chongchu_speed=175;// 
       ruwan_speed=40;
       break; 
     }
     case 0x06: {
       zhidao_speed =240;
       wandao_speed =200;
       chongchu_speed=175;//
       ruwan_speed=40;
       break; 
     }
     case 0x07: {
       zhidao_speed =240;
       wandao_speed =200;
       chongchu_speed=180;// 
       ruwan_speed=50;
       break; 
     }
     case 0x08: {
       zhidao_speed =260;
       wandao_speed =200;
       chongchu_speed=170;// //速度设定拨码
       ruwan_speed=20;
       break; 
     }
     case 0x09: {
       zhidao_speed =270;
       wandao_speed =200;
       chongchu_speed=170;// //速度设定拨码
       ruwan_speed=20;
       break; 
     }
	 case 0x0a: {
       zhidao_speed =280;
       wandao_speed =205;
       chongchu_speed=175;// //速度设定拨码
       ruwan_speed=0;
       break; 
     }
     case 0x0b: {
       zhidao_speed =290;
       wandao_speed =210;
       chongchu_speed=180;// //速度设定拨码
       ruwan_speed=0;
       break; 
     }
     case 0x0c: {				      
       zhidao_speed =300;
       wandao_speed =220;
       chongchu_speed=185;// //速度设定拨码
       ruwan_speed=0;
       break; 
     }
     case 0x0d: {
       zhidao_speed =310;		 //*****比赛速度******
       wandao_speed =220;
       chongchu_speed=190;// //速度设定拨码
       ruwan_speed=0;
       break; 
     }
     case 0x0e: {
       zhidao_speed =320;	  
       wandao_speed =225;
       chongchu_speed=195;// //速度设定拨码
       ruwan_speed=0;
       break; 
     }
     case 0x0f: {
       zhidao_speed =330;  
       wandao_speed =230;
       chongchu_speed=200;// //冲 高速
       ruwan_speed=0;
       break; 
     }
     default:  DDRK=0X06;break; //比赛速度 
  }   
  
      for(i=0;i<2500;i++)
         for(j=0;j<1000;j++)
            asm NOP;
            
  station=0;
  stat=1;
  PWMDTY23=500;
  //tdeg=tdeg_max-tdeg_min;
  //tdeg_k=pow(tdeg_max-tdeg_min,2)/500.0;
  //tdeg_k=sqrt(tdeg_max-tdeg_min)/(9*sen3);          
  //tdeg_k=tdeg_max-tdeg_min;
  
EnableInterrupts;
 
	for(;;)
	{       
        ATD0CTL5=0x90;//启动一个新的转换 ,ch0 begin
        while(!(ATD0STAT0&0x80));  //等待队列转换结束
	      ad_value[0]=ATD0DR0L;
		    ad_value[1]=ATD0DR1L;
		    ad_value[2]=ATD0DR2L;
		    ad_value[3]=ATD0DR3L;
		    
      sen1=sqrt(ad_value[0])+sqrt(ad_value[1]);
    	sen2=sqrt(ad_value[2])+sqrt(ad_value[3]);
    	sen3=sen1-sen2;
    	
    	if(sen3>0) sen3*=1.185;   
    	zhuanjiao=tdeg*sen3;
      if(zhuanjiao>460) zhuanjiao=450;
      else if(zhuanjiao<-460) zhuanjiao=-450;
    	senor=(int)zhuanjiao+MID;

    	tdeg=2.7*pow(fabs(zhuanjiao)/140,2)+27; //good
    	//tdeg=2.3*pow(fabs(zhuanjiao)/130,2)+29;    	  
    	//tdeg=4*pow(fabs(zhuanjiao)/180,2)+30;
    	if(sen3>-3.8&&sen3<3.8&&sen1>12&&sen2>12)  stat=1;    //判断为跑道中央
    	if(!station) speed=zhidao_speed;
    	else
    	{
    	  speed=wandao_speed;
    	}
      if(zhuanjiao<-120) station=-1;
    	  else if(zhuanjiao>120) station=1;    	 	          
  
    if(sen1<8.5&&sen2>9.5)             //(ad_value[0]<9&&ad_value[1]<16&&ad_value[2]>21&&ad_value[3]>21)
    {
      senor=MIDL;
      station=-1;
      speed=chongchu_speed;
    }
    else if(sen2<8.5&&sen1>9.5)       //(ad_value[2]<6&&ad_value[3]<14&&ad_value[0]>21&&ad_value[1]>21)
    {
      senor=MIDR;
      station=1;
      speed=chongchu_speed;
    }
    
    if((sen1+sen2<20)||(ad_value[0]>ad_value[1]&&ad_value[2]<ad_value[3])) 
    {
      if(station==-1)
      {
        senor=MIDL; 
      }
      else if(station==1){
        senor=MIDR;
      }
      else 
      {
          if(lastturn>10) {
            senor=MIDR;
          }
          else if(lastturn<-10){ senor=MIDL;}
          else senor=lastturn;
      }
 
      speed=chongchu_speed;
    }
        	
    PWMDTY01=senor;
    lastturn=senor;   
  } 
}

#pragma CODE_SEG NON_BANKED
/******************RTI中断*******************/   
void interrupt 7 RTI_ISR(void) 
{ 
     //int Spd_Now=0;
     CRGFLG_RTIF=1;
     Spd_Now=PACN32;
     PACN32=0;
     if(stat)
     {
        if(ruwantimer<200) ruwantimer++;
        if(ruwantimer>30) station=0; //判断为直道
        stat=0;   //清标记
     } 
     else 
     {
          if(ruwantimer>10)
          {
            speed=ruwan_speed;
            ruwantimer-=2;
          }
          else ruwantimer=0;
     }   //入弯减速
     
     if(stop>1) speed = 0;
     if(Spd_Count<2)
     {
         Spd_Count++;
         Spd_Avg+=Spd_Now;
     }
     else
     {
         PWMDTY23=Spd_P(speed,Spd_Avg/2);
         //PWMDTY23=speed;
         Spd_Count=0;
         Spd_Avg=0;
     }
     
}

/*****************计数器中断**************/
/*void interrupt 8 ECT_Counter(void)
{
     chesu=TC0;
     TFLG1_C0F = 1;
} */
	
	
/*****************停车中断****************/	
	void interrupt 25 Stop_PortH0(void) 
	{
	     dword i=0;
	     for(i=0;i<100;i++) asm nop;   //去抖动
	     if(PTIH_PTIH0 == 0) stop++;   //低电平即表示检测到的为起跑线
	     PIFH_PIFH0 = 1;               //清中断
	}
	      