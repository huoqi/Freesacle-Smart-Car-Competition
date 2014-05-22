#include <hidef.h>      /* common defines and macros */
#include <mc9s12dg128.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12dg128b"
#include <math.h>
                                       
#define MID 4500;                      //����м�ֵ
#define MAXTURN 450;
#define MIDL 4050;         ////���Ҽ���ֵ
#define MIDR 4950;

int speed=100;
int ruwantimer=0;
int stop=0;

int zhidao_speed=210;           // ��� ֱ���ٶ��趨
int wandao_speed=190;
int chongchu_speed=170;
int ruwan_speed=0;

//int tdeg_min= 35;
//int tdeg_max=57;
float tdeg=13;                ////����ϵ�� 
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
    SYNR=2;                   //���þ������
    REFDV=1;
    PLLCTL=0x60;
    asm NOP;
    asm NOP;
    asm NOP;
    while((CRGFLG&0x08)==0);
    CLKSEL=0x80;
    
    
} 
/**************************ʵʱ�жϳ�ʼ��************************************************************/ 
void RTI_Init(void)
{                 
   //  setup of the RTI interrupt frequency 
    // adjusted to get 1 millisecond (1.024 ms) with 16 MHz oscillator 
         RTICTL = 0b11001111;//RTICTL=0b10001111;//RTICTL = 0b10001111;  //5*2^16:48.8hz //0x1f // set RTI prescaler ::����16384��Ƶ;(����λ+1)*(2^(����λ+9))
         CRGINT = 0x80;      // enable RTI interrupts;   //����λ:1-7,����Ϊ��,�����Ƶ��������
         PACTL=0b01000000;
} 
   
                                  /*���໷����*/

/******************************ECT_Init()************************************************************/                              
                               /*�����ʼ������*/
void ECT_Init(void)        //ECT��ʼ����ʹ�����벶׽����
{  
     TSCR1_TFFCA=1;    //automatically clear flag                
     TSCR2 = 0x87;  //Timer Overflow interrupt open,TCNT prescaler:24MHZ/128=187.5kHz����Ƶ�� 
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
    ATD0CTL2=0xc0;//�����ж�2;  ////�ϵ磬��־λ��������	...	 ATD����ת������ж�ʹ��
    ATD0CTL3=0x20;//sequence length:4   0;	 //ת�����г���4����FIFO�洢ģʽ��
    ATD0CTL4=0x85;  //8λ���ȣ�ADʱ��Ϊbus_clk/(2*(5+1))
    ATD0CTL5=0x90; //�Ҷ��룬�޷�����������ת����?
}

void Servo_Init(void)      //��� 16λPWM����
{
     PWMCTL_CON01= 1;//��0��1������16λ
     PWME_PWME1=0;     //disable PWM1  servo 
     PWMPRCLK|= 0x03;  //CLKA:16��Ƶ
     PWMCLK=0x00 ;    //�����Ĭ��ֵ//ͨ��1��clock Aʱ��Դ
                       //24MHZ/8=3M                         
     PWMPOL_PPOL1 = 1; //�ȸߵ�ƽ��ռ�ձȼ�����֮������
     
     PWMCAE=0X00;			//���뷽ʽĬ�� �����
     
     PWMCTL_CON01 = 1; //ͨ��1:16BIT ����
     PWMPER01  = 60000;//60000:20ms;;;20000//�����Ƶ����50Hz	   1/3M * x=1/50Hz
                       //1ms ���� ��2ms ����
                       //1.5ms�����Ӧ0��
     PWMDTY01 =4500;   //4500:1.5ms;	 //����3500���� 5500�Ҽ���  4500������
     PWMCNT01  = 0;		 //д�����Ĵ���,��ʹPWMDTYx,PWMPERx����������,ͬʱʹ��ͨ��ʱ��Ӵ�ֵ��
     PWME_PWME1   = 1; //PWMͨ��1���
}                                           /*�����ʼ������*/

void Forward_Init(void) // �ٶ�ǰ�����򣬳�ʼ������
{
     PWME_PWME3= 0;
     PWME_PWME5= 0;  
     PWMPRCLK|= 0x20;  //clock b����Ƶ�ʵ�4��Ƶ			
                       //ͨ��3��clock B ʱ��Դ
                       // 24MHZ/4=6M                           
     PWMPOL_PPOL3 = 0; //low electrical level first,,high  electrical level first ,but high level makes mc33886 disabled
     PWMPOL_PPOL5 = 0;			             //���뷽ʽĬ�� �����PWMCAE
     PWMCTL_CON23 = 1; //16BIT ����
     PWMCTL_CON45 = 1;
     PWMPER23  = 600;  //���Ƶ��10k Hz	   	
     PWMPER45  = 600;	
     PWMDTY23  = 0;	 // ռ�ձȾ��� 1/750 
     PWMDTY45 = 0; 
     PWMCNT23  = 0;		 //����PWM
     PWMCNT45  = 0;		 //����PWM
     PWME_PWME3 =1;    //PWMͨ��3���  
     PWME_PWME5 =1;    //PWMͨ��3���
}       /*�����ʼ������*/

void Stop_Init(void) 
{
     DDRH_DDRH0 = 0;  //ͣ����PH0���ж�
     PPSH_PPSH0 = 0;  //�½��ش����ж�
     PIEH_PIEH0 = 1;  //���ж�
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
  PA_Init();                  //����
	


/***************���뿪��***********************/
//*���� ÿ��ѡ���겦��Ҫ������Դ��*//	
	  DDRK=0XF0;
	  boma=PORTK;//���벦�뿪�أ�
	  
    boma=~boma;
    boma&=0x0f;
  
    switch(boma)
    {
     case 0x00:{              //����
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
       chongchu_speed=170;// //�ٶ��趨����
       ruwan_speed=20;
       break; 
     }
     case 0x09: {
       zhidao_speed =270;
       wandao_speed =200;
       chongchu_speed=170;// //�ٶ��趨����
       ruwan_speed=20;
       break; 
     }
	 case 0x0a: {
       zhidao_speed =280;
       wandao_speed =205;
       chongchu_speed=175;// //�ٶ��趨����
       ruwan_speed=0;
       break; 
     }
     case 0x0b: {
       zhidao_speed =290;
       wandao_speed =210;
       chongchu_speed=180;// //�ٶ��趨����
       ruwan_speed=0;
       break; 
     }
     case 0x0c: {				      
       zhidao_speed =300;
       wandao_speed =220;
       chongchu_speed=185;// //�ٶ��趨����
       ruwan_speed=0;
       break; 
     }
     case 0x0d: {
       zhidao_speed =310;		 //*****�����ٶ�******
       wandao_speed =220;
       chongchu_speed=190;// //�ٶ��趨����
       ruwan_speed=0;
       break; 
     }
     case 0x0e: {
       zhidao_speed =320;	  
       wandao_speed =225;
       chongchu_speed=195;// //�ٶ��趨����
       ruwan_speed=0;
       break; 
     }
     case 0x0f: {
       zhidao_speed =330;  
       wandao_speed =230;
       chongchu_speed=200;// //�� ����
       ruwan_speed=0;
       break; 
     }
     default:  DDRK=0X06;break; //�����ٶ� 
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
        ATD0CTL5=0x90;//����һ���µ�ת�� ,ch0 begin
        while(!(ATD0STAT0&0x80));  //�ȴ�����ת������
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
    	if(sen3>-3.8&&sen3<3.8&&sen1>12&&sen2>12)  stat=1;    //�ж�Ϊ�ܵ�����
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
/******************RTI�ж�*******************/   
void interrupt 7 RTI_ISR(void) 
{ 
     //int Spd_Now=0;
     CRGFLG_RTIF=1;
     Spd_Now=PACN32;
     PACN32=0;
     if(stat)
     {
        if(ruwantimer<200) ruwantimer++;
        if(ruwantimer>30) station=0; //�ж�Ϊֱ��
        stat=0;   //����
     } 
     else 
     {
          if(ruwantimer>10)
          {
            speed=ruwan_speed;
            ruwantimer-=2;
          }
          else ruwantimer=0;
     }   //�������
     
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

/*****************�������ж�**************/
/*void interrupt 8 ECT_Counter(void)
{
     chesu=TC0;
     TFLG1_C0F = 1;
} */
	
	
/*****************ͣ���ж�****************/	
	void interrupt 25 Stop_PortH0(void) 
	{
	     dword i=0;
	     for(i=0;i<100;i++) asm nop;   //ȥ����
	     if(PTIH_PTIH0 == 0) stop++;   //�͵�ƽ����ʾ��⵽��Ϊ������
	     PIFH_PIFH0 = 1;               //���ж�
	}
	      