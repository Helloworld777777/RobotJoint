#include "DSP2833x_Device.h"                    // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"                  // DSP2833x Examples Include File
#include <math.h>

/////////////////////controler progress param////////////////
//1.0 :adc sample value ,3 phase current and voltage//////
volatile float Vac=0.0;
volatile float Ia =0.0;    // A相电流采集值
volatile float Ib =0.0;    // B相电流采集值
volatile float Ic =0.0;     // C相电流采集值
volatile float Idc=0.0;     // 直流电流采集值
volatile float Udc=0.0;
volatile float Vdc=0.0;     // 直流电压采集值
volatile float toque=0.0;
volatile float Speed_AD=0.0;
//volatile float Vdc_low=360.0;//直流电压欠压保护点
volatile float Vdc_high=700.0;//直流电压过压保护点
volatile float Uac_low=200.0;//交流电压欠压保护点
volatile float Uac_high=245.0;//交流电压过压保护点

volatile float speed_low = 200.0,speed_start=300;
Uint16 Ch[6];
/////////////////////////////////////////////
volatile float speed;
volatile float theta_mech=0.0;
volatile long Qpos[2]={0};
volatile float we = 0;//每中断周期机械位置差
volatile int pos_last = 0;

////////////////////////////////////////////

/////////////////////////////////
volatile float VdcStr[4]={0,0,0,0};
volatile float VdcNow=0.0,delta_vdc;    //   逆变器母线电压

/************************************SCI communication*******************/
Uint16 ModbusCRCLo[],ModbusCRCHi[];
Uint16 data_tx[20]={0};
Uint16 rdataB[8]={0};

volatile Uint16 RxCount = 0;
volatile int RxFlag=0;
/************************************SCI communication*******************/

/*-----------------------------------------------------------------------------
    Default initalizer for the CLARKE object.
-----------------------------------------------------------------------------*/
#define CLARKE_DEFAULTS { 0.0, \
                          0.0, \
                          0.0, \
                          0.0, \
                          0.0, \
                        }

/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/
#define PARK_DEFAULTS {   0.0, \
                          0.0, \
                          0.0, \
                          0.0, \
                          0.0, \
                          0.0, \
                          0.0,   \
                          0.0,\
                      }

//电流控制PI参数
#define PIDIQ_DEFAULTS   {  0,\
/*  Fdb   */                0,\
/*  Err   */                0,\
/*  Kp   */                 2,\
/*  Up   */                 0,\
/*  Ui   */                 0,\
/*  OutPreSat   */          0,\
/*  OutMax   */             600,\
/*  OutMin   */             -600,\
/*  Out   */                0,\
/*  Ki  */                  0.00005,\
                            8,\
                            -8,\
                         }

#define PIDSPEED_DEFAULTS   { 0,\
/*  Fdb   */                0,\
/*  Err   */                0,\
/*  Kp   */                 0.005,\
/*  Up   */                 0,\
/*  Ui   */                 0,\
/*  OutPreSat   */          0,\
/*  OutMax   */             5,\
/*  OutMin   */             -5,\
/*  Out   */                0,\
/*  Ki  */                  0.01,\
                            200.0,\
                            -200.0,\
                          }

#define Dis_Para_DEFAULTS {\
                            0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
                            0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
                            0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
                            1199,0.0008333333,0,0,{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,},\
                            }


#define IPARK_DEFAULTS {  0.0, \
                          0.0, \
                          0.0, \
                          0.0, \
                          0.0, \
                          0.0, \
                       }
#define CURR_DEFAULTS { 0.00012,\
                        0.0,  \
                        0.0,  \
                        0.0,  \
                        0.0,  \
                        0.0,  \
                        0.0,  \
                        0.0,  \
                        0.0,  \
                       }


#define SVGENDQ_DEFAULTS {  0,\
                            0,\
                            0,\
                            0,\
                            0,\
                         }


#define Uart1_DEFAULTS  {\
/*Tx_Data1*/             {0x08,0x04,0x30,0,0,0,0,0},\
/*Rx_Data1*/             {0,0,0,0,0,0,0,0},\
/*Tx_Data2*/             {0x01,0x00,0,0,0,0,0,0},\
/*Rx_Data2*/             {0,0,0,0,0,0,0,0},\
/*Rx_Disp*/              {0,0,0,0,0,0,0,0},\
/*Tx_Num1*/               8,\
/*Rx_Num1*/               32,\
/*Tx_Num2*/               8,\
/*Rx_Num2*/               8,\
                          NULL,\
/*Tx_Cnt*/                0,\
/*Rx_Cnt*/                0,\
/*Rx_Flag*/               0,\
/*Tx_Flag*/               0,\
                          1,\
                          0,\
                        }
#define Uart2_DEFAULTS  {\
/*Tx_Data1*/             {0x01,0x03,0x08,0,0,0,0,0},\
/*Rx_Data1*/             {0,0,0,0,0,0,0,0},\
/*Tx_Data2*/             {0x01,0x00,0,0,0,0,0,0},\
/*Rx_Data2*/             {0,0,0,0,0,0,0,0},\
/*Rx_Disp*/              {0,0,0,0,0,0,0,0},\
/*Tx_Num1*/               8,\
/*Rx_Num1*/               32,\
/*Tx_Num2*/               8,\
/*Rx_Num2*/               8,\
                          NULL,\
/*Tx_Cnt*/                0,\
/*Rx_Cnt*/                0,\
/*Rx_Flag*/               0,\
/*Tx_Flag*/               0,\
                          1,\
                          0,\
                        }
#define RUNSTS_DEFAULTS {\
                            0,\
                            0,\
                            0,\
                            0x2,\
                            0,\
                            0,\
                            0,\
                            0,\
                            0,\
                            0,\
                            0,\
                            }

#define MOTOR_DATA_DEFAULTS {1000,10000,0,0,0,0,0,0,0,0.005,0.005,2.6}

RUNSTS  run_sts=RUNSTS_DEFAULTS;
MOTOR_DATA     motor_dat   = MOTOR_DATA_DEFAULTS;

CLARKE         I_clarke    = CLARKE_DEFAULTS;
PARK           V_park      = PARK_DEFAULTS;
PARK           I_park      = PARK_DEFAULTS;
IPARK          I_ipark     = IPARK_DEFAULTS;
PIDREG         Id_pidreg   = PIDIQ_DEFAULTS;
PIDREG         Iq_pidreg   = PIDIQ_DEFAULTS;

PIDREG         Speed_pidreg    = PIDSPEED_DEFAULTS;
CURR_ADJ       V_CURR_ADJ  = CURR_DEFAULTS;

SVGENDQ        svgen       = SVGENDQ_DEFAULTS;  //SVPWM初始化

Display         Dis_Para    =Dis_Para_DEFAULTS;
Uart            Uart1       = Uart1_DEFAULTS;
Uart            Uart2       = Uart2_DEFAULTS;

Uint16    *ExRamStart = (Uint16 *)0x100000;
Uint16    *ExRamStart1 = (Uint16 *)0x100600;

char  ReadAd(void);
void  clarke_calc(CLARKE_handle v);
void  park_calc(PARK_handle v);
void  CurrControl(Curr_handle v);
void  ipark_calc(IPARK_handle v);

void  CtrlProcess (void);
void  pid_reg_calc(PIDREG_handle v);
void  svgendq_calc(SVGENDQ_handle v);
void  fault_action(void);
void  Paramcalc2(void);
void  rotor_pos_calc(void);
void Protect(u16 s);

void Scib_Check(void);
void Scic_Check(void);
void Scib_Data_Process(void);
void Scib_Data_Tx(void);
void Scic_Data_Process(void);
void Scic_Data_Tx(void);
void RESET_SCIB(void);
void RESET_SCIC(void);
void GetCrcCode(Uint16 len,Uint16 * str);
void int_char(Uint16 len, Uint16 * Dest,Uint16 * Source);

interrupt void EPwm1ISR(void);
void c46_DOL(void);
void Paramcalc(void);
//extern void InitXintf();

volatile int Fault_I=0,VdcCount=0;

volatile Uint32 rs_time = 0;

float R25_N=0.1,T25_N=0.003354,T0=273.15,Vt=5,B5025_N=0.000253164,Rt=51;//B=3375,T25=298.15,Rt为上端分压电阻
volatile float R1=10;
volatile float Vtemp;

float offset1=60,offset2=60,offset3=60,offset4=60;

volatile int para_set[4]={1,0,50,0};
volatile int mode_select=1;
volatile int modbus_rx_delay1=0,modbus_rx_delay2=0,Torque_set=50,Speed_set=500;


volatile Uint16 data_req=0,data_ready=0;
int step=0,start_done=0,QEP_CLR=0,switch_delay=0,start=0;


void ENIO()
{
    EALLOW;
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO48 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;

    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;

    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;

    GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO61 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;
    EDIS;
}

void main(void)
{
	Uint16 i,j;
	float temp;

	InitSysCtrl();

	InitGpio();
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	InitTzGpio();
	InitXintf();
	InitEQep1Gpio();
	InitSciGpio();
    ENIO();

	DINT;
	
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();

	EALLOW; 
	PieVectTable.EPWM1_INT = &EPwm1ISR;
	EDIS;
	InitEQep();
	InitAdc();
	InitSci();
	InitEPwm();
	

	IER |= M_INT3;
    IER |= M_INT9;

    PieCtrlRegs.PIEIER9.bit.INTx7=1;  //pie group 9 cana int1
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;	// Enable TINT0 in the PIE: Group 3 interrupt 1
	
	EINT;// Enable global Interrupts and higher priority real-time debug events:
	ERTM;
	
   Speed_pidreg.Ref=Speed_set;

   Id_pidreg.Kp    = 0.2;//0.5
   Id_pidreg.Ki    = 0.05;//0.01
   Id_pidreg.OutMax= 50.0;//24
   Id_pidreg.OutMin= -50.0;//-24
   Id_pidreg.ErrMax= 20;//10
   Id_pidreg.ErrMin= -20;//-10
   Iq_pidreg.Kp    = 0.2;//0.5
   Iq_pidreg.Ki    = 0.05;//0.0002
   Iq_pidreg.OutMax= 50.0;//24
   Iq_pidreg.OutMin= -50.0;//-24
   Iq_pidreg.ErrMax= 20;//10
   Iq_pidreg.ErrMin= -20;//-10
   Speed_pidreg.Kp = 0.02;//0.08
   Speed_pidreg.Ki = 0.0005;//0.00003
   Speed_pidreg.OutMax= 25;//20
   Speed_pidreg.OutMin=-25;//-20
   Speed_pidreg.ErrMax=200;//1000
   Speed_pidreg.ErrMin=-200;//-1000

/************************电流调零**************************/
  i=0;
  while(i<10)
  {
      while(!Dis_Para.status);
      Dis_Para.status=0;
      i++;
  }
  Dis_Para.status|=0x04;//调零结束

  temp=Dis_Para.Ia_sum_t * Dis_Para.Count_Ratio;
  offset1=offset1+temp;

  temp=Dis_Para.Ib_sum_t * Dis_Para.Count_Ratio;
  offset2=offset2+temp;

  temp=Dis_Para.Ic_sum_t * Dis_Para.Count_Ratio;
  offset3=offset3+temp;

  temp=Dis_Para.Idc_sum_t * Dis_Para.Count_Ratio;
  offset4=offset4+temp;

	while(1)
	{

       /************************过流保护及恢复*****************************/
         Fault_I = EPwm1Regs.TZFLG.bit.OST;
         if(Fault_I)
         {
             run_sts.fault1 |= 0x20;
             TZCLR();
             fault_action();
         }
         else if(run_sts.fault1&0x20)
         {
             run_sts.fault1 &= 0xffdf;
         }

   /************************过流保护及恢复*****************************/

       if(Dis_Para.status&0x01)
       {
           Dis_Para.status &= 0xfffe;
           Paramcalc2();
       }

       if(modbus_rx_delay1 > 100)//485应答10mS延时
       {
           modbus_rx_delay1=0;
           Scib_Data_Process();
       }
	}
}



interrupt void EPwm1ISR(void)
{
    PieCtrlRegs.PIEACK.all = 0x0004;

    if((ScibRegs.SCICTL2.bit.TXEMPTY)&&(!Uart1.Tx_Flag))//485方向控制 发送完成转为接收
        RS485_RX1;
    Scib_Check();
    rotor_pos_calc();
    ReadAd();

    Protect(1);
///************************全部故障恢复*****************************/
    if((!run_sts.fault1)&&(!run_sts.run_flag))
    {
        TZCLR();
        Speed_pidreg.Ref=0;
        Speed_pidreg.Ui =0;
        Speed_pidreg.Up =0;
        Iq_pidreg.Ui    =0;
        Iq_pidreg.Up    =0;
        Id_pidreg.Ui    =0;
        Id_pidreg.Up    =0;
        RELAY_ON;
        run_sts.run_flag1=1;
        run_sts.run_flag=1;
    }
/************************全部故障恢复*****************************/

    if(run_sts.run_flag)
    {
        CtrlProcess();         //控制参数计算
        svgendq_calc(&svgen);                           //SVPWM计算及更新寄存器
        LoadPWM;
//        run_sts.run_flag = 0;
        if(run_sts.run_flag1)
        {
            run_sts.run_en=1;
            EnablePWM;
            run_sts.run_flag1=0;
        }
    }

    Paramcalc();//参数计算函数

    if(Uart1.Rx_Flag & 0x1)
        modbus_rx_delay1++;
//    if(Uart2.Rx_Flag & 0x1)
//        modbus_rx_delay2++;

    Scib_Data_Tx();

    run_sts.delay_cnt++;
    if(run_sts.delay_cnt>100000)
        run_sts.delay_cnt=0;

    EPwm1Regs.ETCLR.bit.INT = 1;                   //clear ETFLG[INT]
}


char  ReadAd(void)
{
    Uint16 i;
    float temp=0;

    while (AdcRegs.ADCST.bit.INT_SEQ1== 0);//判断 AD是否读完
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;//清除AD标志

    Ch[0]=((AdcRegs.ADCRESULT0)>>4)-12;//IA
    Ch[1]=((AdcRegs.ADCRESULT1)>>4)-12;//IB
    Ch[2]=((AdcRegs.ADCRESULT2)>>4)-12;//IC
    Ch[3]=((AdcRegs.ADCRESULT3)>>4)-12;//IDC
    Ch[4]=((AdcRegs.ADCRESULT4)>>4);//UDC
    Ch[5]=((AdcRegs.ADCRESULT5)>>4);//TEMP


    Ia       = (Ch[0] * 0.0007326 - 1.65) * 22.7272;//30A互感器变比
    Ib       = (Ch[1] * 0.0007326 - 1.65) * 22.7272;
    Ic       = (Ch[2] * 0.0007326 - 1.65) * 22.7272;
    Idc      = (1.65 - Ch[3] * 0.0007326) * 22.7272;
//    toque    = (Ch[5] * 0.0007326 - 1.2)  * 6.25;
//    Speed_AD = (Ch[6] * 0.0007326 - 1.2)  * 3750;
    Udc      = Ch[4] * 0.01465*1.06;//Ch[4]*0.0007326*20

    Vtemp    = Ch[5]* 0.0007326;


    VdcCount&=3;
    VdcStr[VdcCount++] = Udc;

    for(i=0;i<4;i++)
        temp=temp+VdcStr[i];
    Vdc = temp * 0.25;

    return(0);  //无故障返回0
}

void CtrlProcess (void)
{
    float temp;
//  svgen.Ualpha    = -0.05;//       //SVPWM输出
//  svgen.Ubeta     = 0;//
    if(!start_done)
    {
        if(!QEP_CLR)
        {
            QEP_CLR=1;
            EQep1Regs.QCLR.bit.IEL = 1;//Clear index event latch interrupt flag
        }
        svgen.Ualpha    = -0.06 * cos(step * 0.0031416);//      //SVPWM输出
        svgen.Ubeta     = -0.06 * sin(step * 0.0031416);//
        if(Speed_pidreg.Ref<0)//反向转
        {
            temp=svgen.Ualpha;
            svgen.Ualpha=svgen.Ubeta;
            svgen.Ubeta = temp;
        }
        step++;
        if(step>1999)   step=0;
    }
    if(EQep1Regs.QFLG.bit.IEL)
    {
        start_done = 1;
    }
    if(start_done)
    {
        /*电流克拉克坐标变换*/
        I_clarke.AS     =-Ia;      //将实际A相电压采集值送给克拉克函数输入A相
        I_clarke.BS     =-Ib;      //将实际B相电压采集值送给克拉克函数输入B相
        I_clarke.CS     =-Ic;      //将实际C相电压采集值送给克拉克函数输入C相

        clarke_calc(&I_clarke);                //电流clarke变换
        /*电流帕克坐标变换*/
        I_park.PAlpha  = I_clarke.Valpha;
        I_park.PBeta   = I_clarke.Vbeta;
        I_park.PAngle  = motor_dat.theta_mech;

        park_calc(&I_park);             //电流PARK变换

        //速度外环
//        if((!motor_dat.pos_cnt)&&(mode_select&0x1))//1kHz
        if(!motor_dat.pos_cnt)//1kHz
        {
            if(Speed_pidreg.Ref != Speed_set)
            {
               if(Speed_pidreg.Ref > Speed_set + 0.2)
               {
                   Speed_pidreg.Ref -= 0.2;
               }
               else if(Speed_pidreg.Ref < Speed_set - 0.2)//0.2
               {
                   Speed_pidreg.Ref += 0.2;
               }
               else
               {
                   Speed_pidreg.Ref = Speed_set;
               }
            }
//            Speed_pidreg.Ref = Speed_set;
            motor_dat.pos_cnt=1;


            Speed_pidreg.Fdb=motor_dat.speed;
            pid_reg_calc(&Speed_pidreg);
            Iq_pidreg.Ref= Speed_pidreg.Out;
        }

        /*内环电流q轴控制*/
        Iq_pidreg.Fdb   = I_park.PQs;
        pid_reg_calc(&Iq_pidreg);
//        I_ipark.IPQs    = Iq_pidreg.Out;
        V_CURR_ADJ.Vq_Pidout    = Iq_pidreg.Out;

        /*内环电流d轴控制*/
        Id_pidreg.Ref   = 0.0 ; //无功电流
        Id_pidreg.Fdb   = I_park.PDs;
        pid_reg_calc(&Id_pidreg);
//        I_ipark.IPDs    = Id_pidreg.Out;
        V_CURR_ADJ.Vd_Pidout    = Id_pidreg.Out;
        /*前馈解耦*/
        V_CURR_ADJ.Ifdbd = I_park.PDs;
        V_CURR_ADJ.Ifdbq = I_park.PQs;
        V_CURR_ADJ.We= motor_dat.we;
        CurrControl(&V_CURR_ADJ);
        /*电流反帕克坐标变换*/
        I_ipark.IPDs    = V_CURR_ADJ.Vd;             //控制器输出d轴电压
        I_ipark.IPQs    = V_CURR_ADJ.Vq;

        temp=1.732/Vdc;
        I_ipark.IPDs    = I_ipark.IPDs * temp;
        I_ipark.IPQs    = I_ipark.IPQs * temp;
        I_ipark.IPSine  = I_park.PSine;
        I_ipark.IPCosine= I_park.PCosine;
        ipark_calc(&I_ipark);             //PARK反变换
        /*七段SVPWM*/
        svgen.Ualpha    = -I_ipark.IPAlpha;       //SVPWM输
        svgen.Ubeta     = -I_ipark.IPBeta;
    }

}

void Paramcalc(void)
{

    if(Dis_Para.Count>Dis_Para.N)
    {

        Dis_Para.Udc_sum_t=Dis_Para.Udc_sum;        Dis_Para.Udc_sum=0;

        Dis_Para.Ia_sum_t =Dis_Para.Ia_sum;         Dis_Para.Ia_sum=0;
        Dis_Para.Ib_sum_t =Dis_Para.Ib_sum;         Dis_Para.Ib_sum=0;
        Dis_Para.Ic_sum_t =Dis_Para.Ic_sum;         Dis_Para.Ic_sum=0;

        Dis_Para.Idc_sum_t=Dis_Para.Idc_sum;        Dis_Para.Idc_sum=0;
        Dis_Para.Iq_sum_t=Dis_Para.Iq_sum;        Dis_Para.Iq_sum=0;

        Dis_Para.Tem_sum_t =Dis_Para.Tem_sum;       Dis_Para.Tem_sum=0;

        Dis_Para.Speed_sum_t= Dis_Para.Speed_sum;   Dis_Para.Speed_sum=0;

        Dis_Para.Count=0;
        Dis_Para.status |=0x01;
    }
    else
    {

        Dis_Para.Udc_sum+=Vdc;
        Dis_Para.Idc_sum+=Idc;
        Dis_Para.Iq_sum +=I_park.PQs;

        if(Dis_Para.status & 0x04)//显示使用
        {
            Dis_Para.Ia_sum +=Ia * 0.577 * Ia * 0.577;
            Dis_Para.Ib_sum +=Ib * 0.577 * Ib * 0.577;
            Dis_Para.Ic_sum +=Ic * 0.577 * Ic * 0.577;
        }
        else//调零使用
        {
            Dis_Para.Ia_sum +=Ia * 0.577;
            Dis_Para.Ic_sum +=Ic * 0.577;
            Dis_Para.Ib_sum +=Ib * 0.577;
        }

        Dis_Para.Tem_sum+=Vtemp;

        Dis_Para.Speed_sum += motor_dat.speed;

        Dis_Para.Count++;
    }

}

void Paramcalc2(void)
{
    int i;
    float temp1,temp2,T;


    Dis_Para.Udc=Dis_Para.Udc_sum_t * Dis_Para.Count_Ratio;
    Dis_Para.Idc=Dis_Para.Idc_sum_t * Dis_Para.Count_Ratio;
    Dis_Para.Iq=Dis_Para.Iq_sum_t * Dis_Para.Count_Ratio;

    Dis_Para.Ia=sqrt(Dis_Para.Ia_sum_t * Dis_Para.Count_Ratio);
    Dis_Para.Ib=sqrt(Dis_Para.Ib_sum_t * Dis_Para.Count_Ratio);
    Dis_Para.Ic=sqrt(Dis_Para.Ic_sum_t * Dis_Para.Count_Ratio);

    Dis_Para.P = Dis_Para.Udc * Dis_Para.Idc;

    Dis_Para.Speed = Dis_Para.Speed_sum_t * Dis_Para.Count_Ratio + 0.2;

    Dis_Para.Tem=Dis_Para.Tem_sum_t * Dis_Para.Count_Ratio;

    R1 = Rt/(Vt-Dis_Para.Tem) * Dis_Para.Tem;  //NTC阻值
    temp2 = R1 * R25_N;
    temp1 = log(temp2) * B5025_N + T25_N;
    temp2 = 1.0f/temp1;
    temp1 = temp2-T0;
    Dis_Para.Tem = temp1;  //温度℃

//  * Vaver_tcppoint=Dis_Para.Speed;//转速

    i=0;
    if(Dis_Para.Speed>0)
        temp1=Dis_Para.Iq;
    else
        temp1=Dis_Para.Iq;
    temp1=fabs(temp1 * 4.8);
//    temp1=fabs(temp1 * 12);//扭矩=力矩系数*q轴电流*100      //100为组态屏处进行的/100操作,减小误差
    T=fabs(temp1 * 0.01);
//  Dis_Para.Tx_int[i++] = temp1 * 0.039;//定子电压10x  line Voltage,3.9V/1000r
    Dis_Para.Tx_int[i++] = temp1;//扭矩  [0]
    if(Dis_Para.Speed>0)
//        temp1=Dis_Para.Ib-0.98;
        temp1=Dis_Para.Ib;
    else if(Dis_Para.Speed<0)
        temp1=Dis_Para.Ib;
    else
        temp1=0;
    Dis_Para.Tx_int[i++] = temp1 * 10;//定子电流  [1]
    Dis_Para.Tx_int[i++] = Dis_Para.Udc * 10;//   [2]
    if(fabs(Dis_Para.Speed)==0)
        Dis_Para.Idc=0;
    Dis_Para.Tx_int[i++] = (int)(Dis_Para.Idc * 10);//直流电流   [3]
    Dis_Para.Tx_int[i++] = (int)Dis_Para.Speed;//转速   [4]
//  Dis_Para.Tx_int[i++] = (int)(-Dis_Para.Iq * 20.588);//转矩
    Dis_Para.Tx_int[i++] = 0;//保留 [5]
    Dis_Para.Tx_int[i++] = 0;//保留 [6]
    Dis_Para.Tx_int[i++] = 0;//保留 [7]
//  Dis_Para.Tx_int[i++] = Dis_Para.Tem * 10;//温度[9]
    Dis_Para.Tx_int[i++] = fabs((Dis_Para.Speed)*T*1.047);//P=T*N*10000/9550W [8]
    if(fabs(Dis_Para.Speed)==0)
        Dis_Para.Idc=0;
    Dis_Para.Tx_int[i++] = (int)(Dis_Para.Udc * Dis_Para.Idc *10);//直流功率  [9]
    if(run_sts.fault1 == 0)
        run_sts.fault2 = 0;
    Dis_Para.Tx_int[i++] = run_sts.fault2;//status [10]

    Uart2.Tx_Data2[0] = Dis_Para.Speed;//速度
    Uart2.Tx_Data2[1] = Dis_Para.Tx_int[1];//定子电流
    Uart2.Tx_Data2[2] = Dis_Para.Tx_int[0];//定子电压


}

void Scib_Check(void)
{
    if(ScibRegs.SCIRXST.bit.RXRDY)
    {
        Uart1.Rx_Data1[Uart1.Rx_Cnt++]= ScibRegs.SCIRXBUF.bit.RXDT;   // Read data
        if(Uart1.Rx_Data1[0]==0x08)
        {
            Uart1.Rx_Flag = 0x02;//计算超时标志位
            if(Uart1.Rx_Cnt>1)
            {
                if((Uart1.Rx_Data1[1]==0x03)||(Uart1.Rx_Data1[1]==0x04)||(Uart1.Rx_Data1[1]==0x05)||(Uart1.Rx_Data1[1]==0x06))
                {
                    if(Uart1.Rx_Cnt == 8)
                    {
                        Uart1.Rx_Flag=0x1;
                        Uart1.Rx_Cnt=0;
                        modbus_rx_delay1=0;
                        Uart1.Time =0;
                    }
                }
                else    Uart1.Rx_Cnt=0;
            }
        }
        else Uart1.Rx_Cnt=0;
    }
    if(Uart1.Rx_Flag & 0x2)
        Uart1.Time++;
    if(Uart1.Time > 150)//超时15mS
    {
        Uart1.Rx_Cnt=0;
        Uart1.Time=0;
        Uart1.Rx_Flag=0;
    }
}

void Scib_Data_Process(void)
{
    u16 temp;
    int tmp;

    Uart1.Rx_Flag=0;
    Uart1.Rx_Data1[8]=Uart1.Rx_Data1[6];
    Uart1.Rx_Data1[9]=Uart1.Rx_Data1[7];

    GetCrcCode(6,Uart1.Rx_Data1);

    if((Uart1.Rx_Data1[8] != Uart1.Rx_Data1[6])||(Uart1.Rx_Data1[9] != Uart1.Rx_Data1[7]))
        Uart1.Rx_Data1[1] = 0;

    switch(Uart1.Rx_Data1[1])
    {
        case 0x03:
                temp = (Uart1.Rx_Data1[2]<<8) + Uart1.Rx_Data1[3];
                if(temp<2100) temp=2100;
                if(temp>2110) temp=2110;
                temp=temp-2100; //寄存器起始地址偏移量

                Uart1.Tx_Data1[2]=Uart1.Rx_Data1[5];//发送寄存器数目

                int_char(Uart1.Tx_Data1[2],Uart1.Tx_Data1,para_set);
                Uart1.Tx_Data1[2] = Uart1.Tx_Data1[2]<<1;//发送字节数目

                Uart1.Tx_Data1[1]=0x3;
                Uart1.Tx_Num1 = 3 + Uart1.Tx_Data1[2];
                GetCrcCode(Uart1.Tx_Num1, Uart1.Tx_Data1);
                Uart1.Tx_Num1 = 2 + Uart1.Tx_Num1;//加上校验2字节

                Uart1.Tx_Flag=0x02;
                break;
        case 0x04:
                temp = (Uart1.Rx_Data1[2]<<8) + Uart1.Rx_Data1[3];
                if(temp<1000) temp=1000;
                if(temp>1010) temp=1010;
                temp=temp-1000; //寄存器起始地址偏移量

                Uart1.Tx_Data1[2]=Uart1.Rx_Data1[5];//发送寄存器数目
                if(Uart1.Tx_Data1[2]>11)
                    Uart1.Tx_Data1[2]=11;

                int_char(Uart1.Tx_Data1[2],Uart1.Tx_Data1,&Dis_Para.Tx_int[temp]);
                Uart1.Tx_Data1[2] = Uart1.Tx_Data1[2]<<1;//发送字节数目

                Uart1.Tx_Data1[1]=0x4;
                Uart1.Tx_Num1 = 3 + Uart1.Tx_Data1[2];
                GetCrcCode(Uart1.Tx_Num1, Uart1.Tx_Data1);
                Uart1.Tx_Num1 = 2 + Uart1.Tx_Num1;//加上校验2字节

                Uart1.Tx_Flag=0x02;
                break;
        case 0x05://开关机
                if(!Uart1.Rx_Data1[4])
                    run_sts.fault1|=0x02;
                else
                    run_sts.fault1=0;
                    Uart1.Tx_Flag=0x04;
                break;
        case 0x06://模式  转速  转矩
                temp=(Uart1.Rx_Data1[2]<<8) + Uart1.Rx_Data1[3];
                tmp=(Uart1.Rx_Data1[4]<<8) + Uart1.Rx_Data1[5];
                if(temp==2100)//0x0834
                {
                    if(!run_sts.run_flag)
                        mode_select=tmp;
                    para_set[0]=mode_select;
                }
                if(temp==2101)//0x0835
                {
                    if(tmp>3000)
                        tmp=3000;
                    if(tmp<(-3000))
                        tmp=-3000;
                    Speed_set=tmp;
                    para_set[1]=tmp;
//                    Speed_pidreg.Ref = Speed_set;
                }
                if(temp==2102)//0x0836
                {
                    if(tmp>350)//扭矩小于13N
                        tmp=350;
                    if(tmp<(-350))
                        tmp=-350;
                    Torque_set=tmp;
                    para_set[2]=tmp;
                }
                Uart1.Tx_Flag=0x04;
                break;
    }
}

void Scib_Data_Tx(void)
{
    if(ScibRegs.SCIRXST.bit.RXERROR)//串口错误复位
        RESET_SCIB();
    if(ScibRegs.SCICTL2.bit.TXRDY)
    {
        if(Uart1.Tx_Flag & 0x06)
            RS485_TX1;
        switch(Uart1.Tx_Flag)
        {
            case 0x02:
                    ScibRegs.SCITXBUF = Uart1.Tx_Data1[Uart1.Tx_Cnt++];
                    if(Uart1.Tx_Cnt==Uart1.Tx_Num1)//11字数据
                    {
                        Uart1.Tx_Cnt=0;
                        Uart1.Tx_Flag=0;
                    }
                    break;
            case 0x04:
                    ScibRegs.SCITXBUF = Uart1.Rx_Data1[Uart1.Tx_Cnt++];
                    if(Uart1.Tx_Cnt&0x08)
                    {
                        Uart1.Tx_Cnt=0;
                        Uart1.Tx_Flag=0;
                    }
                    break;
            default:
                break;
        }
    }
}

void RESET_SCIB(void)
{
    ScibRegs.SCICTL1.bit.SWRESET=0;
    DELAY_US(1);
    ScibRegs.SCICTL1.bit.SWRESET=1;
}


void int_char(Uint16 len, Uint16 * Dest,Uint16 * Source)
{
    Uint16 i, temp1, temp2;
    for (i = 0; i < len; i++)
    {
        temp1 = (i << 1) + 3;
        temp2 = temp1 +1;
        *(Dest+temp1) = *(Source+i) >> 8;      //高字节在前
        *(Dest+temp2) = *(Source+i) & 0x00ff;
    }
}

void GetCrcCode(Uint16 len, Uint16 * str)
{
    Uint16 hi, lo;
    Uint16 x, j;
    hi = 0xff;
    lo = 0xff;
    for (x = 0; x < len; x++)
    {
        j = lo ^ (*(str + x));
        lo = hi ^ ModbusCRCHi[j];
        hi = ModbusCRCLo[j];
    }
    *(str + len) = lo;
    *(str + len + 1) = hi;
}



void fault_action(void)
{
    DisablePWM;
    DisablePWM1;
//    Fault_Reset_OFF;
//    DELAY_US(100);
//    Fault_Reset_ON;
    run_sts.run_en=0;
    QEP_CLR=0;
    start_done=0;
    start=0;
    run_sts.run_flag=0;
    run_sts.run_flag1=0;
    run_sts.fault2=run_sts.fault1;
    Speed_pidreg.Ref=0;
    Speed_pidreg.Ui =0;
    Speed_pidreg.Up =0;
    Iq_pidreg.Ui    =0;
    Iq_pidreg.Up    =0;
    Id_pidreg.Ui    =0;
    Id_pidreg.Up    =0;
    I_park.PQs      =0;
}


void rotor_pos_calc()
{
    int temp;

    motor_dat.theta_mech = EQep1Regs.QPOSCNT;   //位置
    if(EQep1Regs.QFLG.bit.UTO)//1mS
    {
        motor_dat.pos_last = motor_dat.pos_now;
        motor_dat.pos_now  = motor_dat.theta_mech;//EQep1Regs.QPOSLAT;
        EQep1Regs.QCLR.bit.UTO=1;

        temp = motor_dat.pos_now - motor_dat.pos_last;
        if(temp>6000)//反转
            temp -= motor_dat.encoder_pr;
        if(temp<-6000)//正转
            temp += motor_dat.encoder_pr;

        motor_dat.pos_cnt=0;
//        motor_dat.speed = temp * 14.6484375;
//        motor_dat.we = temp * 2.513274;
        motor_dat.speed = temp * 6;
        motor_dat.we = temp * 3.1415927;
    }
}

void Protect(u16 s)
{
    switch(s)
    {
        case 1://瞬时值
            if(Vdc>70)//30
            {
                run_sts.fault1|=0x08; //母线过压 瞬时值
            }
            else if(Vdc<60)//30
            {
                run_sts.fault1&=0xfff7; //母线过压恢复
            }
            if(Vdc<20)//30
            {
                run_sts.fault1|=0x10; //母线过压 瞬时值
            }
            else if(Vdc>25)//30
            {
                run_sts.fault1&=0xffef; //母线过压 瞬时值
            }
            if((Ia>28)||(Ib>28)||(Ic>28)||(Idc>28)||(Ia<-28)||(Ib<-28)||(Ic<-28))
            {
                run_sts.fault1|=0x20;
            }
            else
                run_sts.fault1&=0xffdf;
            break;

        case 2: //平均值

            if(Dis_Para.Udc>70)
            {
                run_sts.fault1|=0x40;
            }
            else if(Dis_Para.Udc<60)
            {
                run_sts.fault1&=0xffbf;
            }
            if(Dis_Para.Udc<20)//22
            {
                run_sts.fault1|=0x04;//母线欠压 平均值
            }
            else if(Dis_Para.Udc>25)//23
            {
                run_sts.fault1&=0xfffb;
            }
            break;
        default:
            break;
    }

    if(run_sts.run_flag&&run_sts.fault1)
        fault_action();
}

void c46_DOL()
{
    int i,DI1;
    int data1=6,data2=1;

    CS1L;
    DELAY_US(5);
    CS1H;
    DELAY_US(5);

    for(i=0;i<3;i++)
    {
        DI1 =(0x04>>i) & data1 ;
        if(DI1)
            DI1H;
        else
            DI1L;
        DELAY_US(1);
        CLK1H;
        DELAY_US(4);
        CLK1L;
        DELAY_US(3);
    }

    for(i=0;i<7;i++)
    {
        DI1 =(0x40>>i) & data2 ;
        if(DI1)
            DI1H;
        else
            DI1L;
        DELAY_US(1);
        CLK1H;
        DELAY_US(4);
        CLK1L;
        DELAY_US(3);
    }
}


Uint16 ModbusCRCHi[]=
{
0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,
0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,
0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,
0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,
0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,
0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,
0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,
0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40
};


Uint16 ModbusCRCLo[]=
{
0x00,0xc0,0xc1,0x01,0xc3,0x03,0x02,0xc2,0xc6,0x06,0x07,0xc7,0x05,0xc5,0xc4,0x04,0xcc,0x0c,0x0d,0xcd,
0x0f,0xcf,0xce,0x0e,0x0a,0xca,0xcb,0x0b,0xc9,0x09,0x08,0xc8,0xd8,0x18,0x19,0xd9,0x1b,0xdb,0xda,0x1a,
0x1e,0xde,0xdf,0x1f,0xdd,0x1d,0x1c,0xdc,0x14,0xd4,0xd5,0x15,0xd7,0x17,0x16,0xd6,0xd2,0x12,0x13,0xd3,
0x11,0xd1,0xd0,0x10,0xf0,0x30,0x31,0xf1,0x33,0xf3,0xf2,0x32,0x36,0xf6,0xf7,0x37,0xf5,0x35,0x34,0xf4,
0x3c,0xfc,0xfd,0x3d,0xff,0x3f,0x3e,0xfe,0xfa,0x3a,0x3b,0xfb,0x39,0xf9,0xf8,0x38,0x28,0xe8,0xe9,0x29,
0xeb,0x2b,0x2a,0xea,0xee,0x2e,0x2f,0xef,0x2d,0xed,0xec,0x2c,0xe4,0x24,0x25,0xe5,0x27,0xe7,0xe6,0x26,
0x22,0xe2,0xe3,0x23,0xe1,0x21,0x20,0xe0,0xa0,0x60,0x61,0xa1,0x63,0xa3,0xa2,0x62,0x66,0xa6,0xa7,0x67,
0xa5,0x65,0x64,0xa4,0x6c,0xac,0xad,0x6d,0xaf,0x6f,0x6e,0xae,0xaa,0x6a,0x6b,0xab,0x69,0xa9,0xa8,0x68,
0x78,0xb8,0xb9,0x79,0xbb,0x7b,0x7a,0xba,0xbe,0x7e,0x7f,0xbf,0x7d,0xbd,0xbc,0x7c,0xb4,0x74,0x75,0xb5,
0x77,0xb7,0xb6,0x76,0x72,0xb2,0xb3,0x73,0xb1,0x71,0x70,0xb0,0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,
0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9c,0x5c,0x5d,0x9d,0x5f,0x9f,0x9e,0x5e,0x5a,0x9a,0x9b,0x5b,
0x99,0x59,0x58,0x98,0x88,0x48,0x49,0x89,0x4b,0x8b,0x8a,0x4a,0x4e,0x8e,0x8f,0x4f,0x8d,0x4d,0x4c,0x8c,
0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,0x40
};


