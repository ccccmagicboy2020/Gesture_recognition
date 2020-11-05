#include "timer.h"
#include "led.h"
#include "adc.h"
#include "stdio.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器中断驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/4/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


TIM_HandleTypeDef TIM3_Handler;      //定时器句柄 

TIM_HandleTypeDef TIM4_Handler;      	//定时器句柄 
TIM_OC_InitTypeDef TIM4_CH2Handler;	//定时器14通道1句柄

TIM_HandleTypeDef TIM10_Handler;      	//定时器句柄 
TIM_OC_InitTypeDef TIM10_CH1Handler;	//定时器14通道1句柄

u16 Timer_Count = 0;//溢出次数
//u16 AD_Value1[1024]={0};
s16  AD_Value[1024]={0};
float temp_val=0;
u8 mea= 0;
u8 stopflag=0;
extern void fft256( u8 a);
extern void measure_fft(void);

//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)
void TIM3_Init(u16 arr,u16 psc)
{  
    TIM3_Handler.Instance=TIM3;                          //通用定时器3
    TIM3_Handler.Init.Prescaler=psc;                     //分频系数
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM3_Handler.Init.Period=arr;                        //自动装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM3_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM3_Handler); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE   
}

//TIM14 PWM部分初始化 
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
void TIM4_PWM_Init(u16 arr,u16 psc)
{  
    TIM4_Handler.Instance=TIM4;          	//定时器14
    TIM4_Handler.Init.Prescaler=psc;       //定时器分频
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM4_Handler.Init.Period=arr;          //自动重装载值
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM4_Handler);       //初始化PWM
    
    TIM4_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM4_CH2Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
	
    TIM4_CH2Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为低 
	
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH2Handler,TIM_CHANNEL_2);//配置TIM14通道1
	
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_2);//开启PWM通道1
}

void TIM10_PWM_Init(u16 arr,u16 psc)
{  
    TIM10_Handler.Instance=TIM10;          	//定时器14
    TIM10_Handler.Init.Prescaler=psc;       //定时器分频
    TIM10_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM10_Handler.Init.Period=arr;          //自动重装载值
    TIM10_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM10_Handler);       //初始化PWM
    
    TIM10_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM10_CH1Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM10_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为低 
    HAL_TIM_PWM_ConfigChannel(&TIM10_Handler,&TIM10_CH1Handler,TIM_CHANNEL_1);//配置TIM14通道1
	
    HAL_TIM_PWM_Start(&TIM10_Handler,TIM_CHANNEL_1);//开启PWM通道1
}




//定时器底册驱动，开启时钟，设置中断优先级
//此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM3)
	{
		__HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
		HAL_NVIC_SetPriority(TIM3_IRQn,1,3);    //设置中断优先级，抢占优先级1，子优先级3
		HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断   
	}
}

//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM4_CLK_ENABLE();			//使能定时器10
	__HAL_RCC_TIM10_CLK_ENABLE();			//使能定时器10
	
	
	__HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOF时钟
	
	GPIO_Initure.Pin=GPIO_PIN_7;           	//PF6
	
	
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	
	GPIO_Initure.Alternate= GPIO_AF2_TIM4;	//PF9复用为TIM14_CH1
	
	
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	
	GPIO_Initure.Pin=GPIO_PIN_8;           	//PF6
	
	
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	
	GPIO_Initure.Alternate= GPIO_AF3_TIM10;	//PF9复用为TIM14_CH1
	
	
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);	
}



//设置TIM通道4的占空比
//compare:比较值
void TIM_SetTIM4Compare1(u32 compare)
{
	TIM4->CCR2=compare; 
}
void TIM_SetTIM10Compare1(u32 compare1)
{
	TIM10->CCR1=compare1; 
}


//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM3_Handler);
}


//回调函数，定时器中断服务函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
		u16 adc_data,i;
	  if(htim==(&TIM3_Handler))
    {
				adc_data = Get_Adc(5);
				temp_val+=adc_data;
			  AD_Value[Timer_Count] = adc_data;
			  
				//		  printf("%d \r",adc_data);
			
				Timer_Count++;
				
				if(Timer_Count == 868)//1024
				{
					
					temp_val=temp_val/Timer_Count;
					for(i=0;i<868;i++)//1024
					{
							AD_Value[i] = AD_Value[i] - temp_val ; 
					//	  printf("%.2f \r",AD_Value[i]);
					}
				//	delay_ms(100);
					if(stopflag==1)
					{
					+	delay_ms(300);
						stopflag=0;
					}
					else if(stopflag==2)
					{
//						delay_ms(2000);
						stopflag=0;
					}
          mea=1;					
//					measure_fft();	
						
					Timer_Count = 0;
					temp_val = 0;
		//			  printf("\n ");
					
								//	timer_interrupt_flag_clear(TIMER6,TIMER_INT_UP);
				}
					
				//	LED0=!LED0;
    }
}
