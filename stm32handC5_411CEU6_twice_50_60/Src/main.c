/* USER CODE BEGIN Header */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sys.h"
#include "crc.h"
#include "usart.h"
#include "gpio.h"
#include "app_x-cube-ai.h"
#include "stdio.h"
#include "adc.h"
#include "timer.h"
#include "math.h" 
#include "arm_math.h" 
#include "delay.h"
#include "led.h"

#define FFT_LENGTH		128 		//FFT长度
extern s16 AD_Value[868];   //直流有负数且不超过256 1024
extern u8 mea;
extern u8 stopflag;
ai_u16 t1[50][75]={0};
ai_u16 t[50]= {0} ;
ai_float in_data[3000]={0};
ai_float out_data1[4];
ai_float out_data2[4];
//ai_float out_data3[4];
const float threshold=0.99f;
const float threshold2=0.9f;
const float threshold3=0.95f;

ai_u8 upordownbright=0;//up:0;down:1
ai_u8 upordowncolor=0;//up:0;down:1
ai_u8 turnonflag=0;
//ai_u8 notawake=0;//notawake:0;awake:1
//ai_u8 notturnon=0;//notturnon:0~2;turnon:3
ai_u8 turnoncnt=0;
ai_u16 sleepcnt=0;
//u8 status=0,Key=0,Key1=0;

//u16 t2[60] ={0};
ai_u8 sleeporwake=0;//sleep:0;wake:1
ai_u8 offon=1;//off:0;on:1
//u16 led0pwmval=0;
//u16 led1pwmval=0;
//ai_i16 ledpwmval[2]={0};
ai_u8 lightNo=0;
ai_bool keyhot=false;
//ai_u8 dir=0;
ai_i16 brightnessvalue=0;
ai_u8 led01ratio[2]={0,100};
extern u8 mea;

void turnbright2()//return 
{
	if(upordownbright==0)//up
		brightnessvalue+=200;//3dang	100 300 500
	else
		brightnessvalue-=200;	
				//		led1pwmval=0;									

						
	if(brightnessvalue>500)		
	{
		upordownbright = 1;//down
		brightnessvalue=500;
	}
	if(brightnessvalue<100)
	{
		upordownbright = 0;//up
		brightnessvalue=100;

	}
	TIM_SetTIM4Compare1(round(brightnessvalue*led01ratio[0]/100));
	TIM_SetTIM10Compare1(round(brightnessvalue*led01ratio[1]/100));	
	

}

void switchcolor(ai_u8 gearnum)//gearnum dangshu 
{

	if(upordowncolor==0)
	{
		led01ratio[0]=led01ratio[0]+100/gearnum;
		led01ratio[1]=led01ratio[1]-100/gearnum;
	}
	else if(upordowncolor==1)
	{
		led01ratio[1]=led01ratio[1]+100/gearnum;
		led01ratio[0]=led01ratio[0]-100/gearnum;
	}
	if(led01ratio[0]==100 && led01ratio[1]==0)
		upordowncolor=1;
	else if(led01ratio[1]==100 && led01ratio[0]==0)
		upordowncolor=0;
	
	TIM_SetTIM4Compare1(round(brightnessvalue*led01ratio[0]/100));
	TIM_SetTIM10Compare1(round(brightnessvalue*led01ratio[1]/100));	
}

void fft256( ai_u8 a)
{
	float fft_inputbuf[FFT_LENGTH*4]={0.0};
	float fft_outputbuf[FFT_LENGTH*2];	//FFT输入数组//FFT输出数组 
//	static ai_u16 fft_output[60];
	ai_i16 AD_FFT[256] = {0};
	arm_cfft_radix2_instance_f32 scfft;
	arm_cfft_radix2_init_f32(&scfft,256,0,1);//初始化scfft结构体，设定FFT相关参数

	const float hamming_TAB1[128]={0.08,0.080562849,0.082250017,0.085057376,0.088978056,0.094002462,0.100118299,0.1073106,0.115561765,0.124851601,0.135157375,
		0.146453867,0.158713432,0.171906069,0.185999493,0.200959216,0.216748629,0.233329092,0.25066003,0.268699032,0.287401952,0.306723023,0.326614961,0.347029088,
		0.367915447,0.389222926,0.410899381,0.432891768,0.455146266,0.477608416,0.500223248,0.522935421,0.545689353,0.568429363,0.591099802,0.61364519,0.636010357,
		0.65814057,0.679981673,0.701480217,0.722583593,0.743240156,0.763399356,0.78301186,0.802029674,0.820406257,0.838096639,0.855057529,0.87124742,0.886626693,
		0.901157713,0.914804919	,0.927534914,0.939316547,0.950120985,0.959921789,0.968694973,0.976419069,0.983075175,0.988647001,0.993120913,0.996485962,0.998733914,
		0.999859266,0.999859266,0.998733914,0.996485962,0.993120913,0.988647001,0.983075175,0.976419069,0.968694973,0.959921789,0.950120985,0.939316547,0.927534914,
		0.914804919,0.901157713,0.886626693,0.87124742,0.855057529,0.838096639,0.820406257,0.802029674,0.78301186,0.763399356,0.743240156,0.722583593,0.701480217,
		0.679981673,0.65814057,0.636010357,0.61364519,0.591099802,0.568429363,0.545689353,0.522935421,0.500223248,0.477608416,0.455146266,0.432891768,0.410899381,
		0.389222926,0.367915447,0.347029088,0.326614961,0.306723023,0.287401952,0.268699032,0.25066003,0.233329092,0.216748629,0.200959216,0.185999493,0.171906069,
		0.158713432,0.146453867,0.135157375,0.124851601,0.115561765,0.1073106,0.100118299,0.094002462,0.088978056,0.085057376,0.082250017,0.080562849,0.08};
	
	
	for(ai_u8 b=0;b<128;b++)
	{
		AD_FFT[b] = AD_Value[10*a+b]*hamming_TAB1[b];
		fft_inputbuf[2*b] = (float)AD_FFT[b];
		fft_inputbuf[2*b+1]=0.0;//虚部全部为0
	//  printf("%d \r",AD_FFT[b] );
	}
//	for(ai_u8 b=128;b<=255;b++)
//	{
//		fft_inputbuf[2*b] =0.0;
//		fft_inputbuf[2*b+1]=0.0;//虚部全部为0
//	}
	__HAL_TIM_SET_COUNTER(&TIM3_Handler,0);//重设TIM3定时器的计数器值

	arm_cfft_radix2_f32(&scfft,fft_inputbuf);	//FFT计算（基4）
//  time=__HAL_TIM_GET_COUNTER(&TIM3_Handler)+(u32)Timer_Count*500;//计算所用时间
	//sprintf((char*)buf,"%0.3fms\r\n",time/1000);	
	arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,256);	//;, 
	for(ai_u8 i=0;i<50;i++)
		t[i]=(ai_u16)fft_outputbuf[i+10];
//			return fft_output;
}
void measure_fft(void)
{
	
//		arm_cfft_radix2_instance_f32 scfft;
//		if(keyhot==true)
//		{
//			keyhot=false;
//			return;
//		}
	  s16 max=0, min = 0,cut;
    float s;		
    for(ai_u8 a=0;a<75;a++)//90
		{
			fft256(a);
			for(ai_u8 i=0;i<50;i++)
			{
				t1[i][a] = t[i];
			}
		}
		
//quant to 0~255
		min = t1[0][0];
    max = t1[0][0];
		for(ai_u8 c=0;c<75;c++)//90
		{
			for(ai_u8 i=0;i<50;i++)
			{ 
					if(t1[i][c] > max)
					{
							max = t1[i][c] ;	
					}
					if( min > t1[i][c] )
					{
						  min = t1[i][c] ;	
					}
				
			}
					
		}
	 cut = max -min;
//	  printf("%d %d %d \n" ,max,min,cut);
//    printf(" \n");
	 for(ai_u8 c=0;c<75;c++)//90
	 {
		 for(ai_u8 i=0;i<50;i++)
		 {
				s = t1[i][c] - min;
				s*=255;
				s = s/cut;
				t1[i][c] = s ;
		 }
			
	//		printf("%d ", t1[i]);
		
	}
	 
		
//classify by ai
	//	printf("END60*90  \n");
		for(ai_u8 i=0;i<50;i++)
		{
			for(ai_u8 c=0;c<60;c++)
			{
				in_data[i*60+c] = t1[i][c]; 
			}
		  	
		}
		aiRun(in_data, out_data1);
		
		
		
		for(ai_u8 i=0;i<50;i++)
		{
			for(ai_u8 c=15;c<75;c++)
			{
				in_data[i*60+(c-15)] = t1[i][c]; 
			}
		  	
		}
		aiRun(in_data, out_data2);
		
		
		
//		for(ai_u8 i=0;i<60;i++)
//		{
//			for(ai_u8 c=30;c<90;c++)
//			{
//				in_data[i*60+(c-30)] = t1[i][c]; 
//			}
//		  	
//		}
//		aiRun(in_data, out_data3);
		
		
//		printf("out_data [%.2f %.2f %.2f %.2f ] | ", (out_data1[0]),
//																									(out_data1[1]),
//																									(out_data1[2]),
//																									(out_data1[3]));
//		printf("out_data [%.2f %.2f %.2f %.2f ] | ", (out_data2[0]),
//																									(out_data2[1]),
//																									(out_data2[2]),
//																									(out_data2[3]));	
//		printf("out_data [%.2f %.2f %.2f %.2f ]\r\n", (out_data3[0]),
//																									(out_data3[1]),
//																									(out_data3[2]),
//																									(out_data3[3]));



			
  if(offon==1)//on
	{
		if(!(out_data1[1]>threshold && out_data2[1]>threshold) && !(out_data1[2]>threshold && out_data2[2]>threshold)&&
			!(out_data1[3]>threshold && out_data2[3]>threshold))//nomove
		{
			if(sleeporwake==1)//wake up level
			{
				sleepcnt++;
//				sleeporwake--;
				if(sleepcnt>=5)//resleep
				{
					sleepcnt=0;
					sleeporwake=0;
				}
			}	
			
			if(sleeporwake==2)//fully awake level
			{
				sleepcnt++;
				if(sleepcnt>10)
				{
					sleeporwake=0;//sleep
					sleepcnt=0;
//					notawake=0;
				}
			}
			return;
		}
		
		
		if(sleeporwake>=1)//wake
		{
			//leftright light
//			ai_float flag=out_data1[1]-0.8f; // && out_data2[1]>threshold && out_data3[1]>threshold 
//			if(keyhot==true)
//			{
//				keyhot=false;
//				return;
//			}
			if(out_data1[1]>threshold && out_data2[1]>threshold)//ture brightness value 
			{
				turnbright2();
//				notawake=1;
				Beepone();
				sleeporwake=2;//fully awake
				keyhot=true;
				stopflag=1;
//				printf("bright [%d %d]\r\n",(int)round(brightnessvalue*led01ratio[0]/100),(int)round(brightnessvalue*led01ratio[1]/100));
			}	
			else if(out_data1[2]>threshold3 && out_data2[2]>threshold3)//switch light No.
			{
				switchcolor(2);
//				notawake=1;
				Beeptwo();
//				keyhot=true;
//				printf("color [%d %d]\r\n",(int)round(brightnessvalue*led01ratio[0]/100),(int)round(brightnessvalue*led01ratio[1]/100));
				sleeporwake=2;//fully awake
				stopflag=2;

			}
			else if(out_data1[3]>threshold2 && out_data2[3]>threshold2)//turn on light
			{

				if(sleeporwake==10)
				{
					sleeporwake=2;
					return;
				}
				brightnessvalue=0;
				led01ratio[0]=50;
				led01ratio[1]=50;
				TIM_SetTIM4Compare1(round(brightnessvalue*led01ratio[0]/100));
				TIM_SetTIM10Compare1(round(brightnessvalue*led01ratio[1]/100));
				
				turnonflag=0;
				offon=0;
//				printf("turnoff\r\n");
//				notawake=1;
//				keyhot=true;


				Beeplong();
				sleeporwake=2;//fully awake

			}
		}
		else if(sleeporwake==0)//sleep
		{
			if(offon==0)
				return;
			if(out_data1[1]>threshold && out_data2[1]>threshold)//ture brightness value
			{
				sleeporwake++;//wake up first
//				printf("wait\r\n");

				return;
			}
			else if(out_data1[2]>threshold3 && out_data2[2]>threshold3)//switch light No.
			{
				sleeporwake++;//wake up first
//				printf("wait\r\n");

				return;
			}			
			else if(out_data1[3]>threshold2 && out_data2[3]>threshold2)//turn off light
			{
				sleeporwake++;//wake up first
				turnonflag=0;
//				printf("wait\r\n");
				Beeplong();

				return;
			}
		}
	}
	else//off to on
	{
		if(!(out_data1[1]>threshold && out_data2[1]>threshold ) && !(out_data1[2]>threshold && out_data2[2]>threshold )&&
			!(out_data1[3]>threshold && out_data2[3]>threshold))//nomove
		{
			if(turnonflag==1)
			{
				turnoncnt++;
				if(turnoncnt>=5)
				{
					turnoncnt=0;
					turnonflag=0;
				}
			}
			return;
		}
		if(out_data1[3]>threshold2 && out_data2[3]>threshold2)//turn on light
		{
			if(turnonflag<2)
			{
				turnonflag++;//wake up first
				if(turnonflag<2)
				{
					Beeplong();
//					printf("wait turn on\r\n");
				}
				else
				{
					brightnessvalue=250;
					led01ratio[0]=50;
					led01ratio[1]=50;
					TIM_SetTIM4Compare1(round(brightnessvalue*led01ratio[0]/100));
					TIM_SetTIM10Compare1(round(brightnessvalue*led01ratio[1]/100));	

					offon=1;
					Beeplong();
//					printf("turnon\r\n");

					turnoncnt=0;
					turnonflag=0;
					sleeporwake=10;//just turn on the light,can turn brightness,can switch color,can not shutdown right now
				}
			}
		}
	}
		
	
}


int main(void)
{
 
  HAL_Init();
	Stm32_Clock_Init(200,25,2,7);
	
  delay_init(100);               	//初始化延时函数
	MY_ADC_Init();
	LED_Init();		//初始化LED	 
	TIM3_Init(1000-1,50-1);        //1Mhz计数频率,最大计时500US左右超出  //1000-1  1ms
	TIM10_PWM_Init(500-1,50-1);    	//84M/84=1M的计数频率，自动重装载为500，那么PWM频率为1M/500=2kHZ  500-1
	TIM_SetTIM10Compare1(0); 	
	
	TIM4_PWM_Init(500-1,50-1);    	//84M/84=1M的计数频率，自动重装载为500，那么PWM频率为1M/500=2kHZ  500-1
	TIM_SetTIM4Compare1(0); 	
	
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_X_CUBE_AI_Init();
//  printf("test\r\n");
	
  while (1)
  {
			if(mea == 1)
			{
				 measure_fft();
         mea = 0;				
			}
	}

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
