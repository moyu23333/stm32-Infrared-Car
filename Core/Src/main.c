#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdint.h"
#include "stdio.h"
uint8_t RG_FLAG=0;//直角标志位
void SystemClock_Config(void);
//串口重定向
int fgetc(FILE *f) {      
uint8_t ch = 0;
HAL_UART_Receive(&huart1,&ch,1,0xffff);
return ch;
}
int fputc(int ch, FILE *f) {      
HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xffff);
return ch;
}
//电机控制函数
//功能：控制电机正反转及速度
//duty1：控制电机1，2 duty2:控制电机3，4
void CAR_GO(int16_t duty1,int16_t duty2)
{	
	if(duty1>8000)duty1=8000;//对电机的占空比进行限幅，防止转速过大烧坏电机
	else if(duty1<-8000)duty1=-8000;
	if(duty2>8000)duty2=8000;
	else if(duty2<-8000)duty2=-8000;
	if(duty1<0)//判读输入参数正负
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,-duty1);//电机1、2反转，通道3出PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	}
	else
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);//电机1、2正转，通道2出PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,duty1);
	}
	
	if(duty2<0)//判读输入参数正负
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,-duty2);//电机3、4反转，通道3出PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
	}
	else
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0); //电机3、4正转，通道2出PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,duty2);
	}
}
//红外循迹
void SearchRun(void)
{
	if(SEARCH_L_IO == WHITE_AREA && SEARCH_R_IO == WHITE_AREA)//若左右红外读到白线则直行
	{
		CAR_GO(5000,5000);
//		HAL_Delay(20);
	}
	else if (SEARCH_L_IO == BLACK_AREA && SEARCH_R_IO == WHITE_AREA)//若左红外读到白线则左转
	{
		CAR_GO(-5000,5000);
	}
	else if (SEARCH_R_IO == BLACK_AREA && SEARCH_L_IO == WHITE_AREA)//若右红外读到白线则右转
	{
	CAR_GO(5000,-5000);	
	}
    if(SEARCH_R_IO == BLACK_AREA && SEARCH_L_IO == BLACK_AREA&&SEARCH_M_IO==WHITE_AREA)//双直线
	{
	CAR_GO(5000,5000);
	}
	if(SEARCH_R_IO == BLACK_AREA && SEARCH_L_IO == BLACK_AREA&&SEARCH_M_IO==BLACK_AREA)//十字
	{
	 CAR_GO(5000,5000);
	}
	if(SEARCH_R_IO == BLACK_AREA && SEARCH_L_IO == WHITE_AREA&&SEARCH_M_IO==WHITE_AREA&&RG_FLAG==0)//右直角
	{

			CAR_GO(5000,-5000);	
			HAL_Delay(500);
			RG_FLAG=1;

	}
	else if(SEARCH_R_IO == WHITE_AREA && SEARCH_L_IO == BLACK_AREA&&SEARCH_M_IO==WHITE_AREA&&RG_FLAG==0)//左直角
	{

			CAR_GO(-5000,5000);	
			HAL_Delay(500);
			RG_FLAG=1;

	}
}
//超声波测距（主程序执行方式）
uint16_t Measure(void)
{
	uint16_t start_time=0;
	uint16_t end_time=0;
	uint16_t time_val=0;
	__HAL_TIM_SET_COUNTER(&htim6,0);//清除定时器计数
	HAL_TIM_Base_Start(&htim6);//启动定时器
	//启动超声波发送
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);
	//等待超声波返回信号
	while(HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin)==0);
	start_time=__HAL_TIM_GET_COUNTER(&htim6);//开始计时
	while(HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin)==1);
	end_time=__HAL_TIM_GET_COUNTER(&htim6);//获取超声波返回时间
	HAL_TIM_Base_Stop(&htim6);//停止计时
	time_val=time_val+(end_time-start_time)*34/2000;//计算距离
	return time_val;
}
//超声波避障
void Ultra_avoid(void)
{
	if(Measure()<13)
	{
		CAR_GO(0,0);//后退
		HAL_Delay(300);
		CAR_GO(-6000,6000);//左转
		HAL_Delay(500);
		CAR_GO(5000,5000);//直行
		HAL_Delay(1200);		
		CAR_GO(6000,-6000);//右转
		HAL_Delay(700);
		CAR_GO(5000,5000);//直行
		while(SEARCH_L_IO == WHITE_AREA)
		;
		CAR_GO(-6000,6000);//左转
		HAL_Delay(350);
	}		
}
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  //电机驱动PWM初始化	
  TIM_PWM_Int(&htim4,TIM_CHANNEL_1,1,0);
  TIM_PWM_Int(&htim4,TIM_CHANNEL_2,1,0);  
  TIM_PWM_Int(&htim4,TIM_CHANNEL_3,1,0);  
  TIM_PWM_Int(&htim4,TIM_CHANNEL_4,1,0);  
  while (1)
  {
	SearchRun();//红外循迹模块
	Ultra_avoid();//超声波避障
  }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
