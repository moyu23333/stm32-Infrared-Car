#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdint.h"
#include "stdio.h"
uint8_t RG_FLAG=0;//ֱ�Ǳ�־λ
void SystemClock_Config(void);
//�����ض���
int fgetc(FILE *f) {      
uint8_t ch = 0;
HAL_UART_Receive(&huart1,&ch,1,0xffff);
return ch;
}
int fputc(int ch, FILE *f) {      
HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xffff);
return ch;
}
//������ƺ���
//���ܣ����Ƶ������ת���ٶ�
//duty1�����Ƶ��1��2 duty2:���Ƶ��3��4
void CAR_GO(int16_t duty1,int16_t duty2)
{	
	if(duty1>8000)duty1=8000;//�Ե����ռ�ձȽ����޷�����ֹת�ٹ����ջ����
	else if(duty1<-8000)duty1=-8000;
	if(duty2>8000)duty2=8000;
	else if(duty2<-8000)duty2=-8000;
	if(duty1<0)//�ж������������
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,-duty1);//���1��2��ת��ͨ��3��PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	}
	else
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);//���1��2��ת��ͨ��2��PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,duty1);
	}
	
	if(duty2<0)//�ж������������
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,-duty2);//���3��4��ת��ͨ��3��PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
	}
	else
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0); //���3��4��ת��ͨ��2��PWM
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,duty2);
	}
}
//����ѭ��
void SearchRun(void)
{
	if(SEARCH_L_IO == WHITE_AREA && SEARCH_R_IO == WHITE_AREA)//�����Һ������������ֱ��
	{
		CAR_GO(5000,5000);
//		HAL_Delay(20);
	}
	else if (SEARCH_L_IO == BLACK_AREA && SEARCH_R_IO == WHITE_AREA)//������������������ת
	{
		CAR_GO(-5000,5000);
	}
	else if (SEARCH_R_IO == BLACK_AREA && SEARCH_L_IO == WHITE_AREA)//���Һ��������������ת
	{
	CAR_GO(5000,-5000);	
	}
    if(SEARCH_R_IO == BLACK_AREA && SEARCH_L_IO == BLACK_AREA&&SEARCH_M_IO==WHITE_AREA)//˫ֱ��
	{
	CAR_GO(5000,5000);
	}
	if(SEARCH_R_IO == BLACK_AREA && SEARCH_L_IO == BLACK_AREA&&SEARCH_M_IO==BLACK_AREA)//ʮ��
	{
	 CAR_GO(5000,5000);
	}
	if(SEARCH_R_IO == BLACK_AREA && SEARCH_L_IO == WHITE_AREA&&SEARCH_M_IO==WHITE_AREA&&RG_FLAG==0)//��ֱ��
	{

			CAR_GO(5000,-5000);	
			HAL_Delay(500);
			RG_FLAG=1;

	}
	else if(SEARCH_R_IO == WHITE_AREA && SEARCH_L_IO == BLACK_AREA&&SEARCH_M_IO==WHITE_AREA&&RG_FLAG==0)//��ֱ��
	{

			CAR_GO(-5000,5000);	
			HAL_Delay(500);
			RG_FLAG=1;

	}
}
//��������ࣨ������ִ�з�ʽ��
uint16_t Measure(void)
{
	uint16_t start_time=0;
	uint16_t end_time=0;
	uint16_t time_val=0;
	__HAL_TIM_SET_COUNTER(&htim6,0);//�����ʱ������
	HAL_TIM_Base_Start(&htim6);//������ʱ��
	//��������������
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);
	//�ȴ������������ź�
	while(HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin)==0);
	start_time=__HAL_TIM_GET_COUNTER(&htim6);//��ʼ��ʱ
	while(HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin)==1);
	end_time=__HAL_TIM_GET_COUNTER(&htim6);//��ȡ����������ʱ��
	HAL_TIM_Base_Stop(&htim6);//ֹͣ��ʱ
	time_val=time_val+(end_time-start_time)*34/2000;//�������
	return time_val;
}
//����������
void Ultra_avoid(void)
{
	if(Measure()<13)
	{
		CAR_GO(0,0);//����
		HAL_Delay(300);
		CAR_GO(-6000,6000);//��ת
		HAL_Delay(500);
		CAR_GO(5000,5000);//ֱ��
		HAL_Delay(1200);		
		CAR_GO(6000,-6000);//��ת
		HAL_Delay(700);
		CAR_GO(5000,5000);//ֱ��
		while(SEARCH_L_IO == WHITE_AREA)
		;
		CAR_GO(-6000,6000);//��ת
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
  //�������PWM��ʼ��	
  TIM_PWM_Int(&htim4,TIM_CHANNEL_1,1,0);
  TIM_PWM_Int(&htim4,TIM_CHANNEL_2,1,0);  
  TIM_PWM_Int(&htim4,TIM_CHANNEL_3,1,0);  
  TIM_PWM_Int(&htim4,TIM_CHANNEL_4,1,0);  
  while (1)
  {
	SearchRun();//����ѭ��ģ��
	Ultra_avoid();//����������
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
