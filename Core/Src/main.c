/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.14159265
#define f0 0.1
#define f1 10
#define f_sample 100
#define T_run 20
typedef union {
  float floatValue;
  uint8_t byteArray[4];
} FloatByteArray;


 uint8_t uart_buff[100];
BPNN_PID_PARAMETER_Struct YAW_BPNN;
BPNN_PID_PARAMETER_Struct PITCH_BPNN;
volatile uint32_t limit_pitch=0;
volatile FloatByteArray PWM;
volatile FloatByteArray SPEED;
volatile FloatByteArray POSITION;
volatile FloatByteArray omegaX;
volatile FloatByteArray omegaY;
volatile FloatByteArray omegaZ;
volatile FloatByteArray ROLL;
volatile FloatByteArray PITCH;
volatile FloatByteArray YAW;
volatile FloatByteArray KP;
volatile FloatByteArray KI;
volatile FloatByteArray KD;
volatile FloatByteArray CAMERA_YAW_ERR;
volatile 	float value_output[9];
 uint8_t trans_array[34];
volatile double_t k=0;
volatile  double sweap=0;

volatile float pitch_SpeedSet, pitch_PositionSet;
volatile float pitch_ek, pitch_ek_1, pitch_ek_2, pitch_uk, pitch_uk_1, pitch_Kp=0.000285, pitch_Ki=0.08, pitch_Kd;
volatile float pitch_cam_ek, pitch_cam_ek_1, pitch_cam_ek_2, pitch_cam_uk, pitch_cam_uk_1, pitch_cam_Kp=1.5, pitch_cam_Ki=0.0000010, pitch_cam_Kd= 0.0000001;

volatile float yaw_SpeedSet, PositionSet;
volatile float yaw_ek, yaw_ek_1, yaw_ek_2, yaw_uk, yaw_uk_1, yaw_Kp=0.000285, yaw_Ki=0.08, yaw_Kd;
volatile float yaw_cam_ek, yaw_cam_ek_1, yaw_cam_ek_2, yaw_cam_uk, yaw_cam_uk_1, yaw_cam_Kp=1.4, yaw_cam_Ki=0.0000001, yaw_cam_Kd= 0.000000001;

volatile float ek, ek_1, ek_2, uk, uk_1, Kp=12, Ki=0, Kd=0.05;

uint8_t SPEED_CONTROL = 0;
uint8_t TRACKING_CONTROL = 0;
uint8_t PID_CONTROL = 0;
uint8_t	BPNN_CONTROL = 1;
float T = 0.002;
uint8_t EN_ON = 0;
uint8_t count_PD = 0;
int16_t pitch_err,yaw_err;

uint8_t step_counter = 0;
double prev_val,val = 0;
float speed_set_prev_1 = 0;
float speed_set_prev_2 = 0;



//float y_prev = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float lowpass_filter(float u,uint8_t Axis) {
	float ye;
	if(Axis ==1)
	{
    ye = (2.5 / 3.5) * speed_set_prev_1 + (1.0 / 3.5) * u;
//    float ye = (3.0 / 4.0) * speed_set_prev + (1.0 / 4.0) * u;
//    ye = (0.85) * speed_set_prev_1 + (0.15) * u;
    speed_set_prev_1 = ye;
	}
	else
	{
	ye = (2.5 / 3.5) * speed_set_prev_2 + (1.0 / 3.5) * u;
//    float ye = (3.0 / 4.0) * speed_set_prev + (1.0 / 4.0) * u;
//    ye = (0.85) * speed_set_prev_2 + (0.15) * u;
	speed_set_prev_2 = ye;
	}
    return ye;
}

char ngu[] = "ALAHU";
void MOTORDRIVE(uint16_t MOTOR,float PWM,uint8_t DIRECT);
void USB_HANDLER(uint8_t *buff,uint32_t *len)
{
	TRACKING_CONTROL =1;
	yaw_err = (int16_t)(buff[2] << 8 | buff[1]);
	pitch_err = (int16_t)(buff[4] << 8 | buff[3]);
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
 //   if (htim->Instance == TIM6) // Kiểm tra nếu là timer 2
 //   {
//		HAL_GPIO_TogglePin(DIR1_GPIO_Port,DIR1_Pin);
//     	sweap = sin(2 *	pi * 0.1 * 20 * (pow(100, (double)(k) / (100 * 20)) - 1) / log(100));
//      	PWM.floatValue = sweap;
//      	k=k+1;
////		if(k == 2001)
////		{
////			MOTORDRIVE(TIM8_CHANNEL3_PWM1_Pin,0,0);
////			//HAL_TIM_Base_Stop(&htim6);
////		}
////		else if(sweap>=0)
////		{
////			MOTORDRIVE(TIM8_CHANNEL3_PWM1_Pin,sweap,1);
////		}
////		else if (sweap<0)
////		{
////			MOTORDRIVE(TIM8_CHANNEL3_PWM1_Pin,(-sweap),0);
////		}
//		HAL_GPIO_TogglePin(DIR1_GPIO_Port,DIR1_Pin);
//	}
//	HAL_UART_Transmit(&huart6, PWM.byteArray,4,100);
//}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	int32_t value_raw;
	float value_convert;
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	  if (huart->Instance == USART3)
	  {
		  if(uart_buff[0] == 10)
		  {
			  if(uart_buff[70] == 13)
			  {
				  for (uint8_t byte_counter = 1; byte_counter <= 9; byte_counter ++)
				  {
					  value_raw 	= 0;
					  value_convert = 0;
					  if (byte_counter <= 6)
					  {
						  uint8_t start_position = (byte_counter - 1) * 8 + 1;
						  uint8_t stop_position = start_position + 6;
						  int8_t sign_flag = 0;
						  if (uart_buff[start_position] == 45)
						  {
							  sign_flag = -1;
						  }
						  else
						  {
							  sign_flag = 1;
						  }
						  start_position ++;
						  while (start_position <= stop_position)
						  {
							  value_raw = value_raw * 10 + (uart_buff[start_position] - 48);
							  start_position ++;
						  }
						  value_convert = value_raw * sign_flag;
						  value_convert = value_convert * 0.001;
					  }
					  else
					  {
						  uint8_t start_position = (byte_counter - 1) * 7 + 1 + 6;
						  uint8_t stop_position = start_position + 5;
						  int8_t sign_flag = 0;
						  if (uart_buff[start_position] == 45)
						  {
							  sign_flag = -1;
						  }
						  else
						  {
							  sign_flag = 1;
						  }
						  start_position ++;
						  while (start_position <= stop_position)
						  {
							  value_raw = value_raw * 10 + (uart_buff[start_position] - 48);
							  start_position ++;
						  }
						  value_convert = value_raw * sign_flag;
						  value_convert = value_convert * 0.001;
					  }
					  value_output[byte_counter - 1] = value_convert;
				  }
				  SPEED_CONTROL = 1;
				  EN_ON =1;
			  }
			  else
			  {
				  EN_ON =0;
			  }
		  }
		  else
		  {
			  EN_ON =0;
		  }
	  }

	  ROLL.floatValue=value_output[0];
	  PITCH.floatValue=value_output[1];
	  YAW.floatValue=value_output[2];
	  omegaX.floatValue=value_output[3];
	  omegaY.floatValue=value_output[4];
	  omegaZ.floatValue=value_output[5];

//	  PID_CONTROL = 1;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_buff, 71);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);

}

void MOTORDRIVE(uint16_t MOTOR,float PWM,uint8_t DIRECT)
{
	/*nho xu ly de khi chay toi han thi dung dong co lai*/
	if(MOTOR == TIM8_CHANNEL3_PWM1_Pin)
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,(uint16_t)(PWM*8400));
		HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,DIRECT);
	}
	if(MOTOR == TIM2_CHANNEL2_PWM2_Pin)
	{
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,(uint16_t)(PWM*4200));
		HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,DIRECT);
	}
}
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	/*test cam bien gimbal*/
//	switch(GPIO_Pin)
//	{
//		case GPIO_PIN_0://home
//		{
//			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0))
//			{
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);//GREEN
//			}
//			else
//			{
//				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
//			}
//		}
//		case GPIO_PIN_2://doc
//		{
//			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2))
//			{
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);//ORGRANGE
//				MOTORDRIVE(TIM8_CHANNEL3_PWM1_Pin,0,1);
//				limit_pitch =1;
//			}
//			else
//			{
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//			}
//		}
//		case GPIO_PIN_3://ngang
//		{
//			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3))
//			{
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);//RED
//				MOTORDRIVE(TIM8_CHANNEL3_PWM1_Pin,0,1);
//				limit_pitch =1;
//			}
//			else
//			{
//				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//			}
//		}
//		default: break;
//	}
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  BPNN_INIT(&YAW_BPNN,1);
  BPNN_INIT(&PITCH_BPNN,0);
//  MOTORDRIVE(TIM8_CHANNEL3_PWM1_Pin,0.1,0);
  uint8_t HAHA[1];
  HAHA[0]=0;
  HAL_TIM_Base_Start_IT(&htim6);
  while(HAHA[0]!=13)
  {
	  HAL_UART_Receive(&huart3, HAHA, 1, 10);
  }
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart_buff, 71);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
//	EN_ON=1;
//	HAL_UART_Transmit_DMA(&huart6, "$BRATE 2",8 );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{

	  if(BPNN_CONTROL)
	  {
	  if(TRACKING_CONTROL)
	  {
			if(EN_ON)
			{
				yaw_cam_ek_2 	= yaw_cam_ek_1;
				yaw_cam_ek_1 	= yaw_cam_ek;
				yaw_cam_ek 		= yaw_err*0.1 ;
				yaw_cam_uk_1	= yaw_cam_uk;
				yaw_cam_uk		= yaw_cam_uk_1 + yaw_cam_Kp*(yaw_cam_ek - yaw_cam_ek_1) + yaw_cam_Ki*(T/2)*(yaw_cam_ek + yaw_cam_ek_1) + yaw_cam_Kd/T*(yaw_cam_ek - 2*yaw_cam_ek_1 + yaw_cam_ek_2);
/*
				if(yaw_cam_uk >=5)
				{
					yaw_cam_uk =5;
				}
				if(yaw_cam_uk <=-5)
				{
					yaw_cam_uk =-5;
				}
*/
				pitch_cam_ek_2 		= pitch_cam_ek_1;
				pitch_cam_ek_1 		= pitch_cam_ek;
				pitch_cam_ek 		= pitch_err*0.1;
				pitch_cam_uk_1		= pitch_cam_uk;
				pitch_cam_uk		= pitch_cam_uk_1 + pitch_cam_Kp*(pitch_cam_ek - pitch_cam_ek_1) + pitch_cam_Ki*(T/2)*(pitch_cam_ek + pitch_cam_ek_1) + pitch_cam_Kd/T*(pitch_cam_ek - 2*pitch_cam_ek_1 + pitch_cam_ek_2);

			}
	  }
	  else{
		  	  yaw_cam_uk   = 0;
		      yaw_cam_uk_1 = 0;
		      yaw_cam_ek   = 0;
		      yaw_cam_ek_1 = 0;
		      yaw_cam_ek_2 = 0;

		      pitch_cam_uk   = 0;
		      pitch_cam_uk_1 = 0;
		      pitch_cam_ek   = 0;
		      pitch_cam_ek_1 = 0;
		      pitch_cam_ek_2 = 0;
	  	  }

	  if(SPEED_CONTROL==1)
	  {
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
		trans_array[0] = 10;
		trans_array[1] = omegaY.byteArray[0];
		trans_array[2] = omegaY.byteArray[1];
		trans_array[3] = omegaY.byteArray[2];
		trans_array[4] = omegaY.byteArray[3];

//		trans_array[17] = PITCH.byteArray[0];
//		trans_array[18] = PITCH.byteArray[1];
//		trans_array[19] = PITCH.byteArray[2];
//		trans_array[20] = PITCH.byteArray[3];

//		trans_array[1] = omegaZ.byteArray[0];
//		trans_array[2] = omegaZ.byteArray[1];
//		trans_array[3] = omegaZ.byteArray[2];
//		trans_array[4] = omegaZ.byteArray[3];

//		CAMERA_YAW_ERR.floatValue = yaw_err;

//		trans_array[17] = YAW.byteArray[0];
//		trans_array[18] = YAW.byteArray[1];
//		trans_array[19] = YAW.byteArray[2];
//		trans_array[20] = YAW.byteArray[3];

//		trans_array[17] = CAMERA_YAW_ERR.byteArray[0];
//		trans_array[18] = CAMERA_YAW_ERR.byteArray[1];
//		trans_array[19] = CAMERA_YAW_ERR.byteArray[2];
//		trans_array[20] = CAMERA_YAW_ERR.byteArray[3];

		trans_array[33] = 13;

		// PID pitch

	//	PositionSet = 30*sin(2 *pi *k/100);
	//	PositionSet=0;
		POSITION.floatValue = pitch_err;
//		POSITION.floatValue = yaw_err;
		trans_array[13] = POSITION.byteArray[0];
		trans_array[14] = POSITION.byteArray[1];
		trans_array[15] = POSITION.byteArray[2];
		trans_array[16] = POSITION.byteArray[3];



	if(YAW.floatValue>=70)
		{
			EN_ON = 0;
		}
		if(YAW.floatValue<=-70)
		{
			EN_ON = 0;
		}
		if(PITCH.floatValue>=80)
		{
			EN_ON = 0;
		}
		if(PITCH.floatValue<=-80)
		{
			EN_ON = 0;
		}

if(EN_ON)
		{
		k=k+1;
//		if(k >=10)
//		{
//			ek_2 		= ek_1;
//			ek_1 		= ek;
//			ek 			= 0 - PITCH.floatValue;
//			uk_1		= uk;
//			uk			= uk_1 + Kp*(ek - ek_1) + Ki*(T/2)*(ek + ek_1) + Kd/T*(ek - 2*ek_1 + ek_2);
//		k=0;
//		}

//		pitch_SpeedSet = 10*sin(2*pi*k*0.002);
/*		val = sin(2*pi*k*0.004);
		if (prev_val < 0 && val >= 0)
		{
			step_counter = (step_counter + 1) % 4;  // 4 pha: 0 → 1 → 2 → 3 → 0 ...
		}

		prev_val = val;
		float temp_speed_set = 0;

		// Gán giá trị theo pha
		if (step_counter == 0)
			temp_speed_set = 30;
		else if (step_counter == 1)
			temp_speed_set = 0;
		else if (step_counter == 2)
			temp_speed_set = -30;
		else
			temp_speed_set = 0;
*/

//		pitch_SpeedSet = lowpass_filter(temp_speed_set,0);
		pitch_SpeedSet = lowpass_filter(pitch_cam_uk,0);
//		pitch_SpeedSet= val;
//		pitch_SpeedSet = uk;

/*		SPEED.floatValue = pitch_SpeedSet;
		trans_array[9] = SPEED.byteArray[0];
		trans_array[10] = SPEED.byteArray[1];
		trans_array[11] = SPEED.byteArray[2];
		trans_array[12] = SPEED.byteArray[3];
*/
//		float filtered_omegaY = lowpass_filter(omegaY.floatValue, 0);
		PITCH_BPNN.BP.y[1]=PITCH_BPNN.BP.y[0];
		PITCH_BPNN.BP.y[0]=omegaY.floatValue*pi/180;

		PITCH_BPNN.PID.u[1]=PITCH_BPNN.PID.u[0];

		PITCH_BPNN.In.e[2]=PITCH_BPNN.In.e[1];
		PITCH_BPNN.In.e[1]=PITCH_BPNN.In.e[0];
		PITCH_BPNN.In.e[0]=pitch_SpeedSet*pi/180-omegaY.floatValue*pi/180;

		BPNN_PID_Forward_Propagation(&PITCH_BPNN,0);
		BPNN_PID_Back_Propagation(&PITCH_BPNN);
		}

	if(EN_ON)
		{
		k=k+1;
//		if(k >=10)
//		{
//			ek_2 		= ek_1;
//			ek_1 		= ek;
//			ek 			= 0 - YAW.floatValue;
//			uk_1		= uk;
//			uk			= uk_1 + Kp*(ek - ek_1) + Ki*(T/2)*(ek + ek_1) + Kd/T*(ek - 2*ek_1 + ek_2);
//			k=0;
//		}

		//	p_uk = 50*sin(2*pi*k*0.002);

//		yaw_SpeedSet=10.0f;

//		yaw_SpeedSet = 60*sin(2*pi*k*0.002);
/*		val = sin(2*pi*k*0.002);
		if (prev_val < 0 && val >= 0)
		{
			step_counter = (step_counter + 1) % 4;  // 4 pha: 0 → 1 → 2 → 3 → 0 ...
		}

		prev_val = val;
		float temp_speed_set = 0;

//		// Gán giá trị theo pha
		if (step_counter == 0)
			temp_speed_set = 60;
		else if (step_counter == 1)
			temp_speed_set = 0;
		else if (step_counter == 2)
			temp_speed_set = -60;
		else
			temp_speed_set = 0;
//
//			yaw_SpeedSet = yaw_cam_uk;
		yaw_SpeedSet = lowpass_filter(temp_speed_set,1);
//		yaw_SpeedSet= val;
//		yaw_SpeedSet = uk;
 *
 */
		yaw_SpeedSet = lowpass_filter(yaw_cam_uk,1);
/*		SPEED.floatValue = yaw_SpeedSet;
		trans_array[9] = SPEED.byteArray[0];
		trans_array[10] = SPEED.byteArray[1];
		trans_array[11] = SPEED.byteArray[2];
		trans_array[12] = SPEED.byteArray[3];
*/

		YAW_BPNN.BP.y[1]=YAW_BPNN.BP.y[0];
		YAW_BPNN.BP.y[0]=omegaZ.floatValue*pi/180;

		YAW_BPNN.PID.u[1]=YAW_BPNN.PID.u[0];

		YAW_BPNN.In.e[2]=YAW_BPNN.In.e[1];
		YAW_BPNN.In.e[1]=YAW_BPNN.In.e[0];
		YAW_BPNN.In.e[0]=yaw_SpeedSet*pi/180-omegaZ.floatValue*pi/180;

		BPNN_PID_Forward_Propagation(&YAW_BPNN,1);
		BPNN_PID_Back_Propagation(&YAW_BPNN);
		}

		PWM.floatValue =	PITCH_BPNN.PID.u[0];
		trans_array[5] = PWM.byteArray[0];
		trans_array[6] = PWM.byteArray[1];
		trans_array[7] = PWM.byteArray[2];
		trans_array[8] = PWM.byteArray[3];

		KP.floatValue 	=PITCH_BPNN.Out.o_out[0];
		trans_array[21] = KP.byteArray[0];
		trans_array[22] = KP.byteArray[1];
		trans_array[23] = KP.byteArray[2];
		trans_array[24] = KP.byteArray[3];

		KI.floatValue =PITCH_BPNN.Out.o_out[1];
		trans_array[25] = KI.byteArray[0];
		trans_array[26] = KI.byteArray[1];
		trans_array[27] = KI.byteArray[2];
		trans_array[28] = KI.byteArray[3];

		KD.floatValue =PITCH_BPNN.Out.o_out[2];
		trans_array[29] = KD.byteArray[0];
		trans_array[30] = KD.byteArray[1];
		trans_array[31] = KD.byteArray[2];
		trans_array[32] = KD.byteArray[3];
		if(EN_ON)
		{
			if(YAW_BPNN.PID.u[0]>=0)
			{
				MOTORDRIVE(TIM8_CHANNEL3_PWM1_Pin,YAW_BPNN.PID.u[0],1);
			}
			else if (YAW_BPNN.PID.u[0]<0)
			{
				MOTORDRIVE(TIM8_CHANNEL3_PWM1_Pin,(-YAW_BPNN.PID.u[0]),0);
			}

			if(PITCH_BPNN.PID.u[0]>=0)
			{
				MOTORDRIVE(TIM2_CHANNEL2_PWM2_Pin,PITCH_BPNN.PID.u[0],1);
			}
			else if (PITCH_BPNN.PID.u[0]<0)
			{
				MOTORDRIVE(TIM2_CHANNEL2_PWM2_Pin,(-PITCH_BPNN.PID.u[0]),0);
			}
			HAL_UART_Transmit_DMA(&huart6, trans_array, 34);

		}
		else
		{
			MOTORDRIVE(TIM8_CHANNEL3_PWM1_Pin,0,1);
			MOTORDRIVE(TIM2_CHANNEL2_PWM2_Pin,0,1);
		}
		SPEED_CONTROL = 0;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
	  }
	}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8399;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8400;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 230400;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 DIR1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR2_Pin */
  GPIO_InitStruct.Pin = DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
