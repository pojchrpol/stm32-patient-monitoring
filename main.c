  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "math.h"
#include "stdio.h"

// reading threshold with realistic values in comment
#define TEMP_THRESHOLD 31 //37.5
#define ACC_THRESHOLD 0.5
#define GYRO_THRESHOLD 20 //2.2 as default is 2 +-0.1
#define MAG_THRESHOLD 0 //-0.15 as default is about -0.10, but need to be calibrated bases on environment
#define HUMID_THRESHOLD 90 //65
#define PRESSURE_THRESHOLD 1004 //1015

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

static void UART1_Init(void);
static void MX_GPIO_Init(void);

UART_HandleTypeDef huart1;

//handling MODE_TOGGLE
enum device_mode{normal, intensive_care};
enum state{off, on};

uint32_t timestart = 0;
uint32_t report_time;

enum device_mode mode = normal;

int LED_warning = 0;
uint32_t LED_time; //tracking the time of last LED toggle

//printing message
char norm_mode_msg[] = "Entering Normal Mode.\r\n";
char ic_mode_msg[] = "Entering Intensive Care Mode.\r\n";

char temp_msg[] = "Fever is detected! \r\n";
enum state temp_wng = off;

char acc_msg[] = "Fall detected! \r\n";
enum state acc_wng = off;

char gyro_msg[] = "Patient in pain! \r\n";
enum state gyro_wng = off;

char orr_msg[] = "Check patient's abnormal orientation! \r\n";
enum state orr_wng = off;

char resp_msg[] = "Check patient's breath! \r\n";
enum state resp_wng = off;

char msg_print[63];
int msg_cnt = 1;

int pressed = 0;

//password enhancement
enum state pw_mode = off;

int input_cnt = 0;
char input_char[1];
char input_pw[63] = "\0";

char password[63] = "\0";

int main(void)
{
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	UART1_Init();
	MX_GPIO_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	BSP_HSENSOR_Init();
	BSP_PSENSOR_Init();

	//Sensors data
	float temp_data; //Temperature sensor

	//Accelerometer
	float accel_data[3];
	int16_t accel_data_i16[3] = { 0 };

	//Gyroscope
	float gyro_data[3] = { 0 };
	float angular_velocity;

	//Magnetometer
	float magneto_data[3];
	int16_t magneto_data_i16[3] = { 0 };

	float humid_data; //Humidity sensor
	float pressure_data; //Pressure sensor

	//Start the program
	sprintf(msg_print, "%s", "Hello! Welcome to COPEMON \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
	sprintf(msg_print, "%s", "To turn PASSWORD MODE on please press 1 \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
	sprintf(msg_print, "%s", "To skip please press any other keys \r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
	HAL_UART_Receive(&huart1, (uint8_t*)input_char, 1, 0xFFFF);

	//Password setting for user
	HAL_UART_Receive(&huart1, (uint8_t*)input_char, 1, 0xFFFF);
	HAL_UART_Transmit(&huart1, (uint8_t*)input_char, 1,0xFFFF);
	sprintf(msg_print, "%s","\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);

	if(*input_char == '1')
	{
		pw_mode = on;
		sprintf(msg_print, "%s", "Please enter the password \r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);

		int pw_confirmed = 0;

		while(pw_confirmed == 0)
		{
			HAL_UART_Receive(&huart1, (uint8_t*)input_char, 1, 0xFFFF);
			if(*input_char == '\r') // press Enter
			{
				*(input_pw + input_cnt) = '\0';
				sprintf(msg_print, "%s", "\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
				if(input_cnt == 0)
				{
					sprintf(msg_print, "%s", "Password is too short! \r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
				}
				else //valid password
				{
					strcpy(password, input_pw); //take input as password
					sprintf(msg_print, "%s", "Password confirmed! \r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
					pw_confirmed = 1;
				}
				strcpy(input_pw, "\0");
				input_cnt = 0;
				*input_char = '\0';
			}
			else
			{
				*(input_pw + input_cnt) = *input_char;
				HAL_UART_Transmit(&huart1, (uint8_t*)input_char, 1,0xFFFF);
				input_cnt++;
			}
		}
	}

	//Entering Normal Mode at the beginning
	sprintf(msg_print, "%s", norm_mode_msg);
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);

	// Enable LSM6DSL tilt interrupt
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80); // enable interrupts and latch interrupt
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WAKE_UP_DUR, 0x00); // set event duration FF_DUR5
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL, 0x0F); // set free fall threshold & event duration
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, 0x10); // Free Fall interrupt driven to INT1 pin

	//Report timing
	uint32_t report_interval = 10000;
	uint32_t curr_time; //current time
	report_time = HAL_GetTick();

	while (1)
	{
		curr_time = HAL_GetTick();
		if(curr_time - report_time >= report_interval) //report time
		{
			//Temperature sensor
			temp_data = BSP_TSENSOR_ReadTemp();

			if((temp_wng == off) & (temp_data > TEMP_THRESHOLD))
			{
				sprintf(msg_print, "%s", temp_msg);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
				temp_wng = on;
			}

			//Accelerometer
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);
			// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
			accel_data[0] = (float)accel_data_i16[0] / 1000.0f;
			accel_data[1] = (float)accel_data_i16[1] / 1000.0f;
			accel_data[2] = (float)accel_data_i16[2] / 1000.0f;

			if((acc_wng == off) && (accel_data[2] < ACC_THRESHOLD) && (accel_data[1] < ACC_THRESHOLD) && (accel_data[0] < ACC_THRESHOLD))
			{
				sprintf(msg_print, "%s", acc_msg);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
				acc_wng = on;
			} //this part should be edited using interrupt later

			//Humidity sensor
			humid_data = BSP_HSENSOR_ReadHumidity();	//humidity read in RH (relative humidity)

			//Pressure sensor
			pressure_data = BSP_PSENSOR_ReadPressure();

			if((humid_data < HUMID_THRESHOLD) || (pressure_data > PRESSURE_THRESHOLD) && (mode == normal))
			{
				char msg_print[64];
				sprintf(msg_print, "%s", resp_msg);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
				resp_wng = on;

				mode = intensive_care;
				sprintf(msg_print, "%s", ic_mode_msg);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
				temp_wng = off;
				acc_wng = off;
				gyro_wng = off;
				orr_wng = off;
				LED_warning = 0;
				HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
				msg_cnt = 1;
			}

			if(mode == intensive_care)
			{
				//Temp and Acc value printing
				sprintf(msg_print, "%03d TEMP_%0.2f (degreeC) ACC_%0.2f(g)_%0.2f(g)_%0.2f(g) \r\n", msg_cnt, temp_data, accel_data[0], accel_data[1], accel_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print), 0xFFFF);
				msg_cnt++;

				//Gyroscope reading
				BSP_GYRO_GetXYZ(gyro_data);
				gyro_data[0] = (float)gyro_data[0] / 1000.0f;
				gyro_data[1] = (float)gyro_data[1] / 1000.0f;
				gyro_data[2] = (float)gyro_data[2] / 1000.0f;

				//total angular velocity in dps
				angular_velocity = sqrt(pow(gyro_data[0], 2) + pow(gyro_data[1], 2) + pow(gyro_data[2], 2));

				//Magnetometer reading
				BSP_MAGNETO_GetXYZ(magneto_data_i16);
				//the function returns 16 bit integers which are in mGauss
				magneto_data[0] = (float)magneto_data_i16[0] / 1000.0f;
				magneto_data[1] = (float)magneto_data_i16[1] / 1000.0f;
				magneto_data[2] = (float)magneto_data_i16[2] / 1000.0f;

				magneto_data[0] = magneto_data[0] + 0.12;
				magneto_data[1] = magneto_data[1] + 1.94;
				magneto_data[2] = magneto_data[2] + 0.06;

				sprintf(msg_print, "%03d GYRO %.2f(dps) ", msg_cnt, angular_velocity);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
				sprintf(msg_print, "MAGNETO %.2f(G) %.2f(G) %.2f(G) \r\n", magneto_data[0], magneto_data[1], magneto_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
				msg_cnt++;

				if((gyro_wng == off) && (angular_velocity > GYRO_THRESHOLD))
				{
					sprintf(msg_print, "%s", gyro_msg);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
					gyro_wng = on;
				}

				if((orr_wng == off) && (magneto_data[2] < MAG_THRESHOLD))
				{
					sprintf(msg_print, "%s", orr_msg);
					HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
					orr_wng = on;
				}

				sprintf(msg_print, "%03d HUMIDITY %.2f (%%) and PRESSURE %.2f (hPa) \r\n", msg_cnt, humid_data, pressure_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
				msg_cnt++;
			}
			report_time = curr_time;

			//LED warning

			if ((gyro_wng == on) || (orr_wng == on)) {
				LED_warning = 2;
				LED_time = curr_time;
			}

			else if ((temp_wng == on) || (acc_wng == on)) {
				LED_warning = 1;
				LED_time = curr_time;
			}
		}

		if (((LED_warning == 1) && (curr_time - LED_time >= 100))|| //blink at 5Hz
				((LED_warning == 2) && (curr_time - LED_time >= 50))) //blink at 10Hz
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			LED_time = curr_time;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t curr_time = HAL_GetTick();
	if((GPIO_Pin == BUTTON_EXTI13_Pin) && (mode == intensive_care))
	{
		pressed++;

		if(pressed == 1)
		{
			timestart = HAL_GetTick();
		}


		else if(curr_time - timestart > 500) {
			pressed = 0;
		}

		else if(pw_mode == on)//the second pressed is within the 500 ms interval
		{
			sprintf(msg_print, "%s", "Please Enter Password to Enable Normal Mode!\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
			int inc_pw_cnt = 0;
			while(mode != normal)
			{
				HAL_UART_Receive(&huart1, (uint8_t*)input_char, 1, 0xFFFF);
				if(*input_char == '\r') // press Enter
				{
					*(input_pw + input_cnt) = '\0';
					sprintf(msg_print, "%s", "\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
					if(strcmp(password, input_pw) == 0) // correct password
					{
						mode = normal;
						pressed = 0;

						sprintf(msg_print, "%s", norm_mode_msg);
						HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
						temp_wng = off;
						acc_wng = off;
						gyro_wng = off;
						orr_wng = off;
						resp_wng = off;
						LED_warning = 0;
						HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
						report_time = curr_time;
					}
					else if(inc_pw_cnt == 3)
					{
						sprintf(msg_print, "%s", "Incorrect Password! \r\n");
						HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
						inc_pw_cnt = 0;
						break;
					}
					else
					{
						sprintf(msg_print, "%s", "Incorrect Password! Please try again. \r\n");
						HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
						inc_pw_cnt++;
					}
					strcpy(input_pw, "\0");
					input_cnt = 0;
					*input_char = '\0';
				}
				else
				{
					*(input_pw + input_cnt) = *input_char;
					HAL_UART_Transmit(&huart1, (uint8_t*)input_char, 1,0xFFFF);
					input_cnt++;
				}
			}
		}
		else
		{
			sprintf(msg_print, "%s", norm_mode_msg);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
			temp_wng = off;
			acc_wng = off;
			gyro_wng = off;
			orr_wng = off;
			resp_wng = off;
			LED_warning = 0;
			HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
			report_time = curr_time;
		}
	}
	else if ((GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin) && (acc_wng == off)){
		sprintf(msg_print, "%s", acc_msg);
		HAL_UART_Transmit(&huart1, (uint8_t*)msg_print, strlen(msg_print),0xFFFF);
		acc_wng = on;

		if(LED_warning == 0) {
			LED_warning = 1;
			LED_time = curr_time;
		}
	}
}

static void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();	// Enable AHB2 Bus for GPIOA
	__HAL_RCC_GPIOB_CLK_ENABLE();	// Enable AHB2 Bus for GPIOB
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC
	__HAL_RCC_GPIOD_CLK_ENABLE();	// Enable AHB2 Bus for GPIOD

	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET); // Reset the LED2_Pin as 0

	GPIO_InitTypeDef GPIO_InitStructLED = {0};
	GPIO_InitTypeDef GPIO_InitStructBTN = {0};
	GPIO_InitTypeDef GPIO_InitStructACC = {0};

	// Configuration of LED2_Pin (GPIO-B Pin-14) as GPIO output
	GPIO_InitStructLED.Pin = LED2_Pin;
	GPIO_InitStructLED.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructLED.Pull = GPIO_NOPULL;
	GPIO_InitStructLED.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructLED);

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStructBTN.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStructBTN.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructBTN.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructBTN);

	//Configure GPIO pin LSM6DSL_INT1_EXTI11_Pin
	GPIO_InitStructACC.Pin = LSM6DSL_INT1_EXTI11_Pin;
	GPIO_InitStructACC.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructACC.Pull = GPIO_NOPULL;
	GPIO_InitStructACC.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &GPIO_InitStructACC);

	HAL_NVIC_SetPriority((IRQn_Type)(USER_BUTTON_EXTI_IRQn), 0x0E, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type)(USER_BUTTON_EXTI_IRQn));

	HAL_NVIC_SetPriority((IRQn_Type)(LSM6DSL_INT1_EXTI11_EXTI_IRQn), 0x0F, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type)(LSM6DSL_INT1_EXTI11_EXTI_IRQn));


}
