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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEAD_ZONE_LOW 1800
#define DEAD_ZONE_HIGH 2400
#define NOJITTER 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
 uint8_t RxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA}; //Adres waar data naar word gesschreven. Zowel Rx als Tx adress moeten overeen komen bij het zenden en ontvangen van data.
 uint8_t RxData[19]; //Ontvangen data buffer. 18 bytes voor de doorgestuurde ADC's en 1 byte voor de status van de knoppen.
 uint32_t ADC_Values[9]; // Array voor het opslaan van de geconverteerde bytes naar 12bit
 uint8_t buttonStates; // Integer voor het opslaan van de status van de knoppen

 uint32_t dutyCycle_G1 = 0; //dutycycle servo kanon 1
 uint32_t index_G1 = 0; //index voor het rollend gemiddelde
 uint32_t dutyCycle_G1_total = 0; //telt de dutycycle op voor een gemiddelde
 uint32_t dutyCycle_G1_count = 0; //houdt bij hoeveel keer al is opgetelt
 uint32_t values_G1[10] = {0}; // Array voor het opslaan van de weaardes voor de berekening van het gemidelde
 uint32_t av_du_G1 = 0; // gemidelde duty cycle
 uint32_t Last_av_du_G1 = 0; // vorige gemidelde duty cycle

 uint32_t dutyCycle_G2 = 0; //dutycycle servo kanon 2
 uint32_t index_G2 = 0; //index voor het rollend gemiddelde
 uint32_t dutyCycle_G2_total = 0; //telt de dutycycle op voor een gemiddelde
 uint32_t dutyCycle_G2_count = 0; //houdt bij hoeveel keer al is opgetelt
 uint32_t values_G2[10] = {0}; // Array voor het opslaan van de weaardes voor de berekening van het gemidelde
 uint32_t av_du_G2 = 0; // gemidelde duty cycle
 uint32_t Last_av_du_G2 = 0; // vorige gemidelde duty cycle


 uint32_t dutyCycle_T1 = 0; //dutycycle servo kanontoren 1
 uint32_t index_T1 = 0; //index voor het rollend gemiddelde
 uint32_t dutyCycle_T1_total = 0; //telt de dutycycle op voor een gemiddelde
 uint32_t dutyCycle_T1_count = 0; //houdt bij hoeveel keer al is opgetelt
 uint32_t values_T1[10] = {0}; // Array voor het opslaan van de weaardes voor de berekening van het gemidelde
 uint32_t av_du_T1 = 0; // gemidelde duty cycle
 uint32_t Last_av_du_T1 = 0; // vorige gemidelde duty cycle

 uint32_t dutyCycle_T2 = 0; //dutycycle servo kanontoren 2
 uint32_t index_T2 = 0; //index voor het rollend gemiddelde
 uint32_t dutyCycle_T2_total = 0; //telt de dutycycle op voor een gemiddelde
 uint32_t dutyCycle_T2_count = 0; //houdt bij hoeveel keer al is opgetelt
 uint32_t values_T2[10] = {0}; // Array voor het opslaan van de weaardes voor de berekening van het gemidelde
 uint32_t av_du_T2 = 0; // gemidelde duty cycle
 uint32_t Last_av_du_T2 = 0; // vorige gemidelde duty cycle

 uint8_t dutyCycle_CH3 = 0;  // Duty cycle procent voor TIM2 Channel 3
 uint8_t dutyCycle_CH4 = 0;  // Duty cycle in procent voor TIM2 Channel 4
 int lowCountM1 = 0; //Teller om na te gaan of de waarde echt laag is of het gewoon jitter is in motor 1.
 int highCountM1 = 0; //Teller om na te gaan of de waarde echt hoog is of het gewoon jitter is in motor 1.
 int lowCountM2 = 0; //Teller om na te gaan of de waarde echt laag is of het gewoon jitter is in motor 2.
 int highCountM2 = 0; //Teller om na te gaan of de waarde echt hoog is of het gewoon jitter is in motor 2.

 uint32_t ADC; //ADC voor het niveau van de batterij

 uint32_t current_Tick;
 uint32_t prev_Tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
void processData(void);
void adjustDutyCycle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// processData combineert alle ontvangen bytes terug naar hun originele 12 bit staat en onthoudt de status van de knoppen
//////////////////// bv. RxData[j] = lower 8 bits = 0100 1100 en RxData[j+1] = upper 4 bits = 0000 1011
////////////////////	 RxData[j+1] << 8 = 0000 1011 0000 0000
////////////////////     RxData[j] | RxData[j+1] =           0100 1100
////////////////////                            OR 0000 1011 0000 0000
////////////////////                             = 0000 1011 0100 1100 = 12 bit waarde
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void processData(void){
	NRF24_Receive(RxData); // Ontvang data
	for (int i = 0, j = 0; i < 9; i++, j += 2) {
		ADC_Values[i] = (RxData[j] | (RxData[j+1] << 8));  // Combineert 2 ontvangen data bytes naar een 12 bit value.
	}
	buttonStates = RxData[18]; // Haalt de status van de knoppen uit de ontvangen data
	adjustDutyCycle(); // Roept de functie op voor het toepassen van de nieuwe waardes
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// adjustDutyCycle gebruikt de ontvangen waardes om de duyclyes van de motoren aan te passen
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void adjustDutyCycle(void) {
	///////////////////////////////////////////////////////
	//////////////////// DC Motor 1 ///////////////////////
	///////////////////////////////////////////////////////
	if (ADC_Values[0] < 4096){ // zorgt ervoor dat foutieve waardes boven 4096 genegeert worden
		if (ADC_Values[0] < DEAD_ZONE_LOW) { // Als de ADC waarde onder de threshold ligt gaat de motor achteruit.
			lowCountM1 += 1;
			highCountM1 = 0;
			if(lowCountM1 >= NOJITTER) // Als de waarde 3 keer onder de threshold ligt dan weten we dat het geen jitter is.
			{
				lowCountM1 = 0;
				dutyCycle_CH3 = (DEAD_ZONE_LOW - ADC_Values[0]) * 100 / (DEAD_ZONE_LOW - 500); // berekent de dutycycle voor een waarde onder de threshold
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,0); //Configureert IN21
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,1); //Configureert IN11
			}
		}
		else if (ADC_Values[0] > DEAD_ZONE_HIGH) { // Als de ADC waarde boven de threshold ligt gaat de motor vooruit.
			highCountM1 += 1;
			lowCountM1 = 0;
			if(highCountM1 >= NOJITTER) // Als de waarde 3 keer onder de threshold ligt dan weten we dat het geen jitter is.
			{
				highCountM1 = 0;
				dutyCycle_CH3 = (ADC_Values[0] - DEAD_ZONE_HIGH) * 100 / (3550 - DEAD_ZONE_HIGH); // berekent de dutycycle voor een waarde boven de threshold
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,0); //Configureert IN11
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,1); //Configureert IN21
			}
		}
		else {
			highCountM1 = 0;
			lowCountM1 = 0;
			dutyCycle_CH3 = 0; // Motor stops in dead zone
		}
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, (2999 * dutyCycle_CH3) / 100); //Gebruikt de nieuwe duty cycle en laat de motor draaien.
	}
	///////////////////////////////////////////////////////
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	//////////////////// DC Motor 2 ///////////////////////
	///////////////////////////////////////////////////////
	if (ADC_Values[7] < 4096){ // zorgt ervoor dat foutieve waardes boven 4096 genegeert worden
		if (ADC_Values[7] < DEAD_ZONE_LOW) { // Als de ADC waarde onder de threshold ligt gaat de motor achteruit.
			lowCountM2 += 1;
			highCountM2 = 0;
			if(lowCountM2 >= NOJITTER) // Als de waarde 3 keer onder de threshold ligt dan weten we dat het geen jitter is.
			{
				lowCountM2 = 0;
				dutyCycle_CH4 = (DEAD_ZONE_LOW - ADC_Values[7]) * 100 / (DEAD_ZONE_LOW - 500); // berekent de dutycycle voor een waarde onder de threshold
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,0); //Configureert IN10
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,1); //Configureert IN20
			}
		}
		else if (ADC_Values[7] > DEAD_ZONE_HIGH) { // Als de ADC waarde boven de threshold ligt gaat de motor vooruit.
			highCountM2 += 1;
			lowCountM2 = 0;
			if(highCountM2 >= NOJITTER) // Als de waarde 3 keer onder de threshold ligt dan weten we dat het geen jitter is.
			{
				highCountM2 = 0;
				dutyCycle_CH4 = (ADC_Values[7] - DEAD_ZONE_HIGH) * 100 / (3550 - DEAD_ZONE_HIGH); // berekent de dutycycle voor een waarde boven de threshold
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,0); //Configureert IN20
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,1); //Configureert IN10
			}
		}
		else {
			highCountM2 = 0;
			lowCountM2 = 0;
			dutyCycle_CH4 = 0; // Motor stops in dead zone
		}
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, (2999 * dutyCycle_CH4) / 100); //Gebruikt de nieuwe duty cycle en laat de motor draaien.
	}
	///////////////////////////////////////////////////////
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	//////////////////// Kanon servo 1 ////////////////////
	///////////////////////////////////////////////////////
	dutyCycle_G1 = 3600 + ((ADC_Values[5] * (5200 - 3600)) / 4095); // the duty cycly = min value * % * (Max vcalue - Min Value) (20ms = 60khz)
	if (dutyCycle_G1 < 3600) // De waarden limiteren tot hun max/minimum
	{
		dutyCycle_G1 = 3600;
	}
	if (dutyCycle_G1 > 5200)
	{
		dutyCycle_G1 = 5200;
	}
	dutyCycle_G1_total = dutyCycle_G1_total - values_G1[index_G1] + dutyCycle_G1;
	values_G1[index_G1] = dutyCycle_G1;
	index_G1 = (index_G1 + 1) % 10;
	if (dutyCycle_G1_count < 10) { // Het rollend gemiddelde moet eerst tot 10 waarden stijgen nadien blijft deze hier.
		dutyCycle_G1_count++;
	}
	av_du_G1 = dutyCycle_G1_total / dutyCycle_G1_count; // Bereken de gemiddelde duty cycle
	if (av_du_G1 < 3600) // De waarden limiteren tot hun max/minimum
	{
		av_du_G1 = 3600;
	}
	if (av_du_G1 > 5200)
	{
		av_du_G1 = 5200;
	}
	if (av_du_G1 > Last_av_du_G1)
	{
		if ((av_du_G1 - Last_av_du_G1) > 150) // er mag pas bewogen worden van zodra er een verschil is van meer dan 100 eenheiden tegen over vorige waarde.
		{
			Last_av_du_G1 = av_du_G1;
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,av_du_G1); // gebruik de gevonde duty cycle
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_Delay(20);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		}
	}
	if (av_du_G1 < Last_av_du_G1)
	{
		if ((Last_av_du_G1 - av_du_G1) > 150) // er mag pas bewogen worden van zodra er een verschil is van meer dan 100 eenheiden tegen over vorige waarde.
		{
			Last_av_du_G1 = av_du_G1;
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,av_du_G1); // gebruik de gevonde duty cycle
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_Delay(40);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		}
	}
	///////////////////////////////////////////////////////
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	//////////////////// Kanon servo 2 ////////////////////
	///////////////////////////////////////////////////////
	dutyCycle_G2 = 3600 + ((ADC_Values[2] * (5200 - 3600)) / 4095); // the duty cycly = min value * % * (Max vcalue - Min Value) (20ms = 60khz)
	if (dutyCycle_G2 < 3600) // De waarden limiteren tot hun max/minimum
	{
		dutyCycle_G2 = 3600;
	}
	if (dutyCycle_G2 > 5200)
	{
		dutyCycle_G2 = 5200;
	}
	dutyCycle_G2_total = dutyCycle_G2_total - values_G2[index_G2] + dutyCycle_G2; // de totaalsom van het rollent gemiddelde = De som - de oudste waar + de nieuwste waarde
	values_G2[index_G2] = dutyCycle_G2;
	index_G2 = (index_G2 + 1) % 10;
	if (dutyCycle_G2_count < 10) { // Het rollend gemiddelde moet eerst tot 10 waarden stijgen nadien blijft deze hier.
		dutyCycle_G2_count++;
	}
	av_du_G2 = dutyCycle_G2_total / dutyCycle_G2_count; // Bereken de gemiddelde duty cycle
	if (av_du_G2 < 3600) // De waarden limiteren tot hun max/minimum
	{
		av_du_G2 = 3600;
	}
	if (av_du_G2 > 5200)
	{
		av_du_G2 = 5200;
	}
	if (av_du_G2 > Last_av_du_G2)
	{
		if ((av_du_G2 - Last_av_du_G2) > 150) // er mag pas bewogen worden van zodra er een verschil is van meer dan 100 eenheiden tegen over vorige waarde.
		{
			Last_av_du_G2 = av_du_G2;
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,av_du_G2); // gebruik de gevonde duty cycle
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_Delay(20);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		}
	}
	if (av_du_G2 < Last_av_du_G2)
	{
		if ((Last_av_du_G2 - av_du_G2) > 150) // er mag pas bewogen worden van zodra er een verschil is van meer dan 100 eenheiden tegen over vorige waarde.
		{
			Last_av_du_G2 = av_du_G2;
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,av_du_G2); // gebruik de gevonde duty cycle
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_Delay(40);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		}
	}
	///////////////////////////////////////////////////////
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	//////////////////// Kanonentoren servo 1 /////////////
	///////////////////////////////////////////////////////

	dutyCycle_T1 = 3000 + ((ADC_Values[4] * (6000 - 3000)) / 4095); // the duty cycly = min value * % * (Max vcalue - Min Value) (20ms = 60khz)
	if (dutyCycle_T1 < 3000) // De waarden limiteren tot hun max/minimum
	{
		dutyCycle_T1 = 3000;
	}
	if (dutyCycle_T1 > 6000)
	{
		dutyCycle_T1 = 6000;
	}
	dutyCycle_T1_total = dutyCycle_T1_total - values_T1[index_T1] + dutyCycle_T1; // de totaalsom van het rollent gemiddelde = De som - de oudste waar + de nieuwste waarde
	values_T1[index_T1] = dutyCycle_T1;
	index_T1 = (index_T1 + 1) % 10;
	if (dutyCycle_T1_count < 10) { // Het rollend gemiddelde moet eerst tot 10 waarden stijgen nadien blijft deze hier.
		dutyCycle_T1_count++;
	}
	av_du_T1 = dutyCycle_T1_total / dutyCycle_T1_count; // Bereken de gemiddelde duty cycle
	if (av_du_T1 < 3000) // De waarden limiteren tot hun max/minimum
	{
		av_du_T1 = 3000;
	}
	if (av_du_T1 > 6000)
	{
		av_du_T1 = 6000;
	}
	if (av_du_T1 > Last_av_du_T1)
	{
		if ((av_du_T1 - Last_av_du_T1) > 500) // er mag pas bewogen worden van zodra er een verschil is van meer dan 100 eenheiden tegen over vorige waarde.
		{
			Last_av_du_T1 = av_du_T1;
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,av_du_T1); // gebruik de gevonde duty cycle
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
			HAL_Delay(20);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
		}
	}
	if (av_du_T1 < Last_av_du_T1)
	{
		if ((Last_av_du_T1 - av_du_T1) > 500) // er mag pas bewogen worden van zodra er een verschil is van meer dan 100 eenheiden tegen over vorige waarde.
		{
			Last_av_du_T1 = av_du_T1;
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,av_du_T1); // gebruik de gevonde duty cycle
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
			HAL_Delay(40);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
		}
	}
	///////////////////////////////////////////////////////
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	//////////////////// Kanonentoren servo 2 /////////////
	///////////////////////////////////////////////////////

	dutyCycle_T2 = 3000 + ((ADC_Values[3] * (6000 - 3000)) / 4095); // the duty cycly = min value * % * (Max vcalue - Min Value) (20ms = 60khz)
	if (dutyCycle_T2 < 3000) // De waarden limiteren tot hun max/minimum
	{
		dutyCycle_T2 = 3000;
	}
	if (dutyCycle_T2 > 6000)
	{
		dutyCycle_T2 = 6000;
	}
	dutyCycle_T2_total = dutyCycle_T2_total - values_T2[index_T2] + dutyCycle_T2; // de totaalsom van het rollent gemiddelde = De som - de oudste waar + de nieuwste waarde
	values_T2[index_T2] = dutyCycle_T2;
	index_T2 = (index_T2 + 1) % 10;
	if (dutyCycle_T2_count < 10) { // Het rollend gemiddelde moet eerst tot 10 waarden stijgen nadien blijft deze hier.
		dutyCycle_T2_count++;
	}
	av_du_T2 = dutyCycle_T2_total / dutyCycle_T2_count; // Bereken de gemiddelde duty cycle
	if (av_du_T2 < 3000) // De waarden limiteren tot hun max/minimum
	{
		av_du_T2 = 3000;
	}
	if (av_du_T2 > 6000)
	{
		av_du_T2 = 6000;
	}
	if (av_du_T2 > Last_av_du_T2)
	{
		if ((av_du_T2 - Last_av_du_T2) > 150) // er mag pas bewogen worden van zodra er een verschil is van meer dan 100 eenheiden tegen over vorige waarde.
		{
			Last_av_du_T2 = av_du_T2;
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,av_du_T2); // gebruik de gevonde duty cycle
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_Delay(20);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		}
	}
	if (av_du_T2 < Last_av_du_T2)
	{
		if ((Last_av_du_T2 - av_du_T2) > 150) // er mag pas bewogen worden van zodra er een verschil is van meer dan 100 eenheiden tegen over vorige waarde.
		{
			Last_av_du_T2 = av_du_T2;
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,av_du_T2); // gebruik de gevonde duty cycle
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_Delay(40);
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		}
	}
}

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
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  NRF24_Init();	// Initialiseert de NRF24L01+ module.
  NRF24_RXMode(RxAddress, 10); // Configureert de NRF24L01+ module in Receive mode.
  HAL_TIM_Base_Start_IT(&htim1); //Start timer 1 gebruikt voor de servo motoren.
  HAL_TIM_Base_Start_IT(&htim15); //Start timer 2 gebruikt voor de DC motoren.
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); //Start de PWM voor DC motor 1.
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);  //Start de PWM voor DC motor 2.
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0); // Initialisert de counter op 0 van DC motor 1 zodat deze niet automatisch start met rijden.
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0); // Initialisert de counter op 0 van DC motor 2 zodat deze niet automatisch start met rijden.
  HAL_ADC_Start(&hadc1); // Start de ADC voor het batterij niveau.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  current_Tick = HAL_GetTick();
	  if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
	  {
		  ADC = HAL_ADC_GetValue(&hadc1);
	  }
	  if(ADC > 3020) //kijkt na of het batterij niveau nog boven 11.8V is.
	  {
		  if (isDataAvailable(1) == 1) // Kijkt na of er data beschikbaar is.
		  {
			  prev_Tick = HAL_GetTick();
			  processData(); //Als data beschikbaar is gaan we deze data verwerken.
		  }
	  }
	  if (prev_Tick + 500 < current_Tick)
	  {
		  prev_Tick = HAL_GetTick();
		  HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 19;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 59999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 2999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN10_Pin|IN20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN11_Pin|IN21_Pin|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN10_Pin IN20_Pin */
  GPIO_InitStruct.Pin = IN10_Pin|IN20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN11_Pin IN21_Pin PB6 PB7 */
  GPIO_InitStruct.Pin = IN11_Pin|IN21_Pin|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
