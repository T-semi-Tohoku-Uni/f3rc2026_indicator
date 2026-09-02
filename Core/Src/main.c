/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "barSegLED.h"
#include <stdio.h>
#include <stdbool.h> // bool
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define My_SWtact_PRESSED GPIO_PIN_RESET
#define My_SWtact_RELEASED GPIO_PIN_SET
#define My_SWlimit_PRESSED GPIO_PIN_RESET
#define My_SWlimit_RELEASED GPIO_PIN_SET

// 反転させるのだろうか？
#define My_HIGH GPIO_PIN_RESET
#define My_LOW GPIO_PIN_SET


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PI 3.14159

#define KP_OMEGA 2.5f

#define PPR 2000.0 //pulses per revolution
#define R 30.0//mm  //radious of wheel

#define START_POS_X (2000.0f+ 500.0f / 2.0f)
#define START_POS_Y (500.0f / 2.0f)
#define START_THETA PI

#define AVG_WINDOW_SIZE 25

// 機体の機構パラメータ (実測値をmm単位等で設定。CADデータの原点を仮想的な機体中心とした)
const float L = 15.80f; // y軸方向を向く輪のx軸からの距離
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim20;
DMA_HandleTypeDef hdma_tim1_ch4;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader_2;
FDCAN_RxHeaderTypeDef RxHeader_2;

uint8_t TxData[8] = {};
uint8_t RxData[8] = {}; 

const int16_t vel_id = 0x005;
const int16_t odometry_id = 0x015;
const int16_t servo_id = 0x206;

uint16_t timer1000Hz = 0;     // 1000Hzタイマー
uint16_t timer100Hz = 0;      // 100Hzタイマー
uint8_t timer10Hz = 0;        // 10Hzタイマー
uint16_t timerTactSwitch = 0; // タクトスイッチチャタリング除去用タイマー

uint16_t countUpTimer1000Hz = 0; // 1000Hzタイマー用カウンタ

// 機体の縦横サイズ
const float roboWidth = 475.46;
const float roboLength = 491.44;

// ロボマス制御用
volatile float VX = 0, VY = 0; //mm/s
volatile float Omega = 0; // rad/s

// 機体の位置計算用(ローカル座標計算)
const float dt = 0.001f; // 1ms
volatile float deg1 = 0, deg2 = 0, deg3 = 0;
volatile float dwl = 0;
volatile float dxl = 0;
volatile float dyl = 0;
volatile float vx = 0, vy = 0; //mm/s
volatile float omega = 0; // rad/s

// スタート地点から算出するグローバル座標。機体を上から見た時の長方形の幾何学中心を基準点とする
volatile float x = START_POS_X;
volatile float y = START_POS_Y;
volatile float theta = START_THETA;

// 通信で取得するデータ
volatile float yaw = 0;
volatile int enc_value_1;
volatile int enc_value_2;
volatile int enc_value_3;

volatile float prev_yaw = 0.0f;

volatile uint16_t buzzerTimerMs = 0; // ブザーを鳴らす残り時間（ms）

// 自己位置推定のズレのoffset用。主にstep部分で使う予定
float offsets[] = {
    2000.0f,
    1900.0f,
    2700.0f,
    2270.0f,
    512.0f,
    1880.0f,
    1880.0f,
    2700.0f,
    2270.0f,
    1900.0f,
    1900.0f,
    2700.0f,
    2270.0f
};

volatile uint8_t servo_mode = 0; // 0:復帰, 1:動作

uint8_t swstate = 0;// リミットスイッチ上下左右

volatile bool isStarted = false;
volatile bool isSensorReady = false;
volatile uint8_t roboState = 99;

volatile uint16_t timer1 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM20_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void interboard_comms_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings);
void interboard_comms_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef interboard_comms_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader);
HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader);

void Velocity_Tx(void);
void Servo_Tx(uint8_t one_or_zero);
// uint8_t LimitSW_front(void);
// uint8_t LimitSW_back(void);
// uint8_t LimitSW_left(void);
// uint8_t LimitSW_right(void);
// uint8_t LimitSW_Start(void);

typedef struct {
    float buffer[AVG_WINDOW_SIZE]; // 静的に確保
    int index;
    int count;
    double sum;
} MovingAvgData;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static MovingAvgData avg_x, avg_y, avg_omega;

// uint8_t 型のバイト列（4バイト単位）を、IEEE 754 形式の float（単精度浮動小数点数）配列へ復元・変換する関数
void u8_to_float(uint8_t *req, float *des, uint32_t uint8_len)
{
  union IntAndFloat {
    uint32_t ival;
    float fval;
  };
  for(int i = 0; i < uint8_len/4; i++){
    uint32_t f32_u32 = (((uint32_t)req[i*4] << 24) | ((uint32_t)req[i*4+1] << 16) | ((uint32_t)req[i*4+2] << 8) | ((uint32_t)req[i*4+3]));
    union IntAndFloat target;
    target.ival = f32_u32;
    des[i] = target.fval;
  }
}

void u8_to_int16(uint8_t *req, int16_t *des, uint32_t int_len)
{
  for (uint32_t i = 0; i < int_len; i++)
  {
    uint16_t val = ((uint16_t)req[i * 2] << 8) | (uint16_t)req[i * 2 + 1];
    des[i] = (int16_t)val;
  }
}

float update_ma_isr(MovingAvgData *ma, float next_val) {
    // 古い値を引いて新しい値を足す (O(1)の計算)
    if (ma->count == AVG_WINDOW_SIZE) {
        ma->sum -= ma->buffer[ma->index];
    } else {
        ma->count++;
    }

    ma->buffer[ma->index] = next_val;
    ma->sum += next_val;
    ma->index = (ma->index + 1) % AVG_WINDOW_SIZE;

    return ma->sum / (float)ma->count;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {

	  /* Retrieve Rx messages from RX FIFO0 */

		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK) {
			printf("fdcan_getrxmessage is error\r\n");
			Error_Handler();
		}
    
    if (odometry_id == RxHeader.Identifier) {
      int16_t odometry_data[4];
      u8_to_int16(RxData, odometry_data, 4);

      yaw = (float)odometry_data[0] / 1000.0f; // rad
      enc_value_1 = (int)odometry_data[1];
      enc_value_2 = (int)odometry_data[2];
      enc_value_3 = (int)odometry_data[3];

      isSensorReady = !(odometry_data[0] == -32768); // 最低値が送られてきている間はバイアス計算中（未準備）とみなす
    }
	}
}

// // リミットスイッチ
// uint8_t LimitSW_front(){ // 0か1か
//   return (swstate & 0x01);
// }

// uint8_t LimitSW_back(){ // 0か1か
//   return ((swstate >> 1) & 0x01);
// }

// uint8_t LimitSW_left(){ // 0か1か
//   return ((swstate >> 2) & 0x01);
// }

// uint8_t LimitSW_right(){ // 0か1か
//   return ((swstate >> 3) & 0x01);
// }

// uint8_t LimitSW_Start(){ // 0か1か
//   printf("start sw:%d\r\n", (swstate >> 4) & 0x01);
//   return ((swstate >> 4) & 0x01);
// }

// timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (&htim6 == htim)
  { // 1000Hz
    if (timerTactSwitch > 0) // カウントダウン中（クールタイム）
    {
      timerTactSwitch--;
    }
    else if (timerTactSwitch == 0) // タイマーが0になった（ボタンを押してOKな状態）
    {
      // チャタリング除去完了
      if (HAL_GPIO_ReadPin(SW3_PC11_GPIO_Port, SW3_PC11_Pin) == My_SWlimit_PRESSED)
      {
        timerTactSwitch = 500; // 500ms待機

        isStarted = true;
        roboState = 0; // start
      }
    }

    // 姿勢自動補正：角度(yaw)が傾いた分だけ逆向きの角速度(Omega)を与える
    if (!isStarted || !isSensorReady) {
      VX = 0; VY = 0; Omega = 0;
    } else {
      Omega = -KP_OMEGA * yaw;

      // 念のため補正角速度が大きくなりすぎないようリミッターをかける
      if (Omega > 1.0f)  Omega = 1.0f;
      if (Omega < -1.0f) Omega = -1.0f;

      // 計測輪の設定上4で割るっぽい
      deg1 = ((enc_value_1 / (PPR * 4.0f)) * 2.0f * PI) / dt;
      deg2 = ((enc_value_2 / (PPR * 4.0f)) * 2.0f * PI) / dt;
      deg3 = ((enc_value_3 / (PPR * 4.0f)) * 2.0f * PI) / dt;

      // 1msあたりの角度変化量から角速度 [rad/s] を算出
      dwl = (yaw - prev_yaw) / dt; 
      prev_yaw = yaw; // 次回用に保持
      dxl = (deg3 + deg2) * R / 2.0f;
      dyl = deg1 * R - (L * dwl); // Y輪の回転干渉をキャンセル
        
      // 移動平均を計算
      vx = update_ma_isr(&avg_x, dxl);
      vy = update_ma_isr(&avg_y, dyl);
      omega = update_ma_isr(&avg_omega, dwl);

      theta = yaw + START_THETA;

      x += (vx * cosf(theta) - vy * sinf(theta)) * dt;
      y += (vx * sinf(theta) - vy * cosf(theta)) * dt;
    }
    
    if (roboState == 0){
      VX = -0.2; VY = -0.02;
      if(x < 1000 + offsets[0]) { // 基準点がCの白線を踏んだあたりの処理
        
      }

      if(x < 0 + roboWidth/2 + offsets[1]){ // offset必須か。機体がゾーンの端に行った時の処理。この時までに遮断機が降りていると予想
        roboState = 1;
        servo_mode = 1; // 回収機構の起動
      }
    }

    if (roboState == 1){
      VX = 0.2; VY = -0.02;
      if(x > 4500 - roboWidth/2 + offsets[2]) { // 庭の端で荷物を下ろす
        roboState = 2;
        servo_mode = 0;
      }
    }

    if (roboState == 2){
      VX = -0.2; VY = -0.02;
      if(x < 1000 + roboWidth/2 + offsets[3]) { // 領域手前まで移動
        roboState = 3;
      }
    }

    if (roboState == 3){
      VX = 0; VY = 0.1;
      if(y>1200 + offsets[4]) { // フィールドBの手前に来た時
        roboState = 4;
      }
    }

    if (roboState == 4){
      VX = -0.1; VY = 0;
      if(x < 1000 + offsets[5]) { // 基準点がBの白線を踏んだあたりの処理
        servo_mode = 1; // 回収機構の起動
      }

      if(x < 0 + roboWidth/2 + offsets[6]){ // 機体がゾーンの端にまで行ったら回収機を起動。ここに到達するまでには遮断機が降りている想定
        roboState = 5;
        timer1 = 500;
      }
    }

    if (roboState == 5){ // 回収動作
      VX = 0; VY = 0;
      if(timer1 == 0) {
        roboState = 6;
      }
    }

    if (roboState == 6){
      VX = 0.2; VY = 0;
      if(x > 4500 - roboWidth/2 + offsets[7]) { // 庭の端で荷物を下ろす
        roboState = 7;
        servo_mode = 0;
      }
    }

    if (roboState == 7){
      VX = -0.2; VY = 0;
      if(x < 1000 + roboWidth/2 + offsets[8]) { // 領域手前まで移動
        roboState = 8;
      }
    }

    if (roboState == 8){
      VX = 0; VY = 0.08; Omega = 0;
      if(HAL_GPIO_ReadPin(SW1_PC9_GPIO_Port, SW1_PC9_Pin) == My_SWlimit_PRESSED) { // フィールドCの手前に来た時
        roboState = 9;
      }
    }

    if (roboState == 9){
      VX = -0.2; VY = 0.07;
      if(x < 1000 + offsets[9]) { // 基準点がAの白線を踏んだあたりの処理
        servo_mode = 1; // 回収機構の動作
      }

      if(x < 0 + roboWidth/2 + offsets[10]){ // 機体がゾーンの端にまで行った時の処理。ここまでで遮断機が降り切っている想定
        roboState = 10;
      }
    }

    if (roboState == 10){
      VX = 0.1; VY = 0;
      if(x > 4500 - roboWidth/2 + offsets[11]) { // 庭の端で荷物を下ろす
        roboState = 11;
        servo_mode = 0;
      }
    }

    if (roboState == 11){
      VX = -0.1; VY = 0;
      if(x < 0 + roboWidth + offsets[12]) {
        roboState = 99;
      }
    }

    if (roboState == 99){ // end
      VX = 0; VY = 0;
    }

    Velocity_Tx();
  }

  if (&htim7 == htim) { // 100Hz
    if (timer1 > 0) timer1--;
    Servo_Tx(servo_mode);
  }

  if (&htim16 == htim) { // 10Hz
    if (!isStarted) SevenSeg_ToggleAnimate_Slider(0);
    if (roboState < 90){
      SevenSeg_Display_Number(roboState / 10, 0); // 左側に十の位を表示
      SevenSeg_Display_Number(roboState % 10, 1); // 右側に一の位を表示
    }
  }
}

void Velocity_Tx()
{
  FDCAN_TxHeaderTypeDef localTxHeader = TxHeader; // グローバル初期設定をコピー
  localTxHeader.Identifier = vel_id;

  int16_t v_x_tsushin = (int16_t)(VX * 1000); // mm/s
  int16_t v_y_tsushin = (int16_t)(VY * 1000);
  int16_t omega_tsushin = (int16_t)(Omega * 1000); // mrad/s

  uint8_t TxData_vel[8] = {};
  TxData_vel[0] = (uint8_t) ( v_x_tsushin >> 8);
  TxData_vel[1] = (uint8_t) ( v_x_tsushin & 0xff);
  TxData_vel[2] = (uint8_t) ( v_y_tsushin >> 8);
  TxData_vel[3] = (uint8_t) ( v_y_tsushin & 0xff);
  TxData_vel[4] = (uint8_t) ( omega_tsushin >> 8);
  TxData_vel[5] = (uint8_t) ( omega_tsushin & 0xff);

  //printf("Tx %d,%d,%d \r\n",v_x_tsushin,v_y_tsushin,omega_tsushin);
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &localTxHeader, TxData_vel) != HAL_OK){
    printf("add_message_vel is error\r\n");
    Error_Handler();
  }
}

void Servo_Tx(uint8_t one_or_zero)
{
  FDCAN_TxHeaderTypeDef localTxHeader = TxHeader; // グローバル初期設定をコピー
  localTxHeader.Identifier = servo_id;

  uint8_t TxData_Servo[8] = {};

  TxData_Servo[0] = one_or_zero ? (uint8_t)1 : (uint8_t)0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &localTxHeader, TxData_Servo) != HAL_OK){
    printf("add_message_kaishu is error\r\n");
    Error_Handler();
  }
}


int _write(int file,char *ptr,int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 10);
  return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  setbuf(stdout, NULL);
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
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM20_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_OK != interboard_comms_CAN_RxTxSettings_init(&TxHeader))
    Error_Handler();

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // printf("Hello World!\r\n");
    HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_SET); // LED ON
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(10);

    printf("x:%d, y:%d, yaw:%d\r\n", (int)x, (int)y, (int)yaw);
    printf("value1:%d, value2:%d, value3:%d\r\n", enc_value_1, enc_value_2, enc_value_3);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 79;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  htim8.Init.Prescaler = 79;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM20 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM20_Init(void)
{

  /* USER CODE BEGIN TIM20_Init 0 */

  /* USER CODE END TIM20_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM20_Init 1 */

  /* USER CODE END TIM20_Init 1 */
  htim20.Instance = TIM20;
  htim20.Init.Prescaler = 79;
  htim20.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim20.Init.Period = 65535;
  htim20.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim20.Init.RepetitionCounter = 0;
  htim20.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim20) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim20, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim20, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM20_Init 2 */

  /* USER CODE END TIM20_Init 2 */
  HAL_TIM_MspPostInit(&htim20);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Seg2_PC13_GPIO_Port, Seg2_PC13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BarLED_PC0_Pin|BarLED_PC1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BarLED_PA0_Pin|BarLED_PA1_Pin|BarLED_PA4_Pin|BarLED_PA5_Pin
                          |BarLED_PA6_Pin|BarLED_PA7_Pin|BarLED_PA8_Pin|BarLED_PA9_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Seg1_PB0_Pin|Seg1_PB1_Pin|Seg1_PB2_Pin|Seg2_PB10_Pin
                          |Seg2_PB11_Pin|Seg2_PB12_Pin|Seg2_PB13_Pin|Seg2_PB14_Pin
                          |Seg2_PB15_Pin|Seg1_PB4_Pin|Seg1_PB5_Pin|Seg1_PB6_Pin
                          |Seg1_PB7_Pin|Seg1_PB8_Pin|Seg2_PB9_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(buzzer_PA10_GPIO_Port, buzzer_PA10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Seg2_PC13_Pin BarLED_PC0_Pin BarLED_PC1_Pin */
  GPIO_InitStruct.Pin = Seg2_PC13_Pin|BarLED_PC0_Pin|BarLED_PC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BarLED_PA0_Pin BarLED_PA1_Pin BarLED_PA4_Pin BarLED_PA5_Pin
                           BarLED_PA6_Pin BarLED_PA7_Pin BarLED_PA8_Pin BarLED_PA9_Pin
                           buzzer_PA10_Pin */
  GPIO_InitStruct.Pin = BarLED_PA0_Pin|BarLED_PA1_Pin|BarLED_PA4_Pin|BarLED_PA5_Pin
                          |BarLED_PA6_Pin|BarLED_PA7_Pin|BarLED_PA8_Pin|BarLED_PA9_Pin
                          |buzzer_PA10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Seg1_PB0_Pin Seg1_PB1_Pin Seg1_PB2_Pin Seg2_PB10_Pin
                           Seg2_PB11_Pin Seg2_PB12_Pin Seg2_PB13_Pin Seg2_PB14_Pin
                           Seg2_PB15_Pin Seg1_PB4_Pin Seg1_PB5_Pin Seg1_PB6_Pin
                           Seg1_PB7_Pin Seg1_PB8_Pin Seg2_PB9_Pin */
  GPIO_InitStruct.Pin = Seg1_PB0_Pin|Seg1_PB1_Pin|Seg1_PB2_Pin|Seg2_PB10_Pin
                          |Seg2_PB11_Pin|Seg2_PB12_Pin|Seg2_PB13_Pin|Seg2_PB14_Pin
                          |Seg2_PB15_Pin|Seg1_PB4_Pin|Seg1_PB5_Pin|Seg1_PB6_Pin
                          |Seg1_PB7_Pin|Seg1_PB8_Pin|Seg2_PB9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_PC9_Pin SW2_PC10_Pin SW3_PC11_Pin SWtact_PC12_Pin */
  GPIO_InitStruct.Pin = SW1_PC9_Pin|SW2_PC10_Pin|SW3_PC11_Pin|SWtact_PC12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Board_LED_Pin */
  GPIO_InitStruct.Pin = Board_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void interboard_comms_CAN_filter_init(FDCAN_FilterTypeDef *Hfdcan_Filter_Settings)
{
  Hfdcan_Filter_Settings->IdType = FDCAN_STANDARD_ID;
  Hfdcan_Filter_Settings->FilterIndex = 0;
  Hfdcan_Filter_Settings->FilterType = FDCAN_FILTER_RANGE;
  Hfdcan_Filter_Settings->FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  Hfdcan_Filter_Settings->FilterID1 = 0x00;
  Hfdcan_Filter_Settings->FilterID2 = 0x7ff;
}

void interboard_comms_CAN_txheader_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  Htxheader->Identifier = 0x00;
  Htxheader->IdType = FDCAN_STANDARD_ID;
  Htxheader->TxFrameType = FDCAN_DATA_FRAME;
  Htxheader->DataLength = FDCAN_DLC_BYTES_8;
  Htxheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  Htxheader->FDFormat = FDCAN_FD_CAN;
  Htxheader->BitRateSwitch = FDCAN_BRS_ON;
  Htxheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  Htxheader->MessageMarker = 0;
}

HAL_StatusTypeDef interboard_comms_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *Htxheader)
{
  FDCAN_FilterTypeDef FDCAN_Filter_settings;
  interboard_comms_CAN_filter_init(&FDCAN_Filter_settings);
  interboard_comms_CAN_txheader_init(&TxHeader);
  if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter_settings))
  {
    printf("fdcan_configfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE))
  {
    printf("fdcan_configglobalfilter is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_Start(&hfdcan1))
  {
    printf("fdcan_start is error\r\n");
    return HAL_ERROR;
  }
  if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0))
  {
    printf("fdcan_activatenotification is error\r\n");
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef CAN_SEND(uint32_t CANID, uint8_t *txdata, FDCAN_HandleTypeDef *hfdcan, FDCAN_TxHeaderTypeDef *htxheader)
{
  htxheader->Identifier = CANID;
  if (HAL_OK != HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, htxheader, txdata))
  {
    printf("addmessage error\r\n");
    return HAL_ERROR;
  }
  return HAL_OK;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("Error");
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
