/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us);

int _write(int file, char *ptr, int len) {
   HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
   return len;
}

void wheel_on(int LT, int LB, int RB, int RT, int speed);
void wheel_stop(int off_wheel);
// ------------------------------------------------------통신------------------------------------------------------------
uint8_t rxData;
int turn_left = 2;
int turn_back_left = 0;
int move = 0;
int input_state = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   // 💡 라즈베리 파이 5가 연결된 USART1 통로로 데이터가 들어왔을 때만 실행합니다.
   if (huart->Instance == USART1)
   {
      // 허큘러스로 수신 확인 출력
//      char pcMsg[50];
//      sprintf(pcMsg, "[Nucleo] Received: '%c'\r\n", rxData);

      if (rxData == '1')// 뒷문 오픈
      {
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 400);  // 90도
      }
      else if (rxData == '0') // 뒷문 닫기
      {
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1200); // 0도
      }

      else if (rxData == 'r'){ //오른쪽 돌기
         turn_left = 2;
      }

      else if (rxData == 'l'){ //왼쪽 돌기
         turn_left = 1;
      }

      else if (rxData == 'p'){ //오른쪽 돌기
         turn_back_left = 2;
      }


      else if (rxData == 'q'){ //왼쪽 돌기
         turn_back_left = 1;
      }

      else if (rxData == 'f'){// 전진
         move = 1;
         input_state = 1;
      }

      else if (rxData == 'b'){//후진
         move = 2;
         input_state = 1;
      }

      else if (rxData == 'a'){//바로 좌회전
         move = 3;
         input_state = 1;
      }

      else if (rxData == 'd'){//바로 우회전
         move = 4;
         input_state = 1;
      }

      else if (rxData == 's'){// 정지
//         move = 5;
      }

      else if (rxData == 'o'){// 앞문 열기
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000); // 왼
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500); // 오
      }

      else if (rxData == 'c'){// 앞문 닫기
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1100);
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2400);
      }
      // 다음 수신 대기
      HAL_UART_Receive_IT(&huart1, &rxData, 1);
   }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART1)
   {
      __HAL_UART_CLEAR_OREFLAG(huart);

      HAL_UART_AbortReceive(huart);

      HAL_UART_Receive_IT(&huart1, &rxData, 1);

//      printf("UART Error Recovered\r\n");
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
   MX_USART2_UART_Init();
   MX_TIM1_Init();
   MX_TIM2_Init();
   MX_USART1_UART_Init();
   MX_TIM3_Init();
   /* USER CODE BEGIN 2 */
   //---------------------------------------------통신-------------------------------------------
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

   // 2. 초기 위치 0도
   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1200);
   HAL_Delay(500);

   // 3. USART1 인터럽트 수신 시작 (딱 한 번만)
   HAL_UART_Receive_IT(&huart1, &rxData, 1);

   // 4. 허큘러스로 준비 완료 메시지
   char *ready_msg = "=== Ready ===\r\n";
   HAL_UART_Transmit(&huart2, (uint8_t*)ready_msg, strlen(ready_msg), 100);

   //---------------------------------------------바퀴-------------------------------------------
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
   HAL_TIM_Base_Start(&htim2);
   uint32_t time = 0;
   float distance1 = 0.0;
   float distance2 = 0.0;
   int wheel1_moment;  // 왼쪽 위
   int wheel4_moment;  // 오른쪽 위
   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   // 0: 전진, 2: 후진, 3: 좌회전 5: 정지
   int drive_state = 0;
   int move_state = 0;
   int rotate_time = 2500;
   while (1)
   {

      /* 1. 초음파 센서 거리 측정 */
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
      delay_us(10);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
      while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);

      time = __HAL_TIM_GET_COUNTER(&htim2);
      distance1 = time * 0.034f / 2.0f;

      HAL_Delay(50);

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
      delay_us(10);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
      while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);

      time = __HAL_TIM_GET_COUNTER(&htim2);
      distance2 = time * 0.034f / 2.0f;

      printf("state: %d | distance1: %.1f cm | distance2: %.1f cm \r\n", drive_state,distance1,distance2);

      //================================================================
            // ▼▼▼ 여기부터 "움직이는 로직" — 초음파 값(distance1, distance2)은
            //     위에서 이미 구해진 걸 그대로 사용만 함 ▼▼▼
            //================================================================

            // ==========================================================
            // [상황 A] 장애물 감지 → 후진 시작  ***최우선순위, 무조건 실행***
            // 전진 중(drive_state==0)에 30cm 이내 장애물 감지되면
            // 쓰레기를 쫓고 있던 중이어도 무조건 정지 후 후진으로 전환.
            // ==========================================================
            if ((drive_state == 0 && distance1 <= 30) || (drive_state == 0 && distance2 <= 30)) {

               // --- 서서히 감속 정지 ---
               for(int speed = 900; speed >= 0; speed -= 225) {
                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
                  HAL_Delay(60);
               }
               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
               HAL_Delay(500);


               // --- 후진 시작 (폰에서 미리 받은 turn_back_left 값에 따라 방향 다르게) ---
               if(turn_back_left == 1){
                  wheel1_moment = 0; wheel4_moment = 1;
                  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
                  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
                  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,!wheel4_moment);
                  for(int speed = 0; speed <= 1000; speed += 250) {
                     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
                     HAL_Delay(60);
                  }
                  turn_back_left = 0;
               }
               else if(turn_back_left == 2){
                  wheel1_moment = 0; wheel4_moment = 1;
                  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
                  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
                  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,!wheel4_moment);
                  for(int speed = 0; speed <= 1000; speed += 250) {
                     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
                     HAL_Delay(60);
                  }
                  turn_back_left = 0;
               }
               else{
                  wheel_on(0, 0, 0, 0, 1000); // 그냥 직진 후진
               }

               // [중요] 벽 회피가 최우선이므로, 후진 진입 시 YOLO 추적 흔적은 리셋
               // -> 후진 도중 쓰레기가 다시 보이면 아래 [상황 B]에서 새로 판단하게 함
               move = 2;
               move_state = 2;
               drive_state = 2;
               input_state = 0;
            }

            // ==========================================================
            // [상황 B] 후진 중(2) 공간 확보(60cm 이상)
            // 여기서 우선순위 2번 규칙 적용:
            //   - input_state == 0 (라즈베리파이가 쓰레기 추적 명령을 안 보낸 상태)
            //     → 자동 탈출 회전 실행 (폰에서 설정한 turn_left 방향)
            //   - input_state == 1 (후진 중에 라즈베리파이가 'a'/'d'/'f' 등을 보냄
            //     = 쓰레기 감지됨) → 자동 탈출 회전 생략하고 쓰레기 추적 명령을
            //     아래 [명령 처리] 블록이 그대로 실행하게 넘김
            // ==========================================================
            else if ((drive_state == 2 && distance1 >= 60) && (drive_state == 2 && distance2 >= 60)) {

               for(int speed = 900; speed >= 0; speed -= 225) {
                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
                  HAL_Delay(60);
               }
               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
               HAL_Delay(500);

               if(input_state == 0){
                  // --- 쓰레기 추적 명령이 없었음 → 폰에서 설정한 방향으로 자동 탈출 회전 ---
                  if(turn_left == 1){
                     wheel1_moment = 1; wheel4_moment = 0;
                     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
                     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
                     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
                     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,wheel4_moment);
                     for(int speed = 0; speed <= 1000; speed += 250) {
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
                        HAL_Delay(60);
                     }
                     HAL_Delay(rotate_time);
                     for(int speed = 1000; speed >= 0; speed -= 250) {
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
                        HAL_Delay(40);
                     }
                     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
                     HAL_Delay(300);
                  }
                  else if(turn_left == 2){
                     wheel1_moment = 0; wheel4_moment = 1;
                     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
                     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
                     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
                     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,wheel4_moment);
                     for(int speed = 0; speed <= 1000; speed += 250) {
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
                        HAL_Delay(60);
                     }
                     HAL_Delay(rotate_time);
                     for(int speed = 1000; speed >= 0; speed -= 250) {
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
                        HAL_Delay(40);
                     }
                     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
                     HAL_Delay(300);
                  }
                  wheel_on(1, 1, 1, 1, 900);

                  // [수정] move/move_state도 같이 맞춰줘야 아래 "기본 전진 유지" 블록이
                  //        정상 동작하고, 다음 move!=move_state 판정에서 중복 실행 안 됨
                  move = 1;
                  move_state = 1;
                  drive_state = 0;
               }
               else {
                  // --- input_state==1: 쓰레기 감지되어 라즈베리파이가 이미 명령을 보낸 상태 ---
                  // 자동 탈출 회전 하지 않고, 그냥 넘어가서
                  // 아래 [명령 처리] 블록이 라즈베리파이가 보낸 a/d/f 명령을 그대로 실행하게 둠
                  // (= 쓰레기 쪽으로 이동, 우선순위 2번 규칙)
               }
            }

            else {
               HAL_Delay(10);
            }

            // ==========================================================
            // [명령 처리] 라즈베리파이(YOLO 추적)에서 온 move 값이 바뀔 때 1회 실행
            // - 좌/우회전(move==3,4)은 drive_state도 같이 바꿔서 계속 유지되게 함
            //   (예전엔 20ms만 돌고 강제로 전진 전환돼서 회전이 사실상 안 먹혔음)
            // - 명령 실행 직후 INFERENCE_WAIT_MS만큼 대기
            //   → 라즈베리파이5는 YOLO 추론이 느리므로, 로봇이 움직인 뒤
            //     다음 프레임을 추론할 시간을 벌어주기 위함
            // ==========================================================
            #define INFERENCE_WAIT_MS   200   // 필요시 라즈베리파이 추론속도 보고 조절

            if(move != move_state){
               wheel_stop(1); wheel_stop(2); wheel_stop(3); wheel_stop(4);
               HAL_Delay(500);

               if(move == 1){                       // 전진
                  wheel_on(1,1,1,1,1000);
                  drive_state = 0;
                  input_state = 0;
               }
               else if(move == 2){                  // 후진
                  wheel_on(0,0,0,0,1000);
                  drive_state = 2;
                  input_state = 0;
               }
               else if(move == 3){                  // 좌회전 - 다음 명령 올 때까지 유지
                  wheel_on(1,1,1,0,700);
                  drive_state = 3;
                  input_state = 0;
               }
               else if(move == 4){                  // 우회전 - 다음 명령 올 때까지 유지
                  wheel_on(0,1,1,1,700);
                  drive_state = 4;
                  input_state = 0;
               }
               else if(move == 5){                  // 정지
                  wheel_stop(1); wheel_stop(2); wheel_stop(3); wheel_stop(4);
                  drive_state = 5;
                  input_state = 0;
               }

               move_state = move;

               // [추가] 추론 시간 확보용 대기 - 움직임 명령 실행 후에만 적용
               HAL_Delay(INFERENCE_WAIT_MS);
            }

            // ==========================================================
            // [기본 전진 유지] drive_state==0이고 실제로 전진 명령 상태(move==1)일 때만
            // 매 루프마다 바퀴 값을 재확인해서 세팅
            // [수정] "&& move == 1" 조건 추가
            //   → 예전엔 drive_state==0이면 무조건 실행돼서, 방금 세팅한 좌/우회전
            //     방향을 바로 다음 줄에서 다시 전진으로 덮어써버렸음
            //     (쓰레기 봐도 회전 안 하고 그냥 직진하던 버그의 핵심 원인)
            // ==========================================================
            if(drive_state == 0 && move == 1){
               wheel1_moment = 1;
               wheel4_moment = 1;

               HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
               HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
               HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
               HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,wheel4_moment);

               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
            }

            //================================================================
            // ▲▲▲ 움직이는 로직 끝 ▲▲▲
            //================================================================

      //--------------------------------- 건드리지 마삼 ---------------------------------------

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

   /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
   RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
   htim1.Init.Prescaler = 71;
   htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim1.Init.Period = 999;
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
   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

   /* USER CODE BEGIN TIM2_Init 1 */

   /* USER CODE END TIM2_Init 1 */
   htim2.Instance = TIM2;
   htim2.Init.Prescaler = 72-1;
   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim2.Init.Period = 65535;
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
   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
   {
      Error_Handler();
   }
   /* USER CODE BEGIN TIM2_Init 2 */

   /* USER CODE END TIM2_Init 2 */

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

   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};
   TIM_OC_InitTypeDef sConfigOC = {0};

   /* USER CODE BEGIN TIM3_Init 1 */

   /* USER CODE END TIM3_Init 1 */
   htim3.Instance = TIM3;
   htim3.Init.Prescaler = 72-1;
   htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim3.Init.Period = 20000-1;
   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
   {
      Error_Handler();
   }
   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
   {
      Error_Handler();
   }
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
   if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
   {
      Error_Handler();
   }
   if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
   {
      Error_Handler();
   }
   /* USER CODE BEGIN TIM3_Init 2 */

   /* USER CODE END TIM3_Init 2 */
   HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

   /* USER CODE BEGIN USART1_Init 0 */

   /* USER CODE END USART1_Init 0 */

   /* USER CODE BEGIN USART1_Init 1 */

   /* USER CODE END USART1_Init 1 */
   huart1.Instance = USART1;
   huart1.Init.BaudRate = 115200;
   huart1.Init.WordLength = UART_WORDLENGTH_8B;
   huart1.Init.StopBits = UART_STOPBITS_1;
   huart1.Init.Parity = UART_PARITY_NONE;
   huart1.Init.Mode = UART_MODE_TX_RX;
   huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart1.Init.OverSampling = UART_OVERSAMPLING_16;
   if (HAL_UART_Init(&huart1) != HAL_OK)
   {
      Error_Handler();
   }
   /* USER CODE BEGIN USART1_Init 2 */

   /* USER CODE END USART1_Init 2 */

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
   if (HAL_UART_Init(&huart2) != HAL_OK)
   {
      Error_Handler();
   }
   /* USER CODE BEGIN USART2_Init 2 */

   /* USER CODE END USART2_Init 2 */

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
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
         |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
         |GPIO_PIN_7, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

   /*Configure GPIO pin : B1_Pin */
   GPIO_InitStruct.Pin = B1_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

   /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
   GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
         |GPIO_PIN_4|GPIO_PIN_5;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   /*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin
                           PA7 */
   GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
         |GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   /*Configure GPIO pin : PB0 */
   GPIO_InitStruct.Pin = GPIO_PIN_0;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /*Configure GPIO pins : PB6 PB7 */
   GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   /* EXTI interrupt init*/
   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

   /* USER CODE BEGIN MX_GPIO_Init_2 */

   /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void delay_us(uint16_t us){
   __HAL_TIM_SET_COUNTER(&htim2, 0);
   while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void wheel_on(int LT, int LB, int RB, int RT, int speed) {
   int wheel1_moment = LT;   int wheel2_moment = LB;
   int wheel3_moment = RB;   int wheel4_moment = RT;

   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,!wheel2_moment);
   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,wheel2_moment);
   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,wheel3_moment);
   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,!wheel3_moment);
   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,wheel4_moment);

   if (speed <= 0) {
      speed = 900;
   } else {
      speed = speed;
   }

   for(int speeds = 0; speeds <= speed; speeds += speed/4) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speeds);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speeds);
      HAL_Delay(60);
   }

}

void wheel_stop(int off_wheel){
   if(off_wheel == 1){
      int wheel1_moment = 1;
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,!wheel1_moment);
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
   }
   else if(off_wheel == 2){
      int wheel2_moment = 1;
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,!wheel2_moment);
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,!wheel2_moment);
   }
   else if(off_wheel == 3){
      int wheel3_moment = 1;
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,!wheel3_moment);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,!wheel3_moment);
   }
   else if(off_wheel == 4){
      int wheel4_moment = 1;
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,!wheel4_moment);
   }

   for(int speeds = 1000; speeds >= 0; speeds -= 200) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speeds);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speeds);
      HAL_Delay(60);
   }
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
////================================================================
//      // ▼▼▼ 여기부터 "움직이는 로직" — 초음파 값(distance1, distance2)은
//      //     위에서 이미 구해진 걸 그대로 사용만 함 ▼▼▼
//      //================================================================
//
//      // ==========================================================
//      // [상황 A] 장애물 감지 → 후진 시작  ***최우선순위, 무조건 실행***
//      // 전진 중(drive_state==0)에 30cm 이내 장애물 감지되면
//      // 쓰레기를 쫓고 있던 중이어도 무조건 정지 후 후진으로 전환.
//      // ==========================================================
//      if ((drive_state == 0 && distance1 <= 30) || (drive_state == 0 && distance2 <= 30)) {
//
//         // --- 서서히 감속 정지 ---
//         for(int speed = 900; speed >= 0; speed -= 225) {
//            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
//            HAL_Delay(60);
//         }
//         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//         HAL_Delay(500);
//
//         // --- 후진 시작 (폰에서 미리 받은 turn_back_left 값에 따라 방향 다르게) ---
//         if(turn_back_left == 1){
//            wheel1_moment = 0; wheel4_moment = 1;
//            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
//            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
//            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
//            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,!wheel4_moment);
//            for(int speed = 0; speed <= 1000; speed += 250) {
//               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//               HAL_Delay(60);
//            }
//            turn_back_left = 0;
//         }
//         else if(turn_back_left == 2){
//            wheel1_moment = 0; wheel4_moment = 1;
//            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
//            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
//            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
//            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,wheel4_moment);
//            for(int speed = 0; speed <= 1000; speed += 250) {
//               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//               HAL_Delay(60);
//            }
//            turn_back_left = 0;
//         }
//         else{
//            wheel_on(0, 0, 0, 0, 1000); // 그냥 직진 후진
//         }
//
//         // [중요] 벽 회피가 최우선이므로, 후진 진입 시 YOLO 추적 흔적은 리셋
//         // -> 후진 도중 쓰레기가 다시 보이면 아래 [상황 B]에서 새로 판단하게 함
//         move = 2;
//         move_state = 2;
//         drive_state = 2;
//         input_state = 0;
//      }
//
//      // ==========================================================
//      // [상황 B] 후진 중(2) 공간 확보(60cm 이상)
//      // 여기서 우선순위 2번 규칙 적용:
//      //   - input_state == 0 (라즈베리파이가 쓰레기 추적 명령을 안 보낸 상태)
//      //     → 자동 탈출 회전 실행 (폰에서 설정한 turn_left 방향)
//      //   - input_state == 1 (후진 중에 라즈베리파이가 'a'/'d'/'f' 등을 보냄
//      //     = 쓰레기 감지됨) → 자동 탈출 회전 생략하고 쓰레기 추적 명령을
//      //     아래 [명령 처리] 블록이 그대로 실행하게 넘김
//      // ==========================================================
//      else if ((drive_state == 2 && distance1 >= 60) && (drive_state == 2 && distance2 >= 60)) {
//
//         for(int speed = 900; speed >= 0; speed -= 225) {
//            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
//            HAL_Delay(60);
//         }
//         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//         HAL_Delay(500);
//
//         if(input_state == 0){
//            // --- 쓰레기 추적 명령이 없었음 → 폰에서 설정한 방향으로 자동 탈출 회전 ---
//            if(turn_left == 1){
//               wheel1_moment = 1; wheel4_moment = 0;
//               HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
//               HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
//               HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,wheel4_moment);
//               HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,!wheel4_moment);
//               for(int speed = 0; speed <= 1000; speed += 250) {
//                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
//                  HAL_Delay(60);
//               }
//               HAL_Delay(500);
//               for(int speed = 1000; speed >= 0; speed -= 250) {
//                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
//                  HAL_Delay(40);
//               }
//               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//               HAL_Delay(300);
//            }
//            else if(turn_left == 2){
//               wheel1_moment = 0; wheel4_moment = 1;
//               HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,!wheel1_moment);
//               HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,wheel1_moment);
//               HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
//               HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,wheel4_moment);
//               for(int speed = 0; speed <= 1000; speed += 250) {
//                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
//                  HAL_Delay(60);
//               }
//               HAL_Delay(500);
//               for(int speed = 1000; speed >= 0; speed -= 250) {
//                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
//                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);
//                  HAL_Delay(40);
//               }
//               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//               __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//               HAL_Delay(300);
//            }
//            wheel_on(1, 1, 1, 1, 900);
//
//            // [수정] move/move_state도 같이 맞춰줘야 아래 "기본 전진 유지" 블록이
//            //        정상 동작하고, 다음 move!=move_state 판정에서 중복 실행 안 됨
//            move = 1;
//            move_state = 1;
//            drive_state = 0;
//         }
//         else {
//            // --- input_state==1: 쓰레기 감지되어 라즈베리파이가 이미 명령을 보낸 상태 ---
//            // 자동 탈출 회전 하지 않고, 그냥 넘어가서
//            // 아래 [명령 처리] 블록이 라즈베리파이가 보낸 a/d/f 명령을 그대로 실행하게 둠
//            // (= 쓰레기 쪽으로 이동, 우선순위 2번 규칙)
//         }
//      }
//
//      else {
//         HAL_Delay(10);
//      }
//
//      // ==========================================================
//      // [명령 처리] 라즈베리파이(YOLO 추적)에서 온 move 값이 바뀔 때 1회 실행
//      // - 좌/우회전(move==3,4)은 drive_state도 같이 바꿔서 계속 유지되게 함
//      //   (예전엔 20ms만 돌고 강제로 전진 전환돼서 회전이 사실상 안 먹혔음)
//      // - 명령 실행 직후 INFERENCE_WAIT_MS만큼 대기
//      //   → 라즈베리파이5는 YOLO 추론이 느리므로, 로봇이 움직인 뒤
//      //     다음 프레임을 추론할 시간을 벌어주기 위함
//      // ==========================================================
//      #define INFERENCE_WAIT_MS   200   // 필요시 라즈베리파이 추론속도 보고 조절
//
//      if(move != move_state){
//         wheel_stop(1); wheel_stop(2); wheel_stop(3); wheel_stop(4);
//         HAL_Delay(500);
//
//         if(move == 1){                       // 전진
//            wheel_on(1,1,1,1,1000);
//            drive_state = 0;
//            input_state = 0;
//         }
//         else if(move == 2){                  // 후진
//            wheel_on(0,0,0,0,1000);
//            drive_state = 2;
//            input_state = 0;
//         }
//         else if(move == 3){                  // 좌회전 - 다음 명령 올 때까지 유지
//            wheel_on(1,1,1,0,700);
//            drive_state = 3;
//            input_state = 0;
//         }
//         else if(move == 4){                  // 우회전 - 다음 명령 올 때까지 유지
//            wheel_on(0,1,1,1,700);
//            drive_state = 4;
//            input_state = 0;
//         }
//         else if(move == 5){                  // 정지
//            wheel_stop(1); wheel_stop(2); wheel_stop(3); wheel_stop(4);
//            drive_state = 5;
//            input_state = 0;
//         }
//
//         move_state = move;
//
//         // [추가] 추론 시간 확보용 대기 - 움직임 명령 실행 후에만 적용
//         HAL_Delay(INFERENCE_WAIT_MS);
//      }
//
//      // ==========================================================
//      // [기본 전진 유지] drive_state==0이고 실제로 전진 명령 상태(move==1)일 때만
//      // 매 루프마다 바퀴 값을 재확인해서 세팅
//      // [수정] "&& move == 1" 조건 추가
//      //   → 예전엔 drive_state==0이면 무조건 실행돼서, 방금 세팅한 좌/우회전
//      //     방향을 바로 다음 줄에서 다시 전진으로 덮어써버렸음
//      //     (쓰레기 봐도 회전 안 하고 그냥 직진하던 버그의 핵심 원인)
//      // ==========================================================
//      if(drive_state == 0 && move == 1){
//         wheel1_moment = 1;
//         wheel4_moment = 1;
//
//         HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,wheel1_moment);
//         HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,!wheel1_moment);
//         HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,!wheel4_moment);
//         HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,wheel4_moment);
//
//         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
//         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
//      }
//
//      //================================================================
//      // ▲▲▲ 움직이는 로직 끝 ▲▲▲
//      //================================================================
