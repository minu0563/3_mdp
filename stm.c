/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN 0 */

void delay_us(uint16_t us);

int _write(int file, char *ptr, int len)
{
   HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
   return len;
}

/* ================================================================
 * ★★★ 핵심 설계 변경: 연속 주행 -> 펄스(버스트) 주행 ★★★
 *
 * [기존 문제]
 * 라즈베리파이는 YOLO 추론이 끝날 때마다(수백 ms 간격) 명령을 보내는데,
 * 기존 코드는 명령을 받으면 "다음 명령이 올 때까지" 그 동작을 계속했다.
 * 즉 추론 1프레임 사이(예: 250ms) 내내 로봇이 눈을 감은 채 회전/전진했고,
 * 그래서 목표를 지나쳐버리는 오버슈트가 발생했다.
 * pwm을 더 낮출 수 없으므로, "속도"가 아니라 "움직이는 시간"을 줄여야 한다.
 *
 * [새 방식]
 * 추적 명령(A/D/F) 1개 = 아주 짧은 펄스 1회.
 *   -> PULSE_*_MS 만큼만 모터를 돌리고 자동으로 정지한다.
 *   -> 다음 명령이 올 때까지 로봇은 멈춰서 대기한다.
 *   -> 결과적으로 "라파5가 한 프레임 볼 때마다 로봇이 한 걸음씩" 움직인다.
 *      추론이 느려지면 로봇도 자동으로 느려지므로 항상 동기화된다.
 *
 * [워치독]
 * 추적 모드에서 CMD_WATCHDOG_MS 동안 명령이 한 번도 안 오면
 * (라파5 다운, 시리얼 끊김, 네트워크 지연 등) 무조건 정지시킨다.
 * 로봇이 통제 불능으로 계속 달리는 상황을 원천 차단하는 안전장치.
 *
 * [튜닝 가이드]
 * - 여전히 오버슈트(목표를 지나침) -> PULSE_*_MS 를 줄인다.
 * - 너무 찔끔거려서 답답함        -> PULSE_*_MS 를 늘린다.
 * - 로봇이 자꾸 멈춰있다          -> CMD_WATCHDOG_MS 를 늘린다
 *                                   (라파5 추론이 그만큼 느리다는 뜻)
 * ================================================================ */
/* ================================================================
 * ★ 회전 펄스 2단계
 *
 * 펄스가 너무 짧으면 한 번에 몇 도씩만 돌아서, 로봇이 목표를
 * 향해 "지글거리며" 떠는 것처럼 보인다. 그렇다고 무조건 키우면
 * 이번엔 목표를 지나쳐버린다(오버슈트).
 * 그래서 화면상 좌우 오차 크기에 따라 두 단계로 나눈다:
 *   - 많이 틀어져 있음 -> BIG 펄스로 시원하게 돌린다
 *   - 거의 맞았음      -> FINE 펄스로 살살 맞춘다
 * 파이썬이 오차를 보고 L/R(큰 회전) 또는 A/D(미세 회전)를 보낸다.
 *
 * [튜닝]
 *  아직도 찔끔거려 보이면 -> PULSE_TURN_BIG_MS 를 늘린다
 *  큰 회전에서 지나치면   -> PULSE_TURN_BIG_MS 를 줄인다
 *  중앙 근처에서 떨면     -> PULSE_TURN_FINE_MS 를 줄인다
 * ================================================================ */
#define PULSE_TURN_BIG_MS     240    /* 큰 회전 (오차가 클 때) */
#define PULSE_TURN_FINE_MS     60    /* 미세 회전 (거의 정렬됐을 때) */
#define PULSE_FWD_MS          140    /* 전진 1회 펄스 길이(ms) */
#define PULSE_BACK_MS         160    /* 후진 1회 펄스 길이(ms)
                                      * 물체가 화면 너무 아래(= 로봇에 너무 근접)로
                                      * 내려와서 카메라 사각지대에 들어가기 직전일 때,
                                      * 뒤로 살짝 빼서 물체를 다시 화면 위쪽으로
                                      * 올려놓기 위한 펄스. 전진보다 살짝 길게 잡아야
                                      * 정지 마찰을 이기고 실제로 후진이 된다. */
#define CMD_WATCHDOG_MS       700    /* 이 시간 동안 추적 명령 없으면 강제 정지 */

/* 초음파/디버그 출력 주기 (매 루프마다 하면 루프가 느려져서 펄스 타이밍이 밀림) */
#define ULTRASONIC_INTERVAL_MS 200
#define ECHO_TIMEOUT_MS         30   /* 에코 대기 최대 시간. 초과 시 측정 실패 처리 */
#define PRINT_INTERVAL_MS      500

#define CMD_SETTLE_MS           60   /* 수동 명령 전환 시 짧은 안정화 */

/* ================================================================
 * ★★★ 팔(문) 동작 - 양쪽 동시 닫힘 (원통형 대응) ★★★
 *
 * [왜 동시에 닫는가]
 * 캔/페트병은 원통이고 팔은 각진 평면이다. 한쪽 팔만 밀어서
 * 반대쪽 '벽'에 붙이는 방식(sweep)은 미는 힘의 작용선이 물체
 * 무게중심을 정확히 지나지 않는 순간 회전 토크가 생기고,
 * 원통은 그 토크로 굴러서 팔 밖으로 빠져나간다.
 *
 * 양쪽을 동시에 같은 속도로 닫으면 두 접촉면의 힘이 서로
 * 상쇄되어 회전 토크가 생기지 않는다. 선반 척(chuck)이 원통을
 * 물듯이, 닫히는 축 방향으로 물체가 스스로 중앙에 정렬된다.
 *
 * [여전히 천천히 닫는 이유]
 * 목표 펄스폭을 한 번에 써버리면 서보가 전속력으로 휘둘러져서
 * '쓸어담기'가 아니라 '쳐내기'가 된다. 여러 단계로 나눠서
 * 조금씩 이동시켜야 물체를 밀어 넣는 동작이 된다.
 *
 * ★ 남은 한계: 이 방식도 닫히는 축(앞뒤)만 구속한다.
 *   좌우는 여전히 열려 있으므로, 물체가 옆으로 굴러 빠지는 건
 *   측벽 같은 하드웨어 없이는 완전히 막을 수 없다.
 *
 * ARM_A_CH / ARM_B_CH 는 앞/뒤 팔 두 개. 동시에 움직이므로
 * 어느 쪽이 어느 채널인지는 동작에 영향을 주지 않는다.
 * ================================================================ */
#define ARM_A_CH           TIM_CHANNEL_3   /* 팔 1 */
#define ARM_B_CH           TIM_CHANNEL_2   /* 팔 2 */

#define ARM_A_OPEN         1100
#define ARM_A_CLOSE        2500
#define ARM_B_OPEN         2400
#define ARM_B_CLOSE        1100

/* 닫을 때 몇 단계로 나눠 움직일지 / 각 단계 사이 지연(ms)
 *
 * ★ 총 닫힘 시간 = ARM_CLOSE_STEPS * ARM_CLOSE_STEP_MS
 *   현재: 30 * 25 = 750ms. 양쪽이 동시에 움직이므로 이게 곧
 *   전체 닫힘 시간이다 (순차 방식일 때의 약 1.7초에서 단축됨).
 *
 * 팔이 너무 빨리 닫혀서 쓰레기를 쳐내면 두 값을 더 키운다.
 * 반대로 너무 느려서 답답하면 STEP_MS를 줄인다.
 * (STEPS를 줄이면 움직임이 뚝뚝 끊겨 보이므로 STEP_MS부터 조절할 것) */
#define ARM_CLOSE_STEPS      30
#define ARM_CLOSE_STEP_MS    25

/* 여는 건 빨라도 상관없다 (쓰레기와 접촉하지 않는 방향) */
#define ARM_OPEN_STEPS        6
#define ARM_OPEN_STEP_MS     12

/* ================================================================
 * ★ 's' 긴급 정지(estop)
 * ISR에서 즉시 PWM 레지스터를 0으로 만들어 물리적으로 모터를 멈추고,
 * estop_request 플래그를 세팅한다. 메인 루프 최상단에서 가장 먼저 검사한다.
 * ================================================================ */
volatile int estop_request = 0;

static void delay_ms_check_estop(uint32_t ms)
{
   uint32_t start = HAL_GetTick();
   while ((HAL_GetTick() - start) < ms)
   {
      if (estop_request) return;
   }
}

/* ================================================================
 * ★ 두 서보를 '동시에' 목표 위치까지 여러 단계로 나눠 이동시킨다.
 *
 * 한 루프 안에서 두 채널을 같은 단계 비율로 함께 갱신하는 것이
 * 핵심이다. 채널을 하나씩 순차로 램프하면 결국 한쪽이 먼저 닫히는
 * 것과 같아져서, 원통형 물체에 회전 토크가 생긴다.
 * ================================================================ */
static void arm_ramp_both(int a_from, int a_to, int b_from, int b_to,
                          int steps, uint32_t step_ms)
{
   if (steps <= 0) steps = 1;

   for (int i = 1; i <= steps; i++)
   {
      if (estop_request) return;

      int a_val = a_from + ((a_to - a_from) * i) / steps;
      int b_val = b_from + ((b_to - b_from) * i) / steps;

      /* 두 채널을 연속으로 써서 사실상 동시에 움직이게 한다 */
      __HAL_TIM_SET_COMPARE(&htim3, ARM_A_CH, a_val);
      __HAL_TIM_SET_COMPARE(&htim3, ARM_B_CH, b_val);

      delay_ms_check_estop(step_ms);
   }

   if (!estop_request)
   {
      __HAL_TIM_SET_COMPARE(&htim3, ARM_A_CH, a_to);
      __HAL_TIM_SET_COMPARE(&htim3, ARM_B_CH, b_to);
   }
}

/* 팔 열기: 양쪽 동시에 벌린다.
 * 여는 방향은 물체와 부딪히지 않으므로 빠르게 해도 된다. */
static void arm_open(void)
{
   arm_ramp_both(ARM_A_CLOSE, ARM_A_OPEN,
                 ARM_B_CLOSE, ARM_B_OPEN,
                 ARM_OPEN_STEPS, ARM_OPEN_STEP_MS);
}

/* ★ 팔 닫기: 양쪽을 동시에, 천천히 오므린다.
 * 두 접촉면의 힘이 상쇄되어 원통형 물체가 회전하지 않고
 * 닫히는 축 방향으로 스스로 중앙에 정렬된다. */
static void arm_close_sweep(void)
{
   arm_ramp_both(ARM_A_OPEN, ARM_A_CLOSE,
                 ARM_B_OPEN, ARM_B_CLOSE,
                 ARM_CLOSE_STEPS, ARM_CLOSE_STEP_MS);
}

/* 즉시 닫기 (정지/초기화용. 물체를 담는 동작이 아니므로 램프 불필요) */
static void arm_close_fast(void)
{
   __HAL_TIM_SET_COMPARE(&htim3, ARM_A_CH, ARM_A_CLOSE);
   __HAL_TIM_SET_COMPARE(&htim3, ARM_B_CH, ARM_B_CLOSE);
}

static void wheel_drive(int left_dir, int left_speed, int right_dir, int right_speed)
{
   if (left_speed  < 0)   left_speed  = 0;
   if (left_speed  > 999) left_speed  = 999;
   if (right_speed < 0)   right_speed = 0;
   if (right_speed > 999) right_speed = 999;

   if (left_dir) {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, left_speed);
   } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, left_speed);
   }

   if (right_dir) {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right_speed);
   } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right_speed);
   }
}

void wheel_stop_all(void)
{
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}

/* 램프(가속) 주행 - 수동/자율주행 등 "계속 달리는" 동작에만 사용.
 * 추적 펄스에는 쓰지 않는다 (램프에만 240ms 넘게 걸려서 펄스가 성립 안 됨). */
void wheel_on(int left_dir, int right_dir, int speed)
{
   if (speed <= 0) speed = 900;

   int step = speed / 4;
   if (step <= 0) step = speed;

   for (int s = 0; s <= speed; s += step) {
      if (estop_request) { wheel_stop_all(); return; }
      wheel_drive(left_dir, s, right_dir, s);
      delay_ms_check_estop(60);
   }

   if (estop_request) { wheel_stop_all(); return; }
   wheel_drive(left_dir, speed, right_dir, speed);
}

void wheel_decel_stop(int forward)
{
   for (int s = 900; s >= 0; s -= 225) {
      if (estop_request) break;

      if (forward) {
         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, s);
         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, s);
      } else {
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, s);
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s);
      }
      delay_ms_check_estop(60);
   }
   wheel_stop_all();
}

/* ================================================================
 * 초음파 측정 (타임아웃 포함)
 *
 * ★ 기존 코드의 while(ReadPin(...)) 무한 대기는 센서가 응답하지 않으면
 *   메인 루프가 영원히 갇힌다. 그러면 estop 플래그가 세팅돼도 루프가
 *   그걸 검사하러 오지 못해서 로봇이 완전히 먹통이 된다.
 *   여기서는 ECHO_TIMEOUT_MS 초과 시 -1을 리턴하고 빠져나온다.
 * 리턴 0 = 성공, -1 = 측정 실패(센서 이상/타임아웃/정지요청)
 * ================================================================ */
static int us_measure(GPIO_TypeDef *trig_port, uint16_t trig_pin,
                      GPIO_TypeDef *echo_port, uint16_t echo_pin,
                      float *out_cm)
{
   uint32_t t0;

   HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_SET);
   delay_us(10);
   HAL_GPIO_WritePin(trig_port, trig_pin, GPIO_PIN_RESET);

   t0 = HAL_GetTick();
   while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_RESET) {
      if ((HAL_GetTick() - t0) > ECHO_TIMEOUT_MS) return -1;
      if (estop_request) return -1;
   }

   __HAL_TIM_SET_COUNTER(&htim4, 0);

   t0 = HAL_GetTick();
   while (HAL_GPIO_ReadPin(echo_port, echo_pin) == GPIO_PIN_SET) {
      if ((HAL_GetTick() - t0) > ECHO_TIMEOUT_MS) return -1;
      if (estop_request) return -1;
   }

   *out_cm = __HAL_TIM_GET_COUNTER(&htim4) * 0.034f / 2.0f;
   return 0;
}

/* ================================================================
 * 상태 변수
 * ★ ISR과 메인 루프가 함께 쓰는 변수는 전부 volatile.
 *   기존 코드는 estop/eat 관련만 volatile이고 move, front_door 등이
 *   빠져 있었다. 최적화가 걸리면 메인 루프가 이 값을 레지스터에
 *   캐싱해서 UART로 보낸 명령이 반영 안 되는 간헐적 버그가 생긴다.
 * ================================================================ */
uint8_t rxData;

volatile int turn_left        = 1;
volatile int turn_back_left   = 0;
volatile int move             = 0;
volatile int input_state      = 0;
volatile int front_door       = 0;
volatile int last_front_door_state = 0;
volatile char last_rx_char    = 0;
volatile uint8_t rx_print_flag = 0;

/* 추적(펄스) 관련 */
volatile int      pulse_request   = 0;  /* 0=없음 1=좌 2=우 3=전진 (ISR이 세팅) */
volatile int      tracking_mode   = 0;  /* 추적 명령을 받고 있는 중인가 */
volatile uint32_t last_track_tick = 0;  /* 마지막 추적 명령 수신 시각 */

static int      pulse_active   = 0;     /* 펄스 주행 중인가 */
static uint32_t pulse_end_tick = 0;     /* 이 시각이 되면 정지 */

/* eat 시퀀스 */
volatile int eat_request = 0;
volatile int arm_busy    = 0;

/* ================================================================
 * ★ 추적 중 근접 장애물(사람 등) 알림용
 * distance <= 30cm 인 동안 'h'를 한 번만 보내고, 벗어나면 리셋해서
 * 다음 근접 시 다시 알릴 수 있게 한다 (매 200ms마다 스팸 전송 방지).
 * ================================================================ */
volatile int obstacle_alert_sent = 0;

/* ================================================================
 * ★ EAT 시퀀스 타이밍 (반드시 실측)
 *
 * OPEN_HOLD_MS : 문을 연 채로 제자리 정지하는 시간.
 *   라파5가 이미 접근을 끝내고 'e'를 보낸 것이므로 길 이유가 없다.
 *   길면 그동안 아무것도 안 하고 서 있는 낭비 시간이 된다.
 *
 * EAT_TRAVEL_MS : 문이 열린 채로 실제 스쿱 전진하는 시간.
 *   -> 물체를 트리거 지점에 놓고 'e' 수신 순간부터 물체가 입 안에
 *      완전히 들어올 때까지 스톱워치로 재서 넣을 것.
 *   -> 파이썬 FINAL_APPROACH_TIME 과 이 값의 "합"이 실제 거리와
 *      맞아야 한다. 한쪽만 맞추면 안 됨.
 * ================================================================ */
#define OPEN_HOLD_MS    400   /* 팔을 다 연 뒤 전진 시작까지 대기.
                               * 서보가 완전히 열리기 전에 출발하면
                               * 반쯤 열린 팔에 쓰레기가 걸린다. */

#define EAT_TRAVEL_MS  2000   /* ★ 팔을 연 채로 전진하는 시간.
                               * 이 값이 "팔 벌리고 얼마나 더 들어가느냐"를
                               * 결정한다. 쓰레기가 팔 사이 깊숙이 안 들어오면
                               * 이 값을 키운다. 지나쳐서 밀어내면 줄인다.
                               * 반드시 실측: 'e' 수신 순간부터 쓰레기가 팔
                               * 정중앙에 들어올 때까지 스톱워치로 잴 것. */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART1)
   {
      last_rx_char = rxData;
      rx_print_flag = 1;

      /* ============================================================
       * ★ 대문자 = 라즈베리파이 YOLO 추적 명령 (펄스 주행)
       *     L=큰좌회전  R=큰우회전   (오차가 클 때, 시원하게 돌림)
       *     A=미세좌회전 D=미세우회전 (거의 정렬됐을 때)
       *     F=전진      B=후진
       *   소문자 a/d/f/b = 사람이 조작하는 수동 명령 (연속 주행)
       * 둘을 문자로 분리해서, 추적 명령만 짧은 펄스로 처리한다.
       * ISR에서는 플래그만 세우고 실제 구동은 메인 루프가 한다.
       * ============================================================ */
      if (rxData == 'A' || rxData == 'D' || rxData == 'F' ||
          rxData == 'B' || rxData == 'L' || rxData == 'R')
      {
         if (rxData == 'A')      pulse_request = 1;   /* 미세 좌회전 */
         else if (rxData == 'D') pulse_request = 2;   /* 미세 우회전 */
         else if (rxData == 'F') pulse_request = 3;   /* 전진 */
         else if (rxData == 'B') pulse_request = 4;   /* 후진 (너무 가까울 때) */
         else if (rxData == 'L') pulse_request = 5;   /* 큰 좌회전 */
         else                    pulse_request = 6;   /* 큰 우회전 */

         tracking_mode   = 1;
         last_track_tick = HAL_GetTick();   /* 워치독 먹이주기 */
      }
      else if (rxData == '1')
      {
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 400);
      }
      else if (rxData == '0')
      {
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1200);
      }
      else if (rxData == 'l') { turn_left = 2; }
      else if (rxData == 'r') { turn_left = 1; }
      else if (rxData == 'p') { turn_back_left = 2; }
      else if (rxData == 'q') { turn_back_left = 1; }
      else if (rxData == 'f') { move = 1; input_state = 1; tracking_mode = 0; }
      else if (rxData == 'b') { move = 2; input_state = 1; tracking_mode = 0; }
      else if (rxData == 'a') { move = 3; input_state = 1; tracking_mode = 0; }
      else if (rxData == 'd') { move = 4; input_state = 1; tracking_mode = 0; }
      else if (rxData == 's')
      {
         /* ★ 최우선 정지: 메인 루프가 뭘 하고 있든 여기서 즉시 모터를 세운다.
          * 레지스터 쓰기만 하므로 ISR에서 안전하다. */
         move = 5;
         input_state   = 0;
         estop_request = 1;
         tracking_mode = 0;
         pulse_request = 0;

         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
      }
      else if (rxData == 'y') { move = 6; input_state = 0; }
      else if (rxData == 'n') { move = 1; input_state = 1; tracking_mode = 0; }
      else if (rxData == 'o') { front_door = 1; }
      else if (rxData == 'c') { front_door = 0; }
      else if (rxData == 'e')
      {
         /* 이미 eat 시퀀스 처리 중이면 요청 자체를 버린다.
          * (예전엔 pending 됐다가 arm_busy 풀리는 순간 문이 또 열렸음) */
         if (!arm_busy) eat_request = 1;
      }

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
   }
}

/* USER CODE END 0 */

int main(void)
{
   HAL_Init();
   SystemClock_Config();

   MX_GPIO_Init();
   MX_USART2_UART_Init();
   MX_TIM1_Init();
   MX_TIM2_Init();
   MX_USART1_UART_Init();
   MX_TIM3_Init();
   MX_TIM4_Init();

   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1200);
   HAL_Delay(500);

   HAL_UART_Receive_IT(&huart1, &rxData, 1);

   char *ready_msg = "=== Ready (pulse mode) ===\r\n";
   HAL_UART_Transmit(&huart2, (uint8_t*)ready_msg, strlen(ready_msg), 100);

   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
   HAL_TIM_Base_Start(&htim4);

   float distance1 = 999.0f;
   float distance2 = 999.0f;

   int drive_state = 0;
   int move_state  = 0;
   int rotate_time = 1750;
   int pwm         = 650;

   uint32_t last_us_tick    = 0;
   uint32_t last_print_tick = 0;

   /* 팔 초기화: 열림 위치로 한 번 보냈다가 닫아서 기준 위치를 잡는다 */
   __HAL_TIM_SET_COMPARE(&htim3, ARM_A_CH, ARM_A_OPEN);
   HAL_Delay(155);
   __HAL_TIM_SET_COMPARE(&htim3, ARM_B_CH, ARM_B_OPEN);
   HAL_Delay(1500);

   __HAL_TIM_SET_COMPARE(&htim3, ARM_B_CH, 1100);
   HAL_Delay(145); // 150
   __HAL_TIM_SET_COMPARE(&htim3, ARM_A_CH, 2500); // 2600


   HAL_Delay(500);

   wheel_stop_all();
   move        = 5;
   move_state  = 5;
   drive_state = 5;

   while (1)
   {
      uint32_t now = HAL_GetTick();

      /* ============================================================
       * [1] 최우선: 's' 정지 신호 처리
       * ============================================================ */
      if (estop_request)
      {
         wheel_stop_all();
         pulse_active  = 0;
         pulse_request = 0;
         tracking_mode = 0;
         arm_busy      = 0;
         move          = 5;
         move_state    = 5;
         drive_state   = 5;
         input_state   = 0;
         estop_request = 0;
         obstacle_alert_sent = 0;

         arm_close_fast();

         printf("ESTOP: motors halted\r\n");
         continue;
      }

      /* ============================================================
       * [2] 펄스 만료 검사 (논블로킹)
       * 펄스 시간이 지나면 즉시 정지시킨다. 이게 오버슈트를 막는 핵심.
       * 루프가 매번 빠르게 돌아야 정확하므로, 아래 초음파/printf는
       * 전부 주기 게이트를 걸어 두었다.
       * ============================================================ */
      if (pulse_active && (int32_t)(now - pulse_end_tick) >= 0)
      {
         wheel_stop_all();
         pulse_active = 0;
         drive_state  = 5;
      }

      /* ============================================================
       * [3] 추적 명령 워치독
       * 라파5가 죽거나 통신이 끊겨서 명령이 안 오면 무조건 정지.
       * 로봇이 통제 불능으로 계속 달리는 걸 막는 안전장치.
       * ============================================================ */
      if (tracking_mode && (now - last_track_tick) > CMD_WATCHDOG_MS)
      {
         if (pulse_active || drive_state != 5)
         {
            wheel_stop_all();
            pulse_active = 0;
            drive_state  = 5;
            printf("WATCHDOG: no track cmd, stopped\r\n");
         }
         tracking_mode = 0;
         obstacle_alert_sent = 0;
      }

      /* ============================================================
       * [4] 문 상태 변화 처리
       * ============================================================ */
      if (front_door == 0 && last_front_door_state != front_door)
      {
         /* 수동 닫기도 동일하게 양쪽 동시 램프로 천천히 닫는다. */
         arm_close_sweep();
      }
      if (front_door == 1 && last_front_door_state != front_door)
      {
         arm_open();
      }
      last_front_door_state = front_door;

      /* ============================================================
       * [5] 'e' EAT 시퀀스: 정지 -> 문 열기 -> 전진 -> 문 닫기
       * 각 단계마다 estop을 검사해서 's'가 오면 즉시 중단.
       * ============================================================ */
      if (eat_request && !arm_busy)
      {
         arm_busy      = 1;
         eat_request   = 0;
         tracking_mode = 0;      /* eat 중에는 추적 워치독을 끈다 */
         pulse_active  = 0;
         pulse_request = 0;
         obstacle_alert_sent = 0;

         wheel_stop_all();
         delay_ms_check_estop(150);

         if (!estop_request)
         {
            /* 팔 열기 */
            arm_open();
            delay_ms_check_estop(OPEN_HOLD_MS);
         }

         if (!estop_request)
         {
            /* 문 열린 채로 스쿱 전진 */
            wheel_on(1, 1, pwm - 150);
            delay_ms_check_estop(EAT_TRAVEL_MS);
         }

         if (!estop_request)
         {
            /* ★ 반드시 완전히 정지한 뒤에 팔을 닫는다.
             * 움직이면서 닫으면 로봇의 관성 때문에 쓰레기가 상대적으로
             * 뒤로 밀리면서 팔 밖으로 빠져나간다. */
            wheel_decel_stop(1);
            delay_ms_check_estop(250);

            /* 양쪽 팔을 동시에 천천히 오므려 물체를 중앙에 물린다 */
            arm_close_sweep();

            front_door            = 0;
            last_front_door_state = 0;

            /* eat 끝나면 정지 상태로 복귀. 다음 추적 명령을 기다린다.
             * ★ tracking_mode는 이미 0이므로, 이후 장애물을 만나면
             *   섹션 [9]의 자율주행 회피(후진+회전) 로직이 적용된다. */
            wheel_stop_all();
            move        = 5;
            move_state  = 5;
            drive_state = 5;
         }

         arm_busy = 0;
         continue;   /* eat 직후엔 이번 루프의 나머지를 건너뛴다 */
      }

      /* ============================================================
       * [6] 추적 펄스 실행
       * ISR이 세운 pulse_request를 받아 짧게 구동하고 만료 시각만 기록.
       * 여기서 블로킹 대기를 하지 않는다 -> [2]가 알아서 정지시킴.
       * ============================================================ */
      if (pulse_request && !arm_busy)
      {
         int req = pulse_request;
         pulse_request = 0;

         if (!estop_request)
         {
            if (req == 1) {          /* 미세 좌회전 */
               wheel_drive(0, pwm - 50, 1, pwm - 50);
               pulse_end_tick = HAL_GetTick() + PULSE_TURN_FINE_MS;
               drive_state = 3;
            }
            else if (req == 2) {     /* 미세 우회전 */
               wheel_drive(1, pwm - 50, 0, pwm - 50);
               pulse_end_tick = HAL_GetTick() + PULSE_TURN_FINE_MS;
               drive_state = 4;
            }
            else if (req == 5) {     /* 큰 좌회전 */
               wheel_drive(0, pwm - 50, 1, pwm - 50);
               pulse_end_tick = HAL_GetTick() + PULSE_TURN_BIG_MS;
               drive_state = 3;
            }
            else if (req == 6) {     /* 큰 우회전 */
               wheel_drive(1, pwm - 50, 0, pwm - 50);
               pulse_end_tick = HAL_GetTick() + PULSE_TURN_BIG_MS;
               drive_state = 4;
            }
            else if (req == 3) {     /* 전진 펄스 */
               wheel_drive(1, pwm, 1, pwm);
               pulse_end_tick = HAL_GetTick() + PULSE_FWD_MS;
               drive_state = 0;
            }
            else {                   /* 후진 펄스 (물체가 화면 너무 아래일 때) */
               wheel_drive(0, pwm, 0, pwm);
               pulse_end_tick = HAL_GetTick() + PULSE_BACK_MS;
               drive_state = 2;
            }
            pulse_active = 1;
            move_state   = -1;   /* 수동 명령 블록과 상태 충돌 방지 */
         }
      }

      /* ============================================================
       * [7] 수동/모드 명령 처리 (연속 주행)
       * 추적 펄스 중에는 건너뛴다.
       * ★ 기존의 delay_ms_check_estop(YOLO_INFERENCE_MS) 블로킹 대기는
       *   제거했다. 그 대기 동안 루프가 멈춰서 펄스 정지도, estop 반응도
       *   전부 늦어졌다. 이제 타이밍은 펄스와 워치독이 담당한다.
       * ============================================================ */
      if (!tracking_mode && !pulse_active && move != move_state)
      {
         int cmd = move;

         wheel_stop_all();
         delay_ms_check_estop(CMD_SETTLE_MS);

         if (!estop_request)
         {
            if (cmd == 6)      { wheel_on(1, 1, pwm);       drive_state = 0; input_state = 0; }
            else if (cmd == 5) { wheel_stop_all();          drive_state = 5; input_state = 0; }
            else if (cmd == 1) { wheel_on(1, 1, pwm);       drive_state = 0; input_state = 1; }
            else if (cmd == 2) { wheel_on(0, 0, pwm);       drive_state = 2; input_state = 0; }
            else if (cmd == 3) { wheel_on(0, 1, pwm - 50);  drive_state = 3; input_state = 0; }
            else if (cmd == 4) { wheel_on(1, 0, pwm - 50);  drive_state = 4; input_state = 0; }

            move_state = cmd;
         }
      }

      /* ============================================================
       * [8] 초음파 측정 (주기 게이트 + 타임아웃)
       * 펄스 주행 중에는 측정하지 않는다. 측정이 수십 ms를 잡아먹어서
       * 펄스 정지 타이밍이 밀리기 때문 (= 오버슈트 재발).
       * ============================================================ */
      if (!pulse_active && !arm_busy && (now - last_us_tick) >= ULTRASONIC_INTERVAL_MS)
      {
         float d1, d2;

         if (us_measure(GPIOC, GPIO_PIN_5, GPIOB, GPIO_PIN_6, &d1) == 0)
            distance1 = d1;
         else
            distance1 = 999.0f;   /* 측정 실패 = 장애물 없음으로 간주(오동작 방지) */

         HAL_Delay(15);   /* 두 센서 간 크로스토크 방지 */

         if (us_measure(GPIOA, GPIO_PIN_7, GPIOB, GPIO_PIN_7, &d2) == 0)
            distance2 = d2;
         else
            distance2 = 999.0f;

         last_us_tick = HAL_GetTick();

         /* ==========================================================
          * [9] 초음파 장애물 대응
          *
          * ★ 두 갈래로 나뉜다:
          *
          *  (A) tracking_mode == 1 (라파가 F/A/D/... 추적 명령을 계속
          *      보내는 중, 즉 쓰레기를 향해 접근 중인 상황):
          *      사람 등이 갑자기 끼어든 경우이므로 "후진만" 한다.
          *      회전(방향 전환)은 하지 않는다 - 카메라가 타겟을
          *      완전히 놓치면 안 되기 때문이다. 사람이 비켜나면
          *      라파가 계속 보내고 있는 F가 [6]에서 그대로 처리되어
          *      자동으로 전진이 재개된다 (별도 코드 불필요).
          *
          *  (B) tracking_mode == 0 (추적 중이 아님 - 순찰/eat 완료 후
          *      등): 기존 자율주행 회피 로직 그대로. 후진 후 충분히
          *      멀어지면 제자리 회전으로 방향을 바꿔 다시 전진한다.
          * ========================================================== */
         if (tracking_mode)
         {
            /* ------------------------------------------------------
             * (A) 추적 중 근접 장애물 -> 후진 전용, 회전 없음
             * ------------------------------------------------------ */
            if (distance1 <= 30 || distance2 <= 30)
            {
               if (!obstacle_alert_sent)
               {
                  /* 라파에게 "근접 장애물 있음"을 알려 F 스트림을
                   * 잠깐 멈추도록 유도 (경합/지글거림 방지용) */
                  HAL_UART_Transmit(&huart1, (uint8_t*)"h", 1, 100);
                  obstacle_alert_sent = 1;
               }

               /* 짧은 후진 펄스 - 블로킹 없이, 펄스 방식 그대로 유지 */
               wheel_drive(0, pwm, 0, pwm);
               pulse_end_tick = HAL_GetTick() + PULSE_BACK_MS;
               pulse_active   = 1;
               drive_state    = 2;

               /* ★ 큐에 쌓인 F를 취소 - 안 하면 다음 루프에서
                *   [6]이 그 F를 실행해 후진을 즉시 덮어써버린다. */
               pulse_request = 0;
            }
            else
            {
               /* 장애물이 사라짐 - 다음 근접 시 다시 알릴 수 있게 리셋 */
               obstacle_alert_sent = 0;
            }
         }
         else
         {
            /* ------------------------------------------------------
             * (B) 기존 자율주행 회피 로직 (후진 + 방향 전환 회전)
             * 수정 없이 그대로 유지.
             * ------------------------------------------------------ */
            if (drive_state == 0 && (distance1 <= 30 || distance2 <= 30))
            {
               wheel_decel_stop(1);
               delay_ms_check_estop(300);

               if (estop_request) { /* 다음 루프에서 정지로 수렴 */ }
               else if (turn_back_left == 1)
               {
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

                  /* ★ pwm을 쓰기 직전 재확인: 바깥 검사와 이 쓰기 사이에
                   * ISR이 estop을 세팅했다면, ISR이 0으로 만들어놓은
                   * 레지스터를 여기서 덮어써서 정지가 씹히게 된다. */
                  if (!estop_request)
                  {
                     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
                     turn_back_left = 0;
                     move = 2; move_state = 2; drive_state = 2; input_state = 0;
                  }
               }
               else if (turn_back_left == 2)
               {
                  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

                  if (!estop_request)
                  {
                     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
                     turn_back_left = 0;
                     move = 2; move_state = 2; drive_state = 2; input_state = 0;
                  }
               }
               else
               {
                  wheel_on(0, 0, pwm);
                  move = 2; move_state = 2; drive_state = 2; input_state = 0;
               }
            }
            else if (drive_state == 2 && distance1 >= 60 && distance2 >= 60)
            {
               wheel_decel_stop(0);
               delay_ms_check_estop(300);

               if (!estop_request)
               {
                  if (input_state == 0)
                  {
                     if (turn_left == 1)
                     {
                        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
                        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm);
                        delay_ms_check_estop(rotate_time);
                        wheel_stop_all();
                        delay_ms_check_estop(200);
                     }
                     else if (turn_left == 2)
                     {
                        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
                        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
                        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm);
                        delay_ms_check_estop(rotate_time);
                        wheel_stop_all();
                        delay_ms_check_estop(200);
                     }

                     if (!estop_request)
                     {
                        wheel_on(1, 1, pwm);
                        move = 1; move_state = 1; drive_state = 0;
                     }
                  }
                  else
                  {
                     drive_state = 0;
                  }
               }
            }
         }
      }

      /* ============================================================
       * [10] 디버그 출력 (주기 게이트)
       * printf는 115200bps UART라 한 줄에 수 ms가 걸린다.
       * 매 루프마다 찍으면 펄스 타이밍이 밀리므로 주기를 건다.
       * ============================================================ */
      if ((now - last_print_tick) >= PRINT_INTERVAL_MS)
      {
         printf("st:%d trk:%d pulse:%d d1:%.0f d2:%.0f\r\n",
                drive_state, tracking_mode, pulse_active, distance1, distance2);
         last_print_tick = now;
      }

      if (rx_print_flag)
      {
         printf("RX:'%c'\r\n", last_rx_char);
         rx_print_flag = 0;
      }
   }
}

/* ==================== 이하 CubeMX 생성 코드 ==================== */

void SystemClock_Config(void)
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
   RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

   RCC_ClkInitStruct.ClockType =
         RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};
   TIM_OC_InitTypeDef sConfigOC = {0};
   TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

   htim1.Instance = TIM1;
   htim1.Init.Prescaler = 72 - 1;
   htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim1.Init.Period = 999;
   htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim1.Init.RepetitionCounter = 0;
   htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

   if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();
   if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = 0;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

   sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
   sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
   sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
   sBreakDeadTimeConfig.DeadTime = 0;
   sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
   sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
   sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
   if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

   HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};
   TIM_OC_InitTypeDef sConfigOC = {0};

   htim2.Instance = TIM2;
   htim2.Init.Prescaler = 72 - 1;
   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim2.Init.Period = 999;
   htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

   if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
   if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = 0;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
   if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

   HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};
   TIM_OC_InitTypeDef sConfigOC = {0};

   htim3.Instance = TIM3;
   htim3.Init.Prescaler = 72 - 1;
   htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim3.Init.Period = 20000 - 1;
   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

   if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();

   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) Error_Handler();
   if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = 0;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

   if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
   if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
   if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

   HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};

   htim4.Instance = TIM4;
   htim4.Init.Prescaler = 72 - 1;
   htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim4.Init.Period = 65535;
   htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

   if (HAL_TIM_Base_Init(&htim4) != HAL_OK) Error_Handler();

   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) Error_Handler();

   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
   huart1.Instance = USART1;
   huart1.Init.BaudRate = 115200;
   huart1.Init.WordLength = UART_WORDLENGTH_8B;
   huart1.Init.StopBits = UART_STOPBITS_1;
   huart1.Init.Parity = UART_PARITY_NONE;
   huart1.Init.Mode = UART_MODE_TX_RX;
   huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart1.Init.OverSampling = UART_OVERSAMPLING_16;

   if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
   huart2.Instance = USART2;
   huart2.Init.BaudRate = 115200;
   huart2.Init.WordLength = UART_WORDLENGTH_8B;
   huart2.Init.StopBits = UART_STOPBITS_1;
   huart2.Init.Parity = UART_PARITY_NONE;
   huart2.Init.Mode = UART_MODE_TX_RX;
   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart2.Init.OverSampling = UART_OVERSAMPLING_16;

   if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};

   __HAL_RCC_AFIO_CLK_ENABLE();
   __HAL_AFIO_REMAP_TIM2_PARTIAL_2();
   __HAL_AFIO_REMAP_TIM3_ENABLE();

   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();

   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_7, GPIO_PIN_RESET);

   GPIO_InitStruct.Pin = B1_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void delay_us(uint16_t us)
{
   __HAL_TIM_SET_COUNTER(&htim2, 0);
   while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void Error_Handler(void)
{
   __disable_irq();
   while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
