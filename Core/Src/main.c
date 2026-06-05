#include "main.h"

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UART_BAUDRATE             115200UL
#define RX_RING_SIZE              256U
#define RX_LINE_SIZE              128U

#define ACTIVE_LINK_MM            110.0f
#define PASSIVE_LINK_MM           220.0f
#define MOTOR_DISTANCE_MM         160.0f
#define BASE_Y_MM                 0.0f
#define TWO_PI                    6.2831853071795864769f

#define DEFAULT_STEPS_PER_REV     6124L
#define DEFAULT_FEED_MM_S         12.0f
#define DEFAULT_ACCEL_MM_S2       35.0f
#define DEFAULT_JOG_MM            5.0f
#define DEFAULT_HOME_X_MM         0.0f
#define DEFAULT_HOME_Y_MM         314.94f

#define DDA_TICK_US               250UL
#define MIN_SEGMENT_MM            0.10f
#define MAX_BLOCKS                96U
#define MAX_STEP_FREQ_HZ          25000.0f

#define SERVO_UP_CCR              40U
#define SERVO_DOWN_CCR            30U
#define SERVO_SETTLE_MS           300U
#define PHOTO_ACTIVE_LEVEL        GPIO_PIN_RESET

#define MOTOR_EN_ACTIVE           GPIO_PIN_RESET
#define MOTOR_EN_DISABLE          GPIO_PIN_SET
#define MOTOR1_POS_DIR_LEVEL      GPIO_PIN_RESET
#define MOTOR2_POS_DIR_LEVEL      GPIO_PIN_SET
#define STEP_PULSE_DELAY          162U

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

typedef enum
{
  BLOCK_LINE = 0,
  BLOCK_ARC = 1
} BlockType;

typedef enum
{
  MACHINE_IDLE = 0,
  MACHINE_RUN = 1,
  MACHINE_STOP = 2,
  MACHINE_ERROR = 3
} MachineState;

typedef enum
{
  PROFILE_TRAPEZOID = 0,
  PROFILE_S_CURVE = 1
} SpeedProfile;

typedef struct
{
  BlockType type;
  float sx;
  float sy;
  float ex;
  float ey;
  float cx;
  float cy;
  float radius;
  float start_angle;
  float sweep_angle;
  float length;
} PathBlock;

typedef struct
{
  float x;
  float y;
} Point2D;

typedef struct
{
  uint8_t active;
  uint32_t total_ticks;
  uint32_t tick_index;
  SpeedProfile profile;
  float length;
  float vmax;
  float accel;
  float accel_time;
  float flat_time;
  float total_time;
  float last_s;
  float x;
  float y;
  int32_t motor1_pos;
  int32_t motor2_pos;
  uint16_t block_count;
  uint16_t block_index;
  PathBlock blocks[MAX_BLOCKS];
} DdaPlanner;

typedef struct
{
  MachineState state;
  float x;
  float y;
  int32_t motor1_pos;
  int32_t motor2_pos;
  int32_t zero1;
  int32_t zero2;
  int32_t steps_per_rev;
  DdaPlanner planner;
} RobotContext;

static RobotContext robot;
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t estop_request;
static volatile uint8_t planner_done_request;
static volatile uint8_t estop_reply_request;
static char rx_line[RX_LINE_SIZE];
static uint16_t rx_line_len;
static uint8_t pen_is_down;
static uint8_t photo_last_a;
static uint8_t photo_last_b;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void Serial_Init(void);
static void Serial_Poll(void);
static void Process_Command(char *line);
static void Dda_Tick(void);
static void Stop_Motion(uint8_t disable_motors);
static void Handle_EStop_Request(void);
static void Handle_Planner_Done_Request(void);
static int32_t RoundI32(float value);
static uint8_t Scara_IK_Raw(float x_ui, float y_ui, int32_t *p1, int32_t *p2);
static uint8_t Scara_IK(float x_ui, float y_ui, int32_t *p1, int32_t *p2);
static uint8_t Planner_Start(float feed, float accel, SpeedProfile profile);
static void Servo_PenUp(void);
static void Servo_PenDown(void);
static uint8_t Photo_IsActive(GPIO_TypeDef *port, uint16_t pin);
static void Photo_SetLastState(void);
static void Photo_ReportChanges(void);
static void Serial_SendSwitchStatus(void);

static void Serial_Send(const char *text)
{
  while (*text != '\0')
  {
    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
    }
    USART1->DR = (uint8_t)*text;
    text++;
  }
}

static void Serial_Printf(const char *fmt, ...)
{
  char text[192];
  va_list args;

  va_start(args, fmt);
  (void)vsnprintf(text, sizeof(text), fmt, args);
  va_end(args);
  Serial_Send(text);
}

static void Serial_SendFixed3(float value)
{
  int32_t scaled;
  int32_t whole;
  int32_t frac;

  scaled = RoundI32(value * 1000.0f);
  if (scaled < 0)
  {
    Serial_Send("-");
    scaled = -scaled;
  }

  whole = scaled / 1000L;
  frac = scaled % 1000L;
  Serial_Printf("%ld.%03ld", (long)whole, (long)frac);
}

static void Serial_SendStatus(void)
{
  Serial_Send("POS X");
  Serial_SendFixed3(robot.x);
  Serial_Send(" Y");
  Serial_SendFixed3(robot.y);
  Serial_Printf(" M1%ld M2%ld %s P%u\r\n",
                (long)robot.motor1_pos,
                (long)robot.motor2_pos,
                (robot.state == MACHINE_RUN) ? "BUSY" : "IDLE",
                (unsigned int)pen_is_down);
}

static uint8_t Photo_IsActive(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == PHOTO_ACTIVE_LEVEL) ? 1U : 0U;
}

static void Photo_SetLastState(void)
{
  photo_last_a = Photo_IsActive(AIN1_GPIO_Port, AIN1_Pin);
  photo_last_b = Photo_IsActive(BIN1_GPIO_Port, BIN1_Pin);
}

static void Photo_ReportChanges(void)
{
  uint8_t a = Photo_IsActive(AIN1_GPIO_Port, AIN1_Pin);
  uint8_t b = Photo_IsActive(BIN1_GPIO_Port, BIN1_Pin);

  if ((a != photo_last_a) || (b != photo_last_b))
  {
    if ((a != 0U) || (b != 0U))
    {
      Serial_Printf("EV SW A%u B%u\r\n", (unsigned int)a, (unsigned int)b);
    }
    photo_last_a = a;
    photo_last_b = b;
  }
}

static void Serial_SendSwitchStatus(void)
{
  Photo_SetLastState();
  Serial_Printf("SW A%u B%u\r\n",
                (unsigned int)Photo_IsActive(AIN1_GPIO_Port, AIN1_Pin),
                (unsigned int)Photo_IsActive(BIN1_GPIO_Port, BIN1_Pin));
}

void SerialProtocol_RxIrqHandler(void)
{
  uint32_t sr = USART1->SR;

  if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
  {
    (void)USART1->DR;
    return;
  }

  if ((sr & USART_SR_RXNE) != 0U)
  {
    uint8_t data = (uint8_t)(USART1->DR & 0xFFU);
    uint16_t next = (uint16_t)(rx_head + 1U);

    if (next >= RX_RING_SIZE)
    {
      next = 0U;
    }

    if (data == (uint8_t)'!')
    {
      estop_request = 1U;
      return;
    }

    if (next != rx_tail)
    {
      rx_ring[rx_head] = data;
      rx_head = next;
    }
  }
}

static void Serial_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t pclk = HAL_RCC_GetPCLK2Freq();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART1->CR1 = 0U;
  USART1->CR2 = 0U;
  USART1->CR3 = 0U;
  USART1->BRR = (pclk + (UART_BAUDRATE / 2UL)) / UART_BAUDRATE;
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static int32_t RoundI32(float value)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value + 0.5f);
  }
  return (int32_t)(value - 0.5f);
}

static uint32_t AbsI32(int32_t value)
{
  if (value < 0)
  {
    return (uint32_t)(-(value + 1)) + 1U;
  }
  return (uint32_t)value;
}

static float ClampF(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static uint8_t Key_Matches(const char *p, const char *key)
{
  size_t len = strlen(key);

  if (strncmp(p, key, len) != 0)
  {
    return 0U;
  }

  p += len;
  return ((*p == '+') || (*p == '-') || (*p == '.') || ((*p >= '0') && (*p <= '9')));
}

static float ParamF(const char *line, const char *key, float default_value)
{
  const char *p = line;
  size_t len = strlen(key);

  while (*p != '\0')
  {
    while ((*p == ' ') || (*p == '\t'))
    {
      p++;
    }

    if (Key_Matches(p, key) != 0U)
    {
      return strtof(p + len, NULL);
    }

    while ((*p != '\0') && (*p != ' ') && (*p != '\t'))
    {
      p++;
    }
  }

  return default_value;
}

static int32_t ParamI(const char *line, const char *key, int32_t default_value)
{
  return (int32_t)ParamF(line, key, (float)default_value);
}

static uint8_t HasParam(const char *line, const char *key)
{
  const char *p = line;

  while (*p != '\0')
  {
    while ((*p == ' ') || (*p == '\t'))
    {
      p++;
    }

    if (Key_Matches(p, key) != 0U)
    {
      return 1U;
    }

    while ((*p != '\0') && (*p != ' ') && (*p != '\t'))
    {
      p++;
    }
  }

  return 0U;
}

static void Motors_Enable(void)
{
  HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, MOTOR_EN_ACTIVE);
  HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, MOTOR_EN_ACTIVE);
}

static void Motors_Disable(void)
{
  HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, MOTOR_EN_DISABLE);
  HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, MOTOR_EN_DISABLE);
}

static void Servo_SetCompare(uint32_t compare)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, compare);
  htim4.Instance->EGR = TIM_EGR_UG;
}

static void Servo_PenUp(void)
{
  Servo_SetCompare(SERVO_UP_CCR);
  HAL_Delay(SERVO_SETTLE_MS);
  pen_is_down = 0U;
}

static void Servo_PenDown(void)
{
  Servo_SetCompare(SERVO_DOWN_CCR);
  HAL_Delay(SERVO_SETTLE_MS);
  pen_is_down = 1U;
}

static void Set_Dir(uint8_t motor_index, int32_t delta)
{
  GPIO_PinState level;

  if (motor_index == 1U)
  {
    level = (delta >= 0) ? MOTOR1_POS_DIR_LEVEL :
            ((MOTOR1_POS_DIR_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, level);
  }
  else
  {
    level = (delta >= 0) ? MOTOR2_POS_DIR_LEVEL :
            ((MOTOR2_POS_DIR_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, level);
  }
}

static void Pulse_Pin(GPIO_TypeDef *port, uint16_t pin)
{
  volatile uint32_t delay;

  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  for (delay = 0U; delay < STEP_PULSE_DELAY; delay++)
  {
    __NOP();
  }
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

static uint8_t Scara_IK_Raw(float x_ui, float y_ui, int32_t *p1, int32_t *p2)
{
  float x = x_ui;
  float y = y_ui - BASE_Y_MM;
  float half_d = MOTOR_DISTANCE_MM * 0.5f;
  float la = ACTIVE_LINK_MM;
  float lp = PASSIVE_LINK_MM;
  float dx_l = x + half_d;
  float dx_r = x - half_d;
  float rl = sqrtf((dx_l * dx_l) + (y * y));
  float rr = sqrtf((dx_r * dx_r) + (y * y));
  float cl;
  float cr;
  float th_l;
  float th_r;

  if ((p1 == NULL) || (p2 == NULL) || (robot.steps_per_rev <= 0))
  {
    return 0U;
  }

  if ((rl <= 0.0001f) || (rr <= 0.0001f))
  {
    return 0U;
  }

  if ((rl > (la + lp)) || (rr > (la + lp)) ||
      (rl < fabsf(lp - la)) || (rr < fabsf(lp - la)))
  {
    return 0U;
  }

  cl = ((la * la) + (rl * rl) - (lp * lp)) / (2.0f * la * rl);
  cr = ((la * la) + (rr * rr) - (lp * lp)) / (2.0f * la * rr);
  cl = ClampF(cl, -1.0f, 1.0f);
  cr = ClampF(cr, -1.0f, 1.0f);

  th_l = atan2f(y, dx_l) + acosf(cl);
  th_r = atan2f(y, dx_r) - acosf(cr);

  *p1 = RoundI32((th_l / TWO_PI) * (float)robot.steps_per_rev);
  *p2 = RoundI32((th_r / TWO_PI) * (float)robot.steps_per_rev);
  return 1U;
}

static uint8_t Scara_IK(float x_ui, float y_ui, int32_t *p1, int32_t *p2)
{
  int32_t raw1;
  int32_t raw2;

  if (Scara_IK_Raw(x_ui, y_ui, &raw1, &raw2) == 0U)
  {
    return 0U;
  }

  *p1 = robot.zero1 + raw1;
  *p2 = robot.zero2 + raw2;
  return 1U;
}

static void Calc_Trap(DdaPlanner *p, float feed, float accel)
{
  float s_accel;

  p->vmax = ClampF(feed, 0.1f, 1000.0f);
  p->accel = ClampF(accel, 1.0f, 10000.0f);
  p->accel_time = p->vmax / p->accel;
  s_accel = 0.5f * p->accel * p->accel_time * p->accel_time;

  if ((2.0f * s_accel) >= p->length)
  {
    p->accel_time = sqrtf(p->length / p->accel);
    p->flat_time = 0.0f;
    p->vmax = p->accel * p->accel_time;
  }
  else
  {
    p->flat_time = (p->length - (2.0f * s_accel)) / p->vmax;
  }

  p->total_time = (2.0f * p->accel_time) + p->flat_time;
  p->total_ticks = (uint32_t)((p->total_time * 1000000.0f) / (float)DDA_TICK_US) + 1UL;
  if (p->total_ticks < 1UL)
  {
    p->total_ticks = 1UL;
  }
}

static float Trap_DistanceAt(const DdaPlanner *p, float time_s)
{
  float s_accel;
  float s_flat;
  float t_down;

  if (time_s <= 0.0f)
  {
    return 0.0f;
  }

  if (time_s >= p->total_time)
  {
    return p->length;
  }

  if (time_s < p->accel_time)
  {
    return 0.5f * p->accel * time_s * time_s;
  }

  s_accel = 0.5f * p->accel * p->accel_time * p->accel_time;
  if (time_s < (p->accel_time + p->flat_time))
  {
    return s_accel + (p->vmax * (time_s - p->accel_time));
  }

  s_flat = p->vmax * p->flat_time;
  t_down = time_s - p->accel_time - p->flat_time;
  return s_accel + s_flat + (p->vmax * t_down) - (0.5f * p->accel * t_down * t_down);
}

static void Calc_SCurve(DdaPlanner *p, float feed)
{
  p->vmax = ClampF(feed, 0.1f, 1000.0f);
  p->accel = 0.0f;
  p->accel_time = 0.0f;
  p->flat_time = 0.0f;
  p->total_time = p->length / p->vmax;

  if (p->total_time < ((float)DDA_TICK_US / 1000000.0f))
  {
    p->total_time = (float)DDA_TICK_US / 1000000.0f;
  }

  p->total_ticks = (uint32_t)((p->total_time * 1000000.0f) / (float)DDA_TICK_US) + 1UL;
  if (p->total_ticks < 1UL)
  {
    p->total_ticks = 1UL;
  }
}

static float SCurve_DistanceAt(const DdaPlanner *p, float time_s)
{
  float u;
  float smooth;

  if (time_s <= 0.0f)
  {
    return 0.0f;
  }

  if (time_s >= p->total_time)
  {
    return p->length;
  }

  u = ClampF(time_s / p->total_time, 0.0f, 1.0f);
  smooth = (10.0f * u * u * u) -
           (15.0f * u * u * u * u) +
           (6.0f * u * u * u * u * u);
  return p->length * smooth;
}

static float Planner_DistanceAt(const DdaPlanner *p, float time_s)
{
  if (p->profile == PROFILE_S_CURVE)
  {
    return SCurve_DistanceAt(p, time_s);
  }

  return Trap_DistanceAt(p, time_s);
}

static void Block_PointAt(const PathBlock *b, float local_s, float *x, float *y)
{
  float u;

  if (b->length <= 0.0001f)
  {
    *x = b->ex;
    *y = b->ey;
    return;
  }

  u = ClampF(local_s / b->length, 0.0f, 1.0f);

  if (b->type == BLOCK_ARC)
  {
    float angle = b->start_angle + (b->sweep_angle * u);
    *x = b->cx + b->radius * cosf(angle);
    *y = b->cy + b->radius * sinf(angle);
  }
  else
  {
    *x = b->sx + ((b->ex - b->sx) * u);
    *y = b->sy + ((b->ey - b->sy) * u);
  }
}

static uint8_t Planner_TargetAt(float s, float *x, float *y)
{
  DdaPlanner *p = &robot.planner;
  float remain = s;
  uint16_t index = 0U;

  while ((index < p->block_count) && (remain > p->blocks[index].length))
  {
    remain -= p->blocks[index].length;
    index++;
  }

  if (index >= p->block_count)
  {
    PathBlock *last = &p->blocks[p->block_count - 1U];
    *x = last->ex;
    *y = last->ey;
    return 1U;
  }

  p->block_index = index;
  Block_PointAt(&p->blocks[index], remain, x, y);
  return 1U;
}

static void Step_To_Target(int32_t target1, int32_t target2)
{
  int32_t d1 = target1 - robot.motor1_pos;
  int32_t d2 = target2 - robot.motor2_pos;
  uint32_t n1 = AbsI32(d1);
  uint32_t n2 = AbsI32(d2);
  uint32_t major = (n1 > n2) ? n1 : n2;
  uint32_t i;
  uint32_t acc1 = 0U;
  uint32_t acc2 = 0U;

  if (major == 0U)
  {
    return;
  }

  if ((float)major > (MAX_STEP_FREQ_HZ * ((float)DDA_TICK_US / 1000000.0f)))
  {
    Stop_Motion(0U);
    robot.state = MACHINE_ERROR;
    Serial_Send("ER STEP RATE\r\n");
    return;
  }

  Set_Dir(1U, d1);
  Set_Dir(2U, d2);

  for (i = 0U; i < major; i++)
  {
    acc1 += n1;
    acc2 += n2;

    if (acc1 >= major)
    {
      acc1 -= major;
      Pulse_Pin(STEP1_GPIO_Port, STEP1_Pin);
      robot.motor1_pos += (d1 >= 0) ? 1L : -1L;
    }

    if (acc2 >= major)
    {
      acc2 -= major;
      Pulse_Pin(STEP2_GPIO_Port, STEP2_Pin);
      robot.motor2_pos += (d2 >= 0) ? 1L : -1L;
    }
  }
}

static void Planner_Clear(void)
{
  memset(&robot.planner, 0, sizeof(robot.planner));
}

static uint8_t Planner_AddLine(float x0, float y0, float x1, float y1)
{
  PathBlock *b;
  float dx;
  float dy;

  if (robot.planner.block_count >= MAX_BLOCKS)
  {
    return 0U;
  }

  dx = x1 - x0;
  dy = y1 - y0;

  b = &robot.planner.blocks[robot.planner.block_count];
  memset(b, 0, sizeof(*b));
  b->type = BLOCK_LINE;
  b->sx = x0;
  b->sy = y0;
  b->ex = x1;
  b->ey = y1;
  b->length = sqrtf((dx * dx) + (dy * dy));

  if (b->length < MIN_SEGMENT_MM)
  {
    return 0U;
  }

  robot.planner.length += b->length;
  robot.planner.block_count++;
  return 1U;
}

static uint8_t Planner_AddArc(float x0, float y0, float x1, float y1,
                              float cx, float cy, uint8_t cw)
{
  PathBlock *b;
  float r0;
  float r1;
  float a0;
  float a1;
  float sweep;

  if (robot.planner.block_count >= MAX_BLOCKS)
  {
    return 0U;
  }

  r0 = sqrtf(((x0 - cx) * (x0 - cx)) + ((y0 - cy) * (y0 - cy)));
  r1 = sqrtf(((x1 - cx) * (x1 - cx)) + ((y1 - cy) * (y1 - cy)));

  if ((r0 < MIN_SEGMENT_MM) || (fabsf(r0 - r1) > 1.0f))
  {
    return 0U;
  }

  a0 = atan2f(y0 - cy, x0 - cx);
  a1 = atan2f(y1 - cy, x1 - cx);
  sweep = a1 - a0;

  if (cw != 0U)
  {
    if (sweep >= 0.0f)
    {
      sweep -= TWO_PI;
    }
  }
  else
  {
    if (sweep <= 0.0f)
    {
      sweep += TWO_PI;
    }
  }

  b = &robot.planner.blocks[robot.planner.block_count];
  memset(b, 0, sizeof(*b));
  b->type = BLOCK_ARC;
  b->sx = x0;
  b->sy = y0;
  b->ex = x1;
  b->ey = y1;
  b->cx = cx;
  b->cy = cy;
  b->radius = (r0 + r1) * 0.5f;
  b->start_angle = a0;
  b->sweep_angle = sweep;
  b->length = fabsf(b->radius * sweep);

  if (b->length < MIN_SEGMENT_MM)
  {
    return 0U;
  }

  robot.planner.length += b->length;
  robot.planner.block_count++;
  return 1U;
}

static float Point_Distance(Point2D a, Point2D b)
{
  float dx = b.x - a.x;
  float dy = b.y - a.y;
  return sqrtf((dx * dx) + (dy * dy));
}

static Point2D Point_Add(Point2D a, Point2D b)
{
  Point2D out;
  out.x = a.x + b.x;
  out.y = a.y + b.y;
  return out;
}

static Point2D Point_Sub(Point2D a, Point2D b)
{
  Point2D out;
  out.x = a.x - b.x;
  out.y = a.y - b.y;
  return out;
}

static Point2D Point_Scale(Point2D a, float scale)
{
  Point2D out;
  out.x = a.x * scale;
  out.y = a.y * scale;
  return out;
}

static float Point_Dot(Point2D a, Point2D b)
{
  return (a.x * b.x) + (a.y * b.y);
}

static float Point_Cross(Point2D a, Point2D b)
{
  return (a.x * b.y) - (a.y * b.x);
}

static Point2D Point_Normalize(Point2D a)
{
  float len = sqrtf((a.x * a.x) + (a.y * a.y));
  Point2D out = {0.0f, 0.0f};

  if (len > 0.0001f)
  {
    out.x = a.x / len;
    out.y = a.y / len;
  }

  return out;
}

static uint8_t Planner_AddRoundedCorner(Point2D prev, Point2D corner, Point2D next,
                                        Point2D *cursor, float radius)
{
  Point2D in_dir;
  Point2D out_dir;
  Point2D t1;
  Point2D t2;
  Point2D n1;
  Point2D n2;
  Point2D center;
  float len_in;
  float len_out;
  float cos_theta;
  float theta;
  float trim;
  float cross;
  float denom;

  len_in = Point_Distance(prev, corner);
  len_out = Point_Distance(corner, next);
  if ((len_in < MIN_SEGMENT_MM) || (len_out < MIN_SEGMENT_MM))
  {
    (void)Planner_AddLine(cursor->x, cursor->y, corner.x, corner.y);
    *cursor = corner;
    return 1U;
  }

  in_dir = Point_Normalize(Point_Sub(corner, prev));
  out_dir = Point_Normalize(Point_Sub(next, corner));
  cos_theta = ClampF(Point_Dot(Point_Scale(in_dir, -1.0f), out_dir), -0.999f, 0.999f);
  theta = acosf(cos_theta);
  trim = radius / tanf(theta * 0.5f);

  if ((theta < 0.05f) || (trim <= 0.0f))
  {
    (void)Planner_AddLine(cursor->x, cursor->y, corner.x, corner.y);
    *cursor = corner;
    return 1U;
  }

  trim = ClampF(trim, 0.0f, fminf(len_in, len_out) * 0.45f);
  if (trim < MIN_SEGMENT_MM)
  {
    (void)Planner_AddLine(cursor->x, cursor->y, corner.x, corner.y);
    *cursor = corner;
    return 1U;
  }

  t1 = Point_Sub(corner, Point_Scale(in_dir, trim));
  t2 = Point_Add(corner, Point_Scale(out_dir, trim));

  (void)Planner_AddLine(cursor->x, cursor->y, t1.x, t1.y);

  n1.x = -in_dir.y;
  n1.y = in_dir.x;
  n2.x = -out_dir.y;
  n2.y = out_dir.x;
  denom = Point_Cross(n1, n2);
  if (fabsf(denom) < 0.0001f)
  {
    (void)Planner_AddLine(t1.x, t1.y, t2.x, t2.y);
    *cursor = t2;
    return 1U;
  }

  center = Point_Add(t1, Point_Scale(n1, Point_Cross(Point_Sub(t2, t1), n2) / denom));
  cross = Point_Cross(in_dir, out_dir);
  (void)Planner_AddArc(t1.x, t1.y, t2.x, t2.y, center.x, center.y, (cross < 0.0f) ? 1U : 0U);
  *cursor = t2;
  return 1U;
}

static uint8_t Start_DrawPolyline(const Point2D *pts, uint16_t count, float corner_radius,
                                  float start_tol, float feed, float accel,
                                  SpeedProfile profile, const char *name)
{
  Point2D cursor;
  uint16_t i;

  if (robot.state == MACHINE_RUN)
  {
    Serial_Send("ER BUSY\r\n");
    return 0U;
  }

  if ((pts == NULL) || (count < 2U))
  {
    Serial_Send("ER EMPTY\r\n");
    return 0U;
  }

  if (Point_Distance((Point2D){robot.x, robot.y}, pts[0]) > start_tol)
  {
    Serial_Send("ER DRAW START\r\n");
    return 0U;
  }

  Planner_Clear();
  cursor = pts[0];

  if ((corner_radius > 0.0f) && (count > 2U))
  {
    for (i = 1U; i < (count - 1U); i++)
    {
      (void)Planner_AddRoundedCorner(pts[i - 1U], pts[i], pts[i + 1U], &cursor, corner_radius);
    }
    (void)Planner_AddLine(cursor.x, cursor.y, pts[count - 1U].x, pts[count - 1U].y);
  }
  else
  {
    for (i = 1U; i < count; i++)
    {
      (void)Planner_AddLine(cursor.x, cursor.y, pts[i].x, pts[i].y);
      cursor = pts[i];
    }
  }

  if (Planner_Start(feed, accel, profile) != 0U)
  {
    Serial_Printf("OK %s %c\r\n", name, (profile == PROFILE_S_CURVE) ? 'S' : 'T');
    return 1U;
  }

  return 0U;
}

static uint8_t Start_DrawPath1(float feed, float accel, SpeedProfile profile)
{
  const float start_tol = 1.0f;
  const float corner_radius = 0.8f;
  Point2D pts[] = {
      {0.0f, 250.1f},
      {0.0f, 230.1f},
      {-15.0f, 230.1f},
      {-15.0f, 210.1f},
      {-45.0f, 180.1f},
      {-20.0f, 180.1f}};
  Point2D cursor;
  Point2D tail[] = {
      {20.0f, 180.1f},
      {45.0f, 180.1f},
      {15.0f, 210.1f},
      {15.0f, 230.1f},
      {0.0f, 230.1f}};
  uint16_t i;

  if (robot.state == MACHINE_RUN)
  {
    Serial_Send("ER BUSY\r\n");
    return 0U;
  }

  if (Point_Distance((Point2D){robot.x, robot.y}, pts[0]) > start_tol)
  {
    Serial_Send("ER DRAW START\r\n");
    return 0U;
  }

  Planner_Clear();
  cursor = pts[0];

  for (i = 1U; i < ((sizeof(pts) / sizeof(pts[0])) - 1U); i++)
  {
    (void)Planner_AddRoundedCorner(pts[i - 1U], pts[i], pts[i + 1U], &cursor, corner_radius);
  }

  (void)Planner_AddLine(cursor.x, cursor.y, pts[5].x, pts[5].y);
  cursor = pts[5];
  (void)Planner_AddArc(cursor.x, cursor.y, tail[0].x, tail[0].y, 0.0f, 180.0f, 0U);
  cursor = tail[0];

  for (i = 1U; i < ((sizeof(tail) / sizeof(tail[0])) - 1U); i++)
  {
    (void)Planner_AddRoundedCorner(tail[i - 1U], tail[i], tail[i + 1U], &cursor, corner_radius);
  }
  (void)Planner_AddLine(cursor.x, cursor.y, tail[4].x, tail[4].y);
  (void)Planner_AddLine(tail[4].x, tail[4].y, pts[0].x, pts[0].y);

  if (Planner_Start(feed, accel, profile) != 0U)
  {
    Serial_Send((profile == PROFILE_S_CURVE) ? "OK DRAW1 S\r\n" : "OK DRAW1 T\r\n");
    return 1U;
  }

  return 0U;
}

static uint8_t Start_DrawStar(float feed, float accel, SpeedProfile profile)
{
  const float center_x = 0.0f;
  const float center_y = 210.0f;
  const float outer_r = 38.0f;
  const float inner_r = 17.0f;
  static Point2D pts[11];
  uint16_t i;

  for (i = 0U; i < 11U; i++)
  {
    float r = ((i % 2U) == 0U) ? outer_r : inner_r;
    float angle = (TWO_PI * 0.25f) + ((float)i * TWO_PI / 10.0f);
    pts[i].x = center_x + (r * cosf(angle));
    pts[i].y = center_y + (r * sinf(angle));
  }

  return Start_DrawPolyline(pts, 11U, 1.2f, 1.0f, feed, accel, profile, "DRAW2");
}

static uint8_t Start_DrawHeart(float feed, float accel, SpeedProfile profile)
{
  const float center_x = 0.0f;
  const float center_y = 208.0f;
  const float scale = 2.0f;
  static Point2D pts[73];
  uint16_t i;

  for (i = 0U; i < 73U; i++)
  {
    float t = ((float)i * TWO_PI) / 72.0f;
    float st = sinf(t);
    pts[i].x = center_x + scale * 16.0f * st * st * st;
    pts[i].y = center_y + scale *
        ((13.0f * cosf(t)) - (5.0f * cosf(2.0f * t)) -
         (2.0f * cosf(3.0f * t)) - cosf(4.0f * t));
  }

  return Start_DrawPolyline(pts, 73U, 0.0f, 1.0f, feed, accel, profile, "DRAW3");
}

static uint8_t Start_DrawFlower(float feed, float accel, SpeedProfile profile)
{
  const float center_x = 0.0f;
  const float center_y = 210.0f;
  static Point2D pts[73];
  uint16_t i;

  for (i = 0U; i < 73U; i++)
  {
    float t = ((float)i * TWO_PI) / 72.0f;
    float angle = (TWO_PI * 0.25f) + t;
    float r = 22.0f + (10.0f * cosf(6.0f * t));
    pts[i].x = center_x + (r * cosf(angle));
    pts[i].y = center_y + (r * sinf(angle));
  }

  return Start_DrawPolyline(pts, 73U, 0.0f, 1.0f, feed, accel, profile, "DRAW4");
}

static uint8_t Planner_Start(float feed, float accel, SpeedProfile profile)
{
  DdaPlanner *p = &robot.planner;
  int32_t p1;
  int32_t p2;
  PathBlock *last;

  if (p->block_count == 0U)
  {
    Serial_Send("ER EMPTY\r\n");
    return 0U;
  }

  last = &p->blocks[p->block_count - 1U];
  if (Scara_IK(last->ex, last->ey, &p1, &p2) == 0U)
  {
    Planner_Clear();
    Serial_Send("ER IK END\r\n");
    return 0U;
  }

  p->active = 1U;
  p->tick_index = 0UL;
  p->last_s = 0.0f;
  p->block_index = 0U;
  p->x = robot.x;
  p->y = robot.y;
  p->motor1_pos = p1;
  p->motor2_pos = p2;
  p->profile = profile;
  if (profile == PROFILE_S_CURVE)
  {
    Calc_SCurve(p, feed);
  }
  else
  {
    Calc_Trap(p, feed, accel);
  }

  Motors_Enable();
  robot.state = MACHINE_RUN;

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim2);
  return 1U;
}

static uint8_t Start_Line(float x, float y, float feed, float accel, SpeedProfile profile)
{
  int32_t p1;
  int32_t p2;

  if (robot.state == MACHINE_RUN)
  {
    Serial_Send("ER BUSY\r\n");
    return 0U;
  }

  if (Scara_IK(x, y, &p1, &p2) == 0U)
  {
    Serial_Send("ER IK\r\n");
    return 0U;
  }

  if (Point_Distance((Point2D){robot.x, robot.y}, (Point2D){x, y}) < MIN_SEGMENT_MM)
  {
    robot.x = x;
    robot.y = y;
    robot.motor1_pos = p1;
    robot.motor2_pos = p2;
    Serial_Send("OK G1\r\n");
    Serial_Send("RDY\r\n");
    return 1U;
  }

  Planner_Clear();
  if (Planner_AddLine(robot.x, robot.y, x, y) == 0U)
  {
    Serial_Send("ER LINE\r\n");
    return 0U;
  }

  if (Planner_Start(feed, accel, profile) != 0U)
  {
    Serial_Send("OK G1\r\n");
    return 1U;
  }

  return 0U;
}

static uint8_t Start_Arc(float x, float y, float i, float j,
                         uint8_t cw, float feed, float accel, SpeedProfile profile)
{
  float cx = robot.x + i;
  float cy = robot.y + j;
  int32_t p1;
  int32_t p2;

  if (robot.state == MACHINE_RUN)
  {
    Serial_Send("ER BUSY\r\n");
    return 0U;
  }

  if (Scara_IK(x, y, &p1, &p2) == 0U)
  {
    Serial_Send("ER IK\r\n");
    return 0U;
  }

  Planner_Clear();
  if (Planner_AddArc(robot.x, robot.y, x, y, cx, cy, cw) == 0U)
  {
    Serial_Send("ER ARC\r\n");
    return 0U;
  }

  if (Planner_Start(feed, accel, profile) != 0U)
  {
    Serial_Send(cw ? "OK G2\r\n" : "OK G3\r\n");
    return 1U;
  }

  return 0U;
}

static uint8_t Start_Jog(char axis, float distance, float feed, float accel, SpeedProfile profile)
{
  float x = robot.x;
  float y = robot.y;

  if ((axis == 'X') || (axis == 'x'))
  {
    x += distance;
  }
  else if ((axis == 'Y') || (axis == 'y'))
  {
    y += distance;
  }
  else
  {
    Serial_Send("ER JOG AXIS\r\n");
    return 0U;
  }

  return Start_Line(x, y, feed, accel, profile);
}

static void Finish_Planner(void)
{
  DdaPlanner *p = &robot.planner;
  PathBlock *last = &p->blocks[p->block_count - 1U];

  HAL_TIM_Base_Stop_IT(&htim2);
  Step_To_Target(p->motor1_pos, p->motor2_pos);
  robot.x = last->ex;
  robot.y = last->ey;
  robot.motor1_pos = p->motor1_pos;
  robot.motor2_pos = p->motor2_pos;
  Planner_Clear();
  robot.state = MACHINE_IDLE;
  planner_done_request = 1U;
}

static void Dda_Tick(void)
{
  DdaPlanner *p = &robot.planner;
  float now_s;
  float target_s;
  float x;
  float y;
  int32_t target1;
  int32_t target2;

  if (p->active == 0U)
  {
    HAL_TIM_Base_Stop_IT(&htim2);
    return;
  }

  if (estop_request != 0U)
  {
    estop_request = 0U;
    Stop_Motion(1U);
    estop_reply_request = 1U;
    return;
  }

  p->tick_index++;
  now_s = ((float)p->tick_index * (float)DDA_TICK_US) / 1000000.0f;
  target_s = Planner_DistanceAt(p, now_s);

  if (target_s > p->length)
  {
    target_s = p->length;
  }

  (void)Planner_TargetAt(target_s, &x, &y);

  if (Scara_IK(x, y, &target1, &target2) == 0U)
  {
    Stop_Motion(0U);
    robot.state = MACHINE_ERROR;
    Serial_Send("ER IK RUN\r\n");
    return;
  }

  Step_To_Target(target1, target2);
  p->last_s = target_s;
  p->x = x;
  p->y = y;
  robot.x = x;
  robot.y = y;

  if ((target_s >= p->length) || (p->tick_index >= p->total_ticks))
  {
    Finish_Planner();
  }
}

static void Handle_EStop_Request(void)
{
  if (estop_request != 0U)
  {
    estop_request = 0U;
    Stop_Motion(1U);
    estop_reply_request = 1U;
  }

  if (estop_reply_request != 0U)
  {
    estop_reply_request = 0U;
    Serial_Send("OK STOP\r\n");
  }
}

static void Handle_Planner_Done_Request(void)
{
  if (planner_done_request != 0U)
  {
    planner_done_request = 0U;
    Serial_Send("RDY\r\n");
  }
}

static void Stop_Motion(uint8_t disable_motors)
{
  HAL_TIM_Base_Stop_IT(&htim2);
  Planner_Clear();
  robot.state = MACHINE_STOP;
  HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);

  if (disable_motors != 0U)
  {
    Motors_Disable();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    Dda_Tick();
  }
}

static void Serial_Poll(void)
{
  while (rx_tail != rx_head)
  {
    uint8_t ch = rx_ring[rx_tail];

    rx_tail++;
    if (rx_tail >= RX_RING_SIZE)
    {
      rx_tail = 0U;
    }

    if ((ch == '\r') || (ch == '\n'))
    {
      if (rx_line_len > 0U)
      {
        rx_line[rx_line_len] = '\0';
        Process_Command(rx_line);
        rx_line_len = 0U;
      }
    }
    else if (rx_line_len < (RX_LINE_SIZE - 1U))
    {
      rx_line[rx_line_len] = (char)ch;
      rx_line_len++;
    }
    else
    {
      rx_line_len = 0U;
      Serial_Send("ER LINE LONG\r\n");
    }
  }
}

static void Process_Command(char *line)
{
  char axis = 0;
  char sign = 0;
  float distance;
  float feed = ParamF(line, "F", DEFAULT_FEED_MM_S);
  float accel = ParamF(line, "A", DEFAULT_ACCEL_MM_S2);
  SpeedProfile profile = (ParamI(line, "C", 0L) != 0L) ? PROFILE_S_CURVE : PROFILE_TRAPEZOID;

  if ((line[0] == '\0') || (line[0] == ';'))
  {
    return;
  }

  if ((strcmp(line, "?") != 0) && (strcmp(line, "Q") != 0) &&
      (strcmp(line, "STATUS") != 0) && (strcmp(line, "SW") != 0))
  {
    Serial_Send("RX ");
    Serial_Send(line);
    Serial_Send("\r\n");
  }

  if ((strcmp(line, "?") == 0) || (strcmp(line, "Q") == 0) ||
      (strcmp(line, "STATUS") == 0))
  {
    Serial_SendStatus();
  }
  else if (strcmp(line, "SW") == 0)
  {
    Serial_SendSwitchStatus();
  }
  else if ((strcmp(line, "HELP") == 0) || (strcmp(line, "$") == 0))
  {
    Serial_Send("CMD: G1 X.. Y.. F.. A.. C0/C1 | G2/G3 X.. Y.. I.. J.. C0/C1 | DRAW1..DRAW4 F.. A.. | J X+ 5 C0/C1 | SXY X.. Y.. | SZ M1 M2 | PPR N | P0/P1 | SW | M17 | M18 | ! | ?\r\n");
  }
  else if ((strcmp(line, "M17") == 0) || (strcmp(line, "E1") == 0))
  {
    Motors_Enable();
    Serial_Send("OK ENABLE\r\n");
  }
  else if ((strcmp(line, "M18") == 0) || (strcmp(line, "E0") == 0))
  {
    Stop_Motion(0U);
    Motors_Disable();
    Serial_Send("OK DISABLE\r\n");
  }
  else if ((strcmp(line, "!") == 0) || (strcmp(line, "STOP") == 0) ||
           (strcmp(line, "ESTOP") == 0))
  {
    Stop_Motion(1U);
    Serial_Send("OK STOP\r\n");
  }
  else if ((strcmp(line, "P0") == 0) || (strcmp(line, "PEN 0") == 0))
  {
    Servo_PenUp();
    Serial_Send("OK PEN UP\r\n");
  }
  else if ((strcmp(line, "P1") == 0) || (strcmp(line, "PEN 1") == 0))
  {
    Servo_PenDown();
    Serial_Send("OK PEN DOWN\r\n");
  }
  else if (strncmp(line, "PPR", 3) == 0)
  {
    int32_t ppr = ParamI(line, "N", DEFAULT_STEPS_PER_REV);
    int32_t old_ppr = robot.steps_per_rev;
    int32_t old_zero1 = robot.zero1;
    int32_t old_zero2 = robot.zero2;
    if (ppr <= 0)
    {
      Serial_Send("ER PPR\r\n");
    }
    else
    {
      int32_t raw1;
      int32_t raw2;

      robot.steps_per_rev = ppr;
      if (Scara_IK_Raw(robot.x, robot.y, &raw1, &raw2) == 0U)
      {
        robot.steps_per_rev = old_ppr;
        robot.zero1 = old_zero1;
        robot.zero2 = old_zero2;
        Serial_Send("ER PPR IK\r\n");
      }
      else
      {
        robot.zero1 = robot.motor1_pos - raw1;
        robot.zero2 = robot.motor2_pos - raw2;
        Serial_Printf("OK PPR %ld Z1%ld Z2%ld\r\n",
                      (long)robot.steps_per_rev,
                      (long)robot.zero1,
                      (long)robot.zero2);
      }
    }
  }
  else if (strncmp(line, "SZ", 2) == 0)
  {
    robot.zero1 = ParamI(line, "M1", robot.zero1);
    robot.zero2 = ParamI(line, "M2", robot.zero2);
    robot.motor1_pos = robot.zero1;
    robot.motor2_pos = robot.zero2;
    Serial_Printf("OK ZERO M1%ld M2%ld\r\n", (long)robot.zero1, (long)robot.zero2);
  }
  else if (strncmp(line, "SXY", 3) == 0)
  {
    int32_t p1;
    int32_t p2;
    float x = ParamF(line, "X", robot.x);
    float y = ParamF(line, "Y", robot.y);

    if (Scara_IK(x, y, &p1, &p2) == 0U)
    {
      Serial_Send("ER IK\r\n");
    }
    else
    {
      robot.x = x;
      robot.y = y;
      robot.motor1_pos = p1;
      robot.motor2_pos = p2;
      Serial_Send("OK SXY\r\n");
    }
  }
  else if ((strcmp(line, "HOME") == 0) || (strcmp(line, "H") == 0))
  {
    int32_t p1;
    int32_t p2;

    Stop_Motion(0U);
    robot.x = DEFAULT_HOME_X_MM;
    robot.y = DEFAULT_HOME_Y_MM;
    if (Scara_IK(robot.x, robot.y, &p1, &p2) != 0U)
    {
      robot.motor1_pos = p1;
      robot.motor2_pos = p2;
    }
    robot.state = MACHINE_IDLE;
    Serial_Send("OK HOME SET\r\n");
  }
  else if (strncmp(line, "G1", 2) == 0)
  {
    float x = ParamF(line, "X", robot.x);
    float y = ParamF(line, "Y", robot.y);
    (void)Start_Line(x, y, feed, accel, profile);
  }
  else if ((strncmp(line, "G2", 2) == 0) || (strncmp(line, "G3", 2) == 0))
  {
    uint8_t cw = (line[1] == '2') ? 1U : 0U;
    float x = ParamF(line, "X", robot.x);
    float y = ParamF(line, "Y", robot.y);
    float i = ParamF(line, "I", 0.0f);
    float j = ParamF(line, "J", 0.0f);

    if ((HasParam(line, "I") == 0U) || (HasParam(line, "J") == 0U))
    {
      Serial_Send("ER ARC IJ\r\n");
    }
    else
    {
      (void)Start_Arc(x, y, i, j, cw, feed, accel, profile);
    }
  }
  else if (strncmp(line, "DRAW1", 5) == 0)
  {
    (void)Start_DrawPath1(feed, accel, profile);
  }
  else if (strncmp(line, "DRAW2", 5) == 0)
  {
    (void)Start_DrawStar(feed, accel, profile);
  }
  else if (strncmp(line, "DRAW3", 5) == 0)
  {
    (void)Start_DrawHeart(feed, accel, profile);
  }
  else if (strncmp(line, "DRAW4", 5) == 0)
  {
    (void)Start_DrawFlower(feed, accel, profile);
  }
  else if ((line[0] == 'J') || (line[0] == 'j'))
  {
    if (sscanf(line, "J %c%c %f", &axis, &sign, &distance) < 2)
    {
      Serial_Send("ER JOG\r\n");
      return;
    }

    if ((sign != '+') && (sign != '-'))
    {
      Serial_Send("ER JOG SIGN\r\n");
      return;
    }

    if (sscanf(line, "J %c%c %f", &axis, &sign, &distance) < 3)
    {
      distance = DEFAULT_JOG_MM;
    }

    if (sign == '-')
    {
      distance = -distance;
    }

    (void)Start_Jog(axis, distance, feed, accel, profile);
  }
  else
  {
    Serial_Send("ER UNKNOWN\r\n");
  }
}

static void Robot_Init(void)
{
  int32_t p1;
  int32_t p2;

  memset(&robot, 0, sizeof(robot));
  robot.state = MACHINE_IDLE;
  robot.steps_per_rev = DEFAULT_STEPS_PER_REV;
  robot.zero1 = 0L;
  robot.zero2 = 0L;
  robot.x = DEFAULT_HOME_X_MM;
  robot.y = DEFAULT_HOME_Y_MM;

  if (Scara_IK(robot.x, robot.y, &p1, &p2) != 0U)
  {
    robot.motor1_pos = p1;
    robot.motor2_pos = p2;
  }

  Motors_Disable();
  Serial_Send("OK PARALLEL SCARA READY, SEND M17 TO ENABLE\r\n");
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  Serial_Init();
  Robot_Init();
  Photo_SetLastState();

  while (1)
  {
    Photo_ReportChanges();
    Handle_EStop_Request();
    Handle_Planner_Done_Request();
    Serial_Poll();
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72U - 1U;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = DDA_TICK_US - 1UL;
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

  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1439U;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999U;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_UP_CCR;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, STEP1_Pin | STEP2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DIR1_Pin | DIR2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, ENA1_Pin | ENA2_Pin, MOTOR_EN_DISABLE);

  GPIO_InitStruct.Pin = BIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BIN1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AIN1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = STEP1_Pin | STEP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DIR1_Pin | ENA1_Pin | DIR2_Pin | ENA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
