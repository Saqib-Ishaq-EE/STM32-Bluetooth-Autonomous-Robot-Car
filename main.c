/*
 * ==========================================================================
 *  STM32F103 Blue Pill — Rover: Manual + Auto Obstacle Avoidance + Buzzer
 *
 *  BUG FIX: 999 (sensor timeout) now correctly treated as CLEAR.
 *
 *  HC-SR04 behavior:
 *    - Something within range  ? echo returns  ? real distance in cm
 *    - Nothing within range    ? echo never returns ? timeout ? returns 999
 *    - 999 means path is OPEN, not blocked.
 *
 *  BLOCKED   = distance <= 20 cm (something is close)
 *  CLEAR     = distance >  20 cm (including 999 = nothing detected)
 *
 *  Previous code incorrectly filtered out >=400 and 999 as "unsafe",
 *  causing the rover to never move forward on a clear floor.
 * ==========================================================================
 */

#include "stm32f10x.h"

/* ================================================================
 *  TUNING
 * ================================================================ */

#define TURN_INNER_PERCENT       40
#define OBSTACLE_DISTANCE_CM     35     /* blocked if <= this value         */
#define INVALID_DISTANCE_CM      999U   /* sensor timeout value              */
#define AUTO_SPEED               40
#define AUTO_TURN_MS             900
#define AUTO_BACKUP_MS           500
#define SERVO_MOVE_MS            800
#define SENSOR_SETTLE_MS         250
#define SERVO_CENTER_MS          500
#define SCAN_READINGS            3
#define SCAN_READ_DELAY_MS       80
#define RESCAN_PAUSE_MS          350
#define MANUAL_SAFE_DISTANCE_CM 35U
/*
 *  After finding a clear lateral side (left or right > 35cm),
 *  the rover re-checks the front before turning.
 *  This threshold is intentionally lower than OBSTACLE_DISTANCE_CM (35cm)
 *  because a small gap ahead may still allow a turn safely.
 */
#define FRONT_RECHECK_CM         20U

#define SERVO_ANGLE_LEFT         180U
#define SERVO_ANGLE_RIGHT        0U

/* ================================================================
 *  GLOBAL STATE
 * ================================================================ */

uint8_t  speed_percent  = 50;
char     current_action = 'S';
uint8_t  parsing_speed  = 0;
uint16_t speed_value    = 0;
uint8_t manual_distance_clear(uint32_t d)
{
    // Treat distances >35cm as safe
    // Treat 999 as safe
    if(d > MANUAL_SAFE_DISTANCE_CM)
        return 1;   // safe
    else if(d == 999)
        return 1;   // treat 999 as safe
    else
        return 0;   // <=35cm is unsafe
}
/* ================================================================
 *  FUNCTION PROTOTYPES
 * ================================================================ */

void     GPIO_Init_Custom(void);
void     TIM2_PWM_Init(void);
void     TIM3_Servo_Init(void);
void     TIM4_Micros_Init(void);
void     USART1_Init(void);

char     USART1_ReadChar(void);
void     USART1_SendChar(char c);
void     USART1_SendString(char *str);
void     USART1_SendUInt32(uint32_t val);

void     delay(volatile uint32_t t);
void     delay_ms(uint32_t ms);

uint16_t percent_to_ccr(uint8_t percent);
void     set_left_speed(uint8_t percent);
void     set_right_speed(uint8_t percent);
void     set_all_speed(uint8_t percent);

void     left_side_forward(void);
void     left_side_backward(void);
void     right_side_forward(void);
void     right_side_backward(void);

void     stop_motors(void);
void     move_forward(void);
void     move_backward(void);
void     turn_left(void);
void     turn_right(void);
void     apply_current_action(void);

void     buzzer_on(void);
void     buzzer_off(void);

void     servo_set_angle(uint8_t deg);

uint32_t ultrasonic_get_distance_cm(void);
uint32_t ultrasonic_get_stable_distance(void);

uint8_t  distance_is_clear(uint32_t d);
uint8_t  distance_is_blocked(uint32_t d);

void     auto_mode_obstacle_scan(void);
void     auto_mode_run(void);
void     process_received_char(char c);

void     scan_left_right(uint32_t *out_left, uint32_t *out_right);
uint8_t  pick_turn_dir(uint32_t dist_left,  uint8_t clear_left,
                       uint32_t dist_right, uint8_t clear_right);
void     execute_turn(uint8_t dir);

/* ================================================================
 *  DELAY UTILITIES
 * ================================================================ */

void delay(volatile uint32_t t)
{
    while(t--);
}

void delay_ms(uint32_t ms)
{
    uint32_t i;

    for(i = 0U; i < ms; i++)
    {
        uint16_t start = (uint16_t)TIM4->CNT;
        while((uint16_t)(TIM4->CNT - start) < 1000U);
    }
}

/* ================================================================
 *  GPIO INIT
 * ================================================================ */

void GPIO_Init_Custom(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    AFIO->MAPR &= ~(7U << 24);
    AFIO->MAPR |=  (2U << 24);

    /* PA0-PA3 = TIM2 PWM, PA6 = TIM3 servo */
    GPIOA->CRL &= ~(0xFF00FFFFUL);
    GPIOA->CRL |=  (0x0B00BBBBU);

    /* PA5 = buzzer output */
    GPIOA->CRL &= ~(0xFU << 20);
    GPIOA->CRL |=  (0x2U << 20);

    /* PA9 = USART1 TX */
    GPIOA->CRH &= ~(0xFU << 4);
    GPIOA->CRH |=  (0xBU << 4);

    /* PA10 = USART1 RX */
    GPIOA->CRH &= ~(0xFU << 8);
    GPIOA->CRH |=  (0x4U << 8);

    /* PA11 = HC-SR04 ECHO input floating */
    GPIOA->CRH &= ~(0xFU << 12);
    GPIOA->CRH |=  (0x4U << 12);

    /* PB0, PB1, PB3 = motor direction */
    GPIOB->CRL &= ~((0xFU << 0) | (0xFU << 4) | (0xFU << 12));
    GPIOB->CRL |=  ((0x2U << 0) | (0x2U << 4) | (0x2U << 12));

    /* PB8 = TRIG, PB9 = mode switch, PB10-PB14 = motor direction */
    GPIOB->CRH &= ~((0xFU <<  0) | (0xFU <<  4) |
                    (0xFU <<  8) | (0xFU << 12) |
                    (0xFU << 16) | (0xFU << 20) |
                    (0xFU << 24));

    GPIOB->CRH |=  ((0x2U <<  0) |   /* PB8  TRIG output         */
                    (0x8U <<  4) |   /* PB9  input pull-down     */
                    (0x2U <<  8) |   /* PB10 motor direction      */
                    (0x2U << 12) |   /* PB11 motor direction      */
                    (0x2U << 16) |   /* PB12 motor direction      */
                    (0x2U << 20) |   /* PB13 motor direction      */
                    (0x2U << 24));   /* PB14 motor direction      */

    GPIOB->BSRR = (1U << (8U + 16U));  /* TRIG starts LOW  */
    GPIOB->ODR  &= ~(1U << 9U);        /* PB9 pull-down    */

    buzzer_off();
}

/* ================================================================
 *  TIM2 MOTOR PWM
 * ================================================================ */

void TIM2_PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 71U;
    TIM2->ARR = 999U;

    TIM2->CCMR1 &= ~((0xFFU << 0U) | (0xFFU << 8U));
    TIM2->CCMR1 |=  (6U << 4U)  | (1U << 3U);
    TIM2->CCMR1 |=  (6U << 12U) | (1U << 11U);

    TIM2->CCMR2 &= ~((0xFFU << 0U) | (0xFFU << 8U));
    TIM2->CCMR2 |=  (6U << 4U)  | (1U << 3U);
    TIM2->CCMR2 |=  (6U << 12U) | (1U << 11U);

    TIM2->CCER |= (1U << 0U) | (1U << 4U) | (1U << 8U) | (1U << 12U);
    TIM2->CR1  |= (1U << 7U);
    TIM2->EGR  |= (1U << 0U);
    TIM2->CR1  |= (1U << 0U);
}

/* ================================================================
 *  TIM3 SERVO PWM (50 Hz, PA6)
 * ================================================================ */

void TIM3_Servo_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 71U;
    TIM3->ARR = 19999U;

    TIM3->CCMR1 &= ~(0xFFU);
    TIM3->CCMR1 |=  (6U << 4U) | (1U << 3U);

    TIM3->CCR1 = 1500U;   /* 90 degrees = center */

    TIM3->CCER |= (1U << 0U);
    TIM3->CR1  |= (1U << 7U);
    TIM3->EGR  |= (1U << 0U);
    TIM3->CR1  |= (1U << 0U);
}

/* ================================================================
 *  TIM4 MICROSECOND TIMER
 * ================================================================ */

void TIM4_Micros_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    TIM4->PSC = 71U;
    TIM4->ARR = 0xFFFFU;
    TIM4->EGR |= (1U << 0U);
    TIM4->CR1 |= (1U << 0U);
}

/* ================================================================
 *  USART1
 * ================================================================ */

void USART1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    USART1->BRR  = 0x1D4CU;
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

char USART1_ReadChar(void)
{
    if(USART1->SR & USART_SR_RXNE)
    {
        return (char)(USART1->DR & 0xFFU);
    }

    return 0;
}

void USART1_SendChar(char c)
{
    while(!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint32_t)c;
}

void USART1_SendString(char *str)
{
    while(*str)
    {
        USART1_SendChar(*str++);
    }
}

void USART1_SendUInt32(uint32_t val)
{
    char    buf[11];
    uint8_t i = 0U;
    uint8_t j;
    char    tmp;

    if(val == 0U)
    {
        USART1_SendChar('0');
        return;
    }

    while(val > 0U)
    {
        buf[i++] = (char)('0' + (val % 10U));
        val      = val / 10U;
    }

    for(j = 0U; j < i / 2U; j++)
    {
        tmp              = buf[j];
        buf[j]           = buf[i - 1U - j];
        buf[i - 1U - j]  = tmp;
    }

    buf[i] = '\0';

    j = 0U;
    while(buf[j]) USART1_SendChar(buf[j++]);
}

/* ================================================================
 *  SPEED CONTROL
 * ================================================================ */

uint16_t percent_to_ccr(uint8_t percent)
{
    if(percent > 100U) percent = 100U;
    return (uint16_t)((percent * 999U) / 100U);
}

void set_left_speed(uint8_t percent)
{
    uint16_t ccr = percent_to_ccr(percent);
    TIM2->CCR1 = ccr;
    TIM2->CCR2 = ccr;
}

void set_right_speed(uint8_t percent)
{
    uint16_t ccr = percent_to_ccr(percent);
    TIM2->CCR3 = ccr;
    TIM2->CCR4 = ccr;
}

void set_all_speed(uint8_t percent)
{
    set_left_speed(percent);
    set_right_speed(percent);
}

/* ================================================================
 *  DIRECTION HELPERS
 * ================================================================ */

void left_side_forward(void)
{
    GPIOB->BSRR = (1U << 0U);
    GPIOB->BSRR = (1U << (1U  + 16U));
    GPIOB->BSRR = (1U << 3U);
    GPIOB->BSRR = (1U << (10U + 16U));
}

void left_side_backward(void)
{
    GPIOB->BSRR = (1U << (0U  + 16U));
    GPIOB->BSRR = (1U << 1U);
    GPIOB->BSRR = (1U << (3U  + 16U));
    GPIOB->BSRR = (1U << 10U);
}

void right_side_forward(void)
{
    GPIOB->BSRR = (1U << 11U);
    GPIOB->BSRR = (1U << (12U + 16U));
    GPIOB->BSRR = (1U << 13U);
    GPIOB->BSRR = (1U << (14U + 16U));
}

void right_side_backward(void)
{
    GPIOB->BSRR = (1U << (11U + 16U));
    GPIOB->BSRR = (1U << 12U);
    GPIOB->BSRR = (1U << (13U + 16U));
    GPIOB->BSRR = (1U << 14U);
}

/* ================================================================
 *  MOTION FUNCTIONS
 * ================================================================ */

void stop_motors(void)
{
    TIM2->CCR1 = 0U;
    TIM2->CCR2 = 0U;
    TIM2->CCR3 = 0U;
    TIM2->CCR4 = 0U;
    current_action = 'S';
}

void move_forward(void)
{
    left_side_forward();
    right_side_forward();
    set_all_speed(speed_percent);
    current_action = 'F';
}

void move_backward(void)
{
    left_side_backward();
    right_side_backward();
    set_all_speed(speed_percent);
    current_action = 'B';
}

void turn_left(void)
{
    uint8_t inner = (uint8_t)((speed_percent * TURN_INNER_PERCENT) / 100U);
    left_side_forward();
    right_side_forward();
    set_left_speed(inner);
    set_right_speed(speed_percent);
    current_action = 'L';
}

void turn_right(void)
{
    uint8_t inner = (uint8_t)((speed_percent * TURN_INNER_PERCENT) / 100U);
    left_side_forward();
    right_side_forward();
    set_left_speed(speed_percent);
    set_right_speed(inner);
    current_action = 'R';
}

void apply_current_action(void)
{
    if     (current_action == 'F') move_forward();
    else if(current_action == 'B') move_backward();
    else if(current_action == 'L') turn_left();
    else if(current_action == 'R') turn_right();
    else                           stop_motors();
}

/* ================================================================
 *  BUZZER  (PA5 HIGH = ON)
 * ================================================================ */

void buzzer_on(void)
{
    GPIOA->BSRR = (1U << 5U);
}

void buzzer_off(void)
{
    GPIOA->BSRR = (1U << (5U + 16U));
}

/* ================================================================
 *  SERVO
 * ================================================================ */

void servo_set_angle(uint8_t deg)
{
    if(deg > 180U) deg = 180U;
    TIM3->CCR1 = 500U + (uint16_t)((deg * 2000UL) / 180UL);
}

/* ================================================================
 *  ULTRASONIC RAW READING
 *  Returns cm, or 999 on timeout (nothing in range = path clear).
 * ================================================================ */

uint32_t ultrasonic_get_distance_cm(void)
{
    uint16_t t_start;
    uint16_t t_echo_start;
    uint16_t elapsed;
    uint32_t timeout;

    /* 10 us TRIG pulse */
    GPIOB->BSRR = (1U << 8U);
    t_start = (uint16_t)TIM4->CNT;
    while((uint16_t)(TIM4->CNT - t_start) < 10U);
    GPIOB->BSRR = (1U << (8U + 16U));

    /* Wait for ECHO HIGH */
    timeout = 30000U;
    while(!(GPIOA->IDR & (1U << 11U)))
    {
        if(--timeout == 0U) return INVALID_DISTANCE_CM;
    }
    t_echo_start = (uint16_t)TIM4->CNT;

    /* Measure ECHO pulse */
    timeout = 30000U;
    while(GPIOA->IDR & (1U << 11U))
    {
        if(--timeout == 0U) return INVALID_DISTANCE_CM;
    }
    elapsed = (uint16_t)(TIM4->CNT - t_echo_start);

    return (uint32_t)elapsed / 58U;
}

/* ================================================================
 *  ULTRASONIC STABLE READING  [FIXED]
 *
 *  Takes SCAN_READINGS samples and returns the MINIMUM.
 *
 *  WHY: 999 = timeout = nothing detected = path is open.
 *  We accept 999 as a perfectly valid reading.
 *  If one reading is 10 cm (blocked), min = 10 cm = blocked. Safe.
 *  If all readings are 999 (clear), min = 999 = clear. Correct.
 *
 *  PREVIOUS BUG: filtered out readings >= 400, so when all readings
 *  were 999 (open path), valid count was 0 -> returned 999 as
 *  "invalid" -> distance_is_clear(999) returned 0 -> rover thought
 *  path was blocked even on a completely clear floor.
 * ================================================================ */

uint32_t ultrasonic_get_stable_distance(void)
{
    uint32_t min_dist;
    uint32_t d;
    uint8_t  i;

    /* First reading is the starting minimum */
    min_dist = ultrasonic_get_distance_cm();

    for(i = 1U; i < SCAN_READINGS; i++)
    {
        delay_ms(SCAN_READ_DELAY_MS);

        d = ultrasonic_get_distance_cm();

        /* Keep the shortest distance detected (most conservative) */
        if(d < min_dist)
        {
            min_dist = d;
        }
    }

    return min_dist;
}

/* ================================================================
 *  DISTANCE DECISION HELPERS  [FIXED]
 *
 *  BLOCKED = distance <= 20 cm  (something is close)
 *  CLEAR   = distance >  20 cm  (includes 999 = nothing in range)
 *
 *  PREVIOUS BUG: distance_is_clear() required d < 400, so 999
 *  (open floor) was treated as blocked on every main loop tick,
 *  causing the rover to call auto_mode_obstacle_scan() immediately
 *  even when no obstacle existed.
 * ================================================================ */

uint8_t distance_is_clear(uint32_t d)
{
    /*
     *  Only one condition for "blocked": something within 20 cm.
     *  999 means the echo never came back = nothing in range = clear.
     *  400+ means something very far = also clear.
     */
    return (d > (uint32_t)OBSTACLE_DISTANCE_CM) ? 1U : 0U;
}

uint8_t distance_is_blocked(uint32_t d)
{
    return (d <= (uint32_t)OBSTACLE_DISTANCE_CM) ? 1U : 0U;
}

/* ================================================================
 *  SCAN LEFT AND RIGHT — shared helper
 *
 *  Rotates servo left, reads distance, then right, reads distance,
 *  then returns servo to center. Writes results into the pointers.
 * ================================================================ */

void scan_left_right(uint32_t *out_left, uint32_t *out_right)
{
    servo_set_angle(SERVO_ANGLE_LEFT);
    delay_ms(SERVO_MOVE_MS);
    delay_ms(SENSOR_SETTLE_MS);
    *out_left = ultrasonic_get_stable_distance();

    USART1_SendString("  Left  cm: ");
    USART1_SendUInt32(*out_left);
    USART1_SendString("\r\n");

    servo_set_angle(SERVO_ANGLE_RIGHT);
    delay_ms(SERVO_MOVE_MS);
    delay_ms(SENSOR_SETTLE_MS);
    *out_right = ultrasonic_get_stable_distance();

    USART1_SendString("  Right cm: ");
    USART1_SendUInt32(*out_right);
    USART1_SendString("\r\n");

    servo_set_angle(90U);
    delay_ms(SERVO_CENTER_MS);
}

/* ================================================================
 *  PICK TURN DIRECTION
 *
 *  Returns 0 = turn left, 1 = turn right.
 *  Prefers the side with more clearance when both are clear.
 *  Falls back to whichever single side is clear.
 * ================================================================ */

uint8_t pick_turn_dir(uint32_t dist_left,  uint8_t clear_left,
                      uint32_t dist_right, uint8_t clear_right)
{
    if(clear_left && clear_right)
    {
        return (dist_left >= dist_right) ? 0U : 1U;
    }

    return clear_left ? 0U : 1U;
}

/* ================================================================
 *  EXECUTE TURN
 * ================================================================ */

void execute_turn(uint8_t dir)
{
    if(dir == 0U)
    {
        USART1_SendString(">>> Turning LEFT\r\n");
        turn_left();
    }
    else
    {
        USART1_SendString(">>> Turning RIGHT\r\n");
        turn_right();
    }

    delay_ms(AUTO_TURN_MS);
    stop_motors();
}

/* ================================================================
 *  AUTO MODE OBSTACLE SCAN  [UPDATED LOGIC]
 *
 *  PART 1 — Find a clear lateral direction:
 *    Scan left, then right.
 *    If both sides <= 35cm: scan again (2nd pass).
 *    If still both blocked: reverse briefly, then restart the
 *    whole sequence from the top.
 *
 *  PART 2 — Confirm it is safe to move before turning:
 *    Once a clear lateral side is found, re-check the FRONT.
 *    Threshold for front re-check is FRONT_RECHECK_CM (20cm).
 *
 *    Case A — front > 20cm:
 *      Safe to proceed. Turn toward the clear side immediately.
 *
 *    Case B — front <= 20cm:
 *      Still too close. Reverse briefly, then:
 *        - Scan left and right again.
 *        - Check front again (servo at center).
 *        If front now > 20cm: turn toward clear side.
 *        If front still <= 20cm but a lateral side is clear:
 *          Turn anyway (we already reversed to create room).
 *        If all sides blocked: restart from top.
 * ================================================================ */

void auto_mode_obstacle_scan(void)
{
    uint32_t dist_left;
    uint32_t dist_right;
    uint32_t dist_front;
    uint8_t  clear_left;
    uint8_t  clear_right;
    uint8_t  clear_found;
    uint8_t  turn_dir;
    uint8_t  scan_pass;

    stop_motors();
    buzzer_on();

    while(GPIOB->IDR & (1U << 9U))
    {
        clear_found = 0U;
        dist_left   = INVALID_DISTANCE_CM;
        dist_right  = INVALID_DISTANCE_CM;
        turn_dir    = 0U;

        /* ==================================================
         *  PART 1: scan left and right (up to 2 passes)
         *  Goal: find at least one side with distance > 35cm
         * ================================================== */

        for(scan_pass = 0U; scan_pass < 2U; scan_pass++)
        {
            if(scan_pass == 0U)
            {
                USART1_SendString("\r\n--- Obstacle! Scanning L+R ---\r\n");
            }
            else
            {
                USART1_SendString("--- Re-scanning L+R ---\r\n");
                delay_ms(RESCAN_PAUSE_MS);
            }

            scan_left_right(&dist_left, &dist_right);

            clear_left  = distance_is_clear(dist_left);
            clear_right = distance_is_clear(dist_right);

            if(clear_left || clear_right)
            {
                clear_found = 1U;
                break;
            }

            USART1_SendString("  Both sides blocked (<= 35cm).\r\n");
        }

        if(!clear_found)
        {
            /* Both passes showed both sides blocked.
             * Reverse briefly and restart the whole scan. */
            USART1_SendString(">>> Both sides blocked x2. Reversing...\r\n");

            move_backward();
            delay_ms(AUTO_BACKUP_MS);
            stop_motors();
            delay_ms(RESCAN_PAUSE_MS);

            continue;   /* restart outer while loop */
        }

        /* Decide which lateral side to turn toward */
        turn_dir = pick_turn_dir(dist_left,  clear_left,
                                 dist_right, clear_right);

        USART1_SendString("  Clear side found. Rechecking front...\r\n");

        /* ==================================================
         *  PART 2: re-check front before committing to turn
         *  Servo is already centered from scan_left_right().
         * ================================================== */

        dist_front = ultrasonic_get_stable_distance();

        USART1_SendString("  Front cm: ");
        USART1_SendUInt32(dist_front);
        USART1_SendString("\r\n");

        /* --------------------------------------------------
         *  Case A: front > 20cm — safe to turn immediately
         * -------------------------------------------------- */

        if(dist_front > FRONT_RECHECK_CM)
        {
            USART1_SendString("  Front clear (>20cm). Turning now.\r\n");

            execute_turn(turn_dir);
            buzzer_off();
            return;
        }

        /* --------------------------------------------------
         *  Case B: front <= 20cm — reverse first, then
         *  re-scan left + right + front again.
         * -------------------------------------------------- */

        USART1_SendString("  Front still blocked (<=20cm). Reversing...\r\n");

        move_backward();
        delay_ms(AUTO_BACKUP_MS);
        stop_motors();
        delay_ms(RESCAN_PAUSE_MS);

        /* Re-scan left and right after backup */
        USART1_SendString("--- Scanning L+R after reverse ---\r\n");

        scan_left_right(&dist_left, &dist_right);

        clear_left  = distance_is_clear(dist_left);
        clear_right = distance_is_clear(dist_right);

        /* Re-check front (servo is centered from scan_left_right) */
        dist_front = ultrasonic_get_stable_distance();

        USART1_SendString("  Front cm (post-reverse): ");
        USART1_SendUInt32(dist_front);
        USART1_SendString("\r\n");

        /* Update turn direction based on new readings */
        if(clear_left || clear_right)
        {
            turn_dir = pick_turn_dir(dist_left,  clear_left,
                                     dist_right, clear_right);
        }

        if(dist_front > FRONT_RECHECK_CM)
        {
            /* Front is now clear — turn toward safe side */
            USART1_SendString("  Front clear after reverse. Turning.\r\n");

            execute_turn(turn_dir);
            buzzer_off();
            return;
        }

        if(clear_left || clear_right)
        {
            /* Front still close but we reversed and a side is clear.
             * Turn now — the backup already created enough room. */
            USART1_SendString("  Front tight but lateral clear. Turning.\r\n");

            execute_turn(turn_dir);
            buzzer_off();
            return;
        }

        /* All sides still blocked after reverse — restart outer loop */
        USART1_SendString("  All sides blocked after reverse. Retrying...\r\n");
    }

    /* Auto mode switched off during scan */
    stop_motors();
    buzzer_off();
    servo_set_angle(90U);
}

/* ================================================================
 *  AUTO MODE MAIN LOGIC
 * ================================================================ */

void auto_mode_run(void)
{
    uint32_t dist;

    speed_percent = AUTO_SPEED;

    dist = ultrasonic_get_distance_cm();

    if(distance_is_clear(dist))
    {
        /* Path open (includes 999 = nothing detected) */
        buzzer_off();
        if(current_action != 'F') move_forward();
    }
    else
    {
        /* Something within 20 cm ahead */
        stop_motors();
        buzzer_on();
        USART1_SendString("\r\nFRONT blocked: ");
        USART1_SendUInt32(dist);
        USART1_SendString("cm\r\n");
        auto_mode_obstacle_scan();
    }
}

/* ================================================================
 *  BLUETOOTH COMMAND PARSER
 * ================================================================ */

void process_received_char(char c)
{
    uint32_t dist;

    if(parsing_speed)
    {
        if(c >= '0' && c <= '9')
        {
            speed_value = (speed_value * 10U) + (uint16_t)(c - '0');
            if(speed_value > 100U) speed_value = 100U;
            speed_percent = (uint8_t)speed_value;
            apply_current_action();
            return;
        }
        else if(c == ' ' || c == '\r' || c == '\n')
        {
            return;
        }
        else
        {
            parsing_speed = 0U;
            speed_value   = 0U;
        }
    }

    if(c == 'F' || c == 'f')
{
    uint32_t dist = ultrasonic_get_stable_distance();  // use stable readings

    if(!manual_distance_clear(dist))
    {
        buzzer_on();
        stop_motors();
        USART1_SendString("BLOCKED! Distance ");
        USART1_SendUInt32(dist);
        USART1_SendString("cm. Use B/L/R.\r\n");
    }
    else
    {
        buzzer_off();
        move_forward();
        USART1_SendString("Forward\r\n");
    }
}
    else if(c == 'B' || c == 'b')
    {
        buzzer_off(); move_backward();
        USART1_SendString("Backward\r\n");
    }
    else if(c == 'L' || c == 'l')
    {
        buzzer_off(); turn_left();
        USART1_SendString("Left\r\n");
    }
    else if(c == 'R' || c == 'r')
    {
        buzzer_off(); turn_right();
        USART1_SendString("Right\r\n");
    }
    else if(c == 'S' || c == 's')
    {
        buzzer_off(); stop_motors();
        USART1_SendString("Stop\r\n");
    }
    else if(c == 'V' || c == 'v')
    {
        parsing_speed = 1U;
        speed_value   = 0U;
    }
}

/* ================================================================
 *  MAIN
 * ================================================================ */

int main(void)
{
    char    cmd;
    uint8_t auto_mode = 0U;
    uint8_t prev_mode = 0xFFU;
    uint32_t dist;

    SystemInit();

    GPIO_Init_Custom();
    TIM2_PWM_Init();
    TIM3_Servo_Init();
    TIM4_Micros_Init();
    USART1_Init();

    stop_motors();
    buzzer_off();
    servo_set_angle(90U);

    USART1_SendString("\r\n==============================\r\n");
    USART1_SendString("  Rover Ready\r\n");
    USART1_SendString("==============================\r\n");
    USART1_SendString("CLEAR   = distance > 35cm (999 = nothing detected)\r\n");
    USART1_SendString("BLOCKED = distance <= 35cm\r\n");
    USART1_SendString("PB9=0: Manual | PB9=1: Auto\r\n\r\n");

    while(1)
    {
        auto_mode = (GPIOB->IDR & (1U << 9U)) ? 1U : 0U;

        if(auto_mode != prev_mode)
        {
            stop_motors();
            buzzer_off();
            servo_set_angle(90U);

            if(auto_mode)
            {
                USART1_SendString("\r\n>>> AUTO MODE ON <<<\r\n");
                speed_percent = AUTO_SPEED;
            }
            else
            {
                USART1_SendString("\r\n>>> MANUAL MODE ON <<<\r\n");
            }

            prev_mode = auto_mode;
        }

        if(!auto_mode)
        {
            cmd = USART1_ReadChar();
            if(cmd != 0) process_received_char(cmd);

            /* Safety: stop forward if obstacle appears while driving */
            if(current_action == 'F')
            {
                dist = ultrasonic_get_distance_cm();

                if(distance_is_blocked(dist))
                {
                    buzzer_on();
                    stop_motors();
                    USART1_SendString("OBSTACLE! Auto-stopped ");
                    USART1_SendUInt32(dist);
                    USART1_SendString("cm. Use B/L/R.\r\n");
                }
                else
                {
                    buzzer_off();
                }
            }
        }
        else
        {
            auto_mode_run();
        }
    }
}