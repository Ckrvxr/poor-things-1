#include "Grayscale_Sensor.h"
#include "Encoder.h"
#include "PID.h"
#include "AT8236.h"

// 速度环PID控制器（保留你原来的正确配置）
volatile PIDController pid_speed_l;
volatile float_t output_l;
volatile float_t feedback_l;

volatile PIDController pid_speed_r;
volatile float_t output_r;
volatile float_t feedback_r;

// 正方形行驶控制状态
typedef enum {
    MOVE_FORWARD,
    TURN_RIGHT,
    TURN_LEFT
} car_state_t;

static car_state_t current_state = MOVE_FORWARD;
static uint32_t state_timer = 0;
static uint32_t forward_distance = 0;  // 前进距离计数
static int8_t square_side_count = 0;   // 已完成的边数

// 距离检测相关
static int32_t last_encoder_sum = 0;

void TIMG0_TASKS_250Hz_Init(void)
{
    // 保留你的正确速度环配置
    PID_Init(&pid_speed_l, 250.0f, 1.55f, 0.31f, -999.0f, 999.0f);
    PID_Init(&pid_speed_r, 250.0f, 1.55f, 0.31f, -999.0f, 999.0f);

    delay_cycles(96000000);
    PID_SetSetpoint(&pid_speed_l, 0.0f);
    PID_SetSetpoint(&pid_speed_r, 0.0f);
    NVIC_ClearPendingIRQ(TIMG0_TASKS_250Hz_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMG0_TASKS_250Hz_INST_INT_IRQN);
}

// 计算行驶距离（编码器增量）
int32_t get_distance_traveled(void)
{
    int32_t current_sum = encoder_l.delta_count + encoder_r.delta_count;
    int32_t distance = current_sum - last_encoder_sum;
    last_encoder_sum = current_sum;
    return distance;
}

// 正方形行驶控制函数
void square_patrol_control(void)
{
    static float_t target_speed_l = 0.0f;
    static float_t target_speed_r = 0.0f;
    
    // 每个状态的持续时间（根据实际调试调整）
    const uint32_t FORWARD_TIME = 200;   // 前进约0.8秒 (200 * 4ms)
    const uint32_t TURN_TIME = 40;       // 转弯约0.16秒 (40 * 4ms)
    
    switch(current_state)
    {
        case MOVE_FORWARD:
            // 直行
            target_speed_l = 4.0f;
            target_speed_r = 4.0f;
            
            state_timer++;
            if(state_timer >= FORWARD_TIME)
            {
                current_state = TURN_RIGHT;
                state_timer = 0;
                square_side_count++;
                if(square_side_count >= 4)
                {
                    square_side_count = 0;  // 完成一圈
                }
            }
            break;
            
        case TURN_RIGHT:
            // 右转
            target_speed_l = 3.0f;
            target_speed_r = -3.0f;
            
            state_timer++;
            if(state_timer >= TURN_TIME)
            {
                current_state = MOVE_FORWARD;
                state_timer = 0;
            }
            break;
            
        case TURN_LEFT:
            // 左转（备用）
            target_speed_l = -3.0f;
            target_speed_r = 3.0f;
            
            state_timer++;
            if(state_timer >= TURN_TIME)
            {
                current_state = MOVE_FORWARD;
                state_timer = 0;
            }
            break;
    }
    
    // 设置速度环目标值
    PID_SetSetpoint(&pid_speed_l, target_speed_l);
    PID_SetSetpoint(&pid_speed_r, target_speed_r);
}

void TIMG0_TASKS_250Hz_INST_IRQHandler(void)
{
    Grayscale_Sensor_Read();
    Encoder_Update();

    // 获取编码器反馈
    feedback_l = (float_t)encoder_l.delta_count;
    feedback_r = (float_t)encoder_r.delta_count;

    // 执行正方形行驶控制
    square_patrol_control();

    // 速度环控制
    output_l = PID_Update(&pid_speed_l, feedback_l);
    output_r = PID_Update(&pid_speed_r, feedback_r);

    // 输出到电机
    AT8236_PWM_L((int32_t)output_l);
    AT8236_PWM_R((int32_t)output_r);
}