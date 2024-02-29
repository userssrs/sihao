
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 200

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//��ת��ң����ͨ������    
#define CHASSIS_Z_CHANNEL 4

//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.0045f    //0.006f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.00375f    //0.005f
//ң������תҡ�ˣ�max 660��ת���ɳ�����ת�ٶȣ�rad/s���ı���
#define CHASSIS_WZ_RC_SEN 0.006f    //0.006f
//���̸�����̨ģʽ�£�ʳָ���ֿ��Ƶ���Ť���Ķ������ȣ�ƾ�о�����
#define CHASSIS_WRIGGLE_SEN 0.4f    //0.4f

//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
//#define CHASSIS_WZ_RC_SEN 0.01f

//����С�����ٶȣ�rad/s��    
#define CHASSIS_GYROSCOPE_SPEED 2.6f * PI

#define CHASSIS_ACCEL_X_NUM 0.001f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Z_NUM 0.1666666667f

//ҡ������
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.2f           //0.2f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY  KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY  KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
#define CHASSIS_DASH_KEY  KEY_PRESSED_OFFSET_SHIFT
//���°��������̵���ת���󣬲��������ָǡ��ٰ�һ�ιرա�
#define CHASSIS_TURN_ASS_KEY KEY_PRESSED_OFFSET_Z

//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f

//�����˶�����ƽ��ǰ���ٶȣ�ֻ���������
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//�����˶�����ƽ��ƽ���ٶȣ�ֻ���������
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f
//�����˶��������ǰ���ٶȣ���סSHIFT��
#define DASH_MAX_CHASSIS_SPEED_X   3.9f  // 3.9
//�����˶��������ƽ���ٶȣ���סSHIFT��
#define DASH_MAX_CHASSIS_SPEED_Y   3.9f   //3.9

#define CHASSIS_WZ_SET_SCALE -0.00625f       //0.1f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 8500
#define M3505_MOTOR_SPEED_PID_KI 30	
#define M3505_MOTOR_SPEED_PID_KD 0.5f
#define M3505_MOTOR_SPEED_PID_MAX_OUT  MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 16.0f      //40.0f  before 16.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.01f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.4f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 9.0f  //6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //���̻������̨��ԽǶ�
  CHASSIS_SMALL_GYROSCOPE,  //�����е��̽Ƕȿ��Ʊջ�
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //��������ת�ٶȿ���
  CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.

} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               // 底盘使用的遥控器指针
  const gimbal_motor_t *chassis_yaw_motor;   // 底盘使用的yaw电机指针，用于获取电机角度和控制电机转速的PID
  const gimbal_motor_t *chassis_pitch_motor; // 底盘使用的pitch电机指针，用于获取电机角度和控制电机转速的PID
  const fp32 *chassis_INS_angle;             // 获取底盘的惯导角度指针
  chassis_mode_e chassis_mode;               // 底盘的工作模式
  chassis_mode_e last_chassis_mode;          // 上一时刻的底盘工作模式
  chassis_motor_t motor_chassis[4];          // 底盘的四个电机
  pid_type_def motor_speed_pid[4];           // 电机转速PID控制器数组
  pid_type_def chassis_angle_pid;            // 底盘角度PID控制器

  first_order_filter_type_t chassis_cmd_slow_set_vx;  // 底盘x轴线速度的一阶滤波器
  first_order_filter_type_t chassis_cmd_slow_set_vy;  // 底盘y轴线速度的一阶滤波器
  first_order_filter_type_t chassis_cmd_slow_set_wz;  // 底盘z轴角速度的一阶滤波器

  fp32 vx;                          // 底盘当前x轴线速度，正为前进，单位：m/s
  fp32 vy;                          // 底盘当前y轴线速度，正为右移，单位：m/s
  fp32 wz;                          // 底盘当前z轴角速度，正为顺时针旋转，单位：rad/s
  fp32 vx_set;                      // 底盘期望x轴线速度，正为前进，单位：m/s
  fp32 vy_set;                      // 底盘期望y轴线速度，正为右移，单位：m/s
  fp32 wz_set;                      // 底盘期望z轴角速度，正为顺时针旋转，单位：rad/s
  fp32 chassis_relative_angle;      // 底盘相对于云台的角度差，单位：rad
  fp32 chassis_relative_angle_set;  // 底盘期望相对于云台的角度差
  fp32 chassis_yaw_set;             // 底盘期望的yaw角度

  fp32 vx_max_normal_speed;  // 底盘前进的最大线速度，单位：m/s
  fp32 vx_min_normal_speed;  // 底盘后退的最大线速度，单位：m/s
  fp32 vy_max_normal_speed;  // 底盘右移的最大线速度，单位：m/s
  fp32 vy_min_normal_speed;  // 底盘左移的最大线速度，单位：m/s

  fp32 vx_max_dash_speed;    // 底盘前进的最大冲刺线速度，单位：m/s
  fp32 vx_min_dash_speed;    // 底盘后退的最大冲刺线速度，单位：m/s
  fp32 vy_max_dash_speed;    // 底盘右移的最大冲刺线速度，单位：m/s
  fp32 vy_min_dash_speed;    // 底盘左移的最大冲刺线速度，单位：m/s

  fp32 chassis_yaw;   // 底盘绝对yaw角度，即底盘相对于全局坐标系的yaw角度
  fp32 chassis_pitch; // 底盘绝对pitch角度，即底盘相对于全局坐标系的pitch角度
  fp32 chassis_roll;  // 底盘绝对roll角度，即底盘相对于全局坐标系的roll角度

  bool_t is_whipping;  // 是否正在进行鞭打动作

} chassis_move_t;



/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     wz_set: ��ת�ٶ�ָ��     
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          ��ȡ���̵�����״̬��   0�����쳣   1�����̵������   2��׼������   3�������쳣
  * @param[out]     none
  * @retval         uint8_t
  */
extern uint8_t get_chassis_state(void);

/**
  * @brief          ��ȡ�����Ƿ�������С����״̬��   0��δ����   1��������
  * @param[out]     none
  * @retval         uint8_t
  */
extern uint8_t get_chassis_is_whipping_flag(void);


static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
#endif