#include "chassis_task.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "INS_task.h"
#include "referee.h"

// 功率限制算法的变量定义
float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;
double Chassis_pidout_target;
static double Scaling1 = 0, Scaling2 = 0, Scaling3 = 0, Scaling4 = 0;
float Klimit = 1;
float Plimit = 0;
float Chassis_pidout_max;

extern ext_power_heat_data_t power_heat_data_t;

//遥控器输入的死区处理
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
//
chassis_move_t chassis_move;


void chassis_task(void const *pvParameters)
{

	
		     
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis_move);//底盘初始化

	
    while(1)
    {
        //底盘模式
        chassis_set_mode(&chassis_move);
        //反馈函数更新
        chassis_feedback_update(&chassis_move);
        //
        chassis_set_contorl(&chassis_move);
        //
        chassis_control_loop(&chassis_move);

			   //
        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                              chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);

			
				 vTaskDelay(2); //500HZ
    }



}



/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //电机速度PID
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    
    //YAW值PID
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
	//滤波器参数	
	const static fp32 chassis_x_order_filter[1] = { CHASSIS_ACCEL_X_NUM };
    const static fp32 chassis_y_order_filter[1] = { CHASSIS_ACCEL_Y_NUM };
    const static fp32 chassis_z_order_filter[1] = { CHASSIS_ACCEL_Z_NUM };
		
		
		
    uint8_t i;

    //获取遥控器指针、惯导角度指针、底盘偏航电机指针和俯仰电机指针。
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //
    chassis_move_init->chassis_RC = get_remote_control_point();
    //
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //使用循环初始化四个底盘电机的参数，并初始化电机速度 PID 控制器。
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //初始化底盘跟随云台的 PID 控制器
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
		
	//初始化底盘速度的三个一阶滤波器。
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME, chassis_z_order_filter);

	//设置底盘运动结构体中的最大最小正常速度和冲刺速度
    chassis_move_init->vx_max_normal_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_normal_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_normal_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_normal_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vx_max_dash_speed = DASH_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_dash_speed = -DASH_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vy_max_dash_speed = DASH_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_dash_speed = -DASH_MAX_CHASSIS_SPEED_Y;
				
    //调用底盘反馈更新函数
    chassis_feedback_update(chassis_move_init);
}



/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}



static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
     if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {

         chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;         //右1，云台跟随																																																																																						
    }

    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
       chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;               //右2，云台不跟随
    }
     else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
          chassis_move_mode->chassis_mode = CHASSIS_SMALL_GYROSCOPE;        //右1，小陀螺
    } 
}


static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)//无遥控信号
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    
     chassis_rc_to_control_vector(&vx_set, &vy_set, &angle_set,chassis_move_control);
   // chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //云台跟随
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //

        if (chassis_move_control->chassis_RC->key.v & SWING_KEY)
        {
            chassis_move_control->is_whipping = 1;
            chassis_move_control->wz_set = CHASSIS_GYROSCOPE_SPEED;    //
        }
        else
        {
            chassis_move_control->is_whipping = 0;
            chassis_move_control->wz_set = -PID_calc_swing_wz(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);//���̸�����̨,
        }

        //
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_dash_speed, chassis_move_control->vx_max_dash_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_dash_speed, chassis_move_control->vy_max_dash_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_SMALL_GYROSCOPE)//小陀螺
		{  	
			fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
       
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //
        chassis_move_control->is_whipping = 1;
        chassis_move_control->wz_set = CHASSIS_GYROSCOPE_SPEED;    //
        //
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_dash_speed, chassis_move_control->vx_max_dash_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_dash_speed, chassis_move_control->vy_max_dash_speed);	
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW) //云台不跟随
    {
        //
        chassis_move_control->wz_set = angle_set; //
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_dash_speed, chassis_move_control->vx_max_dash_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_dash_speed, chassis_move_control->vy_max_dash_speed);
    }
}


/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     wz_set: ��ת�ٶ�ָ�� 
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel, wz_channel;
    fp32 vx_set_channel, vy_set_channel, wz_set_channel;
    //死区限制
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Z_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
    wz_set_channel = wz_channel * CHASSIS_WZ_RC_SEN;

    //遥控器值转化为输出值
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_DASH_KEY)
            vx_set_channel = chassis_move_rc_to_vector->vx_max_dash_speed;
        else
            vx_set_channel = chassis_move_rc_to_vector->vx_max_normal_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_DASH_KEY)
            vx_set_channel = chassis_move_rc_to_vector->vx_min_dash_speed;
        else
            vx_set_channel = chassis_move_rc_to_vector->vx_min_normal_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_DASH_KEY)
            vy_set_channel = chassis_move_rc_to_vector->vy_max_dash_speed;
        else
            vy_set_channel = chassis_move_rc_to_vector->vy_max_normal_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_DASH_KEY)
            vy_set_channel = chassis_move_rc_to_vector->vy_min_dash_speed;
        else
            vy_set_channel = chassis_move_rc_to_vector->vy_min_normal_speed;
    }

    //一阶滤波
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_wz, wz_set_channel);

    //ֹͣ死区处理
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }
    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
    if (wz_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN && wz_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
    *wz_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out;
}

// 速度限制函数
static void Motor_Speed_limiting(volatile fp32 *motor_speed, int16_t limit_speed)
{
  uint8_t i = 0;
  int16_t max = 0;
  int16_t temp = 0;
  int16_t max_speed = limit_speed;
  fp32 rate = 0;
  for (i = 0; i < 4; i++)
  {
    temp = (motor_speed[i] > 0) ? (motor_speed[i]) : (-motor_speed[i]); // 求绝对值

    if (temp > max)
    {
      max = temp;
    }
  }

  if (max > max_speed)
  {
    rate = max_speed * 1.0 / max; //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
    for (i = 0; i < 4; i++)
    {
      motor_speed[i] *= rate;
    }
  }
}

static void chassis_power_limit(double Chassis_pidout_target_limit,chassis_move_t *chassis_move_power_limit)
{
  // 819.2/A，假设最大功率为120W，那么能够通过的最大电流为5A，取一个保守值：800.0 * 5 = 4000

  Watch_Power_Max = Klimit;
  Watch_Power = power_heat_data_t.chassis_power;
  Watch_Buffer = 60; // Hero_chassis_power_buffer;//限制值，功率值，缓冲能量值，初始值是1，0，0
  // get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);//通过裁判系统和编码器值获取（限制值，实时功率，实时缓冲能量）

  Chassis_pidout_max = 61536; // 32768，40，960			15384 * 4，取了4个3508电机最大电流的一个保守值

  if (Watch_Power > 600)
 
 {// 创建一个数组存放speed_set的值
  fp32 speed_set_array[4] = {
    chassis_move_power_limit->motor_chassis[0].speed_set,
    chassis_move_power_limit->motor_chassis[1].speed_set,
    chassis_move_power_limit->motor_chassis[2].speed_set,
    chassis_move_power_limit->motor_chassis[3].speed_set
  };
  // 调用Motor_Speed_limiting函数
  Motor_Speed_limiting(speed_set_array, 4096); // 限制最大速度 ;//5*4*24;先以最大电流启动，后平滑改变，不知道为啥一开始用的Power>960,可以观测下这个值，看看能不能压榨缓冲功率
 }
  else
  {
    Chassis_pidout = (fabs(chassis_move_power_limit->motor_chassis[0].speed_set - chassis_move_power_limit->motor_chassis[0].speed) +
                      fabs(chassis_move_power_limit->motor_chassis[1].speed_set - chassis_move_power_limit->motor_chassis[1].speed) +
                      fabs(chassis_move_power_limit->motor_chassis[2].speed_set - chassis_move_power_limit->motor_chassis[2].speed) +
                      fabs(chassis_move_power_limit->motor_chassis[3].speed_set - chassis_move_power_limit->motor_chassis[3].speed)); // fabs是求绝对值，这里获取了4个轮子的差值求和

    //	Chassis_pidout_target = fabs(motor_speed_target[0]) + fabs(motor_speed_target[1]) + fabs(motor_speed_target[2]) + fabs(motor_speed_target[3]);

    /*期望滞后占比环，增益个体加速度*/
    if (Chassis_pidout)
    {
      Scaling1 = (chassis_move_power_limit->motor_chassis[0].speed_set - chassis_move_power_limit->motor_chassis[0].speed) / Chassis_pidout;
      Scaling2 = (chassis_move_power_limit->motor_chassis[0].speed_set - chassis_move_power_limit->motor_chassis[1].speed) / Chassis_pidout;
      Scaling3 = (chassis_move_power_limit->motor_chassis[0].speed_set - chassis_move_power_limit->motor_chassis[2].speed) / Chassis_pidout;
      Scaling4 = (chassis_move_power_limit->motor_chassis[0].speed_set - chassis_move_power_limit->motor_chassis[3].speed) / Chassis_pidout; // 求比例，4个scaling求和为1
    }
    else
    {
      Scaling1 = 0.25, Scaling2 = 0.25, Scaling3 = 0.25, Scaling4 = 0.25;
    }

    /*功率满输出占比环，车总增益加速度*/
    //		if(Chassis_pidout_target) Klimit=Chassis_pidout/Chassis_pidout_target;	//375*4 = 1500
    //		else{Klimit = 0;}
    Klimit = Chassis_pidout / Chassis_pidout_target_limit;

    if (Klimit > 1)
      Klimit = 1;
    else if (Klimit < -1)
      Klimit = -1; // 限制绝对值不能超过1，也就是Chassis_pidout一定要小于某个速度值，不能超调

    /*缓冲能量占比环，总体约束*/
    if (Watch_Buffer < 50 && Watch_Buffer >= 40)
      Plimit = 0.9; // 近似于以一个线性来约束比例（为了保守可以调低Plimit，但会影响响应速度）
    else if (Watch_Buffer < 40 && Watch_Buffer >= 35)
      Plimit = 0.75;
    else if (Watch_Buffer < 35 && Watch_Buffer >= 30)
      Plimit = 0.5;
    else if (Watch_Buffer < 30 && Watch_Buffer >= 20)
      Plimit = 0.25;
    else if (Watch_Buffer < 20 && Watch_Buffer >= 10)
      Plimit = 0.125;
    else if (Watch_Buffer < 10 && Watch_Buffer >= 0)
      Plimit = 0.05;
    else
    {
      Plimit = 1;
    }

    chassis_move_power_limit->motor_chassis[0].give_current = Scaling1 * (Chassis_pidout_max * Klimit) * Plimit; // 输出值
    chassis_move_power_limit->motor_chassis[1].give_current = Scaling2 * (Chassis_pidout_max * Klimit) * Plimit;
    chassis_move_power_limit->motor_chassis[2].give_current = Scaling3 * (Chassis_pidout_max * Klimit) * Plimit;
    chassis_move_power_limit->motor_chassis[3].give_current = Scaling4 * (Chassis_pidout_max * Klimit) * Plimit; /*同比缩放电流*/
  }
}

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)//底盘控制循环
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //raw
        return;
    }

    //电机速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //pid计算
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //功率限制
   chassis_power_limit(8000,&chassis_move);

    //输出
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}



static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //轮速计算
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] =  vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] =  vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}
