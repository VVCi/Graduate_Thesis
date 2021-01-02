#include <string.h>

#include "fsm.h"
#include "timer.h"
#include "port.h"
#include "message.h"

#include "sys_ctrl.h"
#include "sys_dbg.h"
#include "xprintf.h"

#include "app.h"
#include "app_dbg.h"

#include "task_list.h"
#include <math.h>
#include <stdio.h>

#include "io_cfg.h"
#include "task_encoder_pid.h"
#include "system.h"

#include "stm32f4xx.h"

encoder_t enc_fb1, enc_fb2, enc_fb3 ;
delta_if_t delta_data;
param_rb_t param_robot_data;
float pid_degree = 30;
float pid_time = 1;

int16_t MPU6050data[7];
float accel_x_OC, accel_y_OC, accel_z_OC, gyro_x_OC,gyro_y_OC, gyro_z_OC; // offset variables
float temp_scalled,accel_x_scalled,accel_y_scalled,accel_z_scalled,gyro_x_scalled,gyro_y_scalled,gyro_z_scalled; //Scalled Data varaibles
float accel_scale_fact, gyro_scale_fact; // Scale factor variables
int16_t temp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

void task_encoder_pid(ak_msg_t* msg) {
	switch (msg->sig) {
	case SL_DELTA_ROBOT_AUTO_RUN_REQ: {
		/* Mode Read_RF */
		/* 1. Read RF */
		/* 2. Steering Control*/

		/* Mode Auto Handle */
		/* 1. Read_MPU return data */

		/*if (MPU6050_TestConnection() == 1){
			xprintf(" Connected! ");
			MPU6050_GetRawAccelTempGyro(MPU6050data);
			MPU6050_OffsetCal();
			xprintf("%f", gyro_x_OC);
		}*/
		/* 2. Steering Control */
		ENCODER_Read_Release(&enc_fb2.Pos_encoder_feedback, &enc_fb2.Dir_feedback, &enc_fb2.Cnt_feedback, TIM5);
		Turn_Release(2, &TIM08_Data_CH2, TM_PWM_Channel_2, pid_feeback2);
		/* Running Control*/
		//Turn_Release(1, &TIM08_Data_CH1, TM_PWM_Channel_1, 70);
		//MPU6050_Initialize();

	}
		break;

	case SL_DELTA_ROBOT_IF_WRITE_BROADCAST_REQ: {
		APP_DBG("SL_DELTA_ROBOT_IF_WRITE_BROADCAST_REQ\n");
		memset(&delta_data, 0, sizeof(delta_if_t));
		memset(&param_robot_data, 0, sizeof(param_rb_t));
		memcpy(&delta_data, ((ak_msg_common_t*)msg)->data, sizeof(delta_if_t));
		/* take data(direction, degree, setting time and division_time) from UART Master to param_robot_data */
		/* BUILD_UINT16(loByte, hiByte) */
		/* param for motor1 */

		param_robot_data.direction		= delta_data.data[15];
		param_robot_data.degree			= BUILD_UINT16(delta_data.data[17], delta_data.data[16]);
		param_robot_data.setting_time	= delta_data.data[18];
		param_robot_data.division_time	= delta_data.data[19];

		xprintf("param_robot_data.direction: %d	-	", param_robot_data.direction);
		xprintf("param_robot_data.degree: %d	-	", param_robot_data.degree);
		xprintf("param_robot_data.setting_time: %d	-	", param_robot_data.setting_time);
		xprintf("param_robot_data.division_time: %d\n", param_robot_data.division_time);
	}
		break;
	case SL_DELTA_ROBOT_IF_START_BROADCAST_REQ: {
		xprintf("START DC4\n");
		if (param_robot_data.direction == POS_DIR) {
			pid_degree = -param_robot_data.degree;
		}
		else if (param_robot_data.direction == NEG_DIR){
			pid_degree = param_robot_data.degree;
		}
		pid_time = (float)param_robot_data.setting_time/param_robot_data.division_time;

		xprintf("pid_degree: %d\n", (int16_t)pid_degree);
		xprintf("pid_time: %d\n", (int8_t)pid_time);
	}
		break;

	case SL_DELTA_ROBOT_IF_RESET_DATA_REQ: {
		APP_DBG("SL_DELTA_ROBOT_IF_RESET_DATA_REQ\n");
		sys_ctrl_reset();
	}
		break;

	default:
		break;
	}
}

void read_MPU(){

}
