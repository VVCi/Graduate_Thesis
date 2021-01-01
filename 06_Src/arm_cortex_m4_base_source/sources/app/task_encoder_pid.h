#ifndef __TASK_ENCODER_PID__
#define __TASK_ENCODER_PID__

#include <stdint.h>
#include "app.h"

#define IFCPU_DELTA_ROBOT_IF_DATA_SIZE		20

#define POS_DIR								1
#define NEG_DIR								0

#define BUILD_UINT16(loByte, hiByte) \
		  ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

typedef struct {
	uint32_t Dir_feedback;
	uint32_t Cnt_feedback;
	uint32_t Cnt_feedback_pre;
	float Pos_encoder_feedback;
} encoder_t;

typedef struct {
	uint8_t cmd;
	uint8_t len;
	uint8_t data[IFCPU_DELTA_ROBOT_IF_DATA_SIZE];
} delta_if_t;

typedef struct {
	uint16_t degree;
	uint8_t setting_time;
	uint8_t division_time;
	uint8_t direction;
} param_rb_t;



extern encoder_t enc_fb1, enc_fb2, enc_fb3;
extern float pid_degree;
extern float pid_time;

#endif //__TASK_ENCODER_PID__
