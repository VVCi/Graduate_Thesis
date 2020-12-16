/*@Author: NinhNN - CoVV
 *@Date: 2020
 *@Note: Open Source GNU/Linux
*/

#ifndef SERIAL_H
#define SERIAL_H

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int if_cpu_serial_opentty(const char* devpath);
int tx_frame_post(uint8_t* data, uint8_t len);

#ifdef __cplusplus
#endif
#endif
