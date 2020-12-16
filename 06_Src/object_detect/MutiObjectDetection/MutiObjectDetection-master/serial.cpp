#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <semaphore.h>

static int if_cpu_serial_fd;

#define IPCPU_SOP_CHAR		0xEF
#define IFCPU_DATA_SIZE		254

#define RX_BUFFER_SIZE		254

static uint8_t tx_buffer[1024];

int if_cpu_serial_opentty(const char* devpath) {
	struct termios options;
	printf("\nOpen Serial OK\n");
	printf("[IF CPU SERIAL][if_cpu_serial_opentty] devpath: %s\n", devpath);

	if_cpu_serial_fd = open(devpath, O_RDWR | O_NOCTTY | O_NDELAY);
	if (if_cpu_serial_fd < 0) {
		return if_cpu_serial_fd;
	}
	else {
		printf("\nTranfer Handing");
		fcntl(if_cpu_serial_fd, F_SETFL, 0);
		/* get current status */
		tcgetattr(if_cpu_serial_fd, &options);

		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);

		/* No parity (8N1) */
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;

		options.c_cflag |= (CLOCAL | CREAD);
		options.c_cflag &=  ~CRTSCTS;

		cfmakeraw(&options);

		tcflush(if_cpu_serial_fd, TCIFLUSH);
		if (tcsetattr (if_cpu_serial_fd, TCSANOW, &options) != 0) {
			printf("error in tcsetattr()\n");
		}
	}
	return 0;
}

/* Calculate IF_CPU_SERIAL frame FCS */
uint8_t if_cpu_serial_calcfcs(uint8_t len, uint8_t *data_ptr) {
	uint8_t xor_result;
	xor_result = len;

	for (int i = 0; i < len; i++, data_ptr++) {
		xor_result = xor_result ^ *data_ptr;
	}
	return xor_result;
}

int tx_frame_post(uint8_t* data, uint8_t len) {
	tx_buffer[0] = IPCPU_SOP_CHAR;
	tx_buffer[1] = len;
	memcpy(&tx_buffer[2], data, len);
	tx_buffer[2 + len] = if_cpu_serial_calcfcs(len, data);
	return write(if_cpu_serial_fd, tx_buffer, (len + 3));
}
