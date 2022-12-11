#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

static uint16_t CRC16_BUYPASS(uint8_t *data, size_t len) {
	uint16_t crc = 0x0000;
	size_t j;
	int i;
	for (j=len; j>0; j--) {
		crc ^= (uint16_t)(*data++) << 8;
		for (i=0; i<8; i++) {
			if (crc & 0x8000) crc = (crc<<1) ^ 0x8005;
			else crc <<= 1;
		}
	}
	return (crc);
}

int main(void) {
	char *portname = "/dev/serial/by-id/usb-067b_2303-if00-port0";
	int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "error %d opening %s: %s", errno, portname, strerror(errno));
		return -1;
	}
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0) {
		fprintf(stderr, "error %d from tcgetattr", errno);
		return -1;
	}
	cfsetispeed(&tty,B9600);
	cfsetospeed(&tty,B9600);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	 // 8-bit chars
	tty.c_cflag &= ~(PARENB | PARODD);			// shut off parity
	tty.c_cflag &= ~CSTOPB;						// CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	tty.c_cflag &= ~CRTSCTS;					// No Hardware flow Control
	tty.c_cflag |= (CLOCAL | CREAD);			// ignore modem controls, enable reading

	// disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
	tty.c_iflag = 0;
	tty.c_iflag &= ~IGNBRK;						// disable break processing
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);		// shut off xon/xoff ctrl
	tty.c_iflag &= ~ICANON;						// Non Cannonical mode

	tty.c_oflag = 0;							// no remapping, no delays

	tty.c_lflag = 0;							// no signaling chars, no echo, no canonical processing

//	tty.c_cc[VMIN] = 255;						// read doesn't block
	tty.c_cc[VMIN] = 0;						// read doesn't block
	tty.c_cc[VTIME] = 20;						// 0.5 seconds read timeout

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "error %d from tcsetattr", errno);
		return -1;
	}
	tcflush(fd, TCIFLUSH);	// Discards old data in the rx buffer
	char write_at_buffer[] = {0x41, 0x54, 0x0D};
	if(write(fd,write_at_buffer,sizeof(write_at_buffer)) < 0) {
		fprintf(stderr, "error writing AT", errno);
	}
//	for(int try=0; try<10; try++) {
		usleep(100000);
		tcflush(fd, TCIFLUSH);	// Discards old data in the rx buffer
		char write_cmd_buffer_time[] = {0x10, 0x02, 0x01, 0x13, 0x00, 0xC1, 0x00, 0x28, 0x08, 0x10, 0x03, 0x6D, 0x13};
/*		if(write(fd,write_cmd_buffer_time,sizeof(write_cmd_buffer_time)) < 0) {
			fprintf(stderr, "error writing command", errno);
		}
		char write_cmd_buffer_time_end[] = {0x10, 0x02, 0x01, 0x13, 0x00, 0xC4, 0x00, 0x00, 0x12, 0x10, 0x03, 0x59, 0x4C};
		if(write(fd,write_cmd_buffer_time_end,sizeof(write_cmd_buffer_time_end)) < 0) {
			fprintf(stderr, "error writing command", errno);
		}
*/		char write_cmd_buffer[] = {0x10, 0x02, 0x01, 0x15, 0x00, 0xC1, 0x00, 0x06, 0x10, 0x03, 0xF1, 0x1B};
		usleep(100000);
		tcflush(fd, TCIFLUSH);	// Discards old data in the rx buffer
		if(write(fd,write_cmd_buffer,sizeof(write_cmd_buffer)) < 0) {
			fprintf(stderr, "error writing command", errno);
		}
		int i = 0;
		char last = 0;
		char read_buffer[400];
		char result_buffer[352];
		int resultCounter = 0;
		int bytes_read = 0;
		for(;read(fd, &read_buffer[bytes_read], 1) != 0; bytes_read++);
		fprintf(stderr, "Bytes Rxed: %d\n", bytes_read);
		for(i=0;i<bytes_read;i++)
			fprintf(stderr, "%3.2x",read_buffer[i]);
		fprintf(stderr, "\n");
		if(read_buffer[0]!=22 || read_buffer[1]!=16 || read_buffer[2]!=2 || read_buffer[3]!=0 || read_buffer[4]!=23) {
			fprintf(stderr, "start bits wrong!\n");
			for(i=0;i<bytes_read;i++)
				fprintf(stderr, "%3.2x",read_buffer[i]);
			fprintf(stderr, "\n\n");
			for(i=0;i<resultCounter;i++) {
				fprintf(stderr, "%3.2x",result_buffer[i]);
			}
			fprintf(stderr, "\n\n");
			//return -1;
		} else {
			if(read_buffer[bytes_read-5]!=16 || read_buffer[bytes_read-4]!=3 || read_buffer[bytes_read-1]!=22) {
				fprintf(stderr, "end bits wrong!\n");
				for(i=0;i<bytes_read;i++)
					fprintf(stderr, "%3.2x",read_buffer[i]);
				fprintf(stderr, "\n\n");
				for(i=0;i<resultCounter;i++) {
					fprintf(stderr, "%3.2x",result_buffer[i]);
				}
				fprintf(stderr, "\n\n");
				//return -1;
			} else {
				for(i=5;i<bytes_read-5;i++) {
					if(read_buffer[i]==16) { // Skip duplicate hex 0x10
						i++;
					}
					result_buffer[resultCounter++]=read_buffer[i];
				}
			/*	fprintf(stderr, "\n\n");
				for(i=0;i<resultCounter;i++) {
					fprintf(stderr, "%3.2x",result_buffer[i]);
				}
				fprintf(stderr, "\n\n");
			*/	uint16_t calcCRC=CRC16_BUYPASS(&read_buffer[3], bytes_read-8);
				uint16_t readCRC = read_buffer[bytes_read-2] | (uint16_t)read_buffer[bytes_read-3] << 8;
				if(calcCRC != readCRC) {
					fprintf(stderr, "CRC mismatch: calculated 0x%04x, read 0x%04x\n", calcCRC, readCRC);
					for(i=0;i<bytes_read;i++)
						fprintf(stderr, "%3.2x",read_buffer[i]);
					fprintf(stderr, "\n\n");
					for(i=0;i<resultCounter;i++) {
						fprintf(stderr, "%3.2x",result_buffer[i]);
					}
					fprintf(stderr, "\n\n");
					//return -1;
					usleep(1000000);
				} else {
					fprintf(stderr, "Hz-Zeit-Ein=%02d:%02d:%02d\n", result_buffer[2], result_buffer[1], result_buffer[0]);
					fprintf(stderr, "Hz-Zeit-Aus=%02d:%02d:%02d\n", result_buffer[5], result_buffer[4], result_buffer[3]);
				}
			}
		}
//	}
	close(fd);
}
