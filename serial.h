#ifndef _SERIAL_H
#define _SERIAL_H
typedef struct serial_config_t{
	char path[1024];
	int index;
	int baudrate; 
	int databit;
	int stopbit;
	char parity[1024];
	unsigned int flowrep;
	int fd;
} serial_config;

union _config_t;

int loadSerialConfig(_tree_node *tmp, serial_config * config);
int serial_read(int fd, struct ring_buf *tx, union _config_t * config);
int serial_write(int fd, struct ring_buf * rx, int wlen, int wllen, union _config_t * config);
unsigned _cflag_sbit(int bitlen);
unsigned _cflag_dbit(int bitlen);
int _set_termois(int fd, struct termios2 *newtio, serial_config * config);
#endif
