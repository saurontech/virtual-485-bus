#ifndef _MAIN_H
#define _MAIN_H

extern struct list_head bus_fds;
struct serial_config_t;
struct udp_config_t;

typedef union _config_t{
	struct serial_config_t serial;
	struct udp_config_t udp;
} _config;

typedef struct {
	struct list_head list;
	int fd;
	int (*read)(int fd, struct ring_buf *tx, _config *config);
	int (*write)(int fd, struct ring_buf *rx, int wlen, int wllen, _config *config);
	_config config;
}bus_fd;

#endif


