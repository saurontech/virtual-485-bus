#ifndef _UDP_H
#define _UDP_H

typedef struct{
	struct list_head list;
	struct sockaddr_in6 saddr;
}udp_target;

typedef struct udp_config_t{
	struct list_head targets;
	char port[64];
} udp_config;

union _config_t;


int loadUdpConfig(_tree_node *tmp, udp_config * config);
int udp_recvfrom(int fd, struct ring_buf *tx, union _config_t * config);
int udp_sendto(int fd, struct ring_buf * rx, int wlen, int wllen, union _config_t * config);

#endif
