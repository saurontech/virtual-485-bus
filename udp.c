#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <netdb.h>
#include <asm-generic/termbits.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <ifaddrs.h>
#include "adv_rb.h"
#include "advioctl.h"
#include "jsmn.h"
#include "jstree.h"
#include "jstree_read.h"
#include "advlist.h"
#include "udp.h"
#include "serial.h"
#include "main.h"

int loadUdpConfig(_tree_node *tmp, udp_config * config)
{
	char content[1024];
	char ipv4to6[2048];
	int i = 0;
	char array[64];
	int ret;


	_tree_node *rnode;
	rnode = 0;

	//rnode = find_node(tmp, "port");
	if(jstree_read(tmp, &rnode, "port") != 1){
		printf("missing udp port\n");
		exit(0);
	}
	//rnode = tmp->r;
	get_node_string(rnode, content, sizeof(content));
	strncpy(config->port, content, sizeof(config->port));	

	do{
		udp_target *_t;
		snprintf(array, sizeof(array), "[%d]", i);
		if(jstree_read(tmp, &rnode, "targets", array, "ip") != 3){
			break;
		}
		_t = malloc(sizeof(udp_target));
		get_node_string(rnode, content, sizeof(content));

		ret = inet_pton(AF_INET6, content, &(_t->saddr.sin6_addr));
		if(ret != 1){
			snprintf(ipv4to6, sizeof(ipv4to6), "::ffff:%s", content);
			ret = inet_pton(AF_INET6, ipv4to6, &(_t->saddr.sin6_addr));
			printf("ret = %d ipv4to6 %s\n", ret, ipv4to6);
		}


		if(jstree_read(tmp, &rnode, "targets", array, "port") != 3){
			break;
		}
		get_node_string(rnode, content, sizeof(content));
		printf("content %s\n", content);
		_t->saddr.sin6_port = htons(atoi(content));
		printf("port = %hu\n", ntohs(_t->saddr.sin6_port));
		_t->saddr.sin6_family = AF_INET6;

	
		list_add( &_t->list, &config->targets);

		printf("%s(%d)\n", __func__, __LINE__);
		i++;
	}while(1);

	printf("i = %d\n", i);

	if(i == 0){
		printf("missing udp targets\n");
		exit(0);
	}

	
	return 0;
}

int udp_recvfrom(int fd, struct ring_buf *tx, _config * config)
{
	int ret;
	int retval;
	int inqlen;
	int ptr;
	int mvlen;
	int lroom = get_rb_lroom(tx);
	int room = get_rb_room(tx);
	struct sockaddr_in6 raddr;
	socklen_t addrlen = sizeof(raddr);

	char rbuf[1500];
//	printf("%s(%d)\n", __func__, __LINE__);
	ret = recvfrom(fd, rbuf, sizeof(rbuf), 0, &raddr, &addrlen);
//	printf("recvfrom ret %d\n", ret);
	if(ret <= 0){
		printf("ret = %d get_rb_llength = %d\n", ret, lroom);
		return ret;
	}
	
	if(ret > lroom){
		memcpy(&tx->mbase[tx->tail], rbuf, lroom);
		if(lroom != room){
			int len = (room > ret)?(ret - lroom):(room -lroom);
			//printf("recvfrom movlen %d %d %d\n", ret, room, lroom);
			memcpy(tx->mbase, &rbuf[lroom], len);
		}
	}else{
		memcpy(&tx->mbase[tx->tail], rbuf, ret);
	}

	
	return ret;
	//ioctl(fd, ADVVCOM_IOCSTXTAIL, &ret);
	//buf_update(fd, &tx, VC_BUF_TX);
}


int udp_sendto(int fd, struct ring_buf * rx, int wlen, int wllen, _config * config)
{
	char sbuf[4096];
	struct list_head *pos;
	//printf("%s(%d)\n", __func__, __LINE__);

	memcpy(sbuf, &rx->mbase[rx->head], wllen);
	if(wlen != wllen){
		memcpy(&sbuf[wllen], rx->mbase, wlen - wllen);
	}
	//printf("%s(%d)\n", __func__, __LINE__);
	list_for_each(pos, &config->udp.targets){
		int ret;
		int len;
		int ptr = 0;
		int txlen = wlen;
		udp_target * _s = container_of(pos, udp_target, list);
		while(txlen > 0){
			len = (txlen > 1500)?1500:txlen;
			ret = sendto(fd, &sbuf[ptr], len, 
				MSG_NOSIGNAL, 
				&_s->saddr, sizeof(struct sockaddr_in6));
			if(ret < 0){
				printf("sendto error:%s\n", strerror(errno));
			}
			//printf("sendtofd %d ptr = %d len = %d ret%d\n", fd, ptr, len , ret);
			txlen -= len;
			ptr += len;
		}
	}
	return wlen;
}

