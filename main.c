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
#include "advlist.h"
#include "advioctl.h"
#include "jsmn.h"
#include "jstree.h"
#include "jstree_read.h"
#include "serial.h"
#include "udp.h"
#include "main.h"

int __set_nonblock(int sock)
{
	int arg;

	if( (arg = fcntl(sock, F_GETFL, NULL)) < 0) {
		printf( "Error fcntl(..., F_GETFL) (%s)\n", strerror(errno));

		return -1;
	}
	arg |= O_NONBLOCK;
	if( fcntl(sock, F_SETFL, arg) < 0) {
		printf( "Error fcntl(..., F_SETFL) (%s)\n", strerror(errno));

		return -1;
	}

	return 0;
}


#define VC_BUF_RX 0
#define VC_BUF_TX 1

int buf_update(int fd, struct ring_buf *buf, int rb_id)
{
	int head;
	int tail;
	int cmd_head;
	int cmd_tail;

	switch(rb_id){
	case VC_BUF_RX:
		//printf("buf update RX\n");
		cmd_head = ADVVCOM_IOCGRXHEAD;
		cmd_tail = ADVVCOM_IOCGRXTAIL;
		break;
	case VC_BUF_TX:
		//printf("buf updat TX\n");
		cmd_head = ADVVCOM_IOCGTXHEAD;
		cmd_tail = ADVVCOM_IOCGTXTAIL;
		break;
	default:
		printf("unknown buf type\n");
		return -1;
	}

	if(ioctl(fd, cmd_head, &head) < 0){
		printf("cannot get head\n");
		return -1;
	}
	
	if(ioctl(fd, cmd_tail, &tail) < 0){
		printf("cannot get tail\n");
		return -1;
	}
	//printf("head %d tail %d\n", head, tail);
	buf->head = head;
	buf->tail = tail;

	return 0;
}

struct list_head bus_fds;


int loadConfig(char *filepath)
{
	int fd;
	int filelen;
	int ret;
	int i;
	int tokcount;
	char * filedata;
	char content[1024];
	char item[1024];

	jsmn_parser p;
	jsmntok_t *tok;
	_tree_node *tmp;
	_tree_node * rnode;

	fd = open(filepath, O_RDONLY);
	printf("%s fd = %d\n", filepath, fd);

	filelen = lseek(fd, 0, SEEK_END);
//	printf("filelen = %d\n", filelen);
	
	lseek(fd, 0, SEEK_SET);
	filedata = malloc(filelen);

	ret = read(fd, filedata, filelen);

	printf("ret = %d\n", ret);

	jsmn_init(&p);

	tokcount = 2;
	tok = malloc(sizeof(*tok) * tokcount);

	ret = 0;

	do{
		ret = jsmn_parse(&p, filedata, filelen, tok, tokcount);
		if(ret == JSMN_ERROR_NOMEM){
			tokcount = tokcount * 2;
			tok = realloc_it(tok, sizeof(*tok) * tokcount);
			if(tok == NULL){
				return -1;
			}
			continue;
		}else if(ret < 0){
			printf("failed ret = %d\n", ret);
		}
		printf("jsmn_parse %d\n", ret);
		break;
	}while(1);

//	dump(filedata, tok, p.toknext, 0);

	jstreeret result;

	result = js2tree(filedata, tok, p.toknext);

	//printf("%s(%d)\n", __func__, __LINE__);

	if(jstree_read(result.node->r, &rnode, "serials") == 1){
		struct termios2 newtio;
		char cntstr[12];
		int i;
		i = 0;
		do{
			//serial_config * _serial;
			bus_fd * _serial;
			snprintf(cntstr, sizeof(cntstr), "[%d]", i);
			printf("reading index:\"%s\"\n", cntstr);
			if(jstree_read(result.node->r, &tmp, "serials", cntstr) != 2){
				break;
			}
			printf("%s(%d)\n", __func__, __LINE__);
			_serial = malloc(sizeof(bus_fd));
			printf("%s(%d)\n", __func__, __LINE__);

			list_add(&(_serial->list), &bus_fds);
			printf("%s(%d)\n", __func__, __LINE__);

			loadSerialConfig(tmp, 
					&_serial->config.serial);

			_serial->fd = open(_serial->config.serial.path, O_RDWR);
			if(_serial->fd < 0){
				printf("cannot open %s sfd = %d\n", 
					_serial->config.serial.path, _serial->fd);
				exit(0);
			}
			__set_nonblock(_serial->fd);
			_set_termois(_serial->fd, &newtio, &_serial->config.serial);

			_serial->write = serial_write;
			_serial->read = serial_read;

			i++;

		}while(1);
	}/*else{
		printf("missing serials\n");
		exit(0);
	}*/
	if(jstree_read(result.node->r, &rnode, "udp") == 1){
		bus_fd * _udp;
		int ret;
		int on = 1;
		int off = 0;
		struct sockaddr_in6 sin6;
		_udp = malloc(sizeof(bus_fd));
		printf("udp rnode type %d\n", rnode->data.type);
		INIT_LIST_HEAD(&_udp->config.udp.targets);
		loadUdpConfig(rnode, &_udp->config.udp);
		printf("%s(%d)\n", __func__, __LINE__);

		_udp->fd = socket(AF_INET6, SOCK_DGRAM, 0);

		printf("%s(%d)\n", __func__, __LINE__);
		printf("socket fd = %d\n", _udp->fd);
		setsockopt(_udp->fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
		setsockopt(_udp->fd, IPPROTO_IP, IP_PKTINFO, &on, sizeof(on));
		setsockopt(_udp->fd, IPPROTO_IPV6, IPV6_RECVPKTINFO, &on, sizeof(on));
		setsockopt(_udp->fd, IPPROTO_IPV6, IPV6_V6ONLY, &off, sizeof(off));
		printf("%s(%d)\n", __func__, __LINE__);
		memset(&sin6, '\0', sizeof(sin6));
		sin6.sin6_family = AF_INET6;
		sin6.sin6_port = htons(atoi(_udp->config.udp.port)/*MY_UDP_PORT*/);
		sin6.sin6_addr =  in6addr_any;
		printf("%s(%d)\n", __func__, __LINE__);

		ret = bind(_udp->fd, (struct sockaddr*)&sin6, sizeof(sin6));

		printf("bind result %d port %hu\n", ret, ntohs(sin6.sin6_port)/*MY_UDP_PORT*/);
		_udp->read = udp_recvfrom;
		_udp->write = udp_sendto;
		list_add(&(_udp->list), &bus_fds);

	}

	close(fd);

	return 0;
}

int main(int argc, char **argv)
{
	fd_set rfds;
	fd_set wfds;
	struct ring_buf rx;
	struct ring_buf tx;
	unsigned int intflags;
	unsigned int lrecv;
	int fd;
	int maxfd;
	char * mbase;
	struct list_head * pos;

	INIT_LIST_HEAD(&bus_fds);


	loadConfig(argv[1]);


	fd = open(argv[2], O_RDWR);
	if(fd < 0){
		printf("cannot open file:%s\n", strerror(errno));
		return 0;
	}
	printf("mmap fd =%d\n", fd);

	mbase = (char *)mmap(0, 4096*3, 
			PROT_READ|PROT_WRITE, MAP_FILE |MAP_SHARED, 
			fd, 0);

	if(mbase == 0){
		printf("failed to mmap\n");
		return 0;
	}

	ioctl(fd, ADVVCOM_IOCGRXBEGIN, &rx.begin);
	ioctl(fd, ADVVCOM_IOCGRXSIZE, &rx.size);
	rx.mbase = &mbase[rx.begin];


	ioctl(fd, ADVVCOM_IOCGTXBEGIN, &tx.begin);
	ioctl(fd, ADVVCOM_IOCGTXSIZE, &tx.size);
	tx.mbase = &mbase[tx.begin];

	maxfd = fd;
	list_for_each(pos, &bus_fds){
		bus_fd * _s = container_of(pos, bus_fd, list);
		if(_s->fd > maxfd){
			maxfd = _s->fd;
		}
	}

	while(1){
		int ret;

		FD_ZERO(&rfds);
		FD_ZERO(&wfds);

		buf_update(fd, &rx, VC_BUF_RX);
		buf_update(fd, &tx, VC_BUF_TX);
		if(is_rb_empty(&tx)){
			list_for_each(pos, &bus_fds){
				bus_fd * _s = container_of(pos, bus_fd, list);
				FD_SET(_s->fd, &rfds);
			}
		}
		if(!is_rb_empty(&rx)){
			list_for_each(pos, &bus_fds){
				bus_fd * _s = container_of(pos, bus_fd, list);
				FD_SET(_s->fd, &wfds);
			}
		}
		if(get_rb_room(&rx) == 0){
			FD_SET(fd, &wfds);
		}
		FD_SET(fd, &rfds);
		
		intflags = ADV_INT_RX|ADV_INT_TX;
		if(ioctl(fd, ADVVCOM_IOCSINTER, &intflags) < 0){
			printf("couldn't set iocstinter\n");
			break;
		}
		

		ret = select(maxfd + 1, &rfds, &wfds, 0, 0);
		
		if(FD_ISSET(fd, &rfds)){
			//printf("ret fd read\n");
			buf_update(fd, &rx, VC_BUF_RX);
		}

		if(FD_ISSET(fd, &wfds)){
			//printf("ret fd write\n");
			buf_update(fd, &tx, VC_BUF_TX);
		}

		list_for_each(pos, &bus_fds){
			bus_fd * _s = container_of(pos, bus_fd, list);
			if(FD_ISSET(_s->fd, &rfds)){
				int ret;
				ret = _s->read(_s->fd, &tx, &_s->config);
				ioctl(fd, ADVVCOM_IOCSTXTAIL, &ret);
				buf_update(fd, &tx, VC_BUF_TX);	
			}
		}
		do{
			int wllen;
			int wlen;
			wlen = get_rb_length(&rx);
			wllen = get_rb_llength(&rx);

			if(wlen == 0)
				break;

			list_for_each(pos, &bus_fds){
					int ret;
					bus_fd * _s = container_of(pos, bus_fd, list);
					_s->write(_s->fd, &rx, wlen, wllen, &_s->config);
			}
				
			ioctl(fd, ADVVCOM_IOCSRXHEAD, &wlen);
			buf_update(fd, &tx, VC_BUF_RX);

		}while(0);
	}
	

	return 0;	
}
