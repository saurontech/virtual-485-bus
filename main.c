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

#include "advioctl.h"

struct ring_buf{
	int head;
	int tail;
	int begin;
	int size;
	char * mbase;
};

static inline int move_rb_head(struct ring_buf * buf, int length)
{
		buf->head += length;
		buf->head %= (buf->size);

	return 0;
}

static inline int move_rb_tail(struct ring_buf * buf, int length)
{
	buf->tail += length;
	buf->tail %= (buf->size);

	return 0;
}


static inline int is_rb_empty(struct ring_buf buf)
{
	return (buf.tail == buf.head);
}

static inline int get_rb_length(struct ring_buf buf)
{
	return (buf.tail >= buf.head)?(buf.tail - buf.head ):(buf.size - buf.head + buf.tail);
}

static inline int get_rb_llength(struct ring_buf buf)
{
	int a = buf.size - buf.head;
	int b = get_rb_length(buf);

	return (a < b)?a:b;
}

static inline int get_rb_room(struct ring_buf buf)
{
	return buf.size - get_rb_length(buf) - 1;
}

static inline int get_rb_lroom(struct ring_buf buf)
{
	int a = (buf.size - buf.tail);
	int b = get_rb_room(buf);
	return a < b?a:b;
}

static inline int get_rb_tail(struct ring_buf buf)
{
	return buf.tail;
}

static inline int get_rb_head(struct ring_buf buf)
{
	return buf.head;
}


int serial_open(char * path, int baud)
{
	struct termios2 newtio;
	int ret;

	int fd;
	fd = open(path, O_RDWR);
	if(fd < 0){
		printf("cannot open %s fd = %d\n", path, fd);
		return 0;
	}
	ret = ioctl(fd, TCGETS2, &newtio);
	newtio.c_iflag &= ~(ISTRIP|IUCLC|IGNCR|ICRNL|INLCR|ICANON|IXON|PARMRK|IXOFF);
	newtio.c_iflag |= (IGNBRK|IGNPAR);
	newtio.c_lflag &= ~(ECHO|ICANON|ISIG);
	newtio.c_cflag &= ~CBAUD;
	newtio.c_cflag |= BOTHER/*|CRTSCTS*/;
	newtio.c_cflag &= ~CRTSCTS;
	newtio.c_oflag &= ~(OPOST);
	newtio.c_ospeed = baud;
	newtio.c_ispeed = baud;
	ret = ioctl(fd, TCSETS2, &newtio);

	return fd;
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

int main(int argc, char **argv)
{
	fd_set rfds;
	fd_set wfds;
	struct ring_buf rx;
	struct ring_buf tx;
	unsigned int intflags;
	unsigned int lrecv;
	int bus_fd[2];
	int fd;
	int maxfd;
	char * mbase;
	int total_tx = 0;
	int total_rx = 0;

	bus_fd[0] = serial_open("/dev/ttyADV0", 921600);
	printf("fd0 = %d\n", bus_fd[0]);
	bus_fd[1] = serial_open("/dev/ttyADV1", 921600);
	printf("fd1 = %d\n", bus_fd[1]);

	fd = open("/proc/vcom/advproc10", O_RDWR);
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
	if(bus_fd[0] > maxfd)
		maxfd = bus_fd[0];
	if(bus_fd[1] > maxfd)
		maxfd = bus_fd[1];
	//timerclear(&zerotv);
	while(1){
		char dbg_msg[1024];
		int dbg_msglen;
		int ret;
		int i;

		FD_ZERO(&rfds);
		FD_ZERO(&wfds);
		dbg_msglen = 0;
		dbg_msg[0] = '\0';
//-----
		buf_update(fd, &rx, VC_BUF_RX);
		buf_update(fd, &tx, VC_BUF_TX);
		if(is_rb_empty(tx)){
			for(i = 0; i < 2; i++){
				dbg_msglen += snprintf(&dbg_msg[dbg_msglen], sizeof(dbg_msg),"sr%d,", i);
				FD_SET(bus_fd[i], &rfds);
			}	
		}
		if(!is_rb_empty(rx)){
			for(i = 0; i < 2; i++){
				dbg_msglen += snprintf(&dbg_msg[dbg_msglen], sizeof(dbg_msg),"sw%d,", i);
				FD_SET(bus_fd[i], &wfds);
			}
		}
		if(get_rb_room(rx) == 0){
			dbg_msglen += snprintf(&dbg_msg[dbg_msglen], sizeof(dbg_msg),"swfd,");
			FD_SET(fd, &wfds);
		}
		//printf("%s loop\n", dbg_msg);
		//FD_SET(fd, &wfds);
		FD_SET(fd, &rfds);
//-----
		
		intflags = ADV_INT_RX|ADV_INT_TX;
		if(ioctl(fd, ADVVCOM_IOCSINTER, &intflags) < 0){
			printf("couldn't set iocstinter\n");
			break;
		}
		/*
		buf_update(fd, &rx, VC_BUF_RX);
		buf_update(fd, &tx, VC_BUF_TX);
		if(is_rb_empty(tx)){
			for(i = 0; i < 2; i++){
				dbg_msglen += snprintf(&dbg_msg[dbg_msglen], sizeof(dbg_msg),"sr%d,", i);
				FD_SET(bus_fd[i], &rfds);
			}	
		}
		if(!is_rb_empty(rx)){
			for(i = 0; i < 2; i++){
				dbg_msglen += snprintf(&dbg_msg[dbg_msglen], sizeof(dbg_msg),"sw%d,", i);
				FD_SET(bus_fd[i], &wfds);
			}
		}
		if(get_rb_room(rx) == 0){
			dbg_msglen += snprintf(&dbg_msg[dbg_msglen], sizeof(dbg_msg),"swfd,");
			FD_SET(fd, &wfds);
		}
		//printf("%s loop\n", dbg_msg);
		//FD_SET(fd, &wfds);
		FD_SET(fd, &rfds);
		*/

		dbg_msglen += snprintf(&dbg_msg[dbg_msglen], sizeof(dbg_msg),"rfds,sel:");
		//printf("--%s--\n", dbg_msg);
		ret = select(maxfd + 1, &rfds, &wfds, 0, 0);
		//printf("rx %d tx %d\n", get_rb_length(rx), get_rb_length(tx));
		
		if(FD_ISSET(fd, &rfds)){
			//printf("ret fd read\n");

			buf_update(fd, &rx, VC_BUF_RX);
		}

		if(FD_ISSET(fd, &wfds)){
			//printf("ret fd write\n");
			buf_update(fd, &tx, VC_BUF_TX);
		}

		for(i = 0; i < 2; i++){
			//printf("check bus fd %d read\n", i);
			if(FD_ISSET(bus_fd[i], &rfds)){
				int lroom = get_rb_lroom(tx);
				ret = read(bus_fd[i], &tx.mbase[tx.tail], lroom);
				dbg_msglen += snprintf(&dbg_msg[dbg_msglen], sizeof(dbg_msg),"rd%d(%d),", i, ret);
				if(ret <= 0){
					printf("ret = %d get_rb_llength = %d\n", ret, lroom);
					continue;
				}
				total_rx += ret;
				
				//printf("bus_fd[%d] read %d\n", i, ret);
				ioctl(fd, ADVVCOM_IOCSTXTAIL, &ret);
				buf_update(fd, &tx, VC_BUF_TX);
				if(ret == lroom){
					int inqlen;
					ioctl(bus_fd[i], TIOCINQ, &inqlen);
					if(inqlen > 0){
						ret = read(bus_fd[i], &tx.mbase[tx.tail], get_rb_lroom(tx));
						if(ret <= 0){
							break;
						}
						ioctl(fd, ADVVCOM_IOCSTXTAIL, &ret);
						buf_update(fd, &tx, VC_BUF_TX);
					}
				}
			}
		}
		do{
			int wlen;
			int clean;
			int twlen;
			int truelen;
			clean = 0;
			twlen = get_rb_length(rx);
			wlen = get_rb_llength(rx);
			//if(twlen != wlen){
			//	printf("\n!!!!!!!!!!!!!!!!!!!twlen = %d wlen = %d\n", twlen, wlen);
			//}

			if(wlen == 0)
				break;

			for(i = 0; i < 2; i++){
				//printf("i = %d\n", i);
				if(FD_ISSET(bus_fd[i], &wfds)){
					int ret;
					int truelen = 0;
					ret = write(bus_fd[i], &rx.mbase[rx.head], wlen);
					dbg_msglen += snprintf(&dbg_msg[dbg_msglen], sizeof(dbg_msg),"wr%d(%d|%d),", i, ret, wlen);
					//printf("write %d(%d) to bus_fd[%d] head %d\n", ret, wlen, i, rx.head);
					if(ret < 0 ){
						printf("5555555555555555555555555555555555\n");
						continue;
					}
					truelen += ret;
					if(ret == wlen && twlen != wlen){
						ret = write(bus_fd[i], rx.mbase, twlen - ret);
						if(ret < 0){
							printf("6666666666666666666666666666666\n");
							continue;
						}
						truelen += ret;
					}
					if(truelen != twlen){
						printf("%d-------------------- %d %d\n", i, truelen, twlen);
					}
					clean = 1;
				}
			}
			if(clean){
				total_tx += wlen;
				//printf("update rxhead\n");
				//ioctl(fd, ADVVCOM_IOCSRXHEAD, &wlen);
				ioctl(fd, ADVVCOM_IOCSRXHEAD, &twlen);

				buf_update(fd, &tx, VC_BUF_RX);
			}

		}while(0);
		//printf("\rtotal_tx %d total_rx %d-----%s", total_tx, total_rx, dbg_msg);
		//fflush(stdout);
	}

	return 0;	
}
