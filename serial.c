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
#include "serial.h"
#include "udp.h"
#include "main.h"

int loadSerialConfig(_tree_node *tmp, serial_config * config)
{
	char content[1024];
	_tree_node *rnode;

	printf("%s(%d)\n", __func__, __LINE__);
	printf("type = %d\n",  tmp->data.type);

	rnode = find_node(tmp, "path");
	get_node_string(rnode->r, content, sizeof(content));
	strncpy(config->path, content, sizeof(config->path));
	printf("found path = %s\n", config->path);

	rnode = find_node(tmp, "baudrate");
	get_node_string(rnode->r, content, sizeof(content));
	config->baudrate = atoi(content);
	printf("get baudrate = %d\n", config->baudrate);

	rnode = find_node(tmp, "databit");
	get_node_string(rnode->r, content, sizeof(content));
	config->databit = atoi(content);
	printf("get databit = %d\n", config->databit);

	rnode = find_node(tmp, "stopbit");
	get_node_string(rnode->r, content, sizeof(content));
	config->stopbit = atoi(content);
	printf("get stopbit = %d\n", config->stopbit);

	rnode = find_node(tmp, "parity");
	if(rnode){
		get_node_string(rnode->r, content, sizeof(content));
		snprintf(config->parity, sizeof(config->parity), "%s", content);
		printf("get parity = %s\n", config->parity);
	}else{
		snprintf(config->parity, sizeof(config->parity), "none");
	}

	rnode = find_node(tmp, "flowctrl");
	get_node_string(rnode->r, content, sizeof(content));

	if(strcmp(content, "rtscts") == 0){
		config->flowrep = CRTSCTS;
		printf("get rtscts\n");
	}else if(strcmp(content, "xonxoff") == 0){
		config->flowrep = IXON;
		printf("get xonxoff\n");
	}else{
		config->flowrep = 0;
		printf("no flow control\n");
	}
	
	rnode = find_node(tmp->r, "index");
	if(rnode){
		printf("index found\n");
		get_node_string(rnode->r, content, sizeof(content));
		config->index = atoi(content);
		printf("serial index = %d\n", config->index);
	}else{
		printf("index not found\n");
		config->index = 0;
	}
	
	return 0;
}

unsigned _cflag_dbit(int bitlen)
{
	switch(bitlen){
		case 5:
			//printf("databit = 5\n");
			return CS5;
		case 6:
			//printf("databit = 6\n");
			return CS6;
		case 7:
			//printf("databit = 7\n");
			return CS7;
		case 8:
		default:
			//printf("databit = 8\n");
			return CS8;
	}
}

unsigned _cflag_sbit(int bitlen)
{
	switch(bitlen){
		case 0://ADV_STOP_1
		case 1://ADV_STOP_1P5 not used
		default:
			//printf("stopbit = 1/1.5\n");
			return 0;
		case 2:
			//printf("stopbit = 2\n");
			return CSTOPB;
	}
}


int serial_read(int fd, struct ring_buf *tx, _config * config)
{
	int ret;
	int retval;
	int inqlen;
	int lroom = get_rb_lroom(tx);
	printf("read serial\n");
	ret = read(fd, &tx->mbase[tx->tail], lroom);
	if(ret <= 0){
		printf("ret = %d get_rb_llength = %d\n", ret, lroom);
		return ret;
	}

	//ioctl(fd, ADVVCOM_IOCSTXTAIL, &ret);
	//buf_update(fd, &tx, VC_BUF_TX);

	if(ret != lroom){
		return ret;
	}

	ioctl(fd, TIOCINQ, &inqlen);
	if(inqlen == 0){
		return ret;
	}
	retval = ret;
	ret = read(fd, &tx->mbase[tx->tail], get_rb_lroom(tx));
	if(ret <= 0){
		return retval;
	}
	return retval + ret;
	//ioctl(fd, ADVVCOM_IOCSTXTAIL, &ret);
	//buf_update(fd, &tx, VC_BUF_TX);
}

int serial_write(int fd, struct ring_buf * rx, int wlen, int wllen, _config * config)
{
	int ret;
	ret = write(fd, &rx->mbase[rx->head], wllen);
	if(ret !=  wllen ){
		//printf("5555555555555555555555555555555555 ret = %d wllen = %d\n", ret, wllen);
		return ret;
	}

	ret = write(fd, rx->mbase, wlen - wllen);
	if(ret != wlen - wllen){
		//printf("6666666666666666666666666666666 ret = %d wlien = %d\n", ret, wlen - wlien);
		return ret;
	}
}

int _set_termois(int fd, struct termios2 *newtio, serial_config * config)
{
	int ret;

	ret = ioctl(fd, TCGETS2, newtio);
	if(ret < 0){
		printf("ioctl : %s\n", strerror(errno));
		return -1;
	}
	printf("BEFORE setting : ospeed %d ispeed %d\n", newtio->c_ospeed, newtio->c_ispeed);
	newtio->c_iflag &= ~(ISTRIP|IUCLC|IGNCR|ICRNL|INLCR|ICANON|IXON|IXOFF|PARMRK);
	newtio->c_iflag |= (IGNBRK|IGNPAR);
	newtio->c_lflag &= ~(ECHO|ICANON|ISIG);
	newtio->c_cflag &= ~CBAUD;
	newtio->c_cflag |= BOTHER;
	newtio->c_cflag |= _cflag_dbit(config->databit);
	if(_cflag_sbit(config->stopbit)){
		newtio->c_cflag |= _cflag_sbit(config->stopbit);
	}

	if(config->flowrep & CRTSCTS){
		printf("enable RTS/CTS\n");
		newtio->c_cflag |= CRTSCTS;
	}else{
		printf("disable RTS/CTS\n");
		newtio->c_cflag &= ~CRTSCTS;
	}

	if(config->flowrep & IXON){
		newtio->c_iflag = IXON;
	}

	newtio->c_cflag &= ~(PARENB|PARODD|CMSPAR);

	if(strcmp(config->parity, "even") == 0){
		newtio->c_iflag |= INPCK;
		newtio->c_cflag |= PARENB;
		//newtio->c_cflag &= 	~(PARODD|CMSPAR);
	}else if(strcmp(config->parity, "odd") == 0){
		newtio->c_iflag |= INPCK;
		newtio->c_cflag |= (PARENB|PARODD);
		//newtio->c_cflag &= ~CMSPAR;
	}else if(strcmp(config->parity, "space") == 0){
		newtio->c_iflag |= INPCK;
		newtio->c_cflag |= (PARENB|CMSPAR);
		//newtio->c_cflag &= 	~PARODD;
	}else if(strcmp(config->parity, "mark") == 0){
		newtio->c_iflag |= INPCK;
		newtio->c_cflag |= (PARENB|PARODD|CMSPAR);
	}else{
		newtio->c_iflag &= ~INPCK;
	}

	/*
	 * Close termios 'OPOST' flag, thus when write serial data
	 * which first byte is '0a', it will not add '0d' automatically !! 
	 * Becaues of '\n' in ASCII is '0a' and '\r' in ASCII is '0d'.
	 * On usual, \n\r is linked, if you only send 0a(\n), 
	 * Kernel will think you forget \r(0d), so add \r(0d) automatically.
	*/
	newtio->c_oflag &= ~(OPOST);
	newtio->c_ospeed = config->baudrate;
	newtio->c_ispeed = config->baudrate;
	ret = ioctl(fd, TCSETS2, newtio);
	if(ret < 0){
		printf("ioctl TCSETS2: %s\n", strerror(errno));
		return -1;
	}
	printf("AFTER setting : ospeed %d ispeed %d\n", newtio->c_ospeed, newtio->c_ispeed);
	
	return 0;
}
