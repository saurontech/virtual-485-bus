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
#include "jsmn.h"
#include "jstree.h"
#include "jstree_read.h"

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


#ifndef _CONTAINER_OF_H
#define _CONTAINER_OF_H
#if __linux__   //  or #if __GNUC__
    #if __LP64__
        #define ENVIRONMENT64
    #else
        #define ENVIRONMENT32
    #endif
#else
    #if _WIN32
        #define ENVIRONMENT32
    #else
        #define ENVIRONMENT64
    #endif
#endif // __linux__

#ifdef ENVIRONMENT64
    #define PTR_OFFSET (unsigned long long int)
#else
    #define PTR_OFFSET (unsigned long int)
#endif // ENVIRONMENT64

#define adv_offsetof(TYPE, MEMBER) (PTR_OFFSET &((TYPE *)0)->MEMBER)

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
               (type *)( (char *)__mptr - adv_offsetof(type,member) );})

#define list_entry(ptr,type,member)     \
    container_of(ptr, type, member)

#endif

struct list_head {
	struct list_head *next, *prev;
};

#define LIST_HEAD_INIT(name) { &(name), &(name) }
#define LIST_HEAD(name) \
struct list_head name = LIST_HEAD_INIT(name)

static inline void INIT_LIST_HEAD(struct list_head *head)
{
        head->next = head;
        head->prev = head;
}

static inline int list_empty(const struct list_head *head)
{
        return head->next == head;
}

static void __list_add(struct list_head *new_lst,
                              struct list_head *prev,
                              struct list_head *next)
{
        next->prev = new_lst;
        new_lst->next = next;
        new_lst->prev = prev;
        prev->next = new_lst;
}

static inline void list_add(struct list_head *new_lst, 
              struct list_head *head)
{
	__list_add(new_lst, head, head->next);
}


static inline void list_add_tail(struct list_head *new_lst, struct list_head *head)
{
        __list_add(new_lst, head->prev, head);
}

static  void __list_del(struct list_head * prev, 
                              struct list_head * next  )
{
        next->prev = prev;
        prev->next = next;
}

#define LIST_POISON1 0
#define LIST_POISON2 0

static inline void list_del(struct list_head * entry)
{
        __list_del(entry->prev,entry->next);
        entry->next = LIST_POISON1;
        entry->prev = LIST_POISON2;
}



#define list_for_each(pos, head) \
        for (pos = (head)->next; pos != (head); pos = pos->next)

#define list_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); \
		pos = n, n = pos->next)

struct list_head bus_fds;


static unsigned _cflag_dbit(int bitlen)
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

static unsigned _cflag_sbit(int bitlen)
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


typedef struct{
	struct list_head list;
	char path[1024];
	int index;
	int baudrate; 
	int databit;
	int stopbit;
	char parity[1024];
	unsigned int flowrep;
	int fd;
} serial_config;

static inline int _set_termois(int fd, struct termios2 *newtio, serial_config * config)
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
		char cntstr[12];
		int i;
		i = 0;
		do{
			serial_config * _serial;
			snprintf(cntstr, sizeof(cntstr), "[%d]", i);
			printf("reading index:\"%s\"\n", cntstr);
			if(jstree_read(result.node->r, &tmp, "serials", cntstr) != 2){
				break;
			}
			printf("%s(%d)\n", __func__, __LINE__);
			_serial = malloc(sizeof(serial_config));
			printf("%s(%d)\n", __func__, __LINE__);

			list_add(&(_serial->list), &bus_fds);
			printf("%s(%d)\n", __func__, __LINE__);

			loadSerialConfig(tmp, 
					_serial);

			i++;

		}while(1);
	}else{
		printf("missing serials\n");
		exit(0);
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


	list_for_each(pos, &bus_fds){
		int sfd;
		struct termios2 newtio;

		serial_config * _s = container_of(pos, serial_config, list);

		sfd = open(_s->path, O_RDWR);
		if(sfd < 0){
			printf("cannot open %s sfd = %d\n", _s->path, sfd);
			return 0;
		}
		_s->fd = sfd;
		__set_nonblock(sfd);

		_set_termois(sfd, &newtio, _s);
		
	}

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
		serial_config * _s = container_of(pos, serial_config, list);
		if(_s->fd > maxfd){
			maxfd = _s->fd;
		}
	}

	while(1){
		int ret;
		int i;

		FD_ZERO(&rfds);
		FD_ZERO(&wfds);

		buf_update(fd, &rx, VC_BUF_RX);
		buf_update(fd, &tx, VC_BUF_TX);
		if(is_rb_empty(tx)){
			list_for_each(pos, &bus_fds){
				serial_config * _s = container_of(pos, serial_config, list);
				FD_SET(_s->fd, &rfds);
			}
		}
		if(!is_rb_empty(rx)){
			list_for_each(pos, &bus_fds){
				serial_config * _s = container_of(pos, serial_config, list);
				FD_SET(_s->fd, &wfds);
			}
		}
		if(get_rb_room(rx) == 0){
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
			serial_config * _s = container_of(pos, serial_config, list);
			//printf("check bus fd %d read\n", i);
			if(FD_ISSET(_s->fd, &rfds)){
				int lroom = get_rb_lroom(tx);
				ret = read(_s->fd, &tx.mbase[tx.tail], lroom);
				if(ret <= 0){
					printf("ret = %d get_rb_llength = %d\n", ret, lroom);
					continue;
				}
				
				ioctl(fd, ADVVCOM_IOCSTXTAIL, &ret);
				buf_update(fd, &tx, VC_BUF_TX);

				if(ret != lroom){
					continue;
				}
				int inqlen;
				ioctl(_s->fd, TIOCINQ, &inqlen);
				if(inqlen == 0){
					continue;
				}
				ret = read(_s->fd, &tx.mbase[tx.tail], get_rb_lroom(tx));
				if(ret <= 0){
					continue;
				}
				ioctl(fd, ADVVCOM_IOCSTXTAIL, &ret);
				buf_update(fd, &tx, VC_BUF_TX);
				
			}
		}
		do{
			int wllen;
			int wlen;
			wlen = get_rb_length(rx);
			wllen = get_rb_llength(rx);

			if(wlen == 0)
				break;

			list_for_each(pos, &bus_fds){
					int ret;
					serial_config * _s = container_of(pos, serial_config, list);
					ret = write(_s->fd, &rx.mbase[rx.head], wllen);
					if(ret !=  wllen ){
						//printf("5555555555555555555555555555555555 ret = %d wllen = %d\n", ret, wllen);
						continue;
					}

					ret = write(_s->fd, rx.mbase, wlen - wllen);
					if(ret != wlen - wllen){
						//printf("6666666666666666666666666666666 ret = %d wlien = %d\n", ret, wlen - wlien);
						continue;
					}
			}
				
			ioctl(fd, ADVVCOM_IOCSRXHEAD, &wlen);
			buf_update(fd, &tx, VC_BUF_RX);

		}while(0);
	}
	

	return 0;	
}
