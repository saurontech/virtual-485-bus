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
	char path[1024];
	int index;
	int baudrate; 
	int databit;
	int stopbit;
	char parity[1024];
	unsigned int flowrep;
	int fd;
} serial_config;

typedef struct{
	struct list_head list;
	struct sockaddr_in6 saddr;
}udp_target;

typedef struct {
	struct list_head targets;
	char port[64];
} udp_config;

typedef union {
	serial_config serial;
	udp_config udp;
} _config;

typedef struct {
	struct list_head list;
	int fd;
	int (*read)(int fd, struct ring_buf *tx, _config *config);
	int (*write)(int fd, struct ring_buf *rx, int wlen, int wllen, _config *config);
	_config config;
}bus_fd;

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

int serial_read(int fd, struct ring_buf *tx, _config * config)
{
	int ret;
	int retval;
	int inqlen;
	int lroom = get_rb_lroom(*tx);
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
	ret = read(fd, &tx->mbase[tx->tail], get_rb_lroom(*tx));
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

int udp_recvfrom(int fd, struct ring_buf *tx, _config * config)
{
	int ret;
	int retval;
	int inqlen;
	int ptr;
	int mvlen;
	int lroom = get_rb_lroom(*tx);
	int room = get_rb_room(*tx);
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
		if(is_rb_empty(tx)){
			list_for_each(pos, &bus_fds){
				bus_fd * _s = container_of(pos, bus_fd, list);
				FD_SET(_s->fd, &rfds);
			}
		}
		if(!is_rb_empty(rx)){
			list_for_each(pos, &bus_fds){
				bus_fd * _s = container_of(pos, bus_fd, list);
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
			wlen = get_rb_length(rx);
			wllen = get_rb_llength(rx);

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
