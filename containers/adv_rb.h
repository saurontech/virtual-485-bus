#ifndef __ADV_RB_H
#define __ADV_RB_H

struct ring_buf{
	int head;
	int tail;
	int begin;
	int size;
	char * mbase;
};

static inline int is_rb_empty(struct ring_buf *buf)
{
	return (buf->tail == buf->head);
}

static inline int get_rb_length(struct ring_buf *buf)
{
	return (buf->tail >= buf->head)?(buf->tail - buf->head ):(buf->size - buf->head + buf->tail);
}

static inline int get_rb_llength(struct ring_buf *buf)
{
	int a = buf->size - buf->head;
	int b = get_rb_length(buf);

	return (a < b)?a:b;
}

static inline int get_rb_room(struct ring_buf *buf)
{
	return buf->size - get_rb_length(buf) - 1;
}

static inline int get_rb_lroom(struct ring_buf *buf)
{
	int a = (buf->size - buf->tail);
	int b = get_rb_room(buf);
	return a < b?a:b;
}

static inline int get_rb_tail(struct ring_buf *buf)
{
	return buf->tail;
}

static inline int get_rb_head(struct ring_buf *buf)
{
	return buf->head;
}

#ifndef SIZE_IS_POWER_OF_2
#define rb_foreach(rb, ptr)	for((ptr) = (rb)->head; \
					ptr != (rb)->tail; \
					ptr++, ptr %= (rb)->size)
#else
#define rb_foreach(rb, ptr)	for((ptr) = (rb)->head;\
					 ptr != (rb)->tail;\
					 ptr++, ptr &= ((rb)->size - 1))
#endif


static inline int move_rb_head(struct ring_buf * buf, int length)
{

	buf->head += length;
#ifndef SIZE_IS_POWER_OF_2
	buf->head %= (buf->size);
#else
	buf->head &= (buf->size - 1);
#endif

	return 0;
}

static inline int move_rb_tail(struct ring_buf * buf, int length)
{
	buf->tail += length;
#ifndef SIZE_IS_POWER_OF_2
	buf->tail %= (buf->size);
#else
	buf->tail &= (buf->size - 1);
#endif

	return 0;
}

#endif
