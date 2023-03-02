/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ADVLIST_SORT_H
#define _ADVLIST_SORT_H

//#include <linux/types.h>

struct list_head;

void list_sort(void *priv, struct list_head *head,
	       int (*cmp)(void *priv, struct list_head *a,
			  struct list_head *b));
#endif
