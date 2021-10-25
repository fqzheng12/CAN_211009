/**
 * @Description: 链表基础操作（参考Linux内核代码）
 * @Author: zengbing
 * @LastEditors: unkown
 * @LastEditTime: Do not edit
 * @Date: 2019-03-05 10:51:57
 */

#ifndef __lIBS_LIST_H__
#define __lIBS_LIST_H__

//#include "lib_types.h"
#include "stdint.h"
/* ------------------------------------------------------------------------ */
/*  */
/* ------------------------------------------------------------------------ */
typedef struct list_t
{
    struct list_t *next;
    struct list_t *pre;
} list_t;

/* ------------------------------------------------------------------------ */
/*  */
/* ------------------------------------------------------------------------ */
#define LIST_DEFINE(list) list_t list = {&list, &list}
#define list_for_each(list, head) for (list = (head)->next; list != (head); list = list->next)
#define list_entry(type, list_ptr, member) (type *)(((unsigned long)list_ptr) - ((unsigned long)&(((type *)0)->member)))
#define list_empty(head) (((head)->pre == (head)) && ((head)->next == (head)))
#define list_tail(head) ((head)->pre)
#define list_first(head) ((head)->next)
#define list_insert_tail(head, item) list_insert((head)->pre, item)

/* ------------------------------------------------------------------------ */
/*  */
/* ------------------------------------------------------------------------ */
extern void list_init(list_t *list);
extern void list_insert(list_t *list, list_t *item);
extern void list_delete(list_t *list);
extern int32_t list_count(list_t *head);
extern void list_insert_after (list_t *list, list_t *node_after, list_t *node_to_insert);
extern void list_insert_before(list_t *list, list_t *node_before, list_t *node_to_insert);
#endif // #ifndef __lIBS_LIST_H__
