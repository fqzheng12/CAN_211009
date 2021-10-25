/*
 * @Description: 精简版列表操作库
 * @Author: zengbing
 * @LastEditors: Please set LastEditors
 * @Date: 2019-03-05 10:04:19
 * @LastEditTime: 2019-03-05 11:02:05
 */
#include "lib_list.h"
#include "stdio.h"

void list_init(list_t *list)
{
    if (list == NULL) {
        return;
    }

    list->pre = list;
    list->next = list;
}

void list_insert(list_t *list, list_t *item)
{
    if ((list == NULL) || (item == NULL)) {
        return;
    }

    list_init(item);

    item->next = list->next;
    item->pre = list;
    ((list_t *)(list->next))->pre = item;
    list->next = item;
}
void list_insert_after(list_t *list, list_t *node_after, list_t *node_to_insert)
{
    node_to_insert->pre = node_after;
    node_to_insert->next = node_after->next;

    node_after->next->pre = node_to_insert;
    node_after->next = node_to_insert;

}
void list_insert_before(list_t *list, list_t *node_before, list_t *node_to_insert)
{
    node_to_insert->pre = node_before->pre;
    node_to_insert->next = node_before;

    node_before->pre->next = node_to_insert;
    node_before->pre = node_to_insert;

}
void list_delete(list_t *list)
{
    if ((list == NULL) || (list->pre == list) || (list->next == list) ||
        (list->pre == NULL) || (list->next == NULL)) {
        return;
    }

    ((list_t *)(list->pre))->next = list->next;
    ((list_t *)(list->next))->pre = list->pre;

    list_init(list);
}

int32_t list_count(list_t *head)
{
    int32_t count = 0;
    list_t *list;

    if (head == NULL) {
        return 0;
    }

    list_for_each(list, head) {
        count++;
    }
    return count;
}

