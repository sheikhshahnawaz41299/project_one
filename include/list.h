#ifndef _LLIST_H_
#define _LLIST_H_

struct list {
    struct list *prev;
    struct list *next;
    void *content;
};

#endif