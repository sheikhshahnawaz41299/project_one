#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include "list.h"

int list_add(struct list **phead, void *content) 
{
    struct list *new_node = NULL;
    
    new_node = (struct list*)malloc(sizeof(struct list));
    if(new_node == NULL) {
        printf("[%s]malloc fail.\n", __func__);
        return -1;
    }    
    new_node->prev = NULL;
    new_node->next = NULL;
    new_node->content = content;
    
    /* if head is null */
    if(*phead == NULL) {
        *phead = new_node;
        return 0;
    }
    
    /* if head is not null */
    /* if just one node */
    if((*phead)->next == NULL) {
        (*phead)->next = new_node;
        new_node->prev = *phead;
    } else {    /* if more than one node */
        new_node->prev = *phead;
        new_node->next = (*phead)->next;
        (*phead)->next->prev = new_node;
        (*phead)->next = new_node;
    }
    
    return 0;
}

int list_add_tail(struct list **head, void *content) 
{
    return 0;
}

int list_del(struct list **head, void *content) 
{
    return 0;
}

int list_free(struct list *head) 
{
    struct list *tmp = head;
    
    while(tmp) {
    
    
        tmp = tmp->next;
        
    }
    return 0;
}

#if 0
/* for test of general list tool */
int main(int argc, char **argv) {
	struct list *head;
	list_add(&head, "char");
	printf("%s\n", (char *)(head->content));
	return 0;
}	

#endif
