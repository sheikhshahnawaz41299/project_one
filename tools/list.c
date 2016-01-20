/*
 * code must checkout by:kernel/scripts/checkpatch.pl -f tools/list.c
 * and must no error,some warnings is allowed
 **/

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <errno.h>
#include "list.h"

int list_add(struct list **phead, void *content)
{
	struct list *new_node = NULL;

	new_node = (struct list *)malloc(sizeof(struct list));
	if (new_node == NULL) {
		printf("[%s]malloc fail.\n", __func__);
		return -ENOMEM;	/* no memory to alloc */
	}
	new_node->prev = NULL;
	new_node->next = NULL;
	new_node->content = content;

	/* if head is null */
	if (*phead == NULL) {
		*phead = new_node;
		return 0;
	}

	/* if head is not null */
	/* if just one node */
	if ((*phead)->next == NULL) {
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
	struct list *tmp = *head;
	struct list *new_node = NULL;

	/* get last node */
	while(tmp) {
		if(tmp->next == NULL)
			break;	
		else
			tmp = tmp->next;	
	}

	new_node = (struct list *)malloc(sizeof(struct list)); 
	if (new_node == NULL) {
		printf("[%s]malloc fail.\n", __func__);
		return -ENOMEM;
	}
	new_node->next = NULL;
	new_node->prev = tmp;
	new_node->content = content;

	tmp->next = new_node;

	return 0;
}

/* del node by it's address */
int list_del(struct list **head, struct list *freed)
{
 struct list *tmp = *head;
	struct list *next_head = NULL;

    /* if content not free */
	     if(freed->content != NULL) {
	      PRINTF_INF("node content not free,please free content first.\n");
	       return -1;
	  }

	while (tmp) {
	     if(tmp == freed) {
	         
	         /* if first node is be freed */
	         if(tmp == *head) {
	             /* if list has more than  1 node */
	             if(tmp->next !=NULL) {
	                 *head = (*head)->next;
	                 (*head)->prev = NULL;
	             } else {
	             /* list has just 1 node */
	                 *head = NULL;
	             }
	         } else {
	             freed->prev-next = freed->next?freed->next:NULL;
	             if(freed->next != NULL) {
	                 freed->next->prev = freed->prev;
	             }
	             
	         }
	         free(tmp);
	         PRINTF_INF("free node succeed.\n");
	         return 0;
	     }
 tmp = tmp->next;
		}
		PRINTF_INF("this node is not in list.\n");
		
	return 0;
}

/* free list*/
int list_free(struct list *head)
{
	struct list *tmp = head;
	struct list *next_head = NULL;

	while (tmp) {
		if (tmp->next)
			next_head = tmp->next;
		else
			next_head = NULL;
		free(tmp);
		tmp = next_head;
	}

	return 0;
}

#if 1
/* for test of general list tool */
int main(int argc, char **argv)
{
	struct list *head;

	list_add(&head, "list test.");
	list_add_tail(&head, "list add tail.");
	printf("%s\n", (char *)(head->content));
	printf("%s\n", (char *)(head->next->content));
	
	list_free(head);
	return 0;
}

#endif
