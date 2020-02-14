/** @author Dario Albani */
#include <stdlib.h>
#include <stdio.h>
#include "message_t_list.h"
#include <stdlib.h>

/* append as tail */
void mtl_push_back(node_t* head, message_t* msg, uint16_t time_stamp) {
  node_t* current = head;
  while (current->next != NULL) {
    current = current->next;
  }

  current->next = malloc(sizeof(node_t));
  current->next->msg = msg;
  current->next->time_stamp = time_stamp;
  current->next->next = NULL;
}

/* push on top as head */
void mtl_push_top(node_t** head, message_t* msg, uint16_t time_stamp) {
    node_t * new_node;
    new_node = malloc(sizeof(node_t));

    new_node->msg = msg;
    new_node->time_stamp = time_stamp;
    new_node->next = *head;
    *head = new_node;
}

/* remove head */
void mtl_remove_first(node_t** head) {
  node_t* next_node = NULL;

  if (!(*head)) {
    return;
  }

  next_node = (*head)->next;
  free((*head)->msg);
  free(*head);
  *head = next_node;
}

/* remove tail */
void mtl_remove_last(node_t* head) {
  /* if there is only one item in the list, remove it */
  if (head->next == NULL) {
    mtl_remove_first(&head);
    return;
  }

  /* iterate the list */
  node_t* current = head;
  while(current->next->next != NULL) {
    current = current->next;
  }

  /* current points to the second to last item of the list */
  free(current->next->msg);
  free(current->next);
  current->next = NULL;
}

/* remove node in position */
void mtl_remove_at(node_t** head, int n) {
  node_t* current = *head;
  node_t* temp_node = NULL;

  if(current && n == 0) {
    mtl_remove_first(head);
    return;
  }

  /* find node to remove */
  while(current && n>1) {
    current = current->next;
    n = n-1;
  }

  /* invalid n */
  if(n>1) {
    return;
  }

  /* re link the list */
  temp_node = current->next;
  if(temp_node) {
    current->next = temp_node->next;
  }
  free(temp_node);
}

/* remove first node with same crc of message_t */
void mtl_remove_node(node_t** head, node_t* to_remove) {
  node_t* current = *head;
  node_t* temp_node = NULL;

  if(current && current->msg->crc == to_remove->msg->crc) {
    mtl_remove_first(head);
    return;
  }

  /* find node to remove */
  while(current->next && current->next->msg) {
    if(current->next->msg->crc == to_remove->msg->crc) {
      temp_node = current->next->next;
      free(current->next);
      current->next = temp_node;
      return;
    }
  }
}

/* check if a node with same data is already present in the list */
/* return the position of the first node found, -1 if none is found */
unsigned mtl_is_node_present(node_t* head, node_t* to_find) {
  unsigned position = 0;
  node_t* current = head;

  while(current && current->msg) {
    if(current->msg->crc == to_find->msg->crc) {
      return position;
    }
    current = current->next;
    position++;
  }
  return -1;
}

/* check if a node with same data is already present in the list */
/* return the position of the first node found, -1 if none is found */
unsigned mtl_is_message_present(node_t* head, message_t* to_find) {
  unsigned position = 0;
  node_t* current = head;

  while(current && current->msg) {
    if(current->msg->crc == to_find->crc) {
      return position;
    }
    current = current->next;
    position++;
  }
  return -1;
}

/* get the node at position pos                   */
/* first element of the list is 0, last is size-1 */
void mtl_get_node_at(node_t* head, unsigned pos, node_t* to_retrieve) {
  node_t* current = head;
  while(current && pos>0) {
    current = current->next;
    pos--;
  }

  if(pos==0 && current) {
    // got it
    to_retrieve = current;
  }
}

/* get first non rebroadcast message   */
void mtl_get_not_rebroadcasted(node_t* head, node_t* not_rebroadcasted) {
  node_t* current = head;

  while(current) {
    if(current->been_rebroadcasted) {
      current = current->next;
    } else {
      not_rebroadcasted = current;
      return;
    }
  }
}

/* return list size */
unsigned mtl_size(node_t* head) {
  node_t* current = head;
  unsigned n = 0;

  while(current) {
    n = n+1;
    current = current->next;
  }

  return n;
}
