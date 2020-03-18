/** @author Dario Albani */
#include <stdlib.h>
#include <stdio.h>
#include "message_t_list.h"

/* append as tail */
void mtl_push_back(node_t *head, node_t* new_node) {
  node_t* current = head;

  while(current->next != NULL) {
    current = current->next;
  }

  current->next = malloc(sizeof(node_t));
  current->next = new_node;
  current->next->next = NULL;
}

/* 0 first element of the list                      */
void mtl_remove_at(node_t **head, u_int16_t position) {
  node_t* current = *head;
  node_t* temp = NULL;

  // empty list
  if(current == NULL) {
    return;
  }

  // asking to remove the head
  if(position == 0) {
    current = current->next;
    free(*head);
    *head = current;
    return;
  }

  // asking to remove any other node
  while(current->next && position>1) {
    current = current->next;
    position = position-1;
  }

  if(position > 1)
    return;
  else {
    temp = current->next;
    if(temp){
      current->next = temp->next;
    }
    free(temp);
  }
}

/* check if a node with same data is already present in the list */
/* return the position of the first node found                   */
/* 0 first element of the list                      */
int mtl_is_message_present(node_t* head, message_t msg) {
  u_int16_t position = 0;
  node_t* current = head;

  while(current != NULL) {
    // check for equality comparing the crc
    fflush(stdout);
    if(current->msg.data[0] == msg.data[0]) {
      return position;
    }
    current = current->next;
    position = position+1;
  }

  return -1;
}

/* get first non rebroadcast message   */
/* 0 first element                                  */
int mtl_get_first_not_rebroadcasted(node_t* head, node_t** not_rebroadcasted) {
  node_t* current = head;
  u_int16_t position = 0;

  while(current != NULL) {
    if(current->been_rebroadcasted != 0) {
      current = current->next;
      position++;
    } else {
      *not_rebroadcasted = current;
      return position;
    }
  }
  return -1;
}

/* return list size */
u_int16_t mtl_size(node_t* head) {
  u_int16_t size = 0;
  node_t* current = head;

  while(current) {
    current = current->next;
    size = size+1;
  }
  return size;
}
