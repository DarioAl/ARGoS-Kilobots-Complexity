/** @author Dario Albani */
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

void mtl_remove_at(node_t **head, uint16_t position) {
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
uint16_t mtl_is_message_present(node_t* head, message_t msg) {
  node_t* current = head;

  while(current != NULL) {
    // check for equality comparing the robot id
    if(current->msg.data[0] == msg.data[0]) {
      return 1;
    }
    current = current->next;
  }

  return 0;
}

/* get first non rebroadcast message   */
node_t* mtl_get_first_not_rebroadcasted(node_t* head) {
  node_t* current = head;

  while(current) {
    if(current->been_rebroadcasted) {
      current = current->next;
    } else {
      return current;
    }
  }
  return NULL;
}

void mtl_clean_list(node_t** head) {
  while((*head)->next) {
    mtl_remove_at(head, 0);
  }

  // now free head and return null
  node_t *temp = *head;
  *head = NULL;
  free(temp);
}

/* clear old messages from the list */
/* clear all messages whose time is less than time */
uint16_t mtl_clean_old(node_t** head, uint32_t time) {
  node_t* current = *head;
  node_t* temp = NULL;
  // keep track of list size while doing this scan
  uint16_t list_size = 0;

  // empty list
  if(current != NULL) {
    // at least the head is there
    list_size = 1;

    // asking to remove any other node, head is parsed last
    while(current->next) {
      // check next one
      if(current->next->time_stamp < time) {
        // store next next for linking
        temp = current->next->next;
        // free current next
        free(current->next);
        // link current and temp
        current->next = temp;
      } else {
        // increase the counter
        list_size = list_size+1;
        // update current for the loop
        current = current->next;
      }
    }

    // now at least the head is there
    // check it
    current = *head;
    if(current->time_stamp < time) {
      // set the head to current next
      *head = current->next;
      // free prev head
      free(current);
      // decrease the counter
      list_size = list_size-1;
    }
  }

  return list_size;
}

/* return list size */
uint16_t mtl_size(node_t* head) {
  uint16_t size = 0;
  node_t* current = head;

  while(current) {
    current = current->next;
    size = size+1;
  }
  return size;
}
