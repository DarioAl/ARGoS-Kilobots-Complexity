#ifndef COMPLEXITY_H
#define COMPLEXITY_H

#include <stdbool.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

  /* struct for kilobot state */
  typedef struct { bool resources[3]; } m_kilobotstate;

#ifdef ARGOS_simulator_BUILD

#define DEBUG_KILOBOT

  /* struct for kilobot debug within ARGoS */
  typedef struct {
    uint8_t ema_resource0;
    uint8_t ema_resource1;
    uint8_t ema_resource2;
    uint8_t decision;
    uint8_t num_messages;
  } debug_info_t;

#endif /* ARGOS_simulator_BUILD */

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#endif /* COMPLEXITY_H */
