/**
 * Custom definition of an area and relative resources contained in it
 *
 * @author Dario Albani
 * @email dario.albani@istc.cnr.it
 */

#ifndef AREA_ALF_H
#define AREA_ALF_H

#include <math.h>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

using namespace argos;

#define SCALE_FACTOR 100 // to slow down utility update
#define REDAREA 0
#define GREENAREA 1

class AreaALF {
 private:
  UInt16 id; // area id

  /************************************/
  /* Area specific variables          */
  /************************************/
  Real eta; /* growht factor */
  Real k; /* maximum population */
  Real umin; /* population threshold @see doStep */

  /************************************/
  /* Kilobots specific variables for  */
  /* area exploitation. Stored here   */
  /* for ease of access               */
  /************************************/
  Real delta; /* collaboration value for the kilobots */
  Real xi; /* interference value for the kilobots */

 public:
  /* constructor */
  AreaALF(UInt8 id, TConfigurationNode& t_tree);
  /* destructor */
  ~AreaALF(){}

  /*
   * do one simulation step during which:
   * - the population is increased according to a logistic function
   * - the population is exploited according to the kilobots over it
   *
   * @return true if a specific min value is reached
   */
  bool doStep(UInt8 kilobotsInArea);

  /************************************/
  /* virtual environment visualization*/
  /************************************/
  CVector2 position; /* Center of the resource */
  Real radius; /* Radius of the circle to plot */
  Real population; /* Population in the current area */
  CColor color; /* Color used to represent the area */

};

#endif /* AREA_ALF_H */
