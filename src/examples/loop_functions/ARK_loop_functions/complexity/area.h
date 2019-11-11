/**
 * Custom definition of an area
 * With the term area we refer to the single area (e.g. the single red circle)
 * Areas disasppears when the population reaches 0
 *
 * @author Dario Albani
 * @email dario.albani@istc.cnr.it
 */

#ifndef AREA_ALF_H
#define AREA_ALF_H

#include <math.h>
#include <stdlib.h>

#include <argos3/core/simulator/simulator.h>

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

using namespace argos;

// base population for every area
#define BASE_POP 1

class AreaALF {
 private:
  // enum for resource color
  std::string enumColor[10] = {
                               "green",
                               "red",
                               "orange",
                               "yellow",
                               "black",
                               "magenta",
                               "cyan",
                               "brown",
                               "purple",
                               "blue",
  };


 public:
  /************************************/
  /* virtual environment visualization*/
  /************************************/
  UInt8 type; // resource type REDAREA or GREENAREA
  UInt8 id; // area id

  CVector2 position; /* Center of the resource */
  Real radius; /* Radius of the circle to plot */
  Real population; /* Population in the current area */
  CColor color; /* Color used to represent the area */
  UInt8 kilobots_in_area; /* keep counts of how many kbs are in the area*/
  std::string exploitation_type; /* determine the exploitation on the area by different kbs */

  /* constructor */
  AreaALF(UInt8 type, UInt8 id, const CVector2& position, Real radius, std::string exploitation_type);
  /* destructor */
  ~AreaALF(){}

  /*
   * do one simulation step during which:
   * - the population is decreased according to the number of agents
   *
   * @return true if a specific the population value reaches 0
   */
  bool doStep();

};

#endif /* AREA_ALF_H */
