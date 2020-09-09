/**
 * Custom definition of an area
 * With the term area we refer to the single circle that together with the other
 * circles sum up to form the resource.
 * An Area disasppears when the population reaches 0.
 * Area's population decrease according to an ODE proportional to the number of agents in it:
 * d u(t)/ d t = u(t)*lambda*n^x
 * where u(t) is the utility, lambda an exploitation parameter, n the number of agents and
 * x a pow coefficient
 * Area's population increase according to a logistic function (similar to the resource)
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

class AreaALF {
 public:
  // enum for resource color
  std::string enumColor[3] = {
                              "red",
                              "green",
                              "blue",
  };

  /************************************/
  /* virtual environment visualization*/
  /************************************/
  UInt8 type; // resource type

  CVector2 position; /* Center of the resource */
  Real radius; /* Radius of the circle to plot */
  CColor color; /* Color used to represent the area */


  Real lambda; /* exploitation coefficient */
  UInt8 kilobots_in_area; /* keep counts of how many kbs are in the area*/
  std::string exploitation_type; /* determine the exploitation on the area by different kbs */
  Real population; /* Population in the current area */
  Real eta; /* Logistic growth regenerative parameter */
  Real agentsExploitation; /* Store here last step exploitatino value */

  /* constructor */
  AreaALF(UInt8 type, const CVector2& position, Real radius, Real population, Real lambda, Real eta);
  /* destructor */
  ~AreaALF(){}

  /*
   * do one simulation step during which:
   * - the population is decreased according to the number of agents
   *
   * @return true if a specific the population value reaches 0
   */
  bool doStep(std::string exploitation_type, Real discretization);

};

#endif /* AREA_ALF_H */
