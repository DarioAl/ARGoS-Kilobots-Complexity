/**
 * A resource is represented by a group of areas.
 * i.e. a group of dots around the environment that shares the same type
 * A resource has a cap and a population (the number of areas) that grows according to
 * a logistic function.
 *
 * @author Dario Albani
 * @email dario.albani@istc.cnr.it
 */

#ifndef RESOURCE_ALF_H
#define RESOURCE_ALF_H

#include <math.h>
#include <stdlib.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include "area.h"

using namespace argos;

class ResourceALF {
 public:
  /************************************/
  /* virtual environment visualization*/
  /************************************/
  UInt8 type; // resource type REDAREA or GREENAREA
  Real area_radius;   // the radius of the circle
  std::vector<AreaALF> areas; /* areas of the resource */
  Real population; /* Total resource population */
  UInt8 discretized_population; /* Total number of areas from population */

  /************************************/
  /* logistic growth                  */
  /************************************/
  Real eta; /* growht factor */
  Real k; /* maximum population for single area */
  Real umin; /* population threshold @see doStep */

  /* constructor */
  inline ResourceALF() {
    type = 0;
    population = 0;
    eta = 0;
    k = 0;
    umin = 0;
    area_radius = 0;
  }

  ResourceALF(UInt8 type, TConfigurationNode& t_tree);

  /* destructor */
  ~ResourceALF(){}

  /*
   * generate areas for the resource by taking into account all other areas positions
   */
  void generate(const std::vector<AreaALF>& oth_areas, const CVector3& arena_size, uint num_of_areas);

  /*
   * do one simulation step during which:
   * - the population is increased according to a logistic function
   * - the population is exploited according to the kilobots over it
   *
   * @return true if a specific min value is reached
   */
  bool doStep(const std::vector<CVector2>& kilobot_positions, const std::vector<AreaALF>& oth_areas, const CVector3& arena_size);

};

#endif /* RESOURCE_ALF_H */
