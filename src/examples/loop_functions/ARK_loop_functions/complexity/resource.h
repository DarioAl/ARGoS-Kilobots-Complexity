/**
 * A resource is represented by a group of areas.
 * i.e. a group of circles around the environment that shares the same type
 * A resource has a cap and a population (the number of areas) that grows according to
 * a logistic function.
 * The population of the resource is computed as the sum of the populations of all areas.
 * Area are not removed when their population reach zero but they can still grow back.
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
  /* random number generator */
  CRandom::CRNG* r_rng;

  /************************************/
  /* virtual environment visualization*/
  /************************************/
  UInt8 type; // resource type REDAREA or GREENAREA
  Real area_radius;   // the radius of the circle
  UInt64 seq_areas_id; // used to sequentially assign ids to areas
  std::vector<AreaALF> areas; /* areas of the resource */
  Real population; /* Total resource population from 0 to k*/

  /************************************/
  /* area logistic growth             */
  /************************************/
  Real eta; /* area growth factor */
  Real k; /* maximum population for single resource -- also determines the number of areas */
  /************************************/
  /* area exploitation function       */
  /************************************/
  std::string exploitation; /* single area exploitation */
  Real lambda; /* exploitation coefficient for the area */

  /* constructor */
  inline ResourceALF() {
    type = 0;
    population = 0;
    eta = 0;
    k = 0;
    area_radius = 0;
    seq_areas_id = 0;
  }

  ResourceALF(UInt8 type, TConfigurationNode& t_tree);

  /* destructor */
  ~ResourceALF(){}

  /*
   * Get population normalized between 0 and 1 instead of 0 and k
   */
  inline Real getNormalizedPopulation() {
    return population/k;
  }
  /*
   * generate areas for the resource by taking into account all other areas positions
   */
  void generate(const std::vector<AreaALF>& oth_areas, const Real arena_size);
  void generate(const std::vector<AreaALF>& oth_areas, const Real arena_size, uint num_of_areas);

  /*
   * do one simulation step during which:
   * - the population is increased according to a logistic function
   * - the population is exploited according to the kilobots over it
   *
   * @return true if a specific min value is reached
   */
  bool doStep(const std::vector<CVector2>& kilobot_positions, const std::vector<UInt8> kilobot_states, const std::vector<CColor> kilobot_colors, const std::vector<AreaALF>& oth_areas, const Real arena_radius);

};

#endif /* RESOURCE_ALF_H */
