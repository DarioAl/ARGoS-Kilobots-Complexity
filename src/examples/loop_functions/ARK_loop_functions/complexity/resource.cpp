#include "resource.h"

ResourceALF::ResourceALF(UInt8 type, TConfigurationNode& t_tree) : type(type), area_radius(0.03) {
  /* Get the virtual environments node from the .argos file */
  TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
  TConfigurationNodeIterator itNodes;

  // to check against xml
  UInt8 tType;

  for(itNodes=itNodes.begin(&tVirtualEnvironmentsNode); itNodes!=itNodes.end(); ++itNodes) {
    GetNodeAttribute(*itNodes, "type", tType);
    if(tType == type) {
      // initial population
      GetNodeAttribute(*itNodes, "initial_population", this->population);
      // growth parameters
      GetNodeAttribute(*itNodes, "k", this->k  );
      GetNodeAttribute(*itNodes, "eta", this->eta);
      GetNodeAttribute(*itNodes, "umin", this->umin);
      // areas of the resource
      this->areas.reserve(k);
      // compute discretized population (it is usefull to have population between 0 and 1)
      this->discretized_population = this->population * this->k;
    }
  }
}

void ResourceALF::generate(std::vector<AreaALF>& oth_areas, const Real arena_radius, uint num_of_areas) {
  CVector2 pos;
  UInt16 tries = 0; // placement tries
  UInt16 maxTries = 999; // max placement tries

  for(UInt32 i=0; i<num_of_areas; ++i) {
    for(tries = 0; tries <= maxTries; tries++) {
      Real rand_angle = ((Real) rand()/(RAND_MAX))*2*CRadians::PI.GetValue();
      Real rand_displacement_x = ((Real) rand()/(RAND_MAX))*(arena_radius-area_radius);
      Real rand_displacement_y = ((Real) rand()/(RAND_MAX))*(arena_radius-area_radius);
      pos = CVector2(rand_displacement_x*sin(rand_angle),
                     rand_displacement_y*cos(rand_angle));

      bool duplicate = false;
      for(const AreaALF& an_area : oth_areas) {
        duplicate = SquareDistance(an_area.position, pos) <= pow(area_radius*2,2);
        if(duplicate)
          break;
      }

      if(!duplicate) {
        AreaALF new_area(type, i, pos, area_radius);
        oth_areas.push_back(new_area);
        areas.push_back(new_area);
        break;
      }

      // too many tries and no valid spot
      if(tries == maxTries-1) {
        std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
      }
    }
  }
}

bool ResourceALF::doStep(const std::vector<CVector2>& kilobot_positions, const std::vector<UInt8> kilobot_states, const std::vector<AreaALF>& oth_areas, const Real arena_radius) {
  // first update kilobots positions in the areas
  // compute only for those kilobots with the right state
  for(UInt8 i=0; i<kilobot_positions.size(); ++i) {
    if(kilobot_states.at(i) == this->type) {
      for(AreaALF& area : areas) {
        if(SquareDistance(kilobot_positions.at(i), area.position) < pow(area_radius,2)) {
          area.kilobots_in_area++;
          break;
        }
      }
    }
  }

  // now call the doStep for every area and remove from the vector if pop is 0
  std::vector<AreaALF>::iterator it = areas.begin();
  while(it != areas.end()) {
    if(it->doStep()) {
      it = areas.erase(it);
    } else {
      // increment iterator
      ++it;
    }
  }

  // update population expressed between 0 and 1
  population += population*eta*(1-(areas.size()/k));
  // k is the DISCRETIZED environment carrying capacity
  discretized_population = ceil(population*k);
  // check how many areas we have to generate now
  UInt8 diff = ceil(discretized_population - areas.size());
  generate(areas, arena_radius, diff);

  return population < umin;
}
