#include "resource.h"

ResourceALF::ResourceALF(UInt8 type, TConfigurationNode& t_tree) : type(type), area_radius(0.025) {
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
      this->areas.reserve(100);
      // compute discretized population (it is usefull to have population between 0 and 1)
      this->discretized_population = this->population * this->k;
    }
  }
}

void ResourceALF::generate(const std::vector<AreaALF>& oth_areas, const CVector3& arena_size, uint num_of_areas) {
  CVector2 pos;
  uint tries;

  for(UInt32 i=0; i<num_of_areas; ++i) {
    for(tries = 0; tries < 500; tries++) {
      // generate first try
      pos = CVector2((arena_size.GetX()*0.9)*((Real) rand()/(RAND_MAX)-0.5),
                     (arena_size.GetY()*0.9)*((Real) rand()/(RAND_MAX)-0.5));

      bool duplicate = false;
      for(const AreaALF& oth_area : oth_areas) {
        duplicate = (duplicate || SquareDistance(oth_area.position,pos) < pow(area_radius,2));

        //found a duplicate, nothing to do anymore
        if(duplicate)
          break;
      }
      // out of the previous loop without a duplicate
      // new spot is valid
      if(!duplicate) {
        areas.push_back(AreaALF(type, i, pos, area_radius));
        break;
      }
      // too many tries and no valid spot
      if(tries == 499) {
        std::cout << "ERROR: too many tries and not an available spot for the area";
        exit(-1);
      }
    }
  }
}

bool ResourceALF::doStep(const std::vector<CVector2>& kilobot_positions, const std::vector<AreaALF>& oth_areas, const CVector3& arena_size) {
  // first update kilobots positions in the areas
  for(CVector2 kb_pos : kilobot_positions) {
    for(AreaALF& area : areas) {
        if(SquareDistance(kb_pos, area.position) < pow(area_radius,2)) {
          area.kilobots_in_area++;
          break;
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

  // update population
  population += population*eta*(1-(areas.size()/k));
  discretized_population = ceil(population*k);
  // check how many areas we have to generate now
  UInt8 diff = ceil(discretized_population - areas.size());
  generate(areas, arena_size, diff);

  return population < umin;
}
