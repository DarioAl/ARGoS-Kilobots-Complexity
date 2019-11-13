#include "resource.h"

ResourceALF::ResourceALF(UInt8 type, TConfigurationNode& t_tree) : type(type), area_radius(0.035) {
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
      GetNodeAttribute(*itNodes, "exploitation", this->exploitation);
      // areas of the resource
      this->areas.reserve(k);
    }
  }
}

void ResourceALF::generate(const std::vector<AreaALF>& oth_areas, const Real arena_radius, uint num_of_areas) {
  // concatenate oth_areas and this->areas to avoid placing a new area on
  // the same spot
  std::vector<AreaALF> all_areas;
  all_areas.insert(all_areas.end(), oth_areas.begin(), oth_areas.end());
  all_areas.insert(all_areas.end(), areas.begin(), areas.end());

  CVector2 pos;
  UInt16 tries = 0; // placement tries
  UInt16 maxTries = 9999; // max placement tries

  for(UInt32 i=0; i<num_of_areas; ++i) {
    for(tries = 0; tries <= maxTries; tries++) {
      do {
      Real rand_angle = ((Real) rand()/(RAND_MAX))*2*CRadians::PI.GetValue();
      Real rand_displacement_x = ((Real) rand()/(RAND_MAX))*(arena_radius);
      Real rand_displacement_y = ((Real) rand()/(RAND_MAX))*(arena_radius);
      pos = CVector2(rand_displacement_x*cos(rand_angle),
                     rand_displacement_y*sin(rand_angle));
      } while(SquareDistance(pos, CVector2(0,0)) > pow(2,arena_radius-area_radius));

      bool duplicate = false;
      for(const AreaALF& an_area : all_areas) {
        duplicate = SquareDistance(an_area.position, pos) <= pow(area_radius*2,2);
        if(duplicate)
          break;
      }

      if(!duplicate) {
        AreaALF new_area(type, seq_areas_id, pos, area_radius, exploitation);
        seq_areas_id++;
        // add to the current areas
        all_areas.push_back(new_area); // used to avoid duplicates
        areas.push_back(new_area);
        break;
      }

      // too many tries and no valid spot
      if(tries >= maxTries-1) {
        std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
        exit(-1);
      }
    }
  }
}

bool ResourceALF::doStep(const std::vector<CVector2>& kilobot_positions, const std::vector<UInt8> kilobot_states, const std::vector<CColor> kilobot_colors, const std::vector<AreaALF>& oth_areas, const Real arena_radius) {
  // first update kilobots positions in the areas
  // compute only for those kilobots with the right state
  for(UInt8 i=0; i<kilobot_positions.size(); ++i) {
    if(kilobot_states.at(i) == this->type &&\
       kilobot_colors.at(i) == CColor::GREEN) {
      for(AreaALF& area : areas) {
        if(SquareDistance(kilobot_positions.at(i), area.position) < pow(area_radius,2)) {
          area.kilobots_in_area++;
          break;
        }
      }
    }
  }

  /* apply exploitation */
  // now call the doStep for every area and remove from the vector if pop is 0
  std::vector<AreaALF>::iterator it = areas.begin();
  while(it != areas.end()) {
    if(it->doStep()) {
      it = areas.erase(it);
      population = areas.size();
    } else {
      // increment iterator
      ++it;
    }
  }

  /* apply growth */
  // the logistic growth up bound for the population here is omitted and is 1
  population += (population)*eta*(1-population/k);

  /* regenerate area */
  // check how many areas we have to generate now
  UInt8 diff = floor(population) - areas.size();
  if(diff>0) {
    this->generate(oth_areas, arena_radius, diff);
  }

  // umin is expressed in percentage
  return population < population*umin;
}
