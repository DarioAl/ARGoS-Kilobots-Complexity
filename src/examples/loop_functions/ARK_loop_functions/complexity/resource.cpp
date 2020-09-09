#include "resource.h"

ResourceALF::ResourceALF(UInt8 type, TConfigurationNode& t_tree) : type(type) {
  /* Get the virtual environments node from the .argos file */
  TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
  TConfigurationNodeIterator itNodes;

  // to check against xml
  UInt8 tType;

  for(itNodes=itNodes.begin(&tVirtualEnvironmentsNode); itNodes!=itNodes.end(); ++itNodes) {
    GetNodeAttribute(*itNodes, "type", tType);
    if(tType == type) {
      /* Read the parameters from the argos file */
      // there are k areas (k is the same as the logistic growth)
      GetNodeAttribute(*itNodes, "k", this->k);
      // initial population for every single area, overall pop is initial_population*k
      GetNodeAttribute(*itNodes, "initial_population", this->population);
      // area radius
      GetNodeAttribute(*itNodes, "radius", this->area_radius);
      // area growth rate (logistic growth)
      GetNodeAttribute(*itNodes, "eta", this->eta);
      // exploitation parameter
      GetNodeAttribute(*itNodes, "lambda", this->lambda);
      // type of exploitation
      GetNodeAttribute(*itNodes, "exploitation", this->exploitation);
      // euler step discretization
      GetNodeAttribute(*itNodes, "discretization", this->discretization);

      // areas of the resource
      this->areas.reserve(k);
    }
    r_rng = CRandom::CreateRNG("argos");
  }
}

void ResourceALF::generate(const Real arena_radius) {
  CVector2 pos;
  UInt16 tries = 0; // placement tries
  UInt16 maxTries = 9999; // max placement tries

  // temp variable to initialize the resources
  Real initial_population = this->population;
  // reset population value (this was a placeholder until now)
  this->population = 0;

  for(UInt32 i=0; i < this->k; ++i) {
    for(tries = 0; tries <= maxTries; tries++) {
      do {
        Real rand_angle = r_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
        Real rand_distance = r_rng->Uniform(CRange<Real>(0,arena_radius-area_radius/2));

        pos = CVector2(rand_distance*cos(rand_angle),
                       rand_distance*sin(rand_angle));
      } while(SquareDistance(pos, CVector2(0,0)) > pow(arena_radius-area_radius, 2));

      bool duplicate = false;
      for(const AreaALF& an_area : areas) {
        duplicate = SquareDistance(an_area.position, pos) <= pow(area_radius*2,2);
        if(duplicate)
          break;
      }

      if(!duplicate) {
        AreaALF new_area(type, pos, area_radius, initial_population, lambda, eta);
        // add to the areas
        areas.push_back(new_area);
        // add area utility to this->population
        this->population += new_area.population;
        // break the cycle and keep going with next area
        break;
      }

      // too many tries and no valid spot
      if(tries >= maxTries-1) {
        std::cerr << "\n \n ERROR: too many tries and not an available spot for the area" << std::endl;
        exit(-1);
      }
    }
  }
}

bool ResourceALF::doStep(const std::vector<CVector2>& kilobot_positions, const std::vector<m_kilobotstate> kilobot_states, const std::vector<CColor> kilobot_colors, Real ticks_per_second) {
  // first update kilobots positions in the areas
  // compute only for those kilobots with the right state
  // i.e. with the correct led color on
  for(UInt8 i=0; i<kilobot_positions.size(); ++i) {
    if(kilobot_states.at(i).resources[this->type] &&
       kilobot_colors.at(i) == areas.at(0).color) {
      for(AreaALF& area : areas) {
        if(SquareDistance(kilobot_positions.at(i), area.position) < pow(area_radius,2)) {
          area.kilobots_in_area++;
          break;
        }
      }
    }
  }

  /* apply exploitation and growth in all areas */
  // reset population
  population = 0;
  // reset lastStepExploitation
  lastStepExploitation = 0;
  // call the doStep of every area and get the population
  for(AreaALF& area : areas) {
    // run it to match the wanted discretization
    for(UInt32 i=0; i<discretization/ticks_per_second; i++){
      // do one step and sum up to resource pop
      area.doStep(exploitation, 1.f/discretization);
    }

    if(area.population==0) {
      std::cerr << "\n \n WARNING: an area has reached zero population" << std::endl;
    }
    // update the population
    population += area.population;
    // update exploitation of this step
    lastStepExploitation += area.agentsExploitation;
    // reset the kilobots in the area
    area.kilobots_in_area = 0;
  }
  // update total exploitation
  totalExploitation += lastStepExploitation;

  return population == 0;
}
