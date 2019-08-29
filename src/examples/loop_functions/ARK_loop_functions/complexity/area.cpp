#include "area.h"

AreaALF::AreaALF(UInt8 id, TConfigurationNode& t_tree) {
  /* Get the virtual environments node from the .argos file */
  TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
  TConfigurationNodeIterator itNodes;
  UInt8 nodeid;
  for(itNodes=itNodes.begin(&tVirtualEnvironmentsNode); itNodes!=itNodes.end(); ++itNodes) {
    GetNodeAttribute(*itNodes, "id", nodeid);
    if(nodeid == id) {
      GetNodeAttribute(*itNodes, "k", this->k  );
      GetNodeAttribute(*itNodes, "eta", this->eta);
      GetNodeAttribute(*itNodes, "umin", this->umin);
      GetNodeAttribute(*itNodes, "position", this->position);
      GetNodeAttribute(*itNodes, "population", this->population);
      GetNodeAttribute(*itNodes, "color", this->color);

      // set area transparency according to the current population
      this->color.SetAlpha(this->population);
      break;
    }
  }
}

bool AreaALF::doStep(UInt8 kilobotsInArea) {
  //logistic growth
  population += (eta*population*((1-population)/k));

  // reduce the resources according to the number of kilobots and a linear function
  this->population *= this->delta*kilobotsInArea + this->xi*std::pow(kilobotsInArea, 2);

  // update color according to the population
  this->color.SetAlpha(this->population*255);

  return this->population < this->umin;
}
