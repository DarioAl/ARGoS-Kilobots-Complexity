#include "area.h"

AreaALF::AreaALF(UInt8 type, UInt8 id, const CVector2& position, Real radius, TConfigurationNode& t_tree) {
  /* Get the virtual environments node from the .argos file */
  TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
  TConfigurationNodeIterator itNodes;
  UInt16 nodetype;

  for(itNodes=itNodes.begin(&tVirtualEnvironmentsNode); itNodes!=itNodes.end(); ++itNodes) {
    GetNodeAttribute(*itNodes, "id", nodetype);
    if(nodetype == type) {
      this->type = type;
      this->radius = radius;

      // random position
      this->position = position;

      // area growth parameters
      GetNodeAttribute(*itNodes, "k", this->k  );
      GetNodeAttribute(*itNodes, "eta", this->eta);
      GetNodeAttribute(*itNodes, "umin", this->umin);

      // area size parameters
      GetNodeAttribute(*itNodes, "population", this->population);
      GetNodeAttribute(*itNodes, "color", this->color);
      // set area color according to the current population
      if(REDAREA == type) {
        this->color.SetRed(255);
        this->color.SetGreen((1-this->population)*255);
      } else if(GREENAREA == type) {
        this->color.SetGreen(255);
        this->color.SetRed((1-this->population)*255);
      }
      this->color.SetBlue((1-this->population)*255);
      // area agent exploitation parameters
      // GetNodeAttribute(*itNodes, "color", this->color);
      // GetNodeAttribute(*itNodes, "color", this->color);
      break;
    }
  }
}

bool AreaALF::doStep(UInt8 kilobotsInArea) {
  // std::cout << "OMG kilobots in " << id << " are " << kilobotsInArea << std::endl;

  //logistic growth
  population += (eta*population*((1-population)/k))/SCALE_FACTOR;

  // reduce the resources according to the number of kilobots and a linear function
  // this->population += (this->population*(this->delta*kilobotsInArea + this->xi*kilobotsInArea*kilobotsInArea))/SCALE_FACTOR;

  // update color according to the population
  if(REDAREA == type) {
    this->color.SetGreen((1-this->population)*255);
  } else if(GREENAREA == type) {
    this->color.SetRed((1-this->population)*255);
  }
  this->color.SetBlue((1-this->population)*255);

  return this->population < this->umin;
}
