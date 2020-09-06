#include "area.h"

AreaALF::AreaALF(UInt8 type, const CVector2& position, Real radius, Real population, Real lambda, Real eta) :
  type(type), position(position), radius(radius), lambda(lambda), eta(eta){

  this->kilobots_in_area = 0; // assume 0 initial kbs
  this->population = population; // from 0 to 1

  this->color.Set(enumColor[type]);
}

/* Decrease population according to the number of the kbs on the area */
bool AreaALF::doStep(std::string exploitation_type, Real discretization) {
  /* exploitation */
  // cubic,quadratic o linear w.r.t agents
  if(exploitation_type == "cubic") {
    population -= population*lambda*pow(kilobots_in_area, 3)*discretization;
  } else if(exploitation_type == "quadratic") {
    population -= population*lambda*pow(kilobots_in_area, 2)*discretization;
  } else {
    population -= population*lambda*kilobots_in_area*discretization;
  }

  /* growth */
  population += population*eta*(1-population)*discretization;
  return this->population == 0;
}
