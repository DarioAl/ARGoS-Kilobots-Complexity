#include "area.h"

AreaALF::AreaALF(UInt8 type, UInt8 id, const CVector2& position, Real radius, std::string exploitation_type) :
  type(type), id(id), position(position), radius(radius), exploitation_type(exploitation_type){

  this->kilobots_in_area = 0;
  this->population = BASE_POP;

  this->color.Set(enumColor[type]);
}

/* Decrease population according to the number of the kbs on the area */
bool AreaALF::doStep() {
  if(exploitation_type == "quadratic") {
    this->population -= pow(kilobots_in_area,2);
  } else if(exploitation_type == "cubic") {
    this->population -= pow(kilobots_in_area,3);
  } else {
    this->population -= kilobots_in_area;
  }

  // reset kbs in the area
  this->kilobots_in_area = 0;
  return this->population <= 0;
}
