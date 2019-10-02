#include "area.h"

AreaALF::AreaALF(UInt8 type, UInt8 id, const CVector2& position, Real radius) :
  type(type), id(id), position(position), radius(radius) {

  this->kilobots_in_area = 0;
  this->population = BASE_POP;

  this->color.Set(enumColor[type]);
}

/* Decrease population according to the number of the kbs on the area */
bool AreaALF::doStep() {
  if(kilobots_in_area > 0) {
    std::cout << "look at me I have kilobots here " << kilobots_in_area << std::endl;
  }
  // fixed decreasing
  this->population -= kilobots_in_area;
  // reset kbs in the area
  this->kilobots_in_area = 0;
  return this->population <= 0;
}
