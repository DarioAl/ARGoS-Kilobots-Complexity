include_directories(argos3/plugins/robot/kilobot/control_interface argos3/plugins/robot/kilobot/simulator)

add_library(ALF_complexity_loop_function
  MODULE
  complexity_ALF.h
  complexity_ALF.cpp
  area.h
  area.cpp
)

target_link_libraries(ALF_complexity_loop_function
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_kilolib
)
