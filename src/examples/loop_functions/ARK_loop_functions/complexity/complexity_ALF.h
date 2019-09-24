/**
 * @file <complexity_ALF.h>
 *
 * @brief TODO
 *
 * @author Dario Albani
 * @email <dario.albani@istc.cnr.it>
 */

#ifndef COMPLEXITY_ALF_H
#define COMPLEXITY_ALF_H

namespace argos {
class CSpace;
class CFloorEntity;
class CSimulator;
}

#include <math.h>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

//resources area
#include "area.h"

#include <array>

using namespace argos;

class CComplexityALF : public CALF {

public:
  /** < constructor */
  CComplexityALF();
  /** < destructor */
  virtual ~CComplexityALF();

  /** < initialization */
  virtual void Init(TConfigurationNode& t_tree);

  /** < reset the simulation to the initial state */
  virtual void Reset();

  /** < close and destroy befor exiting the simulation */
  virtual void Destroy();

  /** < setup the initial state of the Kilobots in the space */
  void SetupInitialKilobotStates();

  /** < setup the initial state of the kilobot pc_kilobot_entity */
  void SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity);

  /** Experiment configuration methods (From .argos files) */
  /** Setup virtual environment */
  void SetupVirtualEnvironments(TConfigurationNode& t_tree);

  /** Get experiment variables */
  void GetExperimentVariables(TConfigurationNode& t_tree);

  /** Triggers the update of the kilobts and is also used to update the areas accordingly */
  void UpdateKilobotStates();

  /** Virtual environment visualization updating */
  /** Get the message to send to a Kilobot according to its position */
  void UpdateKilobotState(CKilobotEntity& c_kilobot_entity);

  /** Get the message to send to a Kilobot according to its position */
  void UpdateVirtualSensor(CKilobotEntity& c_kilobot_entity);

  /** Used to plot the Virtual environment on the floor */
  virtual CColor GetFloorColor(const CVector2& vec_position_on_plane);

private:
  /************************************/
  /*  virtual Environment variables   */
  /************************************/

  UInt8 areas_per_type; // number of subareas per resource type
  std::vector<AreaALF*> resource_a;   // resource a
  std::vector<AreaALF*>resource_b;   // resource b
  UInt16 kilobotsInA;    // number of kbs in resource a
  UInt16 kilobotsInB;    // number of kbs in resource b
  UInt16 kilobotsInNone; // number of kbs in no area

  /************************************/
  /* virtual environment struct*/
  /************************************/

  typedef enum {
                OUTSIDE_AREA=0,
                INSIDE_AREA_A=1,
                INSIDE_AREA_B=2,
  } SRobotState;

  /* used to store the SRobotState of each kilobot */
  std::vector<SRobotState> m_vecKilobotStates;

  /* used to store the last message sent to each kilobot */
  std::vector<Real> m_vecLastTimeMessaged;
  Real m_fMinTimeBetweenTwoMsg;

  /************************************/
  /*       Experiment variables       */
  /************************************/

  /* output file for data acquizition */
  std::ofstream m_cOutput;

  /* output file name*/
  std::string m_strOutputFileName;

  /* data acquisition frequency in ticks */
  UInt16 m_unDataAcquisitionFrequency;
};

#endif
