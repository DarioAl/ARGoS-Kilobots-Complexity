#include "complexity_ALF.h"

// Enable this part of the code to log the perceived utility estimation in the log file
#define DISTRIBUTION_ESTIMATION
#ifdef DISTRIBUTION_ESTIMATION
Real time_last_estimation = 0; // when the last estimation has been computed
UInt8 time_window_for_estimation = 20; // how long an estimation is
std::vector<std::array<UInt8,9>> estimated_by_all; // all other kbs estimations
std::vector<UInt8> hit_resources;
std::vector<UInt8> hit_empties;
std::vector<UInt8> sent_messages;
Real alpha_emas[9] = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9}; // ema alpha value
#endif

/****************************************/
/****************************************/

CComplexityALF::CComplexityALF() :
  m_unDataAcquisitionFrequency(10),
  circular_arena_radius(0.51),
  circular_arena_width(0.01),
  circular_arena_height(0.05),
  circular_arena_walls(50) {
  c_rng = CRandom::CreateRNG("argos");
}

/****************************************/
/****************************************/

CComplexityALF::~CComplexityALF() {}

/****************************************/
/****************************************/

void CComplexityALF::Init(TConfigurationNode& t_node) {
  /* Initialize ALF*/
  CALF::Init(t_node);

  /* Other initializations: Varibales, Log file opening... */
  m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

#ifdef DISTRIBUTION_ESTIMATION
  m_cOutput << "emty, resource, est.1, est.2, est.3, est.4, est.5, est.6, est.7, est.8, eat.9, ut, distance, msgs" << std::endl;

  //TODO REMOVE AFTER ESTIMATION, THIS IS USED ONLY FOR SIMULATED COMMUNICATION
  estimated_by_all.reserve(m_tKilobotEntities.size());
  hit_resources.reserve(m_tKilobotEntities.size());
  hit_empties.reserve(m_tKilobotEntities.size());
  sent_messages.reserve(m_tKilobotEntities.size());
  std::array<UInt8,9> initial_values = {125,125,125,125,125,125,125,125,125};
  for(UInt8 i=0; i<m_tKilobotEntities.size(); i++) {
    estimated_by_all.push_back(initial_values);
    hit_resources.push_back(0);
    hit_empties.push_back(0);
    sent_messages.push_back(c_rng->Uniform(CRange<UInt32>(0,10)));
  }
  sent_messages.at(0) = 0;
  #endif
}

/****************************************/
/****************************************/
void CComplexityALF::Reset() {
  m_cOutput.close();
  /* Reopen the file, erasing its contents */
  m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CComplexityALF::Destroy() {
  /* Close data file */
  m_cOutput.close();
}

/****************************************/
/****************************************/

void CComplexityALF::SetupInitialKilobotStates() {
  /* allocate space for the vectores */
  m_vecKilobotStates.resize(m_tKilobotEntities.size());
  m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
  m_vecKilobotColors.resize(m_tKilobotEntities.size());
  m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
  m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);
  for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
    /* Setup the virtual states of a kilobot(e.g. has food state)*/
    SetupInitialKilobotState(*m_tKilobotEntities[it]);
  }
}

/****************************************/
/****************************************/

void CComplexityALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
  UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
  m_vecKilobotStates[unKilobotID] = 255;
  m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
  m_vecKilobotColors[unKilobotID] = CColor::WHITE;
  m_vecLastTimeMessaged[unKilobotID] = -1000;
  /* Get a non-colliding random position within the circular arena */
  bool distant_enough = false;
  Real rand_angle, rand_displacement_x, rand_displacement_y;
  CVector3 rand_pos;

  UInt16 maxTries = 999;
  UInt16 tries = 0;

  /* Get a random orientation for the kilobot */
  CQuaternion random_rotation;
  CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
  random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);
  do {
    rand_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
    rand_displacement_x = c_rng->Uniform(CRange<Real>(0, circular_arena_radius));
    rand_displacement_y = c_rng->Uniform(CRange<Real>(0, circular_arena_radius));
    rand_pos = CVector3(rand_displacement_x*sin(rand_angle),rand_displacement_y*cos(rand_angle),0);
    distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), rand_pos, random_rotation, false);
    if(tries == maxTries-1) {
      std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
    }
  } while(!distant_enough);
}

/****************************************/
/****************************************/


void CComplexityALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){
  // generate circular arena
  std::ostringstream entity_id;
  CRadians wall_angle = CRadians::TWO_PI / circular_arena_walls;
  CVector3 wall_size(circular_arena_width, 2.0 * circular_arena_radius * Tan(CRadians::PI / circular_arena_walls), circular_arena_height);

  for (UInt32 i = 0; i < circular_arena_walls; i++) {
    entity_id.str("");
    entity_id << "wall_" << i;

    CRadians wall_rotation = wall_angle * i;
    CVector3 wall_position(circular_arena_radius * Cos(wall_rotation), circular_arena_radius * Sin(wall_rotation), 0);
    CQuaternion wall_orientation;
    wall_orientation.FromEulerAngles(wall_rotation, CRadians::ZERO, CRadians::ZERO);

    CBoxEntity *wall = new CBoxEntity(entity_id.str(), wall_position, wall_orientation, false, wall_size);
    AddEntity(*wall);
  }

  // initialize resources
  TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
  TConfigurationNodeIterator itNodes;
  // to check against xml
  UInt8 tType;
  // needed for generation
  for(itNodes=itNodes.begin(&tVirtualEnvironmentsNode); itNodes!=itNodes.end(); ++itNodes) {
    GetNodeAttribute(*itNodes, "type", tType);
    ResourceALF resource(tType, t_tree);
    resources.push_back(resource);
  }

  std::vector<CVector2> area_positions;
  for(ResourceALF& resource : resources) {
    resource.generate(areas, circular_arena_radius, resource.population);
    areas.insert(std::end(areas), std::begin(resource.areas), std::end(resource.areas));
  }
}

/****************************************/
/****************************************/

void CComplexityALF::GetExperimentVariables(TConfigurationNode& t_tree){
  /* Get the experiment variables node from the .argos file */
  TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
  /* Get the output datafile name and open it */
  GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);
  /* Get the frequency of data saving */
  GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
  /* Get the frequency of updating the environment plot */
  GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
  /* Get the time for one kilobot message */
  GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}

/***********************************************/
/***********************************************/

void CComplexityALF::UpdateVirtualEnvironments() {
  // now do step and eventually generate new areas
  for(ResourceALF& resource : resources) {
    resource.doStep(m_vecKilobotsPositions, m_vecKilobotStates, m_vecKilobotColors, areas, circular_arena_radius);
  }
}

/***********************************************/
/***********************************************/

void CComplexityALF::UpdateKilobotStates(){
  for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
    /* Update the virtual states and actuators of the kilobot*/
    UpdateKilobotState(*m_tKilobotEntities[it]);
  }
}


/****************************************/
/* Update both kilobots positions and areas in the environment */
/****************************************/

void CComplexityALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
  // current kb id
  UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);


  // update kb position
  m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);

  // update kb led color
  m_vecKilobotColors[unKilobotID] = GetKilobotLedColor(c_kilobot_entity);

  // update kb status
  m_vecKilobotStates[unKilobotID] = 255;
  // check against resources
  for(const ResourceALF& resource : resources) {
    for(const AreaALF& area : resource.areas) {
      // distance from the center of area
      if(SquareDistance(GetKilobotPosition(c_kilobot_entity), area.position) <= pow(area.radius,2)) {
        // update the state and position of the kilobot and store it for later use
        m_vecKilobotStates[unKilobotID] = resource.type;
        return;
      }
    }
  }
}

/*******************************************/
/* Actual update of the kbs sensors        */
/* Here we fill the message sent to the kb */
/*******************************************/

void CComplexityALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
  // get kb id
  UInt8 unKilobotID = (UInt8) GetKilobotId(c_kilobot_entity); // get kb id

  /* check if enough time has passed from the last message otherwise*/
  if(m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg) {
    // if the time is too short, the kilobot cannot receive a message
    return;
  } else {
    /* Prepare the inividual kilobot's message */
    m_tMessages[unKilobotID].type = 0; // using type 0 to signal ark messages
    /* Save time for next messages */
    m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;

    /* Fill up the kb message */
    m_tMessages[unKilobotID].data[0] = unKilobotID;
    m_tMessages[unKilobotID].data[1] = m_vecKilobotStates[unKilobotID]; // the resource id
    if(m_vecKilobotStates[unKilobotID] != 255) {
      m_tMessages[unKilobotID].data[2] = resources.at(m_vecKilobotStates[unKilobotID]).umin;
    }
    CVector2 kb_position = GetKilobotPosition(c_kilobot_entity);
    Real pdistance = kb_position.Length()/circular_arena_radius;
    CVector2 kb_orientation = CVector2(1,GetKilobotOrientation(c_kilobot_entity));
    Real turning_angle = M_PI-acos(kb_orientation.Normalize().DotProduct(kb_position.Normalize()));
    // 1 byte for r in percentage (0 in the center 100 at the border)
    m_tMessages[unKilobotID].data[3] = (UInt8) (pdistance*100);
    // 1 bytes for turning angle
    m_tMessages[unKilobotID].data[4] = ((UInt8) (turning_angle)*10); // hi part of the uint16

    // UNCOMMENT IF NEEDED
    // m_tMessages[unKilobotID].data[4] = (((UInt16) theta) >> 8); // hi part of the uint16
    // m_tMessages[unKilobotID].data[5] = (((UInt16) theta) & 0xff); // lo part of the uint16

#ifdef DISTRIBUTION_ESTIMATION
  if(m_fTimeInSeconds-time_last_estimation >= time_window_for_estimation) {
      // only print if kb id == 0
      if(unKilobotID == 0) {
        time_last_estimation = m_fTimeInSeconds;
        // estimate
        uint8_t kb_pop = 255;
        if(hit_resources.at(unKilobotID) < sent_messages.at(unKilobotID))
          kb_pop = 255*hit_resources.at(unKilobotID)/(sent_messages.at(unKilobotID));

        // NOTE compute for different EMA values
        for(UInt8 i=0; i<9; i++){
          estimated_by_all.at(unKilobotID)[i] = estimated_by_all.at(unKilobotID)[i]*alpha_emas[i] + (1-alpha_emas[i])*kb_pop;
        }

        float actual_ut = resources.at(0).population;
        m_cOutput << hit_empties.at(unKilobotID) << ", " << hit_resources.at(unKilobotID) << ", ";
        for(UInt8 i=0; i<9; i++){
          m_cOutput << estimated_by_all.at(unKilobotID)[i] << ", ";
        }
        // compute distance from the center in average for all kbs
        float all_distance = 0;
        for(CVector2 kb_vec : m_vecKilobotsPositions) {
          all_distance += SquareDistance(kb_vec, CVector2(0,0));
        }
        m_cOutput << actual_ut << ", " << sqrt(all_distance/m_vecKilobotsPositions.size()) << ", " << sent_messages.at(unKilobotID) << std::endl;
      }

      // reset counters
      hit_resources.at(unKilobotID) = 0;
      hit_empties.at(unKilobotID) = 0;
      sent_messages.at(unKilobotID) = 0;
    } else {
      sent_messages.at(unKilobotID) += 1;
      if(m_vecKilobotStates[unKilobotID] != 255) {
        hit_resources.at(unKilobotID) += 1;
      } else {
        hit_empties.at(unKilobotID) += 1;
      }
    }

    // simulating communication
    if(unKilobotID == 0) {
      UInt8 receivedmsgs = 0;
      for(UInt8 kb_index=1; kb_index < m_vecKilobotsPositions.size(); kb_index++) {
        if(sent_messages.at(kb_index) >= 11) {
          receivedmsgs++;
          // estimate from oth point of view
          uint8_t oth_kb_pop = 255;
          if(hit_resources.at(kb_index) < sent_messages.at(kb_index))
            oth_kb_pop = 255*hit_resources.at(kb_index)/(sent_messages.at(kb_index));

          if(SquareDistance(m_vecKilobotsPositions.at(unKilobotID), m_vecKilobotsPositions.at(kb_index)) < 0.2){
            for(uint8_t i=0; i<9; i++){
              estimated_by_all.at(unKilobotID)[i] = estimated_by_all.at(unKilobotID)[i]*alpha_emas[i] + (1-alpha_emas[i])*oth_kb_pop;
            }
          }
          hit_resources.at(kb_index) = 0;
          hit_empties.at(kb_index) = 0;
          sent_messages.at(kb_index) = 0;
        }
      }
    }
#endif

    /* Sending the message using the overhead controller */
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
  }
}

/****************************************/
/****************************************/

CColor CComplexityALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
  // base background color
  CColor cColor=CColor::WHITE;

  // check if resource
  for(const ResourceALF resource : resources) {
    for(const AreaALF& area : resource.areas) {
      if(SquareDistance(vec_position_on_plane,area.position) < pow(area.radius,2)){
        cColor=area.color;
        return cColor;
      }
    }
  }
  return cColor;
}

REGISTER_LOOP_FUNCTIONS(CComplexityALF, "ALF_complexity_loop_function")
