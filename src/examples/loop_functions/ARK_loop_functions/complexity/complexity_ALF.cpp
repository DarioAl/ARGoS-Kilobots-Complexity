#include "complexity_ALF.h"
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>

// Enable this part of the code to log the perceived utility estimation in the log file
#define DISTRIBUTION_ESTIMATION
#ifdef DISTRIBUTION_ESTIMATION
UInt8 debug_counter;
UInt8 overall_r0_estimate;
UInt8 overall_empty_hits;
UInt8 overall_r0_hits;
Real overall_distance;
UInt8 overall_num_messages;
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
  m_tKBs.clear();
  /* Get the map of all kilobots from the space */
  CSpace::TMapPerType& tKBMap = GetSpace().GetEntitiesByType("kilobot");
  /* Go through them */
  for(CSpace::TMapPerType::iterator it = tKBMap.begin();
      it != tKBMap.end();
      ++it) {
    /* Create a pointer to the current kilobot */
    CKilobotEntity* pcKB = any_cast<CKilobotEntity*>(it->second);
    CCI_KilobotController* pcKBC = &dynamic_cast<CCI_KilobotController&>(pcKB->GetControllableEntity().GetController());
    /* Create debug info for controller */
    debug_info_t* ptDebugInfo = pcKBC->DebugInfoCreate<debug_info_t>();
    /* Append to list */
    m_tKBs.push_back(std::make_pair(pcKBC, ptDebugInfo));
  }

  // initialize log file
  m_cOutput << "e_hits, r0_hits, r0_est_ut, r1_ut, dist, num_msgs" << std::endl;
  debug_counter = 0;
  overall_r0_estimate = 0;
  overall_empty_hits = 0;
  overall_r0_hits = 0;
  overall_distance = 0;
  overall_num_messages = 0;

#endif
}

/****************************************/
/****************************************/
void CComplexityALF::Reset() {}

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

    /* Sending the message using the overhead controller */
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
  }
}

/*********************************************/
/* This is one is only used to store kb logs */
/*********************************************/

void CComplexityALF::PostStep() {
  if(debug_counter < 81) {
    debug_counter++;
    return;
  }

  /* Go through the kilobots to get the positions */
  for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
    /* Update the virtual states and actuators of the kilobot*/
    overall_distance += SquareDistance(GetKilobotPosition(*m_tKilobotEntities[it]), CVector2(0,0));
  }

  /* Go through to get other debug info */
  //TODO check why this does not work m_tKilobotEntities.size();
  UInt8 nkbs = (m_tKilobotEntities.size()>3?3:m_tKilobotEntities.size());
  for(size_t i=0; i<nkbs; ++i) {
    // sum up all
    overall_r0_estimate += m_tKBs[i].second->ema_resource0;
    overall_empty_hits += m_tKBs[i].second->hits_empty;
    overall_r0_hits += m_tKBs[i].second->hits_resource0;
    overall_num_messages += m_tKBs[i].second->num_messages;
  }

  // average
  overall_distance = sqrt(overall_distance);

  // save to log file
  m_cOutput << (overall_empty_hits/nkbs) << " "
            << (overall_r0_hits/nkbs) << " "
            << (overall_r0_estimate/nkbs) << " "
            << resources.at(0).population << " "
            << (overall_distance/nkbs) << " "
            << (overall_num_messages/nkbs) << std::endl;
  // reset counters
  debug_counter = 0;
  overall_r0_estimate = 0;
  overall_empty_hits = 0;
  overall_r0_hits = 0;
  overall_distance = 0;
  overall_num_messages = 0;
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
