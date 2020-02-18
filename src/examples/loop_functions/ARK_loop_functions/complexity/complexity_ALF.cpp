#include "complexity_ALF.h"
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>

#include <bitset>
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
  std::cout << "exiting  setup initial kb s " << std::endl;
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
  Real rand_angle, rand_distance;
  CVector3 rand_pos;

  UInt16 maxTries = 9999;
  UInt16 tries = 0;

  /* Get a random orientation for the kilobot */
  CQuaternion random_rotation;
  CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
  random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);
  do {
    rand_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
    rand_distance = c_rng->Uniform(CRange<Real>(0, circular_arena_radius-0.015));
    rand_pos = CVector3(rand_distance*cos(rand_angle),rand_distance*sin(rand_angle),0);
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

  // update the local areas array
  areas.clear();
  for(ResourceALF& resource : resources) {
    areas.insert(areas.end(), resource.areas.begin(), resource.areas.end());
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
  /* Create ARK-type message variables addressing 3 kbs per message */
  m_tALFKilobotMessage tkilobotMessage, tEmptyMessage, tMessage;

  /* Flag for existance of message to send */
  bool bMessageToSend = false;

  /* Get the kilobot ID */
  UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);

  /* check if enough time has passed from the last message otherwise*/
  if(m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg) {
    // if the time is too short, the kilobot cannot receive a message
    return;
  } else {
    // !!! THE FOLLOWING IS OF EXTREME IMPORTANCE !!!
    // NOTE although the message is defined as type, id and data, in ARK the fields type and id are swapped
    // resulting in a mixed message. If you are not using the whole field this could lead to problems.
    // To avoid it, consider to concatenate the message as ID, type and data.

    /* Prepare the inividual kilobot's message         */
    /* see README.md to understand about ARK messaging */
    /* data has 3x24 bits divided as                   */
    /*   ID 10b    type 4b  data 10b     <- ARK msg    */
    /*  data[0]   data[1]   data[2]      <- kb msg     */
    /* xxxx xxxx xyyz zzzw wwww wwww     <- complexity */
    /* x bits used for kilobot id                      */
    /* y bits used for kilobot arena state             */
    /* z bits used for resource umin                   */
    /* w bits used for kilobot rotattion toward center */

    // clena tkilobotMessage fields
    tkilobotMessage.m_sID = 0;
    tkilobotMessage.m_sType = 0;
    tkilobotMessage.m_sData = 0;

    // 9 bits used for the id of the kilobot store in the first 9 bits of the message
    tkilobotMessage.m_sID = unKilobotID << 1;

    // 2 bits used for the kb state (over a resource?)
    // this is changed as following
    // 0 no resource
    // 1 resource 1
    // 2 resource 2
    // 3 resource 3
    UInt8 kilobotState = m_vecKilobotStates[unKilobotID] + 1;
    tkilobotMessage.m_sID = tkilobotMessage.m_sID | (kilobotState >> 1);
    tkilobotMessage.m_sType = (kilobotState &0x01) << 3;

    // 4 bits used to store umin of current area (this can change dynamically)
    if(kilobotState != 0) { // i.e. 255 in local kilobots_state list
      UInt8 rumin = resources.at(kilobotState-1).umin * 10;
      tkilobotMessage.m_sType = tkilobotMessage.m_sType | (rumin >> 1);
      tkilobotMessage.m_sData = rumin << 9;
    }

    // 9 remaining bits of m_sData are used to store rotation toward the center
    // i.e. if the kilobot is too close to the border then a rotation angle is sent
    CVector2 kb_position = GetKilobotPosition(c_kilobot_entity);
    Real pdistance = kb_position.Length()/circular_arena_radius;
    if (pdistance > 0.92) {
      CVector2 kb_orientation = CVector2(1,GetKilobotOrientation(c_kilobot_entity));
      CRadians turning_angle(M_PI-acos(kb_orientation.Normalize().DotProduct(kb_position.Normalize())));

      // only turn if turning angle greater than 45 degrees
      if(abs(turning_angle.GetValue()) > M_PI/6) {
        UInt16 angle_sign = turning_angle.GetValue()>0?1:-1;
        UInt8 int_turning_angle = ToDegrees(turning_angle).GetValue();
        tkilobotMessage.m_sData = tkilobotMessage.m_sData | (angle_sign << 8) | int_turning_angle;
      }
    }

    /*  Set the message sending flag to True */
    bMessageToSend=true;
    m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
  }

  /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots msg)*/
  if(bMessageToSend) {
    for (int i = 0; i < 9; ++i) {
      m_tMessages[unKilobotID].data[i] = 0;
    }

    // Prepare an empty ARK-type message to fill the gap in the full kilobot message

    tEmptyMessage.m_sID = 511; // invalid id
    tEmptyMessage.m_sType = 0; // empty field
    tEmptyMessage.m_sData = 0; // empty field

    // Fill the kilobot message by the ARK-type messages
    for (int i = 0; i < 3; ++i) {
      if( i == 0){
        tMessage = tkilobotMessage;
      } else{
        tMessage = tEmptyMessage;
      }
      // clean up m_tmessages for safety
      m_tMessages[unKilobotID].data[i*3] = 0;
      m_tMessages[unKilobotID].data[1+i*3] = 0;
      m_tMessages[unKilobotID].data[2+i*3] = 0;
      // fill it up
      m_tMessages[unKilobotID].data[i*3] = (tMessage.m_sID >> 2);
      m_tMessages[unKilobotID].data[1+i*3] = (tMessage.m_sID << 6);
      m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sType << 2);
      m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sData >> 8);
      m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
      if(i==0){
      std::cout << "msID " << std::bitset<10>(tkilobotMessage.m_sID) << std::endl;
      std::cout << "msType " << std::bitset<4>(tkilobotMessage.m_sType) << std::endl;
      std::cout << "msData " << std::bitset<10>(tkilobotMessage.m_sData) << std::endl;
      std::cout << "data[0] " << std::bitset<8>(m_tMessages[unKilobotID].data[i*3]) << std::endl;
      std::cout << "data[1] " << std::bitset<8>(m_tMessages[unKilobotID].data[1+i*3]) << std::endl;
      std::cout << "data[2] " << std::bitset<8>(m_tMessages[unKilobotID].data[2+i*3]) << std::endl;
      std::cout << "\n" << std::endl;
      }
    }
    /* Sending the message */
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
  } else{
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
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
