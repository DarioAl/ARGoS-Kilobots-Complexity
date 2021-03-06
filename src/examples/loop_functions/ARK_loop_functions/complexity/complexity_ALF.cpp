#include "complexity_ALF.h"
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>

#include <bitset>
// Enable this part of the code to log the perceived utility estimation in the log file
#define DISTRIBUTION_ESTIMATION
#ifdef DISTRIBUTION_ESTIMATION
UInt8 debug_counter;
Real overall_r0_estimate;
Real overall_r1_estimate;
Real overall_r2_estimate;
Real overall_distance;
UInt8 overall_num_messages;
UInt32 sequential;
#endif

/****************************************/
/****************************************/

CComplexityALF::CComplexityALF() :
  circular_arena_radius(0.51),
  circular_arena_width(0.01),
  circular_arena_height(0.05),
  circular_arena_walls(50),
  m_unDataAcquisitionFrequency(10) {
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
  // m_cOutput << "r0_est_ut, r1_ut, dist, num_msgs" << std::endl;
  debug_counter = 0;
  overall_r0_estimate = 0;
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
  // initialize kilobot state over empty state
  m_kilobotstate m_state;
  m_state.resources[0] = false;
  m_state.resources[1] = false;
  m_state.resources[2] = false;
  m_vecKilobotStates[unKilobotID] = m_state;
  // store kilobot position
  m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
  // set kilobot color (initially off since uncommitted)
  m_vecKilobotColors[unKilobotID] = CColor::BLACK;
  // set up when last message was sent
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

  for(ResourceALF& resource : resources) {
    resource.generate(circular_arena_radius);
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
    resource.doStep(m_vecKilobotsPositions, m_vecKilobotStates, m_vecKilobotColors, CPhysicsEngine::GetInverseSimulationClockTick());
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
  m_kilobotstate m_state;
  m_state.resources[0] = false;
  m_state.resources[1] = false;
  m_state.resources[2] = false;
  // check against resources
  for(const ResourceALF& resource : resources) {
    for(const AreaALF& area : resource.areas) {
      // distance from the center of area
      if(SquareDistance(GetKilobotPosition(c_kilobot_entity), area.position) <= pow(area.radius,2)) {
        // update the state and position of the kilobot and store it for later use
        m_state.resources[resource.type] = true;
      }
    }
  }
  m_vecKilobotStates[unKilobotID] = m_state;
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
    /*  data[0]   data[1]   data[2]                    */
    /* xxxx xxxx xxaa aabb bbcc ccyy                   */
    /* x(7) bits used for kilobot id                  */
    /* a(5) bits used for resource a utility           */
    /* b(5) bits used for resource b utility           */
    /* c(5) bits used for resource c utility           */
    /* y(2) bits used for turing angle                 */

    /* How to interpret the data sent?                 */
    /* If no resource a,b,c utility is received then   */
    /* means that the kb is on empty space             */
    /* On the opposite, if a resource utility is there */
    /* means that the kb is on resource space          */
    /* The turning angle is divided in 4 equal slices  */
    /* pi/4 to 3/4pi - 3/4pi to 5/4pi - 5/4pi to 7/4pi */


    // clean tkilobotMessage fields
    tkilobotMessage.m_sID = 0;
    tkilobotMessage.m_sType = 0;
    tkilobotMessage.m_sData = 0;

    // 7 bits used for the id of the kilobot
    tkilobotMessage.m_sID = unKilobotID;
    tkilobotMessage.m_sID = tkilobotMessage.m_sID << 3;

    m_kilobotstate kilobotState = m_vecKilobotStates[unKilobotID];
    if(kilobotState.resources[0]) {
      // get the correct area
      for(const AreaALF& area : resources.at(0).areas){
        if(SquareDistance(area.position, m_vecKilobotsPositions[unKilobotID]) < pow(area.radius, 2)) {
          // 5 bits used for resource a (thats why is divided by 8 i.e. 31 slices)
          uint8_t temp = ceil(area.population*31);
          tkilobotMessage.m_sID = tkilobotMessage.m_sID | (temp >> 2);
          tkilobotMessage.m_sType = temp << 2;
          break;
        }
      }
    }

    if(kilobotState.resources[1]) {
      // get the correct area
      for(const AreaALF& area : resources.at(1).areas){
        if(SquareDistance(area.position, m_vecKilobotsPositions[unKilobotID]) < pow(area.radius, 2)) {
          // 5 bits used for resource b (thats why is divided by 8 i.e. 31 slices )
          uint8_t temp = ceil(area.population*31);
          tkilobotMessage.m_sType = tkilobotMessage.m_sType | (temp >> 3);
          tkilobotMessage.m_sData = temp;
          tkilobotMessage.m_sData = tkilobotMessage.m_sData << 7;
          break;
        }
      }
    }

    if(kilobotState.resources[2]) {
      // get the correct area
      for(const AreaALF& area : resources.at(2).areas){
        if(SquareDistance(area.position, m_vecKilobotsPositions[unKilobotID]) < pow(area.radius, 2)) {
          // 5 bits used for resource c (thats why is divided by 8 i.e. 31 slices )
          uint8_t temp = ceil(area.population*31);
          tkilobotMessage.m_sData = tkilobotMessage.m_sData | (temp << 2);
          break;
        }
      }
    }

    // 2 remaining bits of m_sData are used to store rotation toward the center
    // i.e. if the kilobot is too close to the border then a rotation angle is sent
    UInt8 turning_in_msg = 0; // what is actually sent (0 no turn, 1 pi/2, 2 pi, 3 3pi/2)
    CVector2 kb_position = GetKilobotPosition(c_kilobot_entity);
    Real pdistance = kb_position.Length()/circular_arena_radius;
    if(pdistance > 0.92) {
      CVector2 kb_orientation = CVector2(1,GetKilobotOrientation(c_kilobot_entity));
      CRadians turning_angle(M_PI-acos(kb_orientation.Normalize().DotProduct(kb_position.Normalize())));

      // only turn if turning angle greater than 45 degrees
      if(abs(turning_angle.GetValue()) > M_PI/4) {
        if(abs(turning_angle.GetValue()) > 3*M_PI/4) {
          turning_in_msg = 2;
        } else if(abs(turning_angle.GetValue() > 5*M_PI/4)) {
          turning_in_msg = 3;
        }
        turning_in_msg = 1;
      }
      tkilobotMessage.m_sData = tkilobotMessage.m_sData | turning_in_msg;
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
    tEmptyMessage.m_sID = 1023; // invalid id
    tEmptyMessage.m_sType = 0; // empty field
    tEmptyMessage.m_sData = 0; // empty field

    // Fill the kilobot message by the ARK-type messages
    for (int i = 0; i < 3; ++i) {
      if(i == 0){
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
      // if(i==0){
      // std::cout << "msID " << std::bitset<10>(tkilobotMessage.m_sID) << std::endl;
      // std::cout << "msType " << std::bitset<4>(tkilobotMessage.m_sType) << std::endl;
      // std::cout << "msData " << std::bitset<10>(tkilobotMessage.m_sData) << std::endl;
      // std::cout << "data[0] " << std::bitset<8>(m_tMessages[unKilobotID].data[i*3]) << std::endl;
      // std::cout << "data[1] " << std::bitset<8>(m_tMessages[unKilobotID].data[1+i*3]) << std::endl;
      // std::cout << "data[2] " << std::bitset<8>(m_tMessages[unKilobotID].data[2+i*3]) << std::endl;
      // std::cout << "\n" << std::endl;
      // }
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
  // every step (tick)
  if(debug_counter < 9) {
    debug_counter++;
    return;
  }

  /* Go through kilobots to get other debug info */
  UInt8 nkbs = m_tKilobotEntities.size();
  UInt8 status[] = {0,0,0,0,0,0,0}; // as for the enum, last element is 255
  for(size_t i=0; i<nkbs; ++i) {
    /* Get kilobot decision state */
    if(m_tKBs[i].second->decision == 255) {
      status[6] = status[6]+1;
    } else {
      status[m_tKBs[i].second->decision] = status[m_tKBs[i].second->decision]+1;
    }
    /* Get kilobot estimate of resource 0 */
    overall_r0_estimate += (float)m_tKBs[0].second->ema_resource0/255.0;
    overall_r1_estimate += (float)m_tKBs[0].second->ema_resource1/255.0;
    overall_r2_estimate += (float)m_tKBs[0].second->ema_resource2/255.0;
  }

  // average
  overall_distance = sqrt(overall_distance);

  // save to log file
  // pop1 comm1 quor1 pop2 comm2 quor2 pop3 comm3 quor3 uncom step
  m_cOutput << (float)resources.at(0).population/(float)resources.at(0).k << " "
            << status[0] << " " << status[3] << " "
            << (float)resources.at(1).population/(float)resources.at(1).k << " "
            << status[1] << " " << status[4] << " "
            << (float)resources.at(2).population/(float)resources.at(2).k << " "
            << status[2] << " " << status[5] << " "
            << status[6] << " "
            << overall_r0_estimate << " "
            << overall_r1_estimate << " "
            << overall_r2_estimate << " "
            << (resources.at(0).totalExploitation+resources.at(1).totalExploitation+resources.at(2).totalExploitation) << std::endl;

  // reset counters
  debug_counter = 0;
  overall_r0_estimate = 0;
  overall_r1_estimate = 0;
  overall_r2_estimate = 0;
  overall_distance = 0;
  overall_num_messages = 0;
  sequential++;
}

/****************************************/
/****************************************/

CColor CComplexityALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
  // base background color
  CColor cColor=CColor::WHITE;

  // check if resource
  for(const ResourceALF& resource : resources) {
    for(const AreaALF& area : resource.areas) {
      if(SquareDistance(vec_position_on_plane,area.position) < pow(area.radius,2)){
        if(area.population < 0.05) {
          cColor = CColor::GRAY80;
        } else {
          cColor = CColor(area.color.GetRed(), area.color.GetGreen(), area.color.GetBlue(), 0);
        }
       return cColor;
      }
    }
  }
  return cColor;
}

REGISTER_LOOP_FUNCTIONS(CComplexityALF, "ALF_complexity_loop_function")
