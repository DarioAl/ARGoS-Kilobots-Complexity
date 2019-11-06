#include "complexity_ALF.h"

// Enable this part of the code to log the perceived utility estimation in the log file
#define DISTRIBUTION_ESTIMATION
#ifdef DISTRIBUTION_ESTIMATION
Real time_last_estimation = 0; // when the last estimation has been computed
UInt8 time_window_for_estimation = 20; // how long an estimation is
Real sent_messages = 0; // number of messages sent time_last_estimation
UInt8 hit_resource = 0; // number of hits since time_last_estimation
UInt8 hit_empty_space = 0; // number of empty hits since time_last_estimation
#endif

/****************************************/
/****************************************/

CComplexityALF::CComplexityALF() :
  m_unDataAcquisitionFrequency(10),
  circular_arena_radius(1),
  circular_arena_width(0.01),
  circular_arena_height(0.05),
  circular_arena_walls(50) {}

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
  /* The kilobots begins outside the clustering hub*/
  UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
  m_vecKilobotStates[unKilobotID] = 255;
  m_vecLastTimeMessaged[unKilobotID] = -1000;

  /* Get a random rotation within the circular arena */
  CQuaternion rand_rot;
  CRadians rand_rot_angle(((Real) rand()/(RAND_MAX)-0.5)*CRadians::PI.GetValue());
  // rand_rot.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);

  /* Get a non-colliding random position within the circular arena */
  bool distant_enough = false;
  Real rand_angle, rand_displacement_x, rand_displacement_y;
  CVector3 rand_pos;

  UInt16 maxTries = 999;
  UInt16 tries = 0;

  do {
    rand_angle = ((Real) rand()/(RAND_MAX))*2*CRadians::PI.GetValue();
    rand_displacement_x = ((Real) rand()/(RAND_MAX))*(circular_arena_radius-0.025);
    rand_displacement_y = ((Real) rand()/(RAND_MAX))*(circular_arena_radius-0.025);
    rand_pos = CVector3(rand_displacement_x*sin(rand_angle),rand_displacement_y*cos(rand_angle),0);

    distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), rand_pos, rand_rot, false);
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
    areas.insert(std::end(areas), std::begin(resource.areas), std::end(resource.areas));
  }

  std::vector<CVector2> area_positions;
  for(ResourceALF& resource : resources) {
    resource.generate(areas, circular_arena_radius, resource.discretized_population);
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
    resource.doStep(m_vecKilobotsPositions, m_vecKilobotStates, areas, circular_arena_radius);
  }
}

/***********************************************/
/***********************************************/

void CComplexityALF::UpdateKilobotStates(){
  // resets kbs states and positions
  this->m_vecKilobotStates.clear();
  this->m_vecKilobotsPositions.clear();

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
  m_vecKilobotStates[unKilobotID] = 255;

  // check against resources
  for(const ResourceALF& resource : resources) {
    for(const AreaALF& area : resource.areas) {
      // distance from the center of area
      if(SquareDistance(GetKilobotPosition(c_kilobot_entity), area.position) < pow(area.radius,2)) {
        // update the state and position of the kilobot and store it for later use
        m_vecKilobotStates[unKilobotID] = resource.type;
        // if the led color of a kilobot is green  means it is committed and working in place
        // hence store its position to update the resource that are exploited
        if(GetKilobotLedColor(c_kilobot_entity) == CColor::GREEN) {
          m_vecKilobotsPositions.push_back(GetKilobotPosition(c_kilobot_entity));
        }
        // got it, no need to search anymore
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
    // UNCOMMENT IF NEEDED
    // 2 bytes for x coordinate
    // 2 bytes for y coordinate
    // CVector2 kb_position = GetKilobotPosition(c_kilobot_entity);
    // UInt16 x_coord = (UInt16) (kb_position.GetX()*1000);
    // UInt16 y_coord = (UInt16) (kb_position.GetY()*1000);

    // fill up the message of uint8 by splitting the uin16
    // m_tMessages[unKilobotID].data[2] = (x_coord >> 8); // hi part of the uint16
    // m_tMessages[unKilobotID].data[3] = (x_coord & 0xff); // lo part of the uint16
    // m_tMessages[unKilobotID].data[4] = (y_coord >> 8); // hi part of the uint16
    // m_tMessages[unKilobotID].data[5] = (y_coord & 0xff); // lo part of the uint16
    // to concatenate back use
    // UInt16 ycord = (((UInt16)data[4] << 8) | data[5]);

#ifdef DISTRIBUTION_ESTIMATION
      if(unKilobotID == 0) {
        if(m_fTimeInSeconds-time_last_estimation >= time_window_for_estimation) {
        // store in the log file
        m_cOutput << hit_resource << ", " << hit_empty_space << ", " << sent_messages << std::endl;
        // update time_last_estimation
        time_last_estimation = m_fTimeInSeconds;
        // reset counters
        hit_resource = 0;
        hit_empty_space = 0;
      } else {
        sent_messages++;
        if(m_vecKilobotStates[unKilobotID] != 255) {
          hit_empty_space++;
        } else {
          hit_resource++;
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
