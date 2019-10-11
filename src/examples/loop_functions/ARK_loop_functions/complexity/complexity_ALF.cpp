#include "complexity_ALF.h"

/****************************************/
/****************************************/

CComplexityALF::CComplexityALF() :
  m_unDataAcquisitionFrequency(10) {}

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
  m_vecCommittedKilobotsPositions.resize(m_tKilobotEntities.size());
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
}

/****************************************/
/****************************************/


void CComplexityALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){
  arena_size = CVector3(2,2,4);

  // initialize resources
  TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
  TConfigurationNodeIterator itNodes;
  // to check against xml
  UInt8 tType;
  // needed for generation
  std::vector<AreaALF> allAreas;

  for(itNodes=itNodes.begin(&tVirtualEnvironmentsNode); itNodes!=itNodes.end(); ++itNodes) {
    GetNodeAttribute(*itNodes, "type", tType);
    ResourceALF resource(tType, t_tree);
    resources.push_back(resource);
    allAreas.insert(std::end(allAreas), std::begin(resource.areas), std::end(resource.areas));
  }

  std::vector<CVector2> area_positions;
  for(ResourceALF& resource : resources) {
    resource.generate(allAreas, arena_size, resource.discretized_population);
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
/* Override ALF function to enable areas update */
/***********************************************/

void CComplexityALF::UpdateKilobotStates(){
  // resets kbs states and positions
  this->m_vecKilobotStates.clear();
  this->m_vecCommittedKilobotsPositions.clear();

  for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
    /* Update the virtual states and actuators of the kilobot*/
    UpdateKilobotState(*m_tKilobotEntities[it]);
  }

  // now get all areas (FIXME make it more efficient)
  std::vector<AreaALF> allAreas;
  for(const ResourceALF& resource : resources) {
    allAreas.insert(std::end(allAreas), std::begin(resource.areas), std::end(resource.areas));
  }

  // now do step and eventually generate new areas
  for(ResourceALF& resource : resources) {
    resource.doStep(m_vecCommittedKilobotsPositions, allAreas, arena_size);
  }
}


/****************************************/
/* Update both kilobots positions and areas in the environment */
/****************************************/

void CComplexityALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
  // current kb id
  UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
  // pre-check to avoid the two nested loops (those really slow down the simulation)
  // since we only need the kbs position of those committed (showing a RED led)
  if(GetKilobotLedColor(c_kilobot_entity) != CColor::RED) {
    // the kb is not committed to any area
    // 255 is the state value reserved for none
    m_vecKilobotStates[unKilobotID] = 255;
    return;
  }

  // update the position of the kilobots and store it for later use
  this->m_vecCommittedKilobotsPositions.push_back(GetKilobotPosition(c_kilobot_entity));

  // check against resources
  for(const ResourceALF& resource : resources) {
    for(const AreaALF& area : resource.areas) {
      // distance from the center of area
      if(SquareDistance(m_vecCommittedKilobotsPositions.back(), area.position) < pow(area.radius,2)) {
        // update the state of the kilobot and store it for later use
        m_vecKilobotStates[unKilobotID] = resource.type;
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
    // Fill the payload by sending only id and state
    UInt8 state = (UInt8) m_vecKilobotStates[unKilobotID]; // retrieve kb status

    // UNCOMMENT IF NEEDED
    // 2 bytes for x coordinate
    // 2 bytes for y coordinate
    // CVector2 kb_position = GetKilobotPosition(c_kilobot_entity);
    // UInt16 x_coord = (UInt16) (kb_position.GetX()*1000);
    // UInt16 y_coord = (UInt16) (kb_position.GetY()*1000);

    /* Prepare the inividual kilobot's message */
    m_tMessages[unKilobotID].type = 0; // using type 0 to signal ark messages
    /* Save time for next messages */
    m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;

    /* Fill up the kb message */
    m_tMessages[unKilobotID].data[0] = unKilobotID;
    m_tMessages[unKilobotID].data[1] = state; // the resource id
    if(state != 255) {
      m_tMessages[unKilobotID].data[2] = resources.at(state).umin;
      m_tMessages[unKilobotID].data[3] = resources.at(state).k;
    }

    // UNCOMMENT IF NEEDED
    // fill up the message of uint8 by splitting the uin16
    // m_tMessages[unKilobotID].data[2] = (x_coord >> 8); // hi part of the uint16
    // m_tMessages[unKilobotID].data[3] = (x_coord & 0xff); // lo part of the uint16
    // m_tMessages[unKilobotID].data[4] = (y_coord >> 8); // hi part of the uint16
    // m_tMessages[unKilobotID].data[5] = (y_coord & 0xff); // lo part of the uint16
    // to concatenate back use
    // UInt16 ycord = (((UInt16)data[4] << 8) | data[5]);

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
