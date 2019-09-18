/**
 *  TODO
 */

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
  delete resource_a;
  delete resource_b;
  /* Close data file */
  m_cOutput.close();
}

/****************************************/
/****************************************/

void CComplexityALF::SetupInitialKilobotStates() {
  m_vecKilobotStates.resize(m_tKilobotEntities.size());
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
  m_vecKilobotStates[unKilobotID] = OUTSIDE_AREA;
  m_vecLastTimeMessaged[unKilobotID] = -1000;
}

/****************************************/
/****************************************/

void CComplexityALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){
  resource_a = new AreaALF(0, t_tree);
  resource_b = new AreaALF(1, t_tree);
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
  // resets kbs positions
  this->kilobotsInA = 0;
  this->kilobotsInB = 0;

  for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
    /* Update the virtual states and actuators of the kilobot*/
    UpdateKilobotState(*m_tKilobotEntities[it]);
  }

  // update areas now
  this->resource_a->doStep(this->kilobotsInA);
  this->resource_b->doStep(this->kilobotsInB);

  // std::cout << resource_a->population << std::endl;
  // std::cout << resource_b->population << std::endl;
 }


/****************************************/
/* Update both kilobots positions and areas in the environment */
/****************************************/

void CComplexityALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
  // update kilobots positions
  // Update the state of the kilobots (inside or outside the resources)
  UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
  CVector2 cKilobotPosition = GetKilobotPosition(c_kilobot_entity);

  // distance from the center of resource a
  if((pow(cKilobotPosition.GetX()-resource_a->position.GetX(),2) +
      pow(cKilobotPosition.GetY()-resource_a->position.GetY(),2)) <
     resource_a->radius){
    m_vecKilobotStates[unKilobotID]=INSIDE_AREA_A;
    ++this->kilobotsInA;
    return;
  }

  // distance from the center of resource b
  // this is done fore every kb, avoid this computation unless needed
  if((pow(cKilobotPosition.GetX()-resource_b->position.GetX(),2) +
      pow(cKilobotPosition.GetY()-resource_b->position.GetY(),2)) <
     resource_b->radius){
    m_vecKilobotStates[unKilobotID]=INSIDE_AREA_B;
    ++this->kilobotsInB;
    return;
  }

  // the kb is in none of the areas
  m_vecKilobotStates[unKilobotID]=OUTSIDE_AREA;
  ++this->kilobotsInNone;
}

/****************************************/
/* Actual update of the kbs sensors     */
/****************************************/

void CComplexityALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
  /* Flag for existance of message to send*/
  bool bMessageToSend = false;

  // get kb id
  UInt8 unKilobotID = (UInt8) GetKilobotId(c_kilobot_entity); // get kb id

  /* check if enough time has passed from the last message otherwise*/
  if(m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg) {
    // if the time is too short, the kilobot cannot receive a message
    return;
  } else {
    // Fill the 9bytes of the message payload as this:
    // 7th and 8th are free

    // 1st byte for kb state
    UInt8 state = (UInt8) m_vecKilobotStates[unKilobotID]; // retrieve kb status

    // 2nd-3rd bytes for x coordinate
    // 4th-5th bytes for y coordinate
    CVector2 kb_position = GetKilobotPosition(c_kilobot_entity);
    UInt16 x_coord = (UInt16) (kb_position.GetX()*1000);
    UInt16 y_coord = (UInt16) (kb_position.GetY()*1000);

    // 6th byte for area pop (normalazed between [0-255])
    UInt8 area_pop = 255;
    if(state == SRobotState::INSIDE_AREA_A) {
      area_pop = (UInt8) resource_a->population*255; // pop is between [0,1]
    } else if(state == SRobotState::INSIDE_AREA_B) {
      area_pop = (UInt8) resource_b->population*255; // pop is between [0,1]
    }

    /* Prepare the inividual kilobot's message */
    m_tMessages[unKilobotID].type = 0; // using type 0 to signal ark messages
    /* Save time for next messages */
    m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;

    /* Fill up the kb message */
    m_tMessages[unKilobotID].data[0] = unKilobotID;
    m_tMessages[unKilobotID].data[1] = state;
    // fill up the message of uint8 by splitting the uin16
    m_tMessages[unKilobotID].data[2] = (x_coord >> 8); // hi part of the uint16
    m_tMessages[unKilobotID].data[3] = (x_coord & 0xff); // lo part of the uint16
    m_tMessages[unKilobotID].data[4] = (y_coord >> 8); // hi part of the uint16
    m_tMessages[unKilobotID].data[5] = (y_coord & 0xff); // lo part of the uint16
    m_tMessages[unKilobotID].data[6] = area_pop;
    // to concatenate back use
    // UInt16 ycord = (((UInt16)data[4] << 8) | data[5]);

    // TODO set crc
    // m_tMessages[unKilobotID].crc = message_crc(&m_tMessages[unKilobotID]); // set receiver id

    /* Sending the message using the overhead controller */
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
  }

}

/****************************************/
/****************************************/

CColor CComplexityALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
  CColor cColor=CColor::WHITE;
  if((pow(vec_position_on_plane.GetX()-resource_a->position.GetX(),2) +
      pow(vec_position_on_plane.GetY()-resource_a->position.GetY(),2)) <
     resource_a->radius){
    cColor=resource_a->color;
    return cColor;
  }
  if((pow(vec_position_on_plane.GetX()-resource_b->position.GetX(),2) +
      pow(vec_position_on_plane.GetY()-resource_b->position.GetY(),2)) <
     resource_b->radius){
    cColor=resource_b->color;
    return cColor;
  }
  return cColor;
}

REGISTER_LOOP_FUNCTIONS(CComplexityALF, "ALF_complexity_loop_function")
