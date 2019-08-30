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

  std::cout << resource_a->population << std::endl;
  std::cout << resource_b->population << std::endl;
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
  /*Create ARK-type messages variables*/
  m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;

  /* Flag for existance of message to send*/
  bool bMessageToSend = false;

  /* Get the kilobot ID and state (Only Position in this example) */
  UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);

  /* check if enough time has passed from the last message otherwise*/
  if(m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg) {
    return; // if the time is too short, the kilobot cannot receive a message
  } else {
    /*  Prepare the inividual kilobot's message */
    tKilobotMessage.m_sID = unKilobotID; // kilobot id
    tKilobotMessage.m_sType = (int)m_vecKilobotStates[unKilobotID]; // tell the kilobot if it is over a resource

    /*  Set the message sending flag to True */
    bMessageToSend=true;
    m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
  }

  /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
  if(bMessageToSend){
    for(int i=0; i<9; ++i) {
      m_tMessages[unKilobotID].data[i] = 0;
    }

    // Prepare an empty ARK-type message to fill the gap in the full kilobot message
    tEmptyMessage.m_sID=1023;
    tEmptyMessage.m_sType=0;
    tEmptyMessage.m_sData=0;

    // Fill the kilobot message by the ARK-type messages
    for(int i=0; i<3; ++i) {
      if(i==0) {
        tMessage = tKilobotMessage;
      } else {
        tMessage = tEmptyMessage;
      }
      m_tMessages[unKilobotID].data[i*3] = (tMessage.m_sID >> 2); // 2nd kb
      m_tMessages[unKilobotID].data[1+i*3] = (tMessage.m_sID << 6); // 3rd kb
      m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sType << 2);
      m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sData >> 8);
      m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
    }
    /* Sending the message using the overhead controller */
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
  } else {
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
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
