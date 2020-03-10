#ifndef ID_QTUSER_FUNCTIONS_H
#define ID_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>

using namespace argos;

class CIDQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CIDQTUserFunctions();

   virtual ~CIDQTUserFunctions() {}

   void Draw(CKilobotEntity& c_entity);

};

#endif
