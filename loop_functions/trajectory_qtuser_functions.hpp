#pragma once

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#include "trajectory_loop_functions.hpp"

using namespace argos;

class CTrajectoryQTUserFunctions : public CQTOpenGLUserFunctions {
public:
   CTrajectoryQTUserFunctions();
   virtual ~CTrajectoryQTUserFunctions() {}
   virtual void DrawInWorld() override;

private:
   void DrawWaypoints(const std::vector<CVector3>& c_waypoints);
   CTrajectoryLoopFunctions& m_cTrajLF;
};