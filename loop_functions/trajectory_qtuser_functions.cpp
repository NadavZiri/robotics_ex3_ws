#include "trajectory_qtuser_functions.hpp"

using namespace argos;

CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions() :
m_cTrajLF(dynamic_cast<CTrajectoryLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {}

void CTrajectoryQTUserFunctions::DrawInWorld() {
    const auto& map = m_cTrajLF.GetWaypoints();
    for(const auto& pair : map) {
        // draw only if at least 2 points; DrawWaypoints handles the check
        DrawWaypoints(pair.second);
    }
}

void CTrajectoryQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
    if(c_waypoints.size() < 2) return;
    // Iterate once; complexity capped by pruning in loop functions
    for(size_t i = 1; i < c_waypoints.size(); ++i) {
        CVector3 a = c_waypoints[i-1];
        CVector3 b = c_waypoints[i];
        a.SetZ(a.GetZ() + 0.01f); // raise slightly to avoid z-fighting
        b.SetZ(b.GetZ() + 0.01f);
        DrawRay(CRay3(a, b), CColor::BLUE, 2.0f);
    }
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions, "trajectory_qtuser_functions");