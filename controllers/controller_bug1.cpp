#include "controller_bug1.hpp"
#include <limits>

namespace argos {
   
   /****************************************/
   /****************************************/
   
   void ControllerBug1::Init(TConfigurationNode& t_tree) {
      /* Get the actuators and sensors */
      m_pcWheels = GetActuator<CCI_PiPuckDifferentialDriveActuator>("pipuck_differential_drive");
      m_pcColoredLEDs = GetActuator<CCI_PiPuckColorLEDsActuator>("pipuck_leds");
      m_pcSystem = GetSensor<CCI_PiPuckSystemSensor>("pipuck_system");
      m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcCamera->Enable();
      m_pcRangefinders = GetSensor<CCI_PiPuckRangefindersSensor>("pipuck_rangefinders");
      m_pcPositioning = GetSensor<CCI_PositioningSensor>("positioning");
      m_pcPositioning->Enable();
      
      // read target position from .argos file into m_cTargetPosition
      TConfigurationNode& tTargetNode = GetNode(t_tree, "target_position");
      Real targetX, targetY;
      GetNodeAttribute(tTargetNode, "x", targetX);
      GetNodeAttribute(tTargetNode, "y", targetY);
      m_cTargetPosition.Set(targetX, targetY, 0.0);

      // Initialize Bug1 algorithm variables
      m_eState = EBug1State::ALIGN_TO_TARGET;
      m_fBestDist = std::numeric_limits<Real>::max();
      m_bCompletedLoop = false;
      m_cHitPoint.Set(0.0, 0.0);
      m_cBestPoint.Set(0.0, 0.0);
   }
   
   void ControllerBug1::ControlStep() {

      /* your ControlStep code here */
      switch(m_eState) {
         case EBug1State::ALIGN_TO_TARGET:
            AlignToTarget();
            break;
         case EBug1State::MOVE_STRAIGHT:
            MoveStraight();
            break;
         case EBug1State::FOLLOW_OBSTACLE:
            FollowObstacle();
            break;
         case EBug1State::GO_TO_BEST_POINT:
            GoToBestPoint();
            break;
         case EBug1State::FINISHED:
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            m_pcColoredLEDs->SetAllColors(CColor::GREEN);
            break;
      }
   }
void ControllerBug1::AlignToTarget() {

   m_pcColoredLEDs->SetAllColors(CColor::BLUE);

   // current position
   CVector3 currentPos = m_pcPositioning->GetReading().Position;
   CVector2 cPos(currentPos.GetX(), currentPos.GetY());

   // target position
   CVector2 cTarget(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
   CVector2 toTarget = cTarget - cPos;

   
   CRadians cZ, cY, cX;
   m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
   cZ.UnsignedNormalize();

   // target direction angle
   Real line_angle = ATan2(toTarget.GetY(), toTarget.GetX());
   CRadians target_angle = CRadians(line_angle);

   // angle error
   CRadians cAngleError = target_angle - cZ;
   cAngleError.SignedNormalize();

   const Real fThreshold = 0.05;

   if(Abs(cAngleError.GetValue()) > fThreshold) {
      m_pcWheels->SetLinearVelocity(-0.05, 0.05);
   } else {
      m_eState = EBug1State::MOVE_STRAIGHT;
   }
}

   bool ControllerBug1::IsObstacleAhead(){
      const Real OBSTACLE_THRESHOLD = 0.2; 

      for(const auto& reading : m_pcRangefinders->GetReadings()) {
         if(reading.Value < OBSTACLE_THRESHOLD) {
            return true;
         }
      }
      return false;
   }

   void ControllerBug1::MoveStraight(){

      m_pcColoredLEDs->SetAllColors(CColor::BLUE);

      if(IsObstacleAhead()) {
         CVector3 currentPos = m_pcPositioning->GetReading().Position;
         m_cHitPoint.Set(currentPos.GetX(), currentPos.GetY()); // to remeber when a loop is completed

         // initialize bug1 variables
         CVector2 cTarget(m_cTargetPosition.GetX(), m_cTargetPosition.GetY());
         CVector2 cPos(currentPos.GetX(), currentPos.GetY());

         m_cBestPoint = cPos;
         m_fBestDist = (cTarget - cPos).Length();
         m_bCompletedLoop = false;
         m_eState = EBug1State::FOLLOW_OBSTACLE;
         return;
      } else {
      m_pcWheels->SetLinearVelocity(0.5, 0.5);

   }
}

void ControllerBug1::FollowObstacle(){

   m_pcColoredLEDs->SetAllColors(CColor::YELLOW);

   const Real fLinearSpeed = 0.4;
   const Real fTurnSpeed   = 0.3;

   const auto& readings = m_pcRangefinders->GetReadings();

   Real rightMin = 1.0;

   for(size_t i = readings.size() / 2; i < readings.size(); ++i) {
      rightMin = Min(rightMin, readings[i].Value);
   }

   if(rightMin < 0.15) {
      m_pcWheels->SetLinearVelocity(fLinearSpeed,
                                    fLinearSpeed + fTurnSpeed);
   } else {
      m_pcWheels->SetLinearVelocity(fLinearSpeed + fTurnSpeed,
                                    fLinearSpeed);
   }

   CVector3 currentPos3D = m_pcPositioning->GetReading().Position;
   CVector2 cPos(currentPos3D.GetX(), currentPos3D.GetY());

   CVector2 cTarget(m_cTargetPosition.GetX(),
                    m_cTargetPosition.GetY());

   Real currentDist = (cTarget - cPos).Length();

   if(currentDist < m_fBestDist) {
      m_fBestDist = currentDist;
      m_cBestPoint = cPos;
   }

   if((cPos - m_cHitPoint).Length() < 0.1 && !m_bCompletedLoop) {
      m_bCompletedLoop = true;
      m_eState = EBug1State::GO_TO_BEST_POINT;
   }
}
void ControllerBug1::GoToBestPoint(){

   m_pcColoredLEDs->SetAllColors(CColor::YELLOW);

   CVector3 currentPos3D = m_pcPositioning->GetReading().Position;
   CVector2 cPos(currentPos3D.GetX(), currentPos3D.GetY());

   CVector2 toBest = m_cBestPoint - cPos;

   CRadians cZ, cY, cX;
   m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
   cZ.UnsignedNormalize();

   Real line_angle = ATan2(toBest.GetY(), toBest.GetX());
   CRadians target_angle = CRadians(line_angle);

   CRadians angle_error = target_angle - cZ;
   angle_error.SignedNormalize();

   Real fAngular = fAngularGain * angle_error.GetValue();


   const Real fAngularGain = 2.0;
   const Real fLinearSpeed = 0.4;



   m_pcWheels->SetLinearVelocity(
      fLinearSpeed - fAngular,
      fLinearSpeed + fAngular
   );

   if(toBest.Length() < 0.05) {
      m_eState = EBug1State::ALIGN_TO_TARGET;
   }
}





   /****************************************/
   /****************************************/
   /****************************************/
   
   REGISTER_CONTROLLER(ControllerBug1, "controller_bug1");
   
}