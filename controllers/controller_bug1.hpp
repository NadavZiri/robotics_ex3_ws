
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

namespace argos
{

   class ControllerBug1 : public CCI_Controller
   {

   public:
      ControllerBug1() {}
      virtual ~ControllerBug1() {}

      virtual void Init(TConfigurationNode &t_tree) override;
      virtual void ControlStep() override;

   private:
      // states
      enum EState
      {
         STATE_GO_TO_GOAL = 0,
         STATE_ALIGN_TO_OBSTACLE,
         STATE_FOLLOW_OBSTACLE,
         STATE_GO_TO_CLOSEST_POINT,
         STOP
      };

      CCI_PiPuckDifferentialDriveActuator *m_pcWheels;
      CCI_PiPuckColorLEDsActuator *m_pcColoredLEDs;
      CCI_ColoredBlobOmnidirectionalCameraSensor *m_pcCamera;
      CCI_PiPuckRangefindersSensor *m_pcRangefinders;
      CCI_PiPuckSystemSensor *m_pcSystem;
      CCI_PositioningSensor *m_pcPositioning;

      // variables
      EState m_eState;
      CVector3 m_cTargetPosition;
      CVector3 m_cStartPosition;
      CVector3 m_cObstacleHitPoint;
      CVector3 m_cClosestPointToGoal;

      Real m_fClosestDistanceToGoal;
      Real m_fThresholdDistance;
      Real m_fPrevDistToClosest;

      bool m_bLeftHitPoint;
      int m_nLeaveTimer;
      Real m_fKp = 2;
      Real m_fKi = 0;
      Real m_fKd = 0;
      Real m_errorSum = 0;
      Real m_lastError = 0;

      // state functions
      void GoToGoal();
      void FollowObstacle();
      void GoToClosestPoint();
      Real getSensorProximity(UInt8 sensorLabel);
      void AlignToGoal();

      // helper functions
      bool IsObstacleDetected();
      bool IsTargetReached();
      Real DistanceToGoal(const CVector3 &pos);
      void MoveTowardsTarget();
      void MoveTowardsPoint(const CVector3 &point);
      bool ObstacleInFront();
      bool ObstacleOnLeft();
      bool ObstacleOnRight();
      void FollowWallOnly();
   };
}
