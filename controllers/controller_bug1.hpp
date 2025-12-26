#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
// #include <argos3/core/utility/math/radians.h>

namespace argos {

   class ControllerBug1 : public CCI_Controller {

   public:

      ControllerBug1() {}

      virtual ~ControllerBug1() {}

      void Init(TConfigurationNode& t_tree) override;

      void ControlStep() override;

   private:

      /* Sensors and Actuators */
      CCI_PiPuckDifferentialDriveActuator* m_pcWheels = nullptr;
      CCI_PiPuckColorLEDsActuator* m_pcColoredLEDs = nullptr;
      CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera = nullptr;
      CCI_PiPuckRangefindersSensor* m_pcRangefinders = nullptr;
      CCI_PiPuckSystemSensor* m_pcSystem = nullptr;
      CCI_PositioningSensor* m_pcPositioning = nullptr;
      CVector3 m_cTargetPosition;

      /* Bug1 algorithm variables */
      enum class EBug1State {
         ALIGN_TO_TARGET,
         MOVE_STRAIGHT,
         FOLLOW_OBSTACLE,
         GO_TO_BEST_POINT,
         FINISHED
      };
      EBug1State m_eState; 
      CVector2 m_cHitPoint; // Point where the robot first encountered the obstacle
      CVector2 m_cBestPoint; // Closest point to the target found while following the obstacle
      Real m_fBestDist; // Distance to the target at the best point
      bool m_bCompletedLoop; // Flag to indicate if a full loop around the obstacle has been completed

      bool IsObstacleAhead();
      void MoveTowards(const CVector2& cDir);
      void AlignToTarget();
      void MoveStraight();
      void FollowObstacle();
      void GoToBestPoint();


};
}