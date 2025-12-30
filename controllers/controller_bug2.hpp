/* 211388921 Nadav Ziri & 212497564 Shirel Ben Baruch */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_actuator.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_color_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_rangefinders_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_system_sensor.h>
#include <argos3/plugins/robots/pi-puck/control_interface/ci_pipuck_differential_drive_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>


namespace argos
{

   class ControllerBug2 : public CCI_Controller
   {

   public:
      ControllerBug2() {}

      virtual ~ControllerBug2() {}

      void Init(TConfigurationNode &t_tree) override;

      void ControlStep() override;

      bool isObstacleDetected();

      void goToGoal();

      void circumvanteObstacle();

      bool obstacleToMyLeft();

      bool obstacleToMyRight();

      Real getLeftProximity();

      Real getSensorProximity(UInt8 sensorLabel);

      bool isOnLine();

      void traceObstacle();

      enum EState
      {
         STATE_GO_TO_GOAL,
         STATE_CIRCUMNAVIGATE_OBSTACLE,
         STATE_TRACE_OBSTACLE,
         STOP
      };

      int closestAngle();

   private:
      /* Sensors and Actuators */
      CCI_PiPuckDifferentialDriveActuator *m_pcWheels = nullptr;
      CCI_PiPuckColorLEDsActuator *m_pcColoredLEDs = nullptr;
      CCI_ColoredBlobOmnidirectionalCameraSensor *m_pcCamera = nullptr;
      CCI_PiPuckRangefindersSensor *m_pcRangefinders = nullptr;
      CCI_PiPuckSystemSensor *m_pcSystem = nullptr;
      CCI_PositioningSensor *m_pcPositioning = nullptr;
      CVector3 m_cTargetPosition;
      CVector3 startPosition;
      CVector3 obstacleStartPosition;
      EState m_eState;
      Real line_angle;
      Real threshold_distance;
      Real m_fKp = 2;
      Real m_fKi = 0;
      Real m_fKd = 0;
      Real m_errorSum = 0;
      Real m_lastError = 0;
   };
}
