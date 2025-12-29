#include "controller_bug2.hpp"

namespace argos
{

    /****************************************/
    /****************************************/

    void ControllerBug2::Init(TConfigurationNode &t_tree)
    {
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
        TConfigurationNode &tTargetNode = GetNode(t_tree, "target_position");
        Real targetX, targetY;
        GetNodeAttribute(tTargetNode, "x", targetX);
        GetNodeAttribute(tTargetNode, "y", targetY);
        m_cTargetPosition.Set(targetX, targetY, 0.0);
        /* your Init code here */
        startPosition = m_pcPositioning->GetReading().Position;
        line_angle = atan2(targetY - startPosition.GetY(), targetX - startPosition.GetX());
        threshold_distance = 0.05; // 5 cm
        m_eState = STATE_GO_TO_GOAL;
    }

    void ControllerBug2::ControlStep()
    {
        LOG << "Current State: " << m_eState << std::endl;
        /* your ControlStep code here */
        switch (m_eState)
        {
        case STATE_GO_TO_GOAL:
            goToGoal();
            break;
        case STATE_CIRCUMNAVIGATE_OBSTACLE:
            circumvanteObstacle();
            break;
        case STATE_TRACE_OBSTACLE:
            traceObstacle();
            break;
        case STOP:
            m_pcWheels->SetLinearVelocity(0.0, 0.0);
            break;
        }
    }

    void ControllerBug2::goToGoal()
    {
        bool targetReached = false;
        const auto &blobs = m_pcCamera->GetReadings();
        for (const auto &blob : blobs.BlobList)
        {
            if (blob->Color == CColor::CYAN)
            {
                targetReached = true;
                break;
            }
        }
        LOG << "Target Reached: " << targetReached << std::endl;
        if (targetReached)
        {
            m_eState = STOP;
            return;
        }
        CRadians cZ, cY, cX;
        m_pcPositioning->GetReading().Orientation.ToEulerAngles(cZ, cY, cX);
        cZ.UnsignedNormalize();
        CRadians target_angle = CRadians(line_angle);
        bool onLine = isOnLine();
        LOG << "On Line: " << onLine << std::endl;
        target_angle.UnsignedNormalize();
        if (isObstacleDetected() && abs(cZ.GetValue() - target_angle.GetValue()) < threshold_distance)
        {
            m_eState = STATE_CIRCUMNAVIGATE_OBSTACLE;
            
            return;
        }
        if (abs(cZ.GetValue() - target_angle.GetValue()) > threshold_distance)
        {
            // turn towards the goal
            m_pcWheels->SetLinearVelocity(0.05, -0.05);
        }
        else
        {
            // move forward
            m_pcWheels->SetLinearVelocity(0.1, 0.1);
        }
    }

    void ControllerBug2::circumvanteObstacle()
    {
        bool obstacle = obstacleToMyLeft();
        if (obstacle)
        {
            m_eState = STATE_TRACE_OBSTACLE;
            m_errorSum = 0;
            obstacleStartPosition = m_pcPositioning->GetReading().Position;
            return;
        }
        else
        {
            // turn left
            m_pcWheels->SetLinearVelocity(-0.05, 0.05);
        }
    }

    void ControllerBug2::traceObstacle()
    {
        bool onLine = isOnLine();
        LOG << "On Line: " << onLine << std::endl;
        if (onLine && abs(m_pcPositioning->GetReading().Position.GetX() - obstacleStartPosition.GetX()) > 0.1 &&
            abs(m_pcPositioning->GetReading().Position.GetY() - obstacleStartPosition.GetY()) > 0.1)
        {
            m_eState = STATE_GO_TO_GOAL;
            
            return;
        }
        // if (isObstacleDetected())
        // {
        //     // obstacle still in front, keep tracing
        //     m_pcWheels->SetLinearVelocity(-0.05, 0.05);
        //     return;
        // }
        // if (obstacleToMyLeft())
        // {
        //     // keep moving forward
        //     m_pcWheels->SetLinearVelocity(0.1, 0.1);
        // }
        // else
        // {
        //     // turn left
        //     m_pcWheels->SetLinearVelocity(-0.05, 0.05);
        // }
        Real proximity = getSensorProximity(6);
        Real desired_distance = 0.05; // 5 cm
        Real error = desired_distance - proximity;
        m_errorSum += error;
        Real derivative = error - m_lastError;
        Real control_signal = m_fKp * error + m_fKi * m_errorSum + m_fKd * derivative;
        m_lastError = error;
        Real base_speed = 0.1;
        Real left_speed = base_speed + control_signal;
        Real right_speed = base_speed - control_signal;
        m_pcWheels->SetLinearVelocity(left_speed, right_speed);
    }

    bool ControllerBug2::isObstacleDetected()
    {
        bool obstacleDetected = false;
        std::function<void(const CCI_PiPuckRangefindersSensor::SInterface &)> fCheckSensor =
            [&obstacleDetected](const auto &sensor)
        {
            const auto &[id, pos, ori, range] = sensor.Configuration;
            if (sensor.Label != 0 && sensor.Label != 7)
                return; // only front sensors are considered
            if (sensor.Proximity < range)
            {
                obstacleDetected = true;
            }
        };

        // visit each sensor reading, modifying obstacleDetected if any of the front sensors detected
        m_pcRangefinders->Visit(fCheckSensor);
        return obstacleDetected;
    }

    bool ControllerBug2::obstacleToMyLeft()
    {
        bool obstacleDetected = false;
        m_pcRangefinders->Visit([&obstacleDetected](const auto &sensor)
                                {
            if (sensor.Label == 5 || sensor.Label == 6) {
                obstacleDetected = sensor.Proximity < std::get<3>(sensor.Configuration);
            } });
        return obstacleDetected;
    }

    bool ControllerBug2::obstacleToMyRight()
    {
        bool obstacleDetected = false;
        m_pcRangefinders->Visit([&obstacleDetected](const auto &sensor)
                                {
            if (sensor.Label == 1 || sensor.Label == 2) {
                obstacleDetected = sensor.Proximity < std::get<3>(sensor.Configuration);
            } });
        return obstacleDetected;
    }

    bool ControllerBug2::isOnLine()
    {
        const CVector3 p = m_pcPositioning->GetReading().Position;
        const CVector3 a = startPosition;
        const CVector3 b = m_cTargetPosition;
        const CVector3 ab = b - a;
        const Real ab_len2 = ab.SquareLength();
        if(ab_len2 < 1e-6) return true; // degenerate case
        const Real t_raw = ((p - a).DotProduct(ab)) / ab_len2;
        const Real t = std::max<Real>(0.0, std::min<Real>(1.0, t_raw));
        const CVector3 proj = a + t * ab;
        const Real dist = (p - proj).Length();

        return dist < threshold_distance;
    }

    Real ControllerBug2::getLeftProximity()
    {
        Real leftProximity = 0.0;
        m_pcRangefinders->Visit([&leftProximity](const auto &sensor)
                                {
            if (sensor.Label == 5 || sensor.Label == 6) {
                leftProximity += sensor.Proximity;
            } });
        return leftProximity; // average
    }

    Real ControllerBug2::getSensorProximity(UInt8 sensorLabel)
    {
        Real proximity = 0.0;
        m_pcRangefinders->Visit([&proximity, sensorLabel](const auto &sensor)
                                {
            if (sensor.Label == sensorLabel) {
                proximity = sensor.Proximity;
            } });
        return proximity;
    }
    /****************************************/
    /****************************************/

    REGISTER_CONTROLLER(ControllerBug2, "controller_bug2");

}
