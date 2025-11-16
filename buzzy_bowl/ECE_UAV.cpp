#include "ECE_UAV.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>

void threadFunction(ECE_UAV* pUAV) { pUAV->physicsLoop(); }

ECE_UAV::ECE_UAV(const std::array<float, 3>& position,
                 const std::array<float, 3>& velocity,
                 const std::array<float, 3>& acceleration)
{
    m_position     = position;
    m_velocity     = velocity;
    m_acceleration = acceleration;
}

std::array<float, 3> ECE_UAV::calculateOrbitForce()
{
    std::array<float, 3> vectorToUAV = { m_position[0] - SPHERE_CENTER[0],
                                         m_position[1] - SPHERE_CENTER[1],
                                         m_position[2] - SPHERE_CENTER[2] };

    float currentDistance
        = std::sqrt(vectorToUAV[0] * vectorToUAV[0] + vectorToUAV[1] * vectorToUAV[1]
                    + vectorToUAV[2] * vectorToUAV[2]);

    std::array<float, 3> radialDirection = { 0.0f, 0.0f, 0.0f };
    if (currentDistance > 0.001f)
    {
        radialDirection[0] = vectorToUAV[0] / currentDistance;
        radialDirection[1] = vectorToUAV[1] / currentDistance;
        radialDirection[2] = vectorToUAV[2] / currentDistance;
    }

    float error  = SPHERERADIUS - currentDistance;
    float p_term = m_Kp * error;

    m_pidIntegral += error * DELTAT;
    float i_term = m_Ki * m_pidIntegral;

    float derivative   = (error - m_pidPreviousError) / DELTAT;
    m_pidPreviousError = error;
    float d_term       = m_Kd * derivative;

    float pidForceMagnitude = p_term + i_term + d_term;

    std::array<float, 3> radialForce = { radialDirection[0] * pidForceMagnitude,
                                         radialDirection[1] * pidForceMagnitude,
                                         radialDirection[2] * pidForceMagnitude };

    return radialForce;
}

void ECE_UAV::physicsLoop()
{
    while (m_isMoving.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::array<float, 3>        uavForce = { 0.0f, 0.0f, 0.0f };
        std::lock_guard<std::mutex> lock(m_kinematicsMutex);
        auto                        startTime   = std::chrono::high_resolution_clock::now();
        auto                        endTime     = std::chrono::high_resolution_clock::now();
        auto                        elapsedTime = std::chrono::duration<float>(endTime - startTime);
        // we need the state machine here
        UAVState currentState = m_state.load();
        switch (currentState)
        {
            case ECE_UAV::UAVState::WAITING:
            {
                std::cout << "hello";
                break;
            }
            case ECE_UAV::UAVState::LAUNCHING:
            {
                float distance = this->distanceToSphereCenter();
                if (distance <= SPHERERADIUS)
                {
                    m_state          = ECE_UAV::UAVState::ORBITING;
                    m_orbitEntryTime = std::chrono::high_resolution_clock::now();
                }
                break;
            }
            case ECE_UAV::UAVState::ORBITING:
            {
                std::array<float, 3> radialForce     = calculateOrbitForce();
                std::array<float, 3> tangentialForce = {
                    ((rand() % 100) / 100.0f) * 5.0f,
                    ((rand() % 100) / 100.0f) * 5.0f,
                    ((rand() % 100) / 100.0f) * 5.0f,
                };

                uavForce[0] = radialForce[0] + tangentialForce[0];
                uavForce[1] = radialForce[1] + tangentialForce[1];
                uavForce[2] = radialForce[2] + tangentialForce[2];

                auto currTime = std::chrono::high_resolution_clock::now();
                if (currTime - startTime > std::chrono::milliseconds(60))
                {
                    m_state = ECE_UAV::UAVState::DONE;
                }
                break;
            }
            case ECE_UAV::UAVState::DONE:
            {
                uavForce = { 0.0f, 0.0f, 0.0f };
                break;
            }
                applyKinematics(uavForce);
        }

        if (elapsedTime.count() < DELTAT)
        {
            std::this_thread::sleep_for(std::chrono::duration<float>(DELTAT - elapsedTime.count()));
        }
    }
}

void ECE_UAV::applyKinematics(const std::array<float, 3> uavForce)
{
    // first we need to determine if the force needs to be clamped
    std::array<float, 3> netForce = clampMagnitude(uavForce, MAXFORCE);
    netForce[1] += GRAVITY;

    m_acceleration[0] = netForce[0] / MASS;
    m_acceleration[1] = netForce[1] / MASS;
    m_acceleration[2] = netForce[2] / MASS;

    // adjusting the position of the uav
    m_position[0]
        = m_position[0] + (m_velocity[0] * DELTAT) + (0.5 * m_acceleration[0] * m_acceleration[0]);
    m_position[1]
        = m_position[1] + (m_velocity[1] * DELTAT) + (1.5 * m_acceleration[1] * m_acceleration[1]);
    m_position[2]
        = m_position[2] + (m_velocity[2] * DELTAT) + (2.5 * m_acceleration[2] * m_acceleration[2]);

    // adjust the velocity of the uav
    m_velocity[0] = m_velocity[0] + (m_acceleration[0] * DELTAT);
    m_velocity[1] = m_velocity[1] + (m_acceleration[1] * DELTAT);
    m_velocity[2] = m_velocity[2] + (m_acceleration[2] * DELTAT);
}

void ECE_UAV::start()
{
    m_isMoving = true;
    m_movement = std::thread(threadFunction, this);
}

void ECE_UAV::stop()
{
    m_isMoving = false;
    if (m_movement.joinable())
    {
        m_movement.join();
    }
}

float ECE_UAV::distanceToSphereCenter() const
{
    // determine magnitude to center
    return std::sqrt(std::pow(m_position[0] - SPHERE_CENTER[0], 2)
                     + std::pow(m_position[1] - SPHERE_CENTER[1], 2)
                     + std::pow(m_position[2] - SPHERE_CENTER[2], 2));
}
