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
    m_state        = ECE_UAV::UAVState::WAITING;
}

// accesor methods
std::array<float, 3> ECE_UAV::getPosition() const
{
    std::lock_guard<std::mutex> lock(m_kinematicsMutex);
    return m_position;
}

std::atomic<ECE_UAV::UAVState>& ECE_UAV::getState() { return m_state; }

float ECE_UAV::getForceMag() const
{
    return std::sqrt(std::pow(m_acceleration[0] / MASS, 2) + std::pow(m_acceleration[1] / MASS, 2)
                     + std::pow(m_acceleration[2] / MASS, 2));
}

void ECE_UAV::setVelocity(const std::array<float, 3>& velocity)
{
    m_velocity[0] = velocity[0];
    m_velocity[1] = velocity[1];
    m_velocity[2] = velocity[2];
}

float ECE_UAV::getVelMag() const
{
    return std::sqrt(std::pow(m_velocity[0], 2) + std::pow(m_velocity[1], 2)
                     + std::pow(m_velocity[2], 2));
}

float ECE_UAV::distanceTo(const ECE_UAV& otherUAV) const
{
    return std::sqrt(std::pow(m_position[0] - otherUAV.m_position[0], 2)
                     + std::pow(m_position[1] - otherUAV.m_position[1], 2)
                     + std::pow(m_position[2] - otherUAV.m_position[2], 2));
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

    if (std::abs(error) < 2.0f)
    {
        m_pidIntegral += error * DELTAT;
    }

    if (m_pidIntegral > 5.0f) m_pidIntegral = 5.0f;
    if (m_pidIntegral < -5.0f) m_pidIntegral = -5.0f;

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
    auto simStartTime = std::chrono::high_resolution_clock::now();

    while (m_isMoving.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::array<float, 3> uavForce       = { 0.0f, 0.0f, 0.0f };
        auto                 frameStartTime = std::chrono::high_resolution_clock::now();
        auto elapsedTime = std::chrono::duration<float>(frameStartTime - simStartTime);
        // we need the state machine here
        {
            std::lock_guard<std::mutex> lock(m_kinematicsMutex);
            UAVState                    currentState = m_state.load();
            switch (currentState)
            {
                case ECE_UAV::UAVState::WAITING:
                {
                    if (elapsedTime.count() > 5.0f)
                    {
                        m_state = ECE_UAV::UAVState::LAUNCHING;
                    }
                    uavForce = { 0.0f, 0.0f, 0.0f };
                    break;
                }
                case ECE_UAV::UAVState::LAUNCHING:
                {
                    float                distance  = this->distanceToSphereCenter();
                    std::array<float, 3> seekForce = calculateSeekForce(SPHERE_CENTER, 2.0f);

                    float kp_launch = 20.0f;
                    uavForce[0] = seekForce[0] * kp_launch;
                    uavForce[1] = seekForce[1] * kp_launch;
                    uavForce[2] = seekForce[2] * kp_launch;

                    uavForce[2] += 10; // compensate for the gravity

                    if (distance <= SPHERERADIUS)
                    {
                        m_state          = ECE_UAV::UAVState::ORBITING;
                        m_orbitEntryTime = std::chrono::high_resolution_clock::now();
                        m_velocity = {0, 0, 0}; // reset the velocity
                    }
                    break;
                }
                case ECE_UAV::UAVState::ORBITING:
                {
                    std::array<float, 3> radialForce     = calculateOrbitForce();
                    std::array<float, 3> tangentialForce = {
                        ((rand() % 200 - 100) / 100.0f) * 5.0f,
                        ((rand() % 200 - 100) / 100.0f) * 5.0f,
                        ((rand() % 200 - 100) / 100.0f) * 5.0f,
                    };

                    uavForce[0] = radialForce[0] + tangentialForce[0];
                    uavForce[1] = radialForce[1] + tangentialForce[1];
                    uavForce[2] = radialForce[2] + tangentialForce[2];

                    auto currTime      = std::chrono::high_resolution_clock::now();
                    auto orbitDuration = std::chrono::duration<float>(currTime - m_orbitEntryTime);
                    if (orbitDuration.count() >= 60.0f)
                    {
                        std::cout << "finished physics " << std::endl;
                        m_state = ECE_UAV::UAVState::DONE;
                    }
                    break;
                }
                case ECE_UAV::UAVState::DONE:
                {
                    std::cout << "done" << std::endl;
                    uavForce = { 0.0f, 0.0f, 0.0f };
                    m_velocity = {0.0f, 0.0f, 0.0f};
                    m_acceleration = {0.0f, 0.0f, 0.0f};
                    break;
                }
            }

            if (currentState != UAVState::WAITING && currentState != UAVState::DONE)
            {
                applyKinematics(uavForce);
            }

            auto frameEndTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> frameDuration = frameEndTime - frameStartTime;
            if (frameDuration.count() < DELTAT)
            {
                std::this_thread::sleep_for(
                    std::chrono::duration<float>(DELTAT - frameDuration.count()));
            }
        }
    }
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

void ECE_UAV::applyKinematics(const std::array<float, 3> uavForce)
{
    // first we need to determine if the force needs to be clamped
    std::array<float, 3> netForce = clampMagnitude(uavForce, MAXFORCE);
    netForce[2] += GRAVITY;

    m_acceleration[0] = netForce[0] / MASS;
    m_acceleration[1] = netForce[1] / MASS;
    m_acceleration[2] = netForce[2] / MASS;

    // adjusting the position of the uav
    m_position[0] = m_position[0] + (m_velocity[0] * DELTAT)
                    + (0.5 * m_acceleration[0] * std::pow(DELTAT, 2));
    m_position[1] = m_position[1] + (m_velocity[1] * DELTAT)
                    + (0.5 * m_acceleration[1] * std::pow(DELTAT, 2));
    m_position[2] = m_position[2] + (m_velocity[2] * DELTAT)
                    + (0.5 * m_acceleration[2] * std::pow(DELTAT, 2));

    // adjust the velocity of the uav
    m_velocity[0] = m_velocity[0] + (m_acceleration[0] * DELTAT);
    m_velocity[1] = m_velocity[1] + (m_acceleration[1] * DELTAT);
    m_velocity[2] = m_velocity[2] + (m_acceleration[2] * DELTAT);
}

std::array<float, 3> ECE_UAV::clampMagnitude(std::array<float, 3> v, float maxMag)
{
    float vMag = std::sqrt(std::pow(v[0], 2) + std::pow(v[1], 2) + std::pow(v[2], 2));
    if (vMag > maxMag && vMag > 0.001f)
    {
        float scaleFactor = maxMag / vMag;
        v[0] = v[0] * scaleFactor;
        v[1] = v[1] * scaleFactor;
        v[2] = v[2] * scaleFactor;
    }
    return v;
}


std::array<float, 3> ECE_UAV::calculateSeekForce(const std::array<float, 3>& target, float maxSpeed)
{
    std::array<float, 3> desiredVelocity = {
        target[0] - m_position[0],
        target[1] - m_position[1],
        target[2] - m_position[2],
    };

    float dist = std::sqrt(std::pow(desiredVelocity[0], 2) + std::pow(desiredVelocity[1], 2)
                           + std::pow(desiredVelocity[2], 2));

    if (dist > 0.001f)
    {
        desiredVelocity[0] = (desiredVelocity[0] / dist) * maxSpeed;
        desiredVelocity[1] = (desiredVelocity[1] / dist) * maxSpeed;
        desiredVelocity[2] = (desiredVelocity[2] / dist) * maxSpeed;
    }

    std::array<float, 3> steeringForce = {
        desiredVelocity[0] - m_velocity[0],
        desiredVelocity[1] - m_velocity[1],
        desiredVelocity[2] - m_velocity[2],
    };

    return steeringForce;
}
