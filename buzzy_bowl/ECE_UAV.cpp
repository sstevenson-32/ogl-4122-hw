/*
Author: Aaron Marlin, Samir Stevenson
Class: ECE4122
Last Date Modified:

Description:
Implementation of the ECE_UAV class for the buzzy_bowl simulation
*/
#include "ECE_UAV.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <utility>

// threaded function that will be used by the thread to call the physicsLoop function
// takes a pointer to a ECE_UAV object
void threadFunction(ECE_UAV* pUAV) { pUAV->physicsLoop(); }

// ECE_UAV constructor that takes a position array (x, y, z) and initiailizes member position to
// position and initiailizes velocity and accelearation to 0
ECE_UAV::ECE_UAV(const std::array<float, 3>& position)
{
    m_position     = position;
    m_velocity     = { 0.0f, 0.0f, 0.0f };
    m_acceleration = { 0.0f, 0.0f, 0.0f };
    m_state        = ECE_UAV::UAVState::WAITING;
}

// custom destructor for the ECE_UAV used to ensure the threading is stopped if a unique pointer
// goes out of scope
ECE_UAV::~ECE_UAV() { stop(); }

// return the current position of the UAV
std::array<float, 3> ECE_UAV::getPosition() const
{
    // grabs ownership of the m_kinematicsMutex to ensure our position is not randomly changing due
    // to a collision
    std::lock_guard<std::mutex> lock(m_kinematicsMutex);
    return m_position;
}

// return the current velocity of the UAV
std::array<float, 3> ECE_UAV::getVelocity() const
{
    // grabs ownership of the m_kinematicsMutex to ensure our velocity is not randomly changing due
    // to a collision
    std::lock_guard<std::mutex> lock(m_kinematicsMutex);
    return m_velocity;
}

// swap the velocity of two uav's after a collision is detected in our main program
// takes a reference to the uav we collided with
void ECE_UAV::swapVelocity(ECE_UAV& otherUAV)
{
    // grabs ownership of the m_kinematicsMutex to ensure our velocity is a stable value
    std::lock_guard<std::mutex> lock(m_kinematicsMutex);
    // swapping the two velocities
    std::swap(m_velocity, otherUAV.m_velocity);
}

// return the current state of the UAV
std::atomic<ECE_UAV::UAVState>& ECE_UAV::getState() { return m_state; }

// return the magnitude of the force vector
float ECE_UAV::getForceMag() const
{
    // equation for magnitude (Pythagorean theorem)
    return std::sqrt(std::pow(m_acceleration[0] / MASS, 2) + std::pow(m_acceleration[1] / MASS, 2)
                     + std::pow(m_acceleration[2] / MASS, 2));
}

// set the velocity of our UAV to a new vector
void ECE_UAV::setVelocity(const std::array<float, 3>& velocity)
{
    m_velocity[0] = velocity[0];
    m_velocity[1] = velocity[1];
    m_velocity[2] = velocity[2];
}

// return the magnitude of the velocity vector
float ECE_UAV::getVelMag() const
{
    // equation for magnitude (Pythagorean theorem)
    return std::sqrt(std::pow(m_velocity[0], 2) + std::pow(m_velocity[1], 2)
                     + std::pow(m_velocity[2], 2));
}

// return the distance between two UAVs. Used to check for collisions
// takes in a reference to another UAV
float ECE_UAV::distanceTo(const ECE_UAV& otherUAV) const
{
    // returns the magnitude of the distance between the bodies of the two ECE_UAVs
    return std::sqrt(std::pow(m_position[0] - otherUAV.m_position[0], 2)
                     + std::pow(m_position[1] - otherUAV.m_position[1], 2)
                     + std::pow(m_position[2] - otherUAV.m_position[2], 2))
           - (2 * UAVRADIUS);
}

// return the force for the orbiting portion of the ECE_UAVs flight trajectory
std::array<float, 3> ECE_UAV::calculateOrbitForce()
{
    // get a vector from the center of the sphere to the UAV
    std::array<float, 3> vectorToUAV = { m_position[0] - SPHERE_CENTER[0],
                                         m_position[1] - SPHERE_CENTER[1],
                                         m_position[2] - SPHERE_CENTER[2] };

    // determning the magnitude of that vector
    float currentDistance
        = std::sqrt(vectorToUAV[0] * vectorToUAV[0] + vectorToUAV[1] * vectorToUAV[1]
                    + vectorToUAV[2] * vectorToUAV[2]);

    // if our current distance is very low we'll determine the radial direction by dividing our
    // vector from the center by the magnitude of the distance vector
    std::array<float, 3> radialDirection = { 0.0f, 0.0f, 0.0f };
    if (currentDistance > 0.001f)
    {
        radialDirection[0] = vectorToUAV[0] / currentDistance;
        radialDirection[1] = vectorToUAV[1] / currentDistance;
        radialDirection[2] = vectorToUAV[2] / currentDistance;
    }

    // calculate the PID
    float error  = SPHERERADIUS - currentDistance;
    float p_term = m_Kp * error;

    // P portion of PID
    if (std::abs(error) < 2.0f)
    {
        m_pidIntegral += error * DELTAT;
    }

    if (m_pidIntegral > 5.0f)
        m_pidIntegral = 5.0f;
    if (m_pidIntegral < -5.0f)
        m_pidIntegral = -5.0f;

    // I portion of the PID
    float i_term = m_Ki * m_pidIntegral;

    float derivative   = (error - m_pidPreviousError) / DELTAT;
    m_pidPreviousError = error;

    // D term for the PID
    float d_term = m_Kd * derivative;

    float pidForceMagnitude = p_term + i_term + d_term;

    // get the radialForce by multiplying our direction by the force
    std::array<float, 3> radialForce = { radialDirection[0] * pidForceMagnitude,
                                         radialDirection[1] * pidForceMagnitude,
                                         radialDirection[2] * pidForceMagnitude };

    return radialForce;
}

// This method updated the activity of the ECE_UAV based on the current state it is at
void ECE_UAV::physicsLoop()
{
    // remember the start time for our simulation
    auto simStartTime = std::chrono::high_resolution_clock::now();

    // if we are moving
    while (m_isMoving.load())
    {
        // delay the UAV every cycle
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::array<float, 3> uavForce = { 0.0f, 0.0f, 0.0f };
        // keep track of the startTime for this iterations
        auto frameStartTime = std::chrono::high_resolution_clock::now();
        // keep track of the difference in time between the start of our simulation and the current
        // iteration
        auto elapsedTime = std::chrono::duration<float>(frameStartTime - simStartTime);
        // we need the state machine here
        {
            // grab the lock
            std::lock_guard<std::mutex> lock(m_kinematicsMutex);
            // get the current state of the ECE_UAV
            UAVState currentState = m_state.load();

            // based on the state of the ECE_UAV...
            switch (currentState)
            {
                // if we are in the waiting state...
                case ECE_UAV::UAVState::WAITING:
                {
                    // check to see if 5 seconds have passed, and if it has, start launching
                    if (elapsedTime.count() > 5.0f)
                    {
                        m_state = ECE_UAV::UAVState::LAUNCHING;
                    }
                    // make sure to keep the force at 0 if we are still waiting
                    uavForce = { 0.0f, 0.0f, 0.0f };
                    break;
                }
                // if we are launching now...
                case ECE_UAV::UAVState::LAUNCHING:
                {
                    // calculate the distance to the sphere
                    float distance = this->distanceToSphereCenter();
                    // calculate the force to move towards the center
                    std::array<float, 3> seekForce = calculateSeekForce(SPHERE_CENTER, 2.0f);

                    // accelerate the launching speed so that we aren't moving incredibly slow
                    float kp_launch = 20.0f;
                    uavForce[0]     = seekForce[0] * kp_launch;
                    uavForce[1]     = seekForce[1] * kp_launch;
                    uavForce[2]     = seekForce[2] * kp_launch;

                    // account for gravity in the Y plane
                    uavForce[2] += 10;  // compensate for the gravity

                    // if we've reached the 10m radius of the sphere, we can start orbiting and
                    // reset the velocity
                    if (distance <= SPHERERADIUS)
                    {
                        // start orbiting
                        m_state = ECE_UAV::UAVState::ORBITING;
                        // keep track of the time we start orbiting to count to 60s
                        m_orbitEntryTime = std::chrono::high_resolution_clock::now();
                        m_velocity       = { 0, 0, 0 };  // reset the velocity
                    }
                    break;
                }
                // if we are in the orbiting state
                case ECE_UAV::UAVState::ORBITING:
                {
                    // calculate the force for orbiting the sphere
                    std::array<float, 3> radialForce     = calculateOrbitForce();
                    // calculate random force
                    std::array<float, 3> tangentialForce = {
                        ((rand() % 200 - 100) / 100.0f) * 5.0f,
                        ((rand() % 200 - 100) / 100.0f) * 5.0f,
                        ((rand() % 200 - 100) / 100.0f) * 5.0f,
                    };

                    // apply the radial force and random force onto the UAV
                    uavForce[0] = radialForce[0] + tangentialForce[0];
                    uavForce[1] = radialForce[1] + tangentialForce[1];
                    uavForce[2] = radialForce[2] + tangentialForce[2];

                    // get the current time
                    auto currTime      = std::chrono::high_resolution_clock::now();
                    // calculate how long we have been orbiting
                    auto orbitDuration = std::chrono::duration<float>(currTime - m_orbitEntryTime);
                    // if we've been orbiting for 60 seconds we can stop and move to the done phase
                    if (orbitDuration.count() >= 60.0f)
                    {
                        std::cout << "finished physics " << std::endl;
                        m_state = ECE_UAV::UAVState::DONE;
                        m_velocity = {0, 0, 0};
                    }
                    break;
                }
                // if we are done... set everything to zero
                case ECE_UAV::UAVState::DONE:
                {
                    std::cout << "done" << std::endl;
                    uavForce       = { 0.0f, 0.0f, 0.0f };
                    m_velocity     = { 0.0f, 0.0f, 0.0f };
                    m_acceleration = { 0.0f, 0.0f, 0.0f };
                    break;
                }
            }

            // if we are not waiting and we are not finished (aka we are moving)
            if (currentState != UAVState::WAITING && currentState != UAVState::DONE)
            {
                // update the kinematics of the UAV on those iterations
                applyKinematics(uavForce);
            }

            // keep track of when this iteration finished
            auto                         frameEndTime  = std::chrono::high_resolution_clock::now();
            // determine the total time of this iteration
            std::chrono::duration<float> frameDuration = frameEndTime - frameStartTime;
            // if the total time was less than deltat, then we will sleep until deltat has been reached to ensure our threads our synchronized
            if (frameDuration.count() < DELTAT)
            {
                std::this_thread::sleep_for(
                    std::chrono::duration<float>(DELTAT - frameDuration.count()));
            }
        }
    }
}

// start the movement of the ECE_UAV associated with the thread
void ECE_UAV::start()
{
    m_isMoving = true;
    // create a thread calling the threadFunction on the current ECE_UAV
    m_movement = std::thread(threadFunction, this);
}

// stop the movement of the ECE_UAV
void ECE_UAV::stop()
{
    m_isMoving = false;
    // if our thread is joinable
    if (m_movement.joinable())
    {
        // have the thread join back to the main thread
        m_movement.join();
    }
}

// return the distance to the center of the sphere
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
    // account for gravity
    netForce[2] += GRAVITY;

    // A = F/M
    m_acceleration[0] = netForce[0] / MASS;
    m_acceleration[1] = netForce[1] / MASS;
    m_acceleration[2] = netForce[2] / MASS;

    // adjusting the position of the uav, x = x_0 + vt + 1/2at^2
    m_position[0] = m_position[0] + (m_velocity[0] * DELTAT)
                    + (0.5 * m_acceleration[0] * std::pow(DELTAT, 2));
    m_position[1] = m_position[1] + (m_velocity[1] * DELTAT)
                    + (0.5 * m_acceleration[1] * std::pow(DELTAT, 2));
    m_position[2] = m_position[2] + (m_velocity[2] * DELTAT)
                    + (0.5 * m_acceleration[2] * std::pow(DELTAT, 2));

    // adjust the velocity of the uav v = v_0 + at
    m_velocity[0] = m_velocity[0] + (m_acceleration[0] * DELTAT);
    m_velocity[1] = m_velocity[1] + (m_acceleration[1] * DELTAT);
    m_velocity[2] = m_velocity[2] + (m_acceleration[2] * DELTAT);
}

// clamp the magnitude of the force so it does not exceed 20N
// takes the current uavForce and maximum force
std::array<float, 3> ECE_UAV::clampMagnitude(std::array<float, 3> force, float maxMag)
{
    // calculate magnitude of force
    float fMag = std::sqrt(std::pow(force[0], 2) + std::pow(force[1], 2) + std::pow(force[2], 2));
    // if our fMag is larger than 20N and positive
    if (fMag > maxMag && fMag > 0.001f)
    {
        // then scale normalize our force
        float scaleFactor = maxMag / fMag;
        force[0]              = force[0] * scaleFactor;
        force[1]              = force[1] * scaleFactor;
        force[2]              = force[2] * scaleFactor;
    }
    return force;
}


// return the force to seek the center of the sphere
// takes an array of the point we are trying to reach and the max velocity we can travel
std::array<float, 3> ECE_UAV::calculateSeekForce(const std::array<float, 3>& target, float maxSpeed)
{
    // determine the velocity we should have
    std::array<float, 3> desiredVelocity = {
        target[0] - m_position[0],
        target[1] - m_position[1],
        target[2] - m_position[2],
    };

    // calculate the magnitude of the distance vector
    float dist = std::sqrt(std::pow(desiredVelocity[0], 2) + std::pow(desiredVelocity[1], 2)
                           + std::pow(desiredVelocity[2], 2));

    // if the distance is positive
    if (dist > 0.001f)
    {
        // then normalize our desiredVelocity 
        desiredVelocity[0] = (desiredVelocity[0] / dist) * maxSpeed;
        desiredVelocity[1] = (desiredVelocity[1] / dist) * maxSpeed;
        desiredVelocity[2] = (desiredVelocity[2] / dist) * maxSpeed;
    }

    // determine the force we need to reach that desired velocity
    std::array<float, 3> steeringForce = {
        desiredVelocity[0] - m_velocity[0],
        desiredVelocity[1] - m_velocity[1],
        desiredVelocity[2] - m_velocity[2],
    };

    // return that force
    return steeringForce;
}
