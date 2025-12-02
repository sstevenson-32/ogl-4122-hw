/*
Author: Aaron Marlin, Samir Stevenson
Class: ECE4122
Last Date Modified:

Description:
Header file describing the ECE_UAV class for the buzzy_bowl simulation
*/

#include <array>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

class ECE_UAV
{
public:
    // enum to describe the current state of the uav
    enum class UAVState
    {
        WAITING,    // waiting means the uav is not moving and has not started its play path
        LAUNCHING,  // launching is the beginning portion of the uav's movement where they move
                    // towards the center
        ORBITING,   // the uavs are now moving around the sphere in the center randomly
        DONE        // the uavs are finished moving
    };

    // constructor for the UAV object that takes in it's starting position, velocity and
    // acceleration
    ECE_UAV(const std::array<float, 3>& position);
    ~ECE_UAV();

    // start or stop the thread
    void start();
    void stop();

    // get the position of the uav
    // used for main render loop
    std::array<float, 3> getPosition() const;

    // check the status of the UAV
    std::atomic<ECE_UAV::UAVState>& getState();
    bool isDone() const;  // helper to check if the state of the UAV is DONE


    // float distanceToCenter() const;
    float getForceMag() const;  // return the magnitude of the force vector
                                //
    std::array<float, 3> getVelocity() const;

    void swapVelocity(ECE_UAV& otherUAV);

    // used for elastic collisions
    void  setVelocity(const std::array<float, 3>& velocity);  // set m_position to this array;
    float getVelMag() const;                                  // return magnitude of velocity vector
    float distanceTo(const ECE_UAV& otherUAV) const;
    float distanceToSphereCenter() const;
    // void setPosition(const std::array<float, 3>& position);  // set m_position to this array;
    // void setAcceleration(
    //     const std::array<float, 3>& acceleration);  // set m_position to this array;
    // void  update();  // update member variables

private:
    const float MASS              = 1.00;
    const float COLLISIONCONSTANT = 1.0;
    const float MAXFORCE          = 20.0;
    const float GRAVITY           = -10.0;
    const float DELTAT            = 0.01f;
    const float UAVRADIUS         = 0.5f;

    const std::array<float, 3> SPHERE_CENTER = { 0.0f, 0.0f, 50.0f };
    const float                SPHERERADIUS  = 10.0f;

    const float m_Kp = 150.0f;
    const float m_Ki = 20.0f;
    const float m_Kd = 40.0f;

    float m_pidIntegral      = 0.0f;
    float m_pidPreviousError = 0.0f;

    std::array<float, 3> m_position;
    std::array<float, 3> m_velocity;
    std::array<float, 3> m_acceleration;
    std::array<float, 3> m_randTarget = { 1.0f, 0.0f, 0.0f };


    std::thread           m_movement;
    std::atomic<bool>     m_isMoving;
    std::atomic<UAVState> m_state;

    mutable std::mutex m_kinematicsMutex;

    std::chrono::high_resolution_clock::time_point m_startTime;
    std::chrono::high_resolution_clock::time_point m_orbitEntryTime;

    bool slowingDown = false;

    void                 physicsLoop();
    void                 applyKinematics(const std::array<float, 3> uavForce);
    std::array<float, 3> calculateSeekForce(const std::array<float, 3>& target, float maxSpeed);
    std::array<float, 3> calculateOrbitForce();
    std::array<float, 3> clampMagnitude(std::array<float, 3> v, float maxMag);

    friend void threadFunction(ECE_UAV* pUAV);
};
