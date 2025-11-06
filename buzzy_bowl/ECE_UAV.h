#include <thread>
#include <array>


class ECE_UAV {
private:
    const float MASS = 1.00;
    const float COLLISIONCONSTANT = 1.0;
    const float MAXFORCE = 20.0;
    const float GRAVITY = -9.8;
    const float DELTAT = 0.05;

    float m_position[3] = {};
    float m_velocity[3] = {};
    float m_acceleration[3] = {};

    std::thread movement;

public:

    ECE_UAV(const std::array<float, 3> &position, const std::array<float, 3> &velocity, const std::array<float, 3> &m_acceleration);

    void start();
    void stop();
    float distanceToCenter();
    float getForceMag(); // return the magnitude of the force vector
    float getVelMag(); // return magnitude of velocity vector 
    void setPosition(const std::array<float,3> &position); // set m_position to this array;
    void setVelocity(const std::array<float,3> &velocity); // set m_position to this array;
    void setAcceleration(const std::array<float,3> &acceleration); // set m_position to this array;
    void update(); // update member variables     
    bool checkCollision(const ECE_UAV &otherUAV);
    
};
