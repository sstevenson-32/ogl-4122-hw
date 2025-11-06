#include <thread>


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

    ECE_UAV(float position[3], float velocity[3], float m_acceleration[3]);

    void start();
    void stop();
    float distanceToCenter();
    float getForceMag();


    
}
