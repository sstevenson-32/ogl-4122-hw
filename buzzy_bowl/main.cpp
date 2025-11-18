#include "ECE_UAV.h"
#include <iostream>
#include <thread>
#include <vector>
#include <iomanip>

int main() {
    // 1. Setup: Place UAV on the ground at 25 yards (approx 22 meters)
    // Initial Velocity and Acceleration are 0.
    ECE_UAV myUAV({22.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});

    std::cout << "Time(s), State, X, Y, Z, VelMag, DistToSphereCenter" << std::endl;

    // 2. Start the UAV thread
    myUAV.start();

    // 3. Monitoring Loop (Runs for 70 seconds to capture full sequence)
    // The main thread polls every 100ms (0.1s)
    for (int i = 0; i < 1000; ++i) {
        
        // Get thread-safe data
        auto pos = myUAV.getPosition();
        float dist = myUAV.distanceToSphereCenter(); // Make this public for testing or calc manually
        
        // Determine state string
        // (Assuming you made m_state accessible or added a getState() method)
        // You might need to cast the enum to int if you don't want to write a switch
        int stateInt = static_cast<int>(myUAV.getState().load()); 

        // Print CSV format: Time, State, X, Y, Z, Dist
        std::cout << std::fixed << std::setprecision(2) 
                  << (i * 0.1f) << ", " 
                  << stateInt << ", "
                  << pos[0] << ", " 
                  << pos[1] << ", " 
                  << pos[2] << ", "
                  << myUAV.getVelMag() << ", "
                  << dist << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 4. Cleanup
    myUAV.stop();
    return 0;
}
