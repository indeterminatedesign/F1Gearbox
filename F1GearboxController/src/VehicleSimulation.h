#include <Arduino.h>

class VehicleSimulation
{
private:
    
        float ComputeEnginePower(int layshaftRPM);
    

public:
    //VehicleSimulation(/* args */);
    ~VehicleSimulation();
    int percentThrottle;
    int outputLayRPM;
    int inputLayRPM;
    int inputMainRPM;
    int currentGear;
    static const int idleRPM = 300;
    int Simulate(float percentThrottle, int inputLayRPM, int inputMainRPM, int currentGear);
    int RevMatch(int inputLayRPM, int inputMainRPM, int currentGear, int newGear);
};

/*
VehicleSimulation::VehicleSimulation(//args)
{
}
*/
