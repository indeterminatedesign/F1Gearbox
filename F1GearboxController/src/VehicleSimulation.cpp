#include <VehicleSimulation.h>

static const float gearRatios[] =
    {0,
         2.0,
         1.57,
         1.25,
         1.0,
         0.93,
         0.86,
         0.73};
;

 const float finalDriveCoefficent = 3.85;
 const float totalDragProduct = 3.5; //Not an actual drag coefficent, just use to simulate drag increasing with speed and acts as product of all the other factors
 const float totalMassConstant = 25;
 const float powertrainFriction = .6;
 const int idleRPM = 450;
 const int revLimiter = 1200;
 const int computeInterval = 100000; //Time in microseconds between recalculating a target rpm

 const int enginePowerBins[15][2] = {
    {0, 0},
    {100, 0},
    {200, 0},
    {300, 0},
    {400, 0},
    {500, 35},
    {600, 250},
    {700, 575},
    {800, 625},
    {900, 725},
    {1000, 850},
    {1100, 930},
    {1200, 935},
    {1300, 850},
    {1400, 825}};

unsigned long previousSimulateMicros = 0;
int previousGear;

VehicleSimulation::~VehicleSimulation()
{
}

int VehicleSimulation::Simulate(float percentThrottle, int inputLayRPM, int currentGear)
{
    Serial.println("Entered Simulate");

    unsigned long currentMicros = micros();
    unsigned long dt = currentMicros - previousSimulateMicros; //Time elapsed since the vehicle was last simulated
    int newTargetRPM = 0;
    if (dt > computeInterval && inputLayRPM > 1 && currentGear > 0)
    {
        float effectiveGearRatio = gearRatios[currentGear] * finalDriveCoefficent;
        //final drive takes into account tire diameter
        int vehicleSpeed = inputLayRPM / effectiveGearRatio;

        //Effectively adding a floor to the percent throttle
        float effectivePercentThrottle = constrain(percentThrottle, 1, 100);

        //Force available at the current RPM assuming full throttle
        float totalAvailableCurrentForce = ComputeEngineForce(inputLayRPM);

        //Current power at the wheels based on the throttle position & gear ratio
        float currentEffectiveForce = effectivePercentThrottle * totalAvailableCurrentForce * effectiveGearRatio;
        float totalPowerTrainFriction = powertrainFriction * inputLayRPM;

        //A = F/M Thus engine powe r/some mass constant
        // Force of engine at the wheels is dependent on gear ratio
        //Net Decel being experienced based on speed (product of all drag area and coeficients) * vehicleSpeed^2
        float netAcceleration = .0003 * ((currentEffectiveForce / totalMassConstant) - totalPowerTrainFriction - (totalDragProduct * pow(vehicleSpeed, 2)));

        //New target rpm is based on current rpm + delta T * netacceleration, based on V = V0 + AdT
        newTargetRPM = inputLayRPM + dt * netAcceleration;

        previousSimulateMicros = currentMicros;
        // Return newly Calculated target RPM
    }

    //Constrain RPM between idle and rev limiter
    newTargetRPM = constrain(newTargetRPM, idleRPM, revLimiter);

    previousGear = currentGear;

    return newTargetRPM;
}

float VehicleSimulation::ComputeEngineForce(int layshaftRPM)
{
    //Interpolate the current engine power based on RPMs between two bins
    int lowerBinIndex = 0;
    int upperBinIndex = 0;
    for (int i = 0; i < 15; i++)
    {
        if (enginePowerBins[i][0] >= layshaftRPM)
        {
            lowerBinIndex = i - 1;
            upperBinIndex = i;
            break;
        }
    }

    //Use map function to linear interpolate the power between the two bins
    return map(layshaftRPM, enginePowerBins[lowerBinIndex][0], enginePowerBins[upperBinIndex][0], enginePowerBins[lowerBinIndex][ 1], enginePowerBins[upperBinIndex][ 1]);
}

//Cut Throttle based on PWM value
float VehicleSimulation::ThrottleCut()
{
    return 0.9;
}

int RevMatch(int currentGear, int newGear, int layRPM)
{
    //Account for neutral which up or down has no rev match
    if (currentGear == 0 || newGear == 0)
    {
        return 1;
    }
    else
    {
        return layRPM * gearRatios[newGear] / gearRatios[currentGear];
    }
}