#include <VehicleSimulation.h>

static const float gearRatios[] =
    {
        0,
        2.0,
        1.57,
        1.25,
        1.0,
        0.93,
        0.86,
        0.57};

static const float finalDriveCoefficent = 3.38;
static const float totalDragProduct = 2.5; //Not an actual drag coefficent, just use to simulate drag increasing with speed and acts as product of all the other factors
static const float totalMassConstant = 2000;
static const int idleRPM = 300;
static const int revLimiter = 1200;
static const int computeInterval = 50000; //Time in microseconds between recalculating a target rpm

static const int enginePowerBins[15][2] = {
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

VehicleSimulation::~VehicleSimulation()
{
}

int VehicleSimulation::Simulate(float percentThrottle, int inputLayRPM, int inputMainRPM, int currentGear)
{
    Serial.println("Entered Simulate");

    unsigned long currentMicros = micros();
    unsigned long dt = currentMicros - previousSimulateMicros; //Time elapsed since the vehicle was last simulated
    int newTargetRPM = 0;
    if (dt > computeInterval && inputLayRPM > 1 && currentGear > 0)
    {
        int effectiveGearRatio = gearRatios[currentGear] * finalDriveCoefficent;
        int vehicleSpeed = inputMainRPM / effectiveGearRatio;                                //final drive takes into account tire diameter
        float effectivePercentThrottle = constrain(percentThrottle, 1, 100);                 //Effectively adding a floor to the percent throttle
        float totalAvailableCurrentPower = ComputeEnginePower(inputLayRPM);                    //Power available at the current RPM assuming full throttle
        float currentEffectivePower = effectivePercentThrottle * totalAvailableCurrentPower; //current power based on the throttle position

        //A = F/M Thus engine powe r/some mass constant
        // Power of engine at the wheels is dependent on gear ratio
        //Net Decel being experienced based on speed (product of all drag area and coeficients) * vehicleSpeed^2
        float netAcceleration = (currentEffectivePower * effectiveGearRatio) / totalMassConstant - (totalDragProduct * pow(vehicleSpeed, 2));

        //New target rpm is based on current rpm + delta T * netacceleration, based on V = V0 + AdT
        newTargetRPM = inputLayRPM + dt * netAcceleration;

        previousSimulateMicros = currentMicros;
        // Return newly Calculated target RPM
    }

    //Constrain RPM between idle and rev limiter
    newTargetRPM = constrain(newTargetRPM, idleRPM, revLimiter);

    return newTargetRPM;
}

float VehicleSimulation::ComputeEnginePower(int layshaftRPM)
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
    return map(layshaftRPM, enginePowerBins[lowerBinIndex][0], enginePowerBins[upperBinIndex][0], enginePowerBins[lowerBinIndex][1], enginePowerBins[upperBinIndex][1]);
}