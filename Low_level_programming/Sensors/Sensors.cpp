#include "Sensors.h"

// class constructor
Sensors::Sensors(Nucleo_Encoder_16_bits *encoder_A,
                 Nucleo_Encoder_16_bits *encoder_B,
                 AnalogIn *force_sensor_A,
                 AnalogIn *force_sensor_B
                 ) : encoder_A(encoder_A),
                     encoder_B(encoder_B),
                     force_sensor_A(force_sensor_A),
                     force_sensor_B(force_sensor_B)        
{
    printf("Sensors object was constructed.\n");
}

void Sensors::resetEncoders()
{
    encoder_A->ResetCounter();
    encoder_B->ResetCounter();
}

int16_t Sensors::getCounts_A(void)
{
    return encoder_A->GetCounter();
}


int16_t Sensors::getCounts_B(void)
{
    return encoder_B->GetCounter();
}

float Sensors::getForce_A(void)
{
    return force_sensor_A->read();
}

float Sensors::getForce_B(void)
{
    return force_sensor_B->read();
}

// class destructor
Sensors::~Sensors()
{
    printf("Sensors object was destructed.\n");
}