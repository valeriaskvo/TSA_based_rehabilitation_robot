#ifndef SENSORS_H
#define SENSORS_H

#include <stdio.h>

#include "TimEncoders/Nucleo_Encoder_16_bits.h"

// Nucleo_Encoder_16_bits linear_encoder_left(TIM3);
// Nucleo_Encoder_16_bits linear_encoder_right(TIM4);

class Sensors
{
public:
    // class constructor
    Sensors(Nucleo_Encoder_16_bits *encoder_A,
            Nucleo_Encoder_16_bits *encoder_B,
            AnalogIn *force_sensor_A,
            AnalogIn *force_sensor_B
    );

    void resetEncoders(void);

    int16_t getCounts_A();
    int16_t getCounts_B();
    float getForce_A();
    float getForce_B();

    ~Sensors();

private:
    Nucleo_Encoder_16_bits *encoder_A;
    Nucleo_Encoder_16_bits *encoder_B;
    AnalogIn *force_sensor_A;
    AnalogIn *force_sensor_B;
};

#endif