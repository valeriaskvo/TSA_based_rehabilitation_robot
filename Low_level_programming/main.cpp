#include "mbed.h"
#include "CanBus/CanBus.h"

#include "TimEncoders/Nucleo_Encoder_16_bits.h"
#include "Sensors/Sensors.h"

// #define M_PI 3.14159265358979323846

BufferedSerial pc(USBTX, USBRX, 115200); // tx, rx
FileHandle *mbed::mbed_override_console(int fd)
{
  return &pc;
}

AnalogIn force_sensor_A(PF_4);
AnalogIn force_sensor_B(PF_9);
Nucleo_Encoder_16_bits encoder_A(TIM3);
Nucleo_Encoder_16_bits encoder_B(TIM4);
Sensors sensors(&encoder_A, &encoder_B, &force_sensor_A, &force_sensor_B);

CanBus can(PB_8, PB_9, 1000000, &sensors);

int main()
{  
  while (1)
  {

    // printf("%-20s\n", "this is a test");
    // float force_A = force_sensor_A.read();
    // float force_B = force_sensor_B.read();

    // printf("A: %1.4f, B: %1.4f\n",force_A,force_B);
    // thread_sleep_for(5);
  }
}