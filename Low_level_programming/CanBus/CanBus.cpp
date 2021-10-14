#include "CanBus.h"
#include "mbed.h"


// class constructor
CanBus::CanBus(PinName rd, PinName td, int hz, Sensors *sensors) : can(rd, td, hz),
                                                                   sensors(sensors)
{

    NVIC_SetPriority(CAN1_RX0_IRQn, 1);
    can.filter(CAN_ID, 0xFFE00000, CANStandard, 0); //0xFFE00004
    txMsg.id = CAN_MASTER_ID;
    txMsg.len = txSize;
    rxMsg.len = rxSize;
    can.attach(callback(this, &CanBus::onMsgReceived), CAN::RxIrq);

    printf("CanBus object was constructed.\n");
}

void CanBus::onMsgReceived()
{
    if (!can.read(rxMsg))
        return;

    if (rxMsg.id == CAN_ID)
    {
        switch (rxMsg.data[0])
        {
        case MSG_READ_ENCODER:
            read_encoders(rxMsg);
            break;
        case MSG_READ_FORCE:
            read_force(rxMsg);
            break;
        case MSG_RESET:
            reset_counters(rxMsg);
            break;
        case MSG_CONFIG:
            configuration_mode(rxMsg);
            break;  
        default:
            unknown_command(rxMsg);
            break;
        }
    }
}

void CanBus::unknown_command(CANMessage &msg)
{
    txMsg.id = msg.id;
    memcpy(txMsg.data, msg.data, msg.len);
    can.write(txMsg);
}


void CanBus::reset_counters(CANMessage &msg)
{
    txMsg.id = msg.id;
    memcpy(txMsg.data, msg.data, msg.len);
    sensors-> resetEncoders();
    can.write(txMsg);
    printf("Device was reseted.\n");
}


void CanBus::configuration_mode(CANMessage &msg)
{
    txMsg.id = msg.id;
    memcpy(txMsg.data, msg.data, msg.len);
    can.write(txMsg);
}


void CanBus::read_encoders(CANMessage &msg)
{
    txMsg.id = msg.id;
    txMsg.data[0] = msg.data[0];

    uint8_t sensorMsg[2];

    int16_t encoder_data = sensors->getCounts_A(); 
    memcpy(sensorMsg,&encoder_data,sizeof(encoder_data));
    txMsg.data[0] = sensorMsg[0];
    txMsg.data[1] = sensorMsg[1];

    encoder_data = sensors->getCounts_B();
    memcpy(sensorMsg,&encoder_data,sizeof(encoder_data));
    txMsg.data[2] = sensorMsg[0];
    txMsg.data[3] = sensorMsg[1];

    can.write(txMsg);
}

void CanBus::read_force(CANMessage &msg)
{
    txMsg.id = msg.id;
    txMsg.data[0] = msg.data[0];

    uint8_t forceMsg[4];
    
    float force = sensors->getForce_A();
    memcpy(forceMsg,&force,sizeof(force));
    
    for (int i=0;i<4;i++)
    { 
        txMsg.data[i] = forceMsg[i];
    }

    force = sensors->getForce_B();
    memcpy(forceMsg,&force,sizeof(force));
    
    for (int i=4;i<8;i++)
    { 
        txMsg.data[i] = forceMsg[i-4];
    }

    can.write(txMsg);
}


// class destructor
CanBus::~CanBus()
{
    printf("CanBus object was destructed.\n");
}