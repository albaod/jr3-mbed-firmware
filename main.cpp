#include "mbed.h"
#include "Jr3.hpp"
#include "Jr3Controller.hpp"

constexpr auto OFFSET = 4;

enum can_ops : uint16_t
{
    JR3_ACK = 1,
    JR3_START,
    JR3_STOP,
    JR3_ZERO_OFFS,
    JR3_SET_FILTER,
    JR3_GET_STATE,
    JR3_GET_FS,
    JR3_RESET,
    JR3_READ, // forces & moments
    JR3_BOOTUP
};

// uint16_t Fx = 10000 ;
// uint16_t Fy = 20000 ;
// uint16_t Fz = 30000 ;
// uint16_t Mx = 40000 ;
// uint16_t My = 50000 ;
// uint16_t Mz = 60000 ; 
// //float F = 10.50 ;

enum jr3_state : uint8_t
{
    JR3_READY = 0x00,
    JR3_NOT_INITIALIZED = 0x01
};

struct serial_msg
{
    uint16_t op;
    uint8_t data[14]; // make room for 6*2 bytes (6 uint16_t channels) + 2 bytes (frame counter)
    int size;
};

uint16_t parseCutOffFrequency(const serial_msg & msg, size_t offset = 0)
{
    if (msg.size >= sizeof(uint16_t) + offset)
    {
        uint16_t temp;
        memcpy(&temp, msg.data + offset, sizeof(uint16_t));
        return temp;
    }
    else
    {
        return 0;
    }
}

uint32_t parseAsyncPeriod(const serial_msg & msg, size_t offset = 0)
{
    if (msg.size >= sizeof(uint32_t) + offset)
    {
        uint32_t temp;
        memcpy(&temp, msg.data + offset, sizeof(uint32_t));
        return temp;
    }
    else
    {
        return 0;
    }
}

void readMessage(const char * buffer, serial_msg & msg)
{
    msg.size = 0;

    char c = '0';
    int index = 0;

    bool has_op1 = false;
    bool has_op2 = false;

    while (c != '>')
    {
        c = buffer[index++];

        if (c == '<' || c == '>')
        {
            continue;
        }

        if (!has_op1)
        {
            msg.op = int(c - '0') * 10;
            has_op1 = true;
        }
        else if (!has_op2)
        {
            msg.op += int(c - '0');
            has_op2 = true;
        }
        else
        {
            memcpy(msg.data + msg.size, &c, 1);
            msg.size++;
        }
    }
}

int buildMessage(const serial_msg & msg, char * buffer)
{
    sprintf(buffer, "<%02d", msg.op);

    if (msg.size > 0)
    {
        memcpy(buffer + 3, msg.data, msg.size);
    }
    sprintf(buffer + 3 + msg.size, ">");
    return msg.size + OFFSET;
}

void sendData(mbed::BufferedSerial & serial, char * buffer, uint16_t * data)
{
    memcpy(buffer, data, 12);
    memcpy(buffer + 12, data + 12, sizeof(uint16_t)); // frame_counter
    serial.write(buffer, 14);
}

void sendMessage(mbed::BufferedSerial & serial, const serial_msg & msg, char * buffer)
{
    int size = buildMessage(msg, buffer);
    serial.write(buffer, size);

}


void sendFullScales(mbed::BufferedSerial & serial, char * buffer, const Jr3Controller & controller, uint16_t * data)
{
    uint8_t state = controller.getState() == Jr3Controller::READY ? JR3_READY : JR3_NOT_INITIALIZED;
    // state = JR3_NOT_INITIALIZED;
    serial_msg msg {JR3_ACK, {state}, 13}; 
    memcpy(msg.data + 1, data, 12); // fsx, fsy, fsz, msx, msy, msz [uint16_t]
    sendMessage(serial, msg, buffer);
    printf("\n");
}

void sendAcknowledge(mbed::BufferedSerial & serial, char * buffer, const Jr3Controller & controller)
{
    uint8_t state = controller.getState() == Jr3Controller::READY ? JR3_READY : JR3_NOT_INITIALIZED;
    serial_msg msg {JR3_ACK, {state}, 1};
    sendMessage(serial, msg, buffer);
}

int main()
{
    int i = 0;

    while (i < 5)
    {
        rtos::ThisThread::sleep_for(1s);
        i++;
    }
    i=0;
    mbed::BufferedSerial serial(USBTX, USBRX);
    serial.set_baud(MBED_CONF_APP_SERIAL_BAUDRATE);
    // serial.set_format(8, mbed::BufferedSerial::None, 1); // default
        printf("\n"); 
        printf("booting primero\n");

    while (i < 5)
    {
        rtos::ThisThread::sleep_for(1s);
        printf("\n"); 
        printf("booting %d\n", i);
        i++;
    }

    char buffer_in[MBED_CONF_APP_SERIAL_BUFFER_IN_SIZE] = {0};
    char buffer_out[MBED_CONF_APP_SERIAL_BUFFER_OUT_SIZE] = {0};
    // COMO NO VAMOS A TENER CONECTADO EL JR3 TODO LO QUE TENGA RELACIÓN CON LA RECOGIDA/ENVIO DE DATOS DEL JR3 SE TENDRA QUE COMENTAR

    using Jr3Reader = Jr3<MBED_CONF_APP_JR3_PORT, MBED_CONF_APP_JR3_CLOCK_PIN, MBED_CONF_APP_JR3_DATA_PIN>;
    Jr3Reader jr3;
    Jr3Controller controller({&jr3, &Jr3Reader::readFrame});
   

    if (jr3.isConnected()) // Para simular que esta conectado vamos a decir que jr3 esta conectado asi que devuleve un 1 jr3.isConected()
    {
        printf("JR3 sensor is connected\n");
        controller.initialize(); // this blocks until the initialization is completed --> Realmente en la simulación no inicializamos el JR3
        sendMessage(serial, {JR3_BOOTUP, {}, 0}, buffer_out);
    }
    else
    {
        printf("JR3 sensor is not connected\n");
    }

    uint16_t fs_data[6];// helper buffer for full scales (6*2)
    //memcpy(&fs_data[0], &F, sizeof(F)); // float tiene 4 bytes no dos como hemos puesto para Fx
    int size;

    serial_msg msg_in;
    serial_msg msg_data {JR3_READ, {}, 14};


    while ((size = serial.read(buffer_in, MBED_CONF_APP_SERIAL_BUFFER_IN_SIZE)) > 0)
    {
        // printf("ADENTRO DEL WHILE");  
        readMessage(buffer_in, msg_in);
        // printf("El readmessage ha leido un msg_in.op: %d\n", msg_in.op);
        switch (msg_in.op)
        {
        // printf("ADENTRO DEL SWITCH");  
        case JR3_START:
            printf("received JR3 start command (asynchronous)\n");
            controller.startAsync([&serial, &msg_data, &buffer_out](uint16_t * data)
            {
                memcpy(msg_data.data, data, sizeof(msg_data.data));
                sendMessage(serial, msg_data, buffer_out);
            }, parseCutOffFrequency(msg_in), parseAsyncPeriod(msg_in, sizeof(uint16_t)));
            sendAcknowledge(serial, buffer_out, controller);
            break;
        case JR3_STOP:
            printf("received JR3 stop command\n");
            controller.stop();
            sendAcknowledge(serial, buffer_out, controller);
            break;
        case JR3_ZERO_OFFS:
            printf("received JR3 zero offsets command\n");
            controller.calibrate();
            sendAcknowledge(serial, buffer_out, controller);
            break;
        case JR3_SET_FILTER:
            printf("received JR3 set filter command\n");
            controller.setFilter(parseCutOffFrequency(msg_in));
            sendAcknowledge(serial, buffer_out, controller);
            break;
        case JR3_GET_STATE:
            printf("received JR3 get state command\n");
            sendAcknowledge(serial, buffer_out, controller);
            break;
        case JR3_GET_FS:
            printf("received JR3 get full scales (forces) command\n");
            controller.getFullScales(fs_data); 
            // VAMOS A INVENTARNOS LOS VALORES DE fs_data
            sendFullScales(serial, buffer_out, controller , fs_data);
            break;
        case JR3_RESET:
            printf("received JR3 reset command\n");
            controller.initialize();
            sendAcknowledge(serial, buffer_out, controller);
            break;
        default:
            printf("unsupported command: %d\n", msg_in.op);
            break;
        }
        //  printf("FUERA DEL SWITCH");  
        // this is the minimum amount of time that actually sleeps the thread, use AccurateWaiter
        // for the microsecond scale; wait_us(), on the contrary, spins the CPU
        rtos::ThisThread::sleep_for(1s);
    }
}