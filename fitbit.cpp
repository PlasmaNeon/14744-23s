#include "myApp.h"
#include "interface.h"

// sensor libraries for data interperting
#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"
#include "meas_magn/resources.h"
#include "meas_imu/resources.h"
#include <queue>
//Commands formated as byte array with [Command, data?...] (data optional)
enum Commands 
{
    HELLO = 0,
    START = 1,
    STOP = 2,
    LED = 3,
};
//Responses formated as byte array with [Response, tag, data?...] (data optional)
enum Responses 
{
    COMMAND_RESULT = 1,
    DATA = 2,
    ERROR = 3,
};

int REF = 1; 
std::queue<float> q;
int flag = 0;
int step_cnt = 0;

void myApp::handleCommand(uint8_t cmd, const uint8_t values[], size_t len){
    
    switch (cmd)
    {
        case Commands::HELLO:
        {
            // Hello response, for use as a sanity check <3
            uint8_t helloMsg[] = "Hello, hw2!";
            //tags aren't explicitly necessary, but they're a good way of grouping responses.
            //The included console uses them to filter and format responses.
            uint8_t tag=1;
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
            break;
        }
        case Commands::START: {
            uint8_t helloMsg[] = "Hello, hw2! Start";
            uint8_t tag=1;
            step_cnt = 0;
            flag = 0;
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
            char path[] = "/Meas/IMU6/52";
            subscribe(path, sizeof(path), REF);
        }
        break;
        case Commands::STOP: {
            uint8_t helloMsg[] = "Hello, hw2! STOP";
            uint8_t tag=1;
            ledSet(false);
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
            unsubscribe(REF);
            break;
        }
    }
}

void myApp::processData(wb::ResourceId resourceId, const wb::Value &value){
    if (findDataSub(resourceId)->clientReference != REF) {
        return;
    }
    uint8_t helloMsg[] = "Hello, hw2! Process Data";
    
    float magnitudes[16];

    const WB_RES::IMU6Data& data = value.convertTo<WB_RES::IMU6Data&>();
    const wb::Array<wb::FloatVector3D>& accData = data.arrayAcc;

    size_t i;
    float ave = 0;
    const float OFFSET = 10.0f;
    for (i = 0; i < 16 && i < accData.size(); i++) {
        ave += accData[i].x;
    }
    if (i != 0)
        ave = ave / i;

    // process data
    float ave_normalized = ave + OFFSET;
    const float UP_THRESHOLD = 8.0f;
    const float DOWN_THRESHOLD = -5.0f;
    if (flag == 0) {
        if (ave_normalized > UP_THRESHOLD) {
            flag = 1;
            step_cnt++;
            ledSetPattern(100, 1000, 1, true);
        } else if (ave_normalized < DOWN_THRESHOLD) {
            flag = -1;
            step_cnt++;
            ledSetPattern(100, 1000, 1, true);
        }
    } else if (flag == 1) {
        if (ave_normalized < UP_THRESHOLD) {
            flag = 0;
            ledSet(false);
        }
    } else if (flag == -1) {
        if (ave_normalized > DOWN_THRESHOLD) {
            flag = 0;
            ledSet(false);
        }
    }
    
    uint8_t tag = 5;
    uint8_t result[8];
    *((float*)result) = ave_normalized;
    *((int*)result+1) = step_cnt;
    //sendPacket(helloMsg, sizeof(helloMsg), 3, Responses::COMMAND_RESULT);
    sendPacket(result, sizeof(result), tag, Responses::DATA);
}
