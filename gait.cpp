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

enum Posture {
    NONE = 0,
    STANCE = 1,
    SWING1 = 2,
    SWING2 = 3,
};

int REF = 1; 
std::queue<float> q;
std::queue<float> delta;
int step_cnt = 0;
int q_assigned_size = 5;
Posture posture = Posture::NONE;

void myApp::handleCommand(uint8_t cmd, const uint8_t values[], size_t len){
    
    switch (cmd)
    {
        case Commands::HELLO:
        {
            // Hello response, for use as a sanity check <3
            uint8_t helloMsg[] = "Hello, hw3!";
            //tags aren't explicitly necessary, but they're a good way of grouping responses.
            //The included console uses them to filter and format responses.
            uint8_t tag=1;
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
            break;
        }
        case Commands::START: {
            uint8_t helloMsg[] = "Hello, hw3! Start";
            uint8_t tag=1;
            step_cnt = 0;

            while (!q.empty()) {
                q.pop();
            }
            while (!delta.empty()) {
                delta.pop();
            }
            for (int i = 0; i < q_assigned_size; i++) {
                q.push(0);
                if (i) {
                    delta.push(0);
                } 
            }

            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
            char path[] = "/Meas/IMU6/13";
            subscribe(path, sizeof(path), REF);
        }
        break;
        case Commands::STOP: {
            uint8_t helloMsg[] = "Hello, hw3! STOP";
            uint8_t tag=1;
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
    uint8_t helloMsg[] = "Hello, hw3! Process Data";
    
    float magnitudes[16];

    const WB_RES::IMU6Data& data = value.convertTo<WB_RES::IMU6Data&>();
    const wb::Array<wb::FloatVector3D>& gyro_data = data.arrayGyro;
    
    float wn = data.arrayGyro[0].z; // current gyroscope data
    float wn_1 = 0.0; // previous gyroscope data
    float dn, dn_1 = 0.0; // delta data, dn = w_n - w_(n-1)
    
    wn_1 = q.back(); // q saves previous gyroscope data 
    dn_1 = delta.back(); // delta saves previous gyroscope derivative data
    dn = wn - wn_1; // current derivative

    // add new data and clear old data to queue
    q.pop();
    delta.pop();
    delta.push(dn);
    q.push(wn);

    //up peak and down peak threshold
    const float UP_THRESHOLD = 150.0f; 
    const float DOWN_THRESHOLD = -95.0f;

    if (dn_1 > 0 && dn < 0) { // found high peak
        if (wn > UP_THRESHOLD) { // cross the high peak
            posture = Posture::SWING2; // end of SWING
            step_cnt++;
            goto output;
        }
    }   
    if (dn_1 < 0 && dn > 0) { // low peak
        if (posture == Posture::SWING2 && wn < 0) { // Previous posture is SWING, then POSTURE change to STANCE
            posture = Posture::STANCE; // STANCE period
            step_cnt++;
            goto output;
        } else if (posture == Posture::STANCE && wn < DOWN_THRESHOLD) {
            posture = Posture::SWING1; // start of SWING
            goto output;
        }
    }

output:  // Send data
    uint8_t tag = 5;
    uint8_t result[16];
    *((float*)result) = wn;
    *((int*)result+1) = step_cnt;
    char* p = (char*) ((uint8_t*)result+8);
    switch(posture) {
        case Posture::SWING1:
        case Posture::SWING2: {
            char out[]="SWING";
            memcpy(p, out, sizeof(out));   
            break;
        }
        case Posture::STANCE: {
            char out[]="STANCE";
            memcpy(p, out, sizeof(out));   
            break;
        }
        case Posture::NONE: {
            break;
        }
    }
 
    sendPacket(result, sizeof(result), tag, Responses::DATA);
    
            //tags aren't explicitly necessary, but they're a good way of grouping responses.
            //The included console uses them to filter and format responses.
}
