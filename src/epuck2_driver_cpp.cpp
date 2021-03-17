
#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/BatteryState.h>

#define DEBUG_CONNECTION_INIT 1
#define DEBUG_ROS_PARAMS 1
#define DEBUG_UPDATE_SENSORS_TIMING 0
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_COMMUNICATION_ERROR 1
#define DEBUG_ODOMETRY 0
#define DEBUG_IMU 0
#define DEBUG_SPEED_RECEIVED 0
#define DEBUG_LED_RECEIVED 0
#define DEBUG_CAMERA_INIT 0
#define DEBUG_RGB_RECEIVED 0
#define DEBUG_MAG_FIELD 0
#define DEBUG_BATTERY 0

#define READ_TIMEOUT_SEC 10    // 10 seconds, keep it high to avoid desynchronize when there are communication delays due to Bluetooth.
#define READ_TIMEOUT_USEC 0
#define MAX_CONSECUTIVE_TIMEOUT 3

#define CAM_MAX_BUFFER_SIZE 3203
#define CAM_MODE_GRAY 0
#define CAM_MODE_RGB 1

#define WHEEL_DIAMETER 4        // cm.
#define WHEEL_SEPARATION 5.3    // Separation between wheels (cm).
#define WHEEL_DISTANCE 0.053    // Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER*M_PI)/100.0)    // Wheel circumference (meters).
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE/1000.0)      // Distance for each motor step (meters); a complete turn is 1000 steps (0.000125 meters per step (m/steps)).
#define ROBOT_RADIUS 0.035 // meters.

#define LED_NUMBER 6 // total number of LEDs on the robot (0,2,4,6=leds, 8=body, 9=front)
#define RGB_LED_NUMBER 4

#define DEG2RAD(deg) (deg / 180 * M_PI)
#define GYRO_RAW2DPS (250.0/32768.0f)   //250DPS (degrees per second) scale for int16 raw value
#define STANDARD_GRAVITY 9.80665f

#define RESISTOR_R1       220 //kohm
#define RESISTOR_R2       330 //kohm
#define VOLTAGE_DIVIDER   (1.0f * RESISTOR_R2 / (RESISTOR_R1 + RESISTOR_R2))
#define VREF              3.0f //volt correspond to the voltage on the VREF+ pin
#define ADC_RESOLUTION    4096
#define COEFF_ADC_TO_VOLT ((1.0f * ADC_RESOLUTION * VOLTAGE_DIVIDER) / VREF) //convertion from adc value to voltage
#define MAX_VOLTAGE       4.2f	//volt
#define MIN_VOLTAGE       3.4f	//volt

#define INPUT_BUFF_SIZE 103
#define OUTPUT_BUFF_SIZE 22

char output_buffer[OUTPUT_BUFF_SIZE]; // Transmission buffer: set all actuators (0x09) + actuators data (19 bytes, refer to https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development#WiFi_2) + get all sensors (0x08) + end delimiter (0x00)
char input_buffer[INPUT_BUFF_SIZE]; // Receive buffer: sensors data (103 bytes (104 - 1 (button handled by ESP32)), refer to https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development#WiFi_2)
bool camera_enabled, ground_sensors_enabled;
int speedLeft = 0, speedRight = 0;
int stepsLeft = 0, stepsRight = 0;
int rfcommSock;
int sock;
int devId;
unsigned int bytesToReceive;
std::string epuckname;
struct timeval currentTime2, lastTime2;
struct timeval currentTime3, lastTime3;
int consecutiveReadTimeout = 0;
int camWidth, camHeight, camZoom, camMode, camXoffset, camYoffset;

float acceleration, orientation, inclination;
int16_t accData[3];
int16_t gyroData[3];
float magneticField[3];
uint8_t temperature;
int proxData[8];
int lightAvg;
uint16_t distanceMm = 0;
uint16_t micVolume[4];
int16_t motorSteps[2];
uint16_t batteryRaw;
uint8_t microSdState;
uint8_t irCheck, irAddress, irData;
uint8_t selector;
int16_t groundProx[3], groundAmbient[3];
uint8_t buttonState;
unsigned char *camData;
bool newImageReceived = false;
int imageSize = 0;

int16_t gyroOffset[3] = {0, 0, 0}; // Used if making an initial calibration of the gyro.

ros::Publisher proxPublisher[8];
sensor_msgs::Range proxMsg[8];
ros::Publisher laserPublisher;
sensor_msgs::LaserScan laserMsg;
ros::Publisher odomPublisher;
nav_msgs::Odometry odomMsg;
ros::Publisher imagePublisher;
ros::Publisher imuPublisher;
sensor_msgs::Imu imuMsg;
ros::Publisher microphonePublisher;
visualization_msgs::Marker microphoneMsg;
ros::Publisher floorPublisher;
visualization_msgs::Marker floorMsg;
ros::Publisher distSensPublisher;
sensor_msgs::Range distSensMsg;
ros::Publisher magFieldPublisher;
sensor_msgs::MagneticField magFieldMsg;
ros::Publisher magFieldVectorPublisher;
visualization_msgs::Marker magFieldVectorMsg;
ros::Publisher battPublisher;
sensor_msgs::BatteryState battMsg;

ros::Subscriber cmdVelSubscriber, cmdLedSubscriber, cmdRgbLedsSubscriber;

double leftStepsDiff = 0, rightStepsDiff = 0;
double leftStepsPrev = 0, rightStepsPrev = 0;
signed long int leftStepsRawPrev = 0, rightStepsRawPrev = 0;
signed long int motorPositionDataCorrect[2];
double xPos, yPos, theta;
double deltaSteps, deltaTheta;
ros::Time currentTime, lastTime, currentTimeMap, lastTimeMap;
int overflowCountLeft = 0, overflowCountRight = 0;

void clearCommunicationBuffer() {

    char buffer[64];
    struct timeval timeout;
    fd_set readfds;
    int retval;
    int trials = 0;

    if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "sending enter..." << std::endl;
    buffer[0] = '\r';
    FD_ZERO(&readfds);
    FD_SET(rfcommSock, &readfds);
    write(rfcommSock, buffer, 1);
    while(1) {
        timeout.tv_sec=0; // The timeout need to be set every time because the "select" may modify it.
        timeout.tv_usec=500000;
        retval = select(rfcommSock+1,&readfds,NULL,NULL,&timeout);
        if (retval!=0) {
            int n = read(rfcommSock, buffer, 64);
            if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "read " << n << " bytes" << std::endl;
            if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "content: " << buffer << std::endl;
            memset(buffer, 0x0, 64);
        } else {
            break;
        }
    }

    if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "requesting version..." << std::endl;
    buffer[0] = 'V';
    buffer[1] = '\r';
    FD_ZERO(&readfds);
    FD_SET(rfcommSock, &readfds);
    write(rfcommSock, buffer, 2);
    trials = 0;
    while(1) {
        timeout.tv_sec=0; // The timeout need to be set every time because the "select" may modify it.
        timeout.tv_usec=500000;
        retval = select(rfcommSock+1, &readfds, NULL, NULL, &timeout);
        if (retval!=0) {
            int n = read(rfcommSock, buffer, 64);
            if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "read " << n << " bytes" << std::endl;
            if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "content: " << buffer << std::endl;
            memset(buffer, 0x0, 64);
        } else {
            trials++;
            if(trials >= 1) {
                break;
            }
        }
    }

}

int initConnectionWithRobotId(int robotId) {
     std::stringstream ss;

    // open device
    devId = hci_get_route(NULL);
    if (devId < 0) {
        ss.str("");
        ss << "[" << epuckname << "] " << "Error, can't get bluetooth adapter ID";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
        return -1;
    }

    // open socket
    sock = hci_open_dev(devId);
    if (sock < 0) {
        ss.str("");
        ss << "[" << epuckname << "] " << "Error, can't open bluetooth adapter";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
        return -1;
    }

    int trials = 3;     // Try looking for the robot 3 times before giving up.
    while(trials) {
        // query
        if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "Scanning bluetooth:" << std::endl;
        //int length  = 8; /* ~10 seconds */
        int length  = 4; /* ~5 seconds */
        inquiry_info *info = NULL;
        int devicesCount = 0;
        while(1) {
            // device id, query length (last 1.28 * length seconds), max devices, lap ??, returned array, flag
            devicesCount = hci_inquiry(devId, length, 255, NULL, &info, IREQ_CACHE_FLUSH);
            if (devicesCount < 0) {
                ss.str("");
                ss << "[" << epuckname << "] " << "Error, can't query bluetooth";
                if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
                if(errno!=EBUSY) {      // EBUSY means the Bluetooth device is currently used by another process (this happens when
                    close(sock);        // we want to connect to multiple robots simultaneously); in this case wait a little and retry.
                    return -1;          // All others errors are treated normally.
                } else {
                    usleep(1000000);
                }
            } else {
                break;
            }
        }

        bool found = false;
        for (int i = 0; i < devicesCount; i++) {
            char addrString[19];
            char addrFriendlyName[256];
            ba2str(&(info+i)->bdaddr, addrString);
            if (hci_read_remote_name(sock, &(info+i)->bdaddr, 256, addrFriendlyName, 0) < 0) {
                strcpy(addrFriendlyName, "[unknown]");
            }
            if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "\t" <<  addrString << " " << addrFriendlyName << std::endl;
            if (strncmp("e-puck_", addrFriendlyName, 7) == 0) {
                int id;
                if (sscanf(addrFriendlyName + 7, "%d", &id) && (id == robotId)) {
                    if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "Contacting e-puck " << id << std::endl;

                    // set the connection parameters (who to connect to)
                    struct sockaddr_rc addr;
                    addr.rc_family = AF_BLUETOOTH;
                    addr.rc_channel = (uint8_t) 2;
                    addr.rc_bdaddr = (info+i)->bdaddr;

                    // allocate a socket
                    rfcommSock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

                    // connect to server
                    int status = ::connect(rfcommSock, (struct sockaddr *)&addr, sizeof(addr));

                    if (status == 0) {
                        clearCommunicationBuffer();
                        found = true;
                    } else {
                        ss.str("");
                        ss << "[" << epuckname << "] " << "Error, can't connect to rfcomm socket";
                        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
                        return -1;
                    }
                    break;
                }
            }
        }
        if(found) {
            if(hci_close_dev(sock) < 0) {
                ss.str("");
                ss << "[" << epuckname << "] " << "Can't close HCI device";
                if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
            }
            return 0;
        } else {
            trials--;
        }
    }
    if(hci_close_dev(sock) < 0) {
        ss.str("");
        ss << "[" << epuckname << "] " << "Can't close HCI device";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
    }
    return -1;

}

int initConnectionWithRobotAddress(const char *address) {

    std::stringstream ss;
    struct sockaddr_rc addr;    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 2;
    str2ba(address, &addr.rc_bdaddr);

    // allocate a socket
    rfcommSock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // connect to server
    int status = ::connect(rfcommSock, (struct sockaddr *)&addr, sizeof(addr));

    if (status == 0) {
        clearCommunicationBuffer();
        return 0;
    } else {
        ss.str("");
        ss << "[" << epuckname << "] " << "Error, can't connect to rfcomm socket";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
        return -1;
    }

}

void closeConnection() {
    std::stringstream ss;

    if(close(rfcommSock) < 0) {
        ss.str("");
        ss << "[" << epuckname << "] " << "Can't close rfcomm socket";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
    }

//    if(shutdown(rfcommSock, SHUT_RDWR) < 0) {
//        ss.str("");
//        ss << "[" << epuckname << "] " << "Can't shutdown rfcomm socket";
//        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
//    } else {
//        if(DEBUG_CONNECTION_INIT)std::cout << "[" << epuckname << "] " << "rfcomm shutdown correctly" << std::endl;
//    }
}

void updateSensorsAndActuators() {
    struct timeval timeout;
    fd_set readfds;
    int retval;
    int trials = 0;
    unsigned int bufIndex = 0;
    unsigned int bytesRead = 0;

    memset(input_buffer, 0x0, INPUT_BUFF_SIZE);
    FD_ZERO(&readfds);
    FD_SET(rfcommSock, &readfds);
    write(rfcommSock, output_buffer, OUTPUT_BUFF_SIZE);
    bytesRead = 0;
    if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&lastTime3, NULL);
    while(bytesRead < INPUT_BUFF_SIZE) {
      timeout.tv_sec=READ_TIMEOUT_SEC; // The timeout need to be set every time because the "select" may modify it.
      timeout.tv_usec=READ_TIMEOUT_USEC;
      if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&lastTime2, NULL);
      retval = select(rfcommSock+1, &readfds, NULL, NULL, &timeout);
      if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&currentTime2, NULL);
      if(DEBUG_UPDATE_SENSORS_TIMING)std::cout << "[" << epuckname << "] " << "sensors data read in " << double((currentTime2.tv_sec*1000000 + currentTime2.tv_usec)-(lastTime2.tv_sec*1000000 + lastTime2.tv_usec))/1000000.0 << " sec" << std::endl;
      if (retval>0) {
        int n = read(rfcommSock, &input_buffer[bytesRead], INPUT_BUFF_SIZE-bytesRead);
        //if(DEBUG_OTHERS)std::cout << "read " << n << " / " << bytesToReceive << " bytes" << std::endl;
        bytesRead += n;
        consecutiveReadTimeout = 0;
      } else if(retval==0) {
        if(DEBUG_COMMUNICATION_ERROR)std::cout << "[" << epuckname << "] " << "sensors read timeout" << std::endl;
        consecutiveReadTimeout++;
        break;
      } else {
        if(DEBUG_COMMUNICATION_ERROR)perror("sensors read error");
        break;
      }
    }

    if(bytesRead == INPUT_BUFF_SIZE) {
        if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&currentTime3, NULL);
        if(DEBUG_UPDATE_SENSORS_TIMING)std::cout << "[" << epuckname << "] " << "sensors tot read time " << double((currentTime3.tv_sec*1000000 + currentTime3.tv_usec)-(lastTime3.tv_sec*1000000 + lastTime3.tv_usec))/1000000.0 << " sec" << std::endl;

        // Accelerometer
        accData[0] = (unsigned char)input_buffer[0] | input_buffer[1]<<8;
        accData[1] = (unsigned char)input_buffer[2] | input_buffer[3]<<8;
        accData[2] = (unsigned char)input_buffer[4] | input_buffer[5]<<8;
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "acc: " << accData[0] << "," << accData[1] << "," << accData[2] << std::endl;

        // Acceleration
				acceleration = *((float*)&input_buffer[6]);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "acceleration: " << acceleration << std::endl;

				// Orientation
				orientation = *((float*)&input_buffer[10]);
				if (orientation < 0.0 )
					orientation=0.0;
				if (orientation > 360.0 )
					orientation=360.0;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "orientation: " << orientation << std::endl;

				// Inclination
				inclination = *((float*)&input_buffer[14]);
				if (inclination < 0.0 )
					inclination=0.0;
				if (inclination > 180.0 )
					inclination=180.0;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "inclination: " << inclination << std::endl;

        // Gyro
        gyroData[0] = (unsigned char)input_buffer[18] | input_buffer[19]<<8;
        gyroData[1] = (unsigned char)input_buffer[20] | input_buffer[21]<<8;
        gyroData[2] = (unsigned char)input_buffer[22] | input_buffer[23]<<8;
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "gyro: " << gyroData[0] << "," << gyroData[1] << "," << gyroData[2] << std::endl;

        // Magnetometer
        magneticField[1] = *((float*)&input_buffer[28]);
        magneticField[2] = *((float*)&input_buffer[32]);
        magneticField[0] = *((float*)&input_buffer[24]);
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "mag: " << magneticField[0] << "," << magneticField[1] << "," << magneticField[2] << std::endl;

        // Temperature.
        temperature = input_buffer[36];
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "temperature: " << (int)temperature << std::endl;

        // Proximity sensors data.
        proxData[0] = (unsigned char)input_buffer[37] | input_buffer[38]<<8;
        proxData[1] = (unsigned char)input_buffer[39] | input_buffer[40]<<8;
        proxData[2] = (unsigned char)input_buffer[41] | input_buffer[42]<<8;
        proxData[3] = (unsigned char)input_buffer[43] | input_buffer[44]<<8;
        proxData[4] = (unsigned char)input_buffer[45] | input_buffer[46]<<8;
        proxData[5] = (unsigned char)input_buffer[47] | input_buffer[48]<<8;
        proxData[6] = (unsigned char)input_buffer[49] | input_buffer[50]<<8;
        proxData[7] = (unsigned char)input_buffer[51] | input_buffer[52]<<8;
        if(proxData[0]<0) {
					proxData[0]=0;
				}
				if(proxData[1]<0) {
					proxData[1]=0;
				}
				if(proxData[2]<0) {
					proxData[2]=0;
				}
				if(proxData[3]<0) {
					proxData[3]=0;
				}
				if(proxData[4]<0) {
					proxData[4]=0;
				}
				if(proxData[5]<0) {
					proxData[5]=0;
				}
				if(proxData[6]<0) {
					proxData[6]=0;
				}
				if(proxData[7]<0) {
					proxData[7]=0;
				}
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "prox: " << proxData[0] << "," << proxData[1] << "," << proxData[2] << "," << proxData[3] << "," << proxData[4] << "," << proxData[5] << "," << proxData[6] << "," << proxData[7] << std::endl;

        // Compute abmient light.
				lightAvg += (input_buffer[53]+input_buffer[54]*256);
				lightAvg += (input_buffer[55]+input_buffer[56]*256);
				lightAvg += (input_buffer[57]+input_buffer[58]*256);
				lightAvg += (input_buffer[59]+input_buffer[60]*256);
				lightAvg += (input_buffer[61]+input_buffer[62]*256);
				lightAvg += (input_buffer[63]+input_buffer[64]*256);
				lightAvg += (input_buffer[65]+input_buffer[66]*256);
				lightAvg += (input_buffer[67]+input_buffer[68]*256);
				lightAvg = (int) (lightAvg/8);
				lightAvg = (lightAvg>4000)?4000:lightAvg;
				if(lightAvg<0) {
					lightAvg=0;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "lightAvg: " << lightAvg << std::endl;

        // ToF
        distanceMm = (uint16_t)((uint8_t)input_buffer[70]<<8)|((uint8_t)input_buffer[69]);
        if(distanceMm > 2000) {
					distanceMm = 2000;
				}
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "distanceMm: " << distanceMm << "(" << (int)input_buffer[69] << "," << (int)input_buffer[70] << ")" << std::endl;

        // Microphone
				micVolume[0] = ((uint8_t)input_buffer[71]+(uint8_t)input_buffer[72]*256);
				micVolume[1] = ((uint8_t)input_buffer[73]+(uint8_t)input_buffer[74]*256);
				micVolume[2] = ((uint8_t)input_buffer[75]+(uint8_t)input_buffer[76]*256);
				micVolume[3] = ((uint8_t)input_buffer[77]+(uint8_t)input_buffer[78]*256);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "mic: " << micVolume[0] << "," << micVolume[1] << "," << micVolume[2] << "," << micVolume[3] << std::endl;

        // Left steps
				motorSteps[0] = (input_buffer[79]+input_buffer[80]*256);
				// Right steps
				motorSteps[1] = (input_buffer[81]+input_buffer[82]*256);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "steps: " << motorSteps[0] << "," << motorSteps[1] << std::endl;

        // Battery
				batteryRaw = (uint8_t)input_buffer[83]+(uint8_t)input_buffer[84]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "batteryRaw: " << batteryRaw << std::endl;

        // Micro sd state.
				microSdState = input_buffer[85];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "microSdState: " << (int)microSdState << std::endl;

				// Tv remote.
				irCheck = input_buffer[86];
				irAddress = input_buffer[87];
				irData = input_buffer[88];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "tv remote: " << (int)irCheck << "," << (int)irAddress << "," << (int)irData << std::endl;

				// Selector.
				selector = input_buffer[89];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "selector: " << (int)selector << std::endl;

        // Ground sensor proximity.
        groundProx[0] = input_buffer[90]+input_buffer[91]*256;
        groundProx[1] = input_buffer[92]+input_buffer[93]*256;
        groundProx[2] = input_buffer[94]+input_buffer[95]*256;
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "groundProx: " << groundProx[0] << "," << groundProx[1] << "," << groundProx[2] << std::endl;

        // Ground sensor ambient light.
        groundAmbient[0] = input_buffer[96]+input_buffer[97]*256;
        groundAmbient[1] = input_buffer[98]+input_buffer[99]*256;
        groundAmbient[2] = input_buffer[100]+input_buffer[101]*256;
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "groundAmbient: " << groundAmbient[0] << "," << groundAmbient[1] << "," << groundAmbient[2] << std::endl;

        // Button state.
				buttonState = input_buffer[102];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "buttonState: " << (int)buttonState << std::endl;

    } else {
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "discard the sensors data" << std::endl;
    }

    if(camera_enabled) {
        char buff[2];
        int imageBytesToReceive = imageSize;
        buff[0] = -'I';
        buff[1] = 0;
        FD_ZERO(&readfds);
        FD_SET(rfcommSock, &readfds);
        write(rfcommSock, buff, 2);
        bytesRead = 0;
        if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&lastTime3, NULL);
        while(bytesRead < imageBytesToReceive) {
            timeout.tv_sec=READ_TIMEOUT_SEC;
            timeout.tv_usec=READ_TIMEOUT_USEC;
            if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&lastTime2, NULL);
            retval = select(rfcommSock+1, &readfds, NULL, NULL, &timeout);
            if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&currentTime2, NULL);
            if(DEBUG_UPDATE_SENSORS_TIMING)std::cout << "[" << epuckname << "] " << "camera data read in " << double((currentTime2.tv_sec*1000000 + currentTime2.tv_usec)-(lastTime2.tv_sec*1000000 + lastTime2.tv_usec))/1000000.0 << " sec" << std::endl;
            if (retval>0) {
                int n = read(rfcommSock, &camData[bytesRead], imageBytesToReceive-bytesRead);
                //if(DEBUG_OTHERS)std::cout << "[" << epuckname << "] " << "camera: read " << bytesRead << " / " << imageBytesToReceive << " bytes" << std::endl;
                bytesRead += n;
                consecutiveReadTimeout = 0;
            } else if(retval==0) {
                if(DEBUG_COMMUNICATION_ERROR)std::cout << "[" << epuckname << "] " << "camera read timeout" << std::endl;
                consecutiveReadTimeout++;
                break;
            } else {
                if(DEBUG_COMMUNICATION_ERROR)perror("camera read error");
                break;
            }
        }
        if(bytesRead == imageBytesToReceive) {
            if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&currentTime3, NULL);
            if(DEBUG_UPDATE_SENSORS_TIMING)std::cout << "camera tot read time " << double((currentTime3.tv_sec*1000000 + currentTime3.tv_usec)-(lastTime3.tv_sec*1000000 + lastTime3.tv_usec))/1000000.0 << " sec" << std::endl;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "camera read correctly" << std::endl;
            newImageReceived = true;
        } else {
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "wrong camera data" << std::endl;
            newImageReceived = false;
        }

    }

//    while(1) {
//        char tempBuff[255];
//        timeout.tv_sec=0;
//        timeout.tv_usec=300000;
//        retval = select(rfcommSock+1, &readfds, NULL, NULL, &timeout);
//        if (retval!=0) {
//            int n = read(rfcommSock, &tempBuff[0], 255);
//            if(DEBUG_OTHERS)std::cout << "[" << epuckname << "] " << "resync: read " << n << " bytes" << std::endl;
//        } else {
//            if(DEBUG_OTHERS)std::cout << "[" << epuckname << "] " << "resync: read timeout" << std::endl;
//            break;
//        }
//    }

}

void RGB565toRGB888(int width, int height, unsigned char *src, unsigned char *dst) {
    int line, column;
    int index_src=0, index_dst=0;

    for (line = 0; line < height; ++line) {
        for (column = 0; column < width; ++column) {
            dst[index_dst++] = (unsigned char)(src[index_src] & 0xF8);
            dst[index_dst++] = (unsigned char)((src[index_src]&0x07)<<5) | (unsigned char)((src[index_src+1]&0xE0)>>3);
            dst[index_dst++] = (unsigned char)((src[index_src+1]&0x1F)<<3);
            index_src+=2;
        }
    }
}

void updateRosInfo() {
    static tf::TransformBroadcaster br;
    std::stringstream ss;
    geometry_msgs::Quaternion orientQuat;
    std::stringstream parent;
    std::stringstream child;
    tf::Transform transform;
    tf::Quaternion q;
    int i = 0;

    //#############################################################################################################################################
  	// Proximity topics
    for(i=0; i<8; i++) {
        if(proxData[i] > 0) {
            proxMsg[i].range = 0.5/sqrt(proxData[i]);  // Transform the analog value to a distance value in meters (given from field tests).
        } else {
            proxMsg[i].range = proxMsg[i].max_range;
        }
        if(proxMsg[i].range > proxMsg[i].max_range) {
            proxMsg[i].range = proxMsg[i].max_range;
        }
        if(proxMsg[i].range < proxMsg[i].min_range) {
            proxMsg[i].range = proxMsg[i].min_range;
        }
        proxMsg[i].header.stamp = ros::Time::now();
        proxPublisher[i].publish(proxMsg[i]);
    }

    // e-puck proximity positions (cm), x pointing forward, y pointing left
    //           P7(3.5, 1.0)   P0(3.5, -1.0)
    //       P6(2.5, 2.5)           P1(2.5, -2.5)
    //   P5(0.0, 3.0)                   P2(0.0, -3.0)
    //       P4(-3.5, 2.0)          P3(-3.5, -2.0)
    //
    // e-puck proximity orentations (degrees)
    //           P7(10)   P0(350)
    //       P6(40)           P1(320)
    //   P5(90)                   P2(270)
    //       P4(160)          P3(200)

    transform.setOrigin( tf::Vector3(0.035, -0.010, 0.034) );
    q.setRPY(0, 0, 6.11);
    transform.setRotation(q);
    parent << epuckname << "/base_prox0";
    child << epuckname << "/base_link";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));

    transform.setOrigin( tf::Vector3(0.025, -0.025, 0.034) );
    q.setRPY(0, 0, 5.59);
    transform.setRotation(q);
    parent.str("");
    parent << epuckname << "/base_prox1";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));

    transform.setOrigin( tf::Vector3(0.000, -0.030, 0.034) );
    q.setRPY(0, 0, 4.71);
    transform.setRotation(q);
    parent.str("");
    parent << epuckname << "/base_prox2";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));

    transform.setOrigin( tf::Vector3(-0.035, -0.020, 0.034) );
    q.setRPY(0, 0, 3.49);
    transform.setRotation(q);
    parent.str("");
    parent << epuckname << "/base_prox3";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));

    transform.setOrigin( tf::Vector3(-0.035, 0.020, 0.034) );
    q.setRPY(0, 0, 2.8);
    transform.setRotation(q);
    parent.str("");
    parent << epuckname << "/base_prox4";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));

    transform.setOrigin( tf::Vector3(0.000, 0.030, 0.034) );
    q.setRPY(0, 0, 1.57);
    transform.setRotation(q);
    parent.str("");
    parent << epuckname << "/base_prox5";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));

    transform.setOrigin( tf::Vector3(0.025, 0.025, 0.034) );
    q.setRPY(0, 0, 0.70);
    transform.setRotation(q);
    parent.str("");
    parent << epuckname << "/base_prox6";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));

    transform.setOrigin( tf::Vector3(0.035, 0.010, 0.034) );
    q.setRPY(0, 0, 0.17);
    transform.setRotation(q);
    parent.str("");
    parent << epuckname << "/base_prox7";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
    //#############################################################################################################################################

    //#############################################################################################################################################
  	// Laserscan topic
    currentTimeMap = ros::Time::now();
    parent.str("");
    parent << epuckname << "/base_laser";
    //populate the LaserScan message
    laserMsg.header.stamp = ros::Time::now();
    laserMsg.header.frame_id = parent.str();
    laserMsg.angle_min = -M_PI/2.0;
    laserMsg.angle_max = M_PI/2.0;
    laserMsg.angle_increment = M_PI/18.0; // 10 degrees.
    //laserMsg.time_increment = (currentTimeMap-lastTimeMap).toSec()/180; //0.003; //(1 / laser_frequency) / (num_readings);
    //laserMsg.scan_time = (currentTimeMap-lastTimeMap).toSec();
    // The laser is placed in the center of the robot, but the proximity sensors are placed around the robot thus add "ROBOT_RADIUS" to get correct values.
    laserMsg.range_min = 0.005+ROBOT_RADIUS; // 0.5 cm + ROBOT_RADIUS.
    laserMsg.range_max = 0.05+ROBOT_RADIUS; // 5 cm + ROBOT_RADIUS.
    laserMsg.ranges.resize(19);
    laserMsg.intensities.resize(19);
    lastTimeMap = ros::Time::now();

    // We use the information from the 6 proximity sensors on the front side of the robot to get 19 laser scan points. The interpolation used is the following:
    // -90 degrees: P2
    // -80 degrees: 4/5*P2 + 1/5*P1
    // -70 degrees: 3/5*P2 + 2/5*P1
    // -60 degrees: 2/5*P2 + 3/5*P1
    // -50 degrees: 1/5*P2 + 4/5*P1
    // -40 degrees: P1
    // -30 degrees: 2/3*P1 + 1/3*P0
    // -20 degrees: 1/3*P1 + 2/3*P0
    // -10 degrees: P0
    // 0 degrees: 1/2*P0 + 1/2*P7
    // 10 degrees: P7
    // 20 degrees: 1/3*P6 + 2/3*P7
    // 30 degrees: 2/3*P6 + 1/3*P7
    // 40 degrees: P6
    // 50 degrees: 1/5*P5 + 4/5*P6
    // 60 degrees: 2/5*P5 + 3/5*P6
    // 70 degrees: 3/5*P5 + 2/5*P6
    // 80 degrees: 4/5*P5 + 1/5*P6
    // 90 degrees: P5

    float tempProx;
    tempProx = proxData[2];
    if(tempProx > 0) {
        laserMsg.ranges[0] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[0] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[0] = laserMsg.range_max;
        laserMsg.intensities[0] = 0;
    }

    tempProx = proxData[2]*4/5 + proxData[1]*1/5;
    if(tempProx > 0) {
        laserMsg.ranges[1] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[1] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[1] = laserMsg.range_max;
        laserMsg.intensities[1] = 0;
    }

    tempProx = proxData[2]*3/5 + proxData[1]*2/5;
    if(tempProx > 0) {
        laserMsg.ranges[2] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[2] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[2] = laserMsg.range_max;
        laserMsg.intensities[2] = 0;
    }

    tempProx = proxData[2]*2/5 + proxData[1]*3/5;
    if(tempProx > 0) {
        laserMsg.ranges[3] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[3] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[3] = laserMsg.range_max;
        laserMsg.intensities[3] = 0;
    }

    tempProx = proxData[2]*1/5 + proxData[1]*4/5;
    if(tempProx > 0) {
        laserMsg.ranges[4] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[4] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[4] = laserMsg.range_max;
        laserMsg.intensities[4] = 0;
    }

    tempProx = proxData[1];
    if(tempProx > 0) {
        laserMsg.ranges[5] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[5] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[5] = laserMsg.range_max;
        laserMsg.intensities[5] = 0;
    }

    tempProx = proxData[1]*2/3 + proxData[0]*1/3;
    if(tempProx > 0) {
        laserMsg.ranges[6] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[6] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[6] = laserMsg.range_max;
        laserMsg.intensities[6] = 0;
    }

    tempProx = proxData[1]*1/3 + proxData[0]*2/3;
    if(tempProx > 0) {
        laserMsg.ranges[7] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[7] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[7] = laserMsg.range_max;
        laserMsg.intensities[7] = 0;
    }

    tempProx = proxData[0];
    if(tempProx > 0) {
        laserMsg.ranges[8] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[8] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[8] = laserMsg.range_max;
        laserMsg.intensities[8] = 0;
    }

    tempProx = (proxData[0]+proxData[7])>>1;
    if(tempProx > 0) {
        laserMsg.ranges[9] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[9] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[9] = laserMsg.range_max;
        laserMsg.intensities[9] = 0;
    }

    tempProx = proxData[7];
    if(tempProx > 0) {
        laserMsg.ranges[10] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[10] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[10] = laserMsg.range_max;
        laserMsg.intensities[10] = 0;
    }

    tempProx = proxData[7]*2/3 + proxData[6]*1/3;
    if(tempProx > 0) {
        laserMsg.ranges[11] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[11] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[11] = laserMsg.range_max;
        laserMsg.intensities[11] = 0;
    }

    tempProx = proxData[7]*1/3 + proxData[6]*2/3;
    if(tempProx > 0) {
        laserMsg.ranges[12] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[12] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[12] = laserMsg.range_max;
        laserMsg.intensities[12] = 0;
    }

    tempProx = proxData[6];
    if(tempProx > 0) {
        laserMsg.ranges[13] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[13] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[13] = laserMsg.range_max;
        laserMsg.intensities[13] = 0;
    }

    tempProx = proxData[6]*4/5 + proxData[5]*1/5;
    if(tempProx > 0) {
        laserMsg.ranges[14] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[14] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[14] = laserMsg.range_max;
        laserMsg.intensities[14] = 0;
    }

    tempProx = proxData[6]*3/5 + proxData[5]*2/5;
    if(tempProx > 0) {
        laserMsg.ranges[15] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[15] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[15] = laserMsg.range_max;
        laserMsg.intensities[15] = 0;
    }

    tempProx = proxData[6]*2/5 + proxData[5]*3/5;
    if(tempProx > 0) {
        laserMsg.ranges[16] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[16] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[16] = laserMsg.range_max;
        laserMsg.intensities[16] = 0;
    }

    tempProx = proxData[6]*1/5 + proxData[5]*4/5;
    if(tempProx > 0) {
        laserMsg.ranges[17] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[17] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[17] = laserMsg.range_max;
        laserMsg.intensities[17] = 0;
    }

    tempProx = proxData[5];
    if(tempProx > 0) {
        laserMsg.ranges[18] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
        laserMsg.intensities[18] = tempProx;
    } else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
        laserMsg.ranges[18] = laserMsg.range_max;
        laserMsg.intensities[18] = 0;
    }

    for(i=0; i<19; i++) {
        if(laserMsg.ranges[i] > laserMsg.range_max) {
            laserMsg.ranges[i] = laserMsg.range_max;
        }
        if(laserMsg.ranges[i] < laserMsg.range_min) {
            laserMsg.ranges[i] = laserMsg.range_min;
        }
    }

    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.034) );
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    parent.str("");
    child.str("");
    parent << epuckname << "/base_laser";
    child << epuckname << "/base_link";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
    laserPublisher.publish(laserMsg);
    //#############################################################################################################################################

    //#############################################################################################################################################
  	// Odometry topic
    // The encoders values coming from the e-puck are 2 bytes signed int thus we need to handle the overflows otherwise the odometry will be wrong after a while (about 4 meters).
    if((leftStepsRawPrev>0) && (motorSteps[0]<0) && (abs(motorSteps[0]-leftStepsRawPrev)>30000)) {     // Overflow detected (positive).
        overflowCountLeft++;
    }
    if((leftStepsRawPrev<0) && (motorSteps[0]>0) && (abs(motorSteps[0]-leftStepsRawPrev)>30000)) {     // Overflow detected (negative).
        overflowCountLeft--;
    }
    motorPositionDataCorrect[0] = (overflowCountLeft*65536) + motorSteps[0];

    if((rightStepsRawPrev>0) && (motorSteps[1]<0) && (abs(motorSteps[1]-rightStepsRawPrev)>30000)) {     // Overflow detected (positive).
        overflowCountRight++;
    }
    if((rightStepsRawPrev<0) && (motorSteps[1]>0) && (abs(motorSteps[1]-rightStepsRawPrev)>30000)) {     // Overflow detected (negative).
        overflowCountRight--;
    }
    motorPositionDataCorrect[1] = (overflowCountRight*65536) + motorSteps[1];

    leftStepsRawPrev = motorSteps[0];
    rightStepsRawPrev = motorSteps[1];

    if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "left, right raw: " << motorSteps[0] << ", " << motorSteps[1] << std::endl;
    if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "left, right raw corrected: " << motorPositionDataCorrect[0] << ", " << motorPositionDataCorrect[1] << std::endl;

    // Compute odometry.
    leftStepsDiff = motorPositionDataCorrect[0]*MOT_STEP_DIST - leftStepsPrev; // Expressed in meters.
    rightStepsDiff = motorPositionDataCorrect[1]*MOT_STEP_DIST - rightStepsPrev;   // Expressed in meters.
    if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "left, right steps diff: " << leftStepsDiff << ", " << rightStepsDiff << std::endl;

    deltaTheta = (rightStepsDiff - leftStepsDiff)/WHEEL_DISTANCE;   // Expressed in radiant.
    deltaSteps = (rightStepsDiff + leftStepsDiff)/2;        // Expressed in meters.
    if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "delta theta, steps: " << deltaTheta << ", " << deltaSteps << std::endl;

    xPos += deltaSteps*cos(theta + deltaTheta/2);   // Expressed in meters.
    yPos += deltaSteps*sin(theta + deltaTheta/2);   // Expressed in meters.
    theta += deltaTheta;    // Expressed in radiant.
    if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "x, y, theta: " << xPos << ", " << yPos << ", " << theta << std::endl;

    leftStepsPrev = motorPositionDataCorrect[0]*MOT_STEP_DIST;     // Expressed in meters.
    rightStepsPrev = motorPositionDataCorrect[1]*MOT_STEP_DIST;    // Expressed in meters.

    // Publish the odometry message over ROS.
    odomMsg.header.stamp = ros::Time::now();
    odomMsg.header.frame_id = "odom";
    ss.str(std::string());
    ss << epuckname << "/base_link";
    odomMsg.child_frame_id = ss.str();
    odomMsg.pose.pose.position.x = xPos;
    odomMsg.pose.pose.position.y = yPos;
    odomMsg.pose.pose.position.z = 0;
    odomMsg.pose.covariance[0] = 0.001;
  	odomMsg.pose.covariance[7] = 0.001;
  	odomMsg.pose.covariance[14] = 1000.0;
  	odomMsg.pose.covariance[21] = 1000.0;
  	odomMsg.pose.covariance[28] = 1000.0;
  	odomMsg.pose.covariance[35] = 0.001;
    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    orientQuat = tf::createQuaternionMsgFromYaw(theta);
    odomMsg.pose.pose.orientation = orientQuat;
    currentTime = ros::Time::now();
    odomMsg.twist.twist.linear.x = deltaSteps / ((currentTime-lastTime).toSec());   // "deltaSteps" is the linear distance covered in meters from the last update (delta distance);
                                                                                    // the time from the last update is measured in seconds thus to get m/s we multiply them.
    odomMsg.twist.twist.angular.z = deltaTheta / ((currentTime-lastTime).toSec());  // "deltaTheta" is the angular distance covered in radiant from the last update (delta angle);
                                                                                    // the time from the last update is measured in seconds thus to get rad/s we multiply them.
    if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "time elapsed = " << (currentTime-lastTime).toSec() << " seconds" << std::endl;
    lastTime = ros::Time::now();

    odomPublisher.publish(odomMsg);

    // Publish the transform over tf.
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp = odomMsg.header.stamp;
    odomTrans.header.frame_id = odomMsg.header.frame_id;
    odomTrans.child_frame_id = odomMsg.child_frame_id;
    odomTrans.transform.translation.x = xPos;
    odomTrans.transform.translation.y = yPos;
    odomTrans.transform.translation.z = 0.0;
    odomTrans.transform.rotation = orientQuat;
    br.sendTransform(odomTrans);
    //#############################################################################################################################################

    //#############################################################################################################################################
  	// IMU topic
    ss.str(std::string());
    ss << epuckname << "/base_link";
    imuMsg.header.frame_id = ss.str();
    imuMsg.header.stamp = ros::Time::now();
    imuMsg.linear_acceleration.x = (accData[1]-2048.0)/800.0*STANDARD_GRAVITY; // 1 g = about 800, then transforms in m/s^2.
    imuMsg.linear_acceleration.y = (accData[0]-2048.0)/800.0*STANDARD_GRAVITY;
    imuMsg.linear_acceleration.z = (accData[2]-2048.0)/800.0*STANDARD_GRAVITY;
    imuMsg.linear_acceleration_covariance[0] = 0.01;
    imuMsg.linear_acceleration_covariance[1] = 0.0;
    imuMsg.linear_acceleration_covariance[2] = 0.0;
    imuMsg.linear_acceleration_covariance[3] = 0.0;
    imuMsg.linear_acceleration_covariance[4] = 0.01;
    imuMsg.linear_acceleration_covariance[5] = 0.0;
    imuMsg.linear_acceleration_covariance[6] = 0.0;
    imuMsg.linear_acceleration_covariance[7] = 0.0;
    imuMsg.linear_acceleration_covariance[8] = 0.01;
    if(DEBUG_IMU)std::cout << "[" << epuckname << "] " << "accel raw: " << accData[0] << ", " << accData[1] << ", " << accData[2] << std::endl;
    if(DEBUG_IMU)std::cout << "[" << epuckname << "] " << "accel (m/s2): " << ((accData[0]-2048.0)/800.0*STANDARD_GRAVITY) << ", " << ((accData[1]-2048.0)/800.0*STANDARD_GRAVITY) << ", " << ((accData[2]-2048.0)/800.0*STANDARD_GRAVITY) << std::endl;
    imuMsg.angular_velocity.x = (gyroData[0] - gyroOffset[0]) * DEG2RAD(GYRO_RAW2DPS); // rad/s
    imuMsg.angular_velocity.y = (gyroData[1] - gyroOffset[1]) * DEG2RAD(GYRO_RAW2DPS);
    imuMsg.angular_velocity.z = (gyroData[2] - gyroOffset[2]) * DEG2RAD(GYRO_RAW2DPS);
    imuMsg.angular_velocity_covariance[0] = 0.01;
    imuMsg.angular_velocity_covariance[1] = 0.0;
    imuMsg.angular_velocity_covariance[2] = 0.0;
    imuMsg.angular_velocity_covariance[3] = 0.0;
    imuMsg.angular_velocity_covariance[4] = 0.01;
    imuMsg.angular_velocity_covariance[5] = 0.0;
    imuMsg.angular_velocity_covariance[6] = 0.0;
    imuMsg.angular_velocity_covariance[7] = 0.0;
    imuMsg.angular_velocity_covariance[8] = 0.01;
    if(DEBUG_IMU)std::cout << "[" << epuckname << "] " << "gyro raw: " << gyroData[0] << ", " << gyroData[1] << ", " << gyroData[2] << std::endl;
  	if(DEBUG_IMU)std::cout << "[" << epuckname << "] " << "gyro (rad/s): " << imuMsg.angular_velocity.x << ", " << imuMsg.angular_velocity.y << ", " << imuMsg.angular_velocity.z << std::endl;
    // Pitch and roll computed assuming the aerospace rotation sequence Rxyz
  	double roll = atan2(imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z);
  	double pitch = atan2(-imuMsg.linear_acceleration.x, sqrt(imuMsg.linear_acceleration.y*imuMsg.linear_acceleration.y + imuMsg.linear_acceleration.z*imuMsg.linear_acceleration.z));
  	double yaw = 0.0;
    orientQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    imuMsg.orientation = orientQuat;
    imuMsg.orientation_covariance[0] = 0.01;
    imuMsg.orientation_covariance[1] = 0.0;
    imuMsg.orientation_covariance[2] = 0.0;
    imuMsg.orientation_covariance[3] = 0.0;
    imuMsg.orientation_covariance[4] = 0.01;
    imuMsg.orientation_covariance[5] = 0.0;
    imuMsg.orientation_covariance[6] = 0.0;
    imuMsg.orientation_covariance[7] = 0.0;
    imuMsg.orientation_covariance[8] = 0.01;
    if(DEBUG_IMU)std::cout << "[" << epuckname << "] " << "roll=" << roll << ", pitch=" << pitch << std::endl;
    imuPublisher.publish(imuMsg);
    //#############################################################################################################################################

    //#############################################################################################################################################
  	// Magnetic field topic
  	ss.str(std::string());
  	ss << epuckname << "/base_link";
  	magFieldMsg.header.frame_id = ss.str();
  	magFieldMsg.header.stamp = ros::Time::now();
  	magFieldMsg.magnetic_field_covariance[0] = 0.01;
  	magFieldMsg.magnetic_field_covariance[4] = 0.01;
  	magFieldMsg.magnetic_field_covariance[8] = 0.01;
  	magFieldMsg.magnetic_field.x = magneticField[0]/1000000.0; // given in Tesla
  	magFieldMsg.magnetic_field.y = magneticField[1]/1000000.0; // given in Tesla
  	magFieldMsg.magnetic_field.z = magneticField[2]/1000000.0; // given in Tesla
  	if(DEBUG_MAG_FIELD)std::cout << "[" << epuckname << "] " << "mag field (Tesla): " << magFieldMsg.magnetic_field.x << ", " << magFieldMsg.magnetic_field.y << ", " << magFieldMsg.magnetic_field.z << std::endl;
  	magFieldPublisher.publish(magFieldMsg);
  	// Magnetic field vector (normalized) topic
  	// The resulting vector will have a length of 1 meter
  	ss.str(std::string());
  	ss << epuckname << "/base_link";
  	magFieldVectorMsg.header.frame_id = ss.str();
  	magFieldVectorMsg.header.stamp = ros::Time::now();
  	magFieldVectorMsg.type = visualization_msgs::Marker::ARROW;
  	geometry_msgs::Point p;
  	// Start point
  	p.x = 0.0;
  	p.y = 0.0;
  	p.z = 0.0;
  	magFieldVectorMsg.points.clear();
  	magFieldVectorMsg.points.push_back(p);
  	double mod = sqrt(magFieldMsg.magnetic_field.x*magFieldMsg.magnetic_field.x + magFieldMsg.magnetic_field.y*magFieldMsg.magnetic_field.y);
  	// End point
  	p.x = magFieldMsg.magnetic_field.x/mod;
  	p.y = magFieldMsg.magnetic_field.y/mod;
  	magFieldVectorMsg.points.push_back(p);
  	magFieldVectorMsg.scale.x = 0.002;
  	magFieldVectorMsg.scale.y = 0.003;
  	magFieldVectorMsg.scale.z = 0.005;
  	magFieldVectorMsg.color.a = 1.0;
  	magFieldVectorMsg.color.r = 1.0;
  	magFieldVectorMsg.color.g = 1.0;
  	magFieldVectorMsg.color.b = 0.0;
  	magFieldVectorPublisher.publish(magFieldVectorMsg);
  	//#############################################################################################################################################

    //#############################################################################################################################################
  	// Microphone topic
    ss.str(std::string());
    ss << epuckname << "/base_link";
    microphoneMsg.header.frame_id = ss.str();
    microphoneMsg.header.stamp = ros::Time::now();
    microphoneMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    microphoneMsg.pose.position.x = 0.15;
    microphoneMsg.pose.position.y = 0;
    microphoneMsg.pose.position.z = 0.11;
    orientQuat = tf::createQuaternionMsgFromYaw(0);
    microphoneMsg.pose.orientation = orientQuat;
    microphoneMsg.scale.z = 0.01;
    microphoneMsg.color.a = 1.0;
    microphoneMsg.color.r = 1.0;
    microphoneMsg.color.g = 1.0;
    microphoneMsg.color.b = 1.0;
    ss.str(std::string());
    ss << "mic: [" << micVolume[0] << ", " << micVolume[1] << ", " << micVolume[2] << ", " << micVolume[3] << "]";
    microphoneMsg.text = ss.str();
    microphonePublisher.publish(microphoneMsg);
    //#############################################################################################################################################

    //#############################################################################################################################################
  	// Time of flight topic
		distSensMsg.range = distanceMm/1000.0;
		if(distSensMsg.range > distSensMsg.max_range) {
			distSensMsg.range = distSensMsg.max_range;
		}
		if(distSensMsg.range < distSensMsg.min_range) {
			distSensMsg.range = distSensMsg.min_range;
		}
		distSensMsg.header.stamp = ros::Time::now();
		distSensPublisher.publish(distSensMsg);
    transform.setOrigin( tf::Vector3(0.035, 0.0, 0.034) );
    q.setRPY(0, -0.21, 0.0);
    transform.setRotation(q);
    parent.str("");
    parent << epuckname << "/base_dist_sens";
    child.str("");
    child << epuckname << "/base_link";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
    //#############################################################################################################################################

    //#############################################################################################################################################
  	// Battery topic
  	ss.str(std::string());
  	ss << epuckname << "/base_link";
  	battMsg.header.frame_id = ss.str();
  	battMsg.header.stamp = ros::Time::now();
  	battMsg.voltage = (float)batteryRaw / COEFF_ADC_TO_VOLT;
  	battMsg.percentage = (battMsg.voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE);
  	battMsg.present = 1;
  	if(DEBUG_BATTERY)std::cout << "[" << epuckname << "] " << "battery V: " << battMsg.voltage << ", " << battMsg.percentage*100.0 << " %" << std::endl;
  	battPublisher.publish(battMsg);
  	//#############################################################################################################################################

    //#############################################################################################################################################
  	// Ground sensor topic
    if(ground_sensors_enabled) {
        ss.str(std::string());
        ss << epuckname << "/base_link";
        floorMsg.header.frame_id = ss.str();
        floorMsg.header.stamp = ros::Time::now();
        floorMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        floorMsg.pose.position.x = 0.15;
        floorMsg.pose.position.y = 0;
        floorMsg.pose.position.z = 0.13;
        orientQuat = tf::createQuaternionMsgFromYaw(0);
        floorMsg.pose.orientation = orientQuat;
        floorMsg.scale.z = 0.01;
        floorMsg.color.a = 1.0;
        floorMsg.color.r = 1.0;
        floorMsg.color.g = 1.0;
        floorMsg.color.b = 1.0;
        ss.str(std::string());
        ss << "floor: [" << groundProx[0] << ", " << groundProx[1] << ", " << groundProx[2] << "]";
        floorMsg.text = ss.str();
        floorPublisher.publish(floorMsg);
    }
    //#############################################################################################################################################

    //#############################################################################################################################################
  	// Camera image topic
    if(camera_enabled) {
        if(newImageReceived) {
            newImageReceived = false;
            cv::Mat rgb888;
            cv_bridge::CvImage out_msg;
            out_msg.header.stamp = ros::Time::now();; // Same timestamp and tf frame as input image
            if(camMode == CAM_MODE_RGB) {
                rgb888 = cv::Mat(camHeight, camWidth, CV_8UC3);
                RGB565toRGB888(camWidth, camHeight, &camData[3], rgb888.data);
                out_msg.encoding = sensor_msgs::image_encodings::RGB8;
            } else {
                rgb888 = cv::Mat(camHeight, camWidth, CV_8UC1);
                rgb888.data = &camData[3];
                out_msg.encoding = sensor_msgs::image_encodings::MONO8;
            }
            out_msg.image = rgb888;
            imagePublisher.publish(out_msg.toImageMsg());
        }
    }
    //#############################################################################################################################################


}

void handlerVelocity(const geometry_msgs::Twist::ConstPtr& msg) {
    // Controls the velocity of each wheel based on linear and angular velocities.
    double linear = msg->linear.x/3;    // Divide by 3 to adapt the values received from the rviz "teleop" module that are too high.
    double angular = msg->angular.z/3;

    // Kinematic model for differential robot.
    double wl = (linear - (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;
    double wr = (linear + (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;

    // At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
    speedLeft = int(wl * 1000.0);
    speedRight = int(wr * 1000.0);

    output_buffer[2] = speedLeft & 0xFF;
    output_buffer[3] = (speedLeft>>8) & 0xFF;
    output_buffer[4] = speedRight & 0xFF;
    output_buffer[5] = (speedRight>>8) & 0xFF;

    if(DEBUG_SPEED_RECEIVED)std::cout << "[" << epuckname << "] " << "new speed: " << speedLeft << ", " << speedRight << std::endl;

}

void handlerLED(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    // Controls the state of each LED on the standard robot
    for (int i = 0; i < LED_NUMBER; i++) {
		    if(msg->data[i] == 0) {
			       output_buffer[7] &= ~(1<<i);
        } else {
			       output_buffer[7] |= (1<<i);
		    }
    }

    if(DEBUG_LED_RECEIVED) {
        std::cout << "[" << epuckname << "] " << "new LED status: " << std::endl;
        for (int i = 0; i < LED_NUMBER; i++)
            std::cout << msg->data[i] << ", ";
    }
}

void handlerRgbLeds(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    output_buffer[8] = msg->data[0];		// LED2 red
    output_buffer[9] = msg->data[1];		// LED2 green
    output_buffer[10] = msg->data[2];		// LED2 blue
    output_buffer[11] = msg->data[3];		// LED4 red
    output_buffer[12] = msg->data[4];		// LED4 green
    output_buffer[13] = msg->data[5];		// LED4 blue
    output_buffer[14] = msg->data[6];		// LED6 red
    output_buffer[15] = msg->data[7];		// LED6 green
    output_buffer[16] = msg->data[8];		// LED6 blue
    output_buffer[17] = msg->data[9];		// LED8 red
    output_buffer[18] = msg->data[10];	// LED8 green
    output_buffer[19] = msg->data[11];	// LED8 blue

    if(DEBUG_RGB_RECEIVED) {
        std::cout << "[" << epuckname << "] " << "new RGB status: " << std::endl;
        for (int i = 0; i < RGB_LED_NUMBER; i++) {
            std::cout << i << ": " << msg->data[i*3] << ", " << msg->data[i*3+1] << ", " << msg->data[i*3+2];
        }
    }
}

int main(int argc,char *argv[]) {

   int robotId = 0;
   double init_xpos, init_ypos, init_theta;
   int rosRate = 0;
   unsigned int bufIndex = 0;
   int i = 0;
   std::string epuckAddress("");

    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "epuck_driver_cpp");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle np("~"); // Private.
    ros::NodeHandle n; // Public.

    np.getParam("epuck2_id", robotId);
    np.param<std::string>("epuck2_address", epuckAddress, "");
    np.param<std::string>("epuck2_name", epuckname, "epuck2");
    np.param("xpos", init_xpos, 0.0);
    np.param("ypos", init_ypos, 0.0);
    np.param("theta", init_theta, 0.0);
    np.param("camera", camera_enabled, false);
    np.param("cam_width", camWidth, 160);
    np.param("cam_height", camHeight, 2);
    np.param("cam_zoom", camZoom, 1);
    np.param("cam_mode", camMode, 0);
    np.param("cam_x_offset", camXoffset, 240);
    np.param("cam_y_offset", camYoffset, 239);
    np.param("floor", ground_sensors_enabled, false);
    //np.param("ros_rate", rosRate, 7);

    if(camWidth < 0 || camWidth > 640) {
        camWidth = 160;
    }
    if(camHeight < 0 || camHeight > 480) {
        camHeight = 2;
    }
    if(camZoom < 0 || camZoom > 8) {
        camZoom = 1;
    }
    if(camMode < 0 || camMode > 1) {
        camMode = CAM_MODE_GRAY;
    }
    if(camXoffset < 0 || camXoffset > 640) {
        camXoffset = (640-camWidth*camZoom)/2;  // Center the slice.
    }
    if(camYoffset < 0 || camYoffset > 480) {
        camYoffset = (480-camHeight*camZoom)/2;  // Center the slice.
    }

    if(camWidth*camHeight*(camMode+1) > CAM_MAX_BUFFER_SIZE) {  // If the parameters exceed the maximum buffer then use default values for all params.
        std::cerr << "[" << epuckname << "] " << "Camera parameters exceed max buffer size, using default values" << std::endl;
        camWidth = 160;
        camHeight = 2;
        camZoom = 1;
        camMode = CAM_MODE_GRAY;
        camXoffset = (640-camWidth*camZoom)/2;
        camYoffset = (480-camWidth*camZoom)/2;
    }

    if(DEBUG_ROS_PARAMS) {
        std::cout << "[" << epuckname << "] " << "epuck id: " << robotId << std::endl;
        std::cout << "[" << epuckname << "] " << "epuck address: " << epuckAddress << std::endl;
        std::cout << "[" << epuckname << "] " << "epuck name: " << epuckname << std::endl;
        std::cout << "[" << epuckname << "] " << "init pose: " << init_xpos << ", " << init_ypos << ", " << theta << std::endl;
        std::cout << "[" << epuckname << "] " << "ground sensors enabled: " << ground_sensors_enabled << std::endl;
        std::cout << "[" << epuckname << "] " << "camera enabled: " << camera_enabled << std::endl;
        //std::cout << "[" << epuckname << "] " << "ros rate: " << rosRate << std::endl;
        std::cout << "[" << epuckname << "] " << "image size: " << camWidth << " x " << camHeight << std::endl;
        std::cout << "[" << epuckname << "] " << "image zoom: " << camZoom << std::endl;
        std::cout << "[" << epuckname << "] " << "image mode: " << (camMode?"RGB":"GRAY") << std::endl;
        std::cout << "[" << epuckname << "] " << "image offset: " << camXoffset << ", " << camYoffset << std::endl;
    }

    if(epuckAddress.compare("")==0) {   // Search the robot id
        if(initConnectionWithRobotId(robotId)<0) {
            return -1;
        }
    } else {    // Connect directly to the address
        if(initConnectionWithRobotAddress(epuckAddress.c_str())<0) {
            return -1;
        }
    }

    memset(input_buffer, 0x00, INPUT_BUFF_SIZE);
    memset(output_buffer, 0x00, OUTPUT_BUFF_SIZE);
    output_buffer[0] = 0xF7; //-0x09;
    output_buffer[20] = 0xF8; //-0x08;
    output_buffer[21] = 0x00;

    imuPublisher = n.advertise<sensor_msgs::Imu>("imu", 10);

    for(i=0; i<8; i++) {
        std::stringstream ss;
        ss.str("");
        ss << "proximity" << i;
        proxPublisher[i] = n.advertise<sensor_msgs::Range>(ss.str(), 10);
        //proxMsg[i] = new sensor_msgs::Range();
        proxMsg[i].radiation_type = sensor_msgs::Range::INFRARED;
        ss.str("");
        ss << epuckname << "/base_prox" << i;
        proxMsg[i].header.frame_id =  ss.str();
        proxMsg[i].field_of_view = 0.26;    // About 15 degrees...to be checked!
        proxMsg[i].min_range = 0.005;       // 0.5 cm.
        proxMsg[i].max_range = 0.05;        // 5 cm.
    }

    laserPublisher = n.advertise<sensor_msgs::LaserScan>("scan", 10);

    odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 10);
    currentTime = ros::Time::now();
    lastTime = ros::Time::now();

    microphonePublisher = n.advertise<visualization_msgs::Marker>("microphone", 10);

		distSensPublisher = n.advertise<sensor_msgs::Range>("dist_sens", 10);
		distSensMsg.radiation_type = sensor_msgs::Range::INFRARED;
		std::stringstream ss;
		ss.str("");
		ss << epuckname << "/base_dist_sens";
		distSensMsg.header.frame_id =  ss.str();
		distSensMsg.field_of_view = 0.43;	// About 25 degrees (+/- 12.5)
		distSensMsg.min_range = 0.005;		// 5 mm.
		distSensMsg.max_range = 2;			// 2 m.

    magFieldPublisher = n.advertise<sensor_msgs::MagneticField>("mag_field", 10);
  	magFieldVectorPublisher = n.advertise<visualization_msgs::Marker>("mag_field_vector", 10);

  	battPublisher = n.advertise<sensor_msgs::BatteryState>("battery", 10);

    cmdVelSubscriber = n.subscribe("mobile_base/cmd_vel", 10, handlerVelocity);
    cmdLedSubscriber = n.subscribe("mobile_base/cmd_led", 10, handlerLED);
    cmdRgbLedsSubscriber = n.subscribe("mobile_base/rgb_leds", 10, handlerRgbLeds);

    if(ground_sensors_enabled) {
        floorPublisher = n.advertise<visualization_msgs::Marker>("floor", 10);
    }

    if(camera_enabled) {
        imageSize = camWidth*camHeight*(camMode+1)+3; // The image data header contains "mode", "width", "height" in the first 3 bytes.
        camData = (unsigned char *) malloc (imageSize);

        // Configure camera params.
        char buff[30];
        struct timeval timeout;
        fd_set readfds;
        int retval;
        int bytesRead;
        memset(buff, 0x0, 30);
        sprintf(buff,"J,%d,%d,%d,%d,%d,%d\r", camMode, camWidth, camHeight, camZoom, camXoffset, camYoffset);
        FD_ZERO(&readfds);
        FD_SET(rfcommSock, &readfds);
        write(rfcommSock, buff, strlen(buff));
        bytesRead = 0;
        while(bytesRead < 3) {
            timeout.tv_sec=READ_TIMEOUT_SEC;
            timeout.tv_usec=READ_TIMEOUT_USEC;
            retval = select(rfcommSock+1, &readfds, NULL, NULL, &timeout);
            if (retval!=0) {
                int n = read(rfcommSock, &buff[bytesRead], 3-bytesRead);
                if(DEBUG_CAMERA_INIT)std::cout << "[" << epuckname << "] " << "cam init: read " << bytesRead << " / " << 3 << "bytes" << std::endl;
                bytesRead += n;
            } else {
                if(DEBUG_CAMERA_INIT)std::cout << "[" << epuckname << "] " << "cam init: read timeout" << std::endl;
                break;
            }
        }
        if(bytesRead == 3) {
            if(DEBUG_CAMERA_INIT)std::cout << "[" << epuckname << "] " << "camera init correctly (" << buff[0] << buff[1] << buff[2] << ")" << std::endl;
            imagePublisher = n.advertise<sensor_msgs::Image>("camera", 1);
        } else {
            if(DEBUG_CAMERA_INIT)std::cout << "[" << epuckname << "] " << "cannot init camera" << std::endl;
            camera_enabled = false;
        }
    }

    theta = init_theta;
    xPos = init_xpos;
    yPos = init_ypos;

    //ros::Rate loop_rate(rosRate);

    while (ros::ok()) {
        updateSensorsAndActuators();
        updateRosInfo();
        ros::spinOnce();
        //loop_rate.sleep();    // Do not call "sleep" otherwise the bluetooth communication will hang.
                                // We communicate as fast as possible, this shouldn't be a problem...
        if(consecutiveReadTimeout >= MAX_CONSECUTIVE_TIMEOUT) { // We have connection problems, stop here.
            break;
        }
    }

    closeConnection();

    if(camera_enabled) {
        free(camData);
    }

}
