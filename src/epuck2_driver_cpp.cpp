
#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
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
#include <opencv/cv.h>
//#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/BatteryState.h>

#define DEBUG_CONNECTION_INIT 1
#define DEBUG_ROS_PARAMS 1
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_ODOMETRY 0
#define DEBUG_IMU 0
#define DEBUG_SPEED_RECEIVED 0
#define DEBUG_LED_RECEIVED 0
#define DEBUG_RGB_RECEIVED 0
#define DEBUG_MAG_FIELD 0
#define DEBUG_BATTERY 0

#define READ_TIMEOUT_SEC 10
#define READ_TIMEOUT_USEC 0
#define MAX_CONNECTION_TRIALS 3

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

#define RESISTOR_R1             220 //kohm
#define RESISTOR_R2             330 //kohm
#define VOLTAGE_DIVIDER         (1.0f * RESISTOR_R2 / (RESISTOR_R1 + RESISTOR_R2))
#define VREF                    3.0f //volt correspond to the voltage on the VREF+ pin
#define ADC_RESOLUTION          4096
#define COEFF_ADC_TO_VOLT       ((1.0f * ADC_RESOLUTION * VOLTAGE_DIVIDER) / VREF) //convertion from adc value to voltage
#define MAX_VOLTAGE				4.2f	//volt
#define MIN_VOLTAGE				3.4f	//volt

// Communication variables
struct sockaddr_in robot_addr;
int fd;
unsigned char command[21];
unsigned char header, sensor[104];
int bytes_sent = 0, bytes_recv = 0;
bool camera_enabled, ground_sensors_enabled;
uint8_t expected_recv_packets = 0;
bool newImageReceived = false;
std::string epuckAddress("");
	

// Sensors data variables
unsigned char image[160*120*2];
float acceleration, orientation, inclination;		/**< acceleration data*/
int16_t accData[3];
int16_t gyroRaw[3];
float magneticField[3];
uint8_t temperature;
int proxData[8]; /**< proximity sensors data*/
int lightAvg;										/**< light sensor data*/
uint16_t distanceMm;
uint16_t micVolume[4];								/**< microphone data*/
int16_t motorSteps[2];
uint16_t batteryRaw;
uint8_t microSdState;
uint8_t irCheck, irAddress, irData;
uint8_t selector;
int16_t groundProx[3], groundAmbient[3];
uint8_t buttonState;


// ROS variables
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
int16_t gyroOffset[3] = {0, 0, 0}; // Used if making an initial calibration of the gyro.
int speedLeft = 0, speedRight = 0;


// General variables
std::string epuckname;


int initConnectionWithRobot() {
	int ret_value;
	std::stringstream ss;
    struct timeval tv;
    socklen_t len = sizeof(tv);
	uint8_t trials = 0;
	
   	robot_addr.sin_family = AF_INET;
   	robot_addr.sin_addr.s_addr = inet_addr(epuckAddress.c_str());
   	robot_addr.sin_port = htons(1000);

	if(DEBUG_CONNECTION_INIT)fprintf(stderr, "Try to connect to %s:%d (TCP)\n", inet_ntoa(robot_addr.sin_addr), htons(robot_addr.sin_port));
	
   	fd = socket(AF_INET, SOCK_STREAM, 0);
	if(fd < 0) {
		perror("TCP cannot create socket: ");
		return -1;
	}
	
	// Set to non-blocking mode during connection otherwise it will block for too much time if the robot isn't ready to accept connections
    if( (ret_value = fcntl(fd, F_GETFL, 0)) < 0) {
		perror("Cannot get flag status: ");
		return -1;
    }// else {
	//	if((ret_value & O_NONBLOCK) > 0) {
	//		std::cout << "Non-blocking socket" << std::endl;
	//	} else {
	//		std::cout << "Blocking socket" << std::endl;
	//	}
	//}
	ret_value |= O_NONBLOCK;
	if(fcntl(fd, F_SETFL, ret_value) < 0) {
		perror("Cannot set non-blocking mode: ");
		return -1;
	}
	
	while(trials < MAX_CONNECTION_TRIALS) {
		// Connection to the robot (server).
		ret_value = connect(fd, (struct sockaddr *) &robot_addr, sizeof(robot_addr));					
		if (ret_value == 0) {
			break;
		} else {
			trials++;
			if(DEBUG_CONNECTION_INIT)fprintf(stderr, "Connection trial %d\n", trials);
			sleep(3);			
		}
	}
	
	if(trials == MAX_CONNECTION_TRIALS) {
		ss.str("");
		ss << "[" << epuckname << "] " << "Error, can't connect to tcp socket";
		//if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
		return -1;
	}
	
	// Set to blocking mode.
    if( (ret_value = fcntl(fd, F_GETFL, 0)) < 0) {
		perror("Cannot get flag status: ");
		return -1;
    }// else {
	//	if((ret_value & O_NONBLOCK) > 0) {
	//		std::cout << "Non-blocking socket" << std::endl;
	//	} else {
	//		std::cout << "Blocking socket" << std::endl;
	//	}
	//}
	ret_value &= (~O_NONBLOCK);
	if(fcntl(fd, F_SETFL, ret_value) < 0) {
		perror("Cannot set blocking mode: ");
		return -1;
	}	
	
	// Set the reception timeout. This is used when blocking mode is activated after connection.
	tv.tv_sec = READ_TIMEOUT_SEC;
	tv.tv_usec = READ_TIMEOUT_USEC;
	ret_value = setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	if(ret_value < 0) {
		perror("Cannot set rx timeout: ");
		return -1;
	}	
	//ret_value = getsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, &len);
	//if(ret_value < 0) {
	//	perror("Cannot read rx timeout: ");
	//} else {
	//	std::cout << "rx timeout: " << tv.tv_sec << " s and " <<  tv.tv_usec << " us" << std::endl;
	//}	
	
	return 0;
}

void closeConnection() {
    std::stringstream ss;
    if(close(fd) < 0) {
        ss.str("");
        ss << "[" << epuckname << "] " << "Can't close tcp socket";
        if(DEBUG_CONNECTION_INIT)perror(ss.str().c_str());
    }
}

void updateSensorsAndActuators() {
	int bytes_sent = 0, bytes_recv = 0, ret_value;
	long mantis = 0;
	short exp = 0;
	float flt = 0.0;
				
	bytes_sent = 0;
	while(bytes_sent < sizeof(command)) {
		bytes_sent += send(fd, (char *)&command[bytes_sent], sizeof(command)-bytes_sent, 0);
	}
	command[2] = 0; // Stop proximity calibration.
			
	while(expected_recv_packets > 0) {
		bytes_recv = recv(fd, (char *)&header, 1, 0);
		if (bytes_recv <= 0) {
			closeConnection();
			if(initConnectionWithRobot() < 0) {
				std::cerr << "Lost connection with the robot" << std::endl;
				exit(1);
			} else {
				return; // Wait for the next sensor request
			}
		}
		
		switch(header) {
			case 0x01:	// Camera.				
				bytes_recv = 0;
				while(bytes_recv < sizeof(image)) {
					ret_value = recv(fd, (char *)&image[bytes_recv], sizeof(image)-bytes_recv, 0);
					if(ret_value <= 0) {
						closeConnection();
						if(initConnectionWithRobot() < 0) {
							std::cerr << "Lost connection with the robot" << std::endl;
							exit(1);
						} else {
							return; // Wait for the next sensor request
						}
					} else {
						bytes_recv += ret_value;
						//std::cout << "image read = " << bytes_recv << std::endl;
					}
				}
				
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "camera read correctly" << std::endl;
				newImageReceived = true;
				
				//red = image[0] & 0xf8;
				//green = image[0] << 5;
				//green += (image[1] & 0xf8) >> 3;
				//blue = image[1] << 3;
				//printf("1st pixel = %d, %d, %d\r\n", red, green, blue);
				break;
			
			case 0x02: // Sensors.
				bytes_recv = 0;
				while(bytes_recv < sizeof(sensor)) {
					ret_value = recv(fd, (char *)&sensor[bytes_recv], sizeof(sensor)-bytes_recv, 0);
					if(ret_value <= 0) {
						closeConnection();
						if(initConnectionWithRobot() < 0) {
							std::cerr << "Lost connection with the robot" << std::endl;
							exit(1);
						} else {
							return; // Wait for the next sensor request
						}
					} else {
						bytes_recv += ret_value;
						//std::cout << "sensors read = " << bytes_recv << std::endl;
					}
				}

                accData[0] = sensor[0] + sensor[1]*256;
                accData[1] = sensor[2] + sensor[3]*256;
                accData[2] = sensor[4] + sensor[5]*256;			
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "acc: " << accData[0] << "," << accData[1] << "," << accData[2] << std::endl;
				
				// Compute acceleration
				mantis = (sensor[6] & 0xff) + ((sensor[7] & 0xffl) << 8) + (((sensor[8] &0x7fl) | 0x80) << 16);
				exp = (sensor[9] & 0x7f) * 2 + ((sensor[8] & 0x80) ? 1 : 0);
				if (sensor[9] & 0x80) {
					mantis = -mantis;
				}
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				acceleration=flt;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "acceleration: " << acceleration << std::endl;

				// Compute orientation.
				mantis = (sensor[10] & 0xff) + ((sensor[11] & 0xffl) << 8) + (((sensor[12] &0x7fl) | 0x80) << 16);
				exp = (sensor[13] & 0x7f) * 2 + ((sensor[12] & 0x80) ? 1 : 0);
				if (sensor[13] & 0x80)
					mantis = -mantis;
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				orientation=flt;
				if (orientation < 0.0 )
					orientation=0.0;
				if (orientation > 360.0 )
					orientation=360.0;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "orientation: " << orientation << std::endl;

				// Compute inclination.
				mantis = (sensor[14] & 0xff) + ((sensor[15] & 0xffl) << 8) + (((sensor[16] &0x7fl) | 0x80) << 16);
				exp = (sensor[17] & 0x7f) * 2 + ((sensor[16] & 0x80) ? 1 : 0);
				if (sensor[17] & 0x80)
					mantis = -mantis;
				flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
				inclination=flt;
				if (inclination < 0.0 )
					inclination=0.0;
				if (inclination > 180.0 )
					inclination=180.0;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "inclination: " << inclination << std::endl;

				// Gyro
				gyroRaw[0] = sensor[18]+sensor[19]*256;
				gyroRaw[1] = sensor[20]+sensor[21]*256;
				gyroRaw[2] = sensor[22]+sensor[23]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "gyro: " << gyroRaw[0] << "," << gyroRaw[1] << "," << gyroRaw[2] << std::endl;					

				// Magnetometer
				magneticField[0] = *((float*)&sensor[24]);
				magneticField[1] = *((float*)&sensor[28]);
				magneticField[2] = *((float*)&sensor[32]);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "mag: " << magneticField[0] << "," << magneticField[1] << "," << magneticField[2] << std::endl;	

				// Temperature.
				temperature = sensor[36];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "temperature: " << (int)temperature << std::endl;

				// Proximity sensors data.
				proxData[0] = sensor[37]+sensor[38]*256;
				proxData[1] = sensor[39]+sensor[40]*256;
				proxData[2] = sensor[41]+sensor[42]*256;
				proxData[3] = sensor[43]+sensor[44]*256;
				proxData[4] = sensor[45]+sensor[46]*256;
				proxData[5] = sensor[47]+sensor[48]*256;
				proxData[6] = sensor[49]+sensor[50]*256;
				proxData[7] = sensor[51]+sensor[52]*256;
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
				lightAvg += (sensor[53]+sensor[54]*256);
				lightAvg += (sensor[55]+sensor[56]*256);
				lightAvg += (sensor[57]+sensor[58]*256);
				lightAvg += (sensor[59]+sensor[60]*256);
				lightAvg += (sensor[61]+sensor[62]*256);
				lightAvg += (sensor[63]+sensor[64]*256);
				lightAvg += (sensor[65]+sensor[66]*256);
				lightAvg += (sensor[67]+sensor[68]*256);
				lightAvg = (int) (lightAvg/8);
				lightAvg = (lightAvg>4000)?4000:lightAvg;
				if(lightAvg<0) {
					lightAvg=0;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "lightAvg: " << lightAvg << std::endl;
				
				// ToF
				distanceMm = (uint16_t)((uint8_t)sensor[70]<<8)|((uint8_t)sensor[69]);
				if(distanceMm > 2000) {
					distanceMm = 2000;
				}
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "distanceMm: " << distanceMm << "(" << (int)sensor[69] << "," << (int)sensor[70] << ")" << std::endl;

				// Microphone
				micVolume[0] = ((uint8_t)sensor[71]+(uint8_t)sensor[72]*256);
				micVolume[1] = ((uint8_t)sensor[73]+(uint8_t)sensor[74]*256);
				micVolume[2] = ((uint8_t)sensor[75]+(uint8_t)sensor[76]*256);
				micVolume[3] = ((uint8_t)sensor[77]+(uint8_t)sensor[78]*256);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "mic: " << micVolume[0] << "," << micVolume[1] << "," << micVolume[2] << "," << micVolume[3] << std::endl;

				// Left steps
				motorSteps[0] = (sensor[79]+sensor[80]*256);
				// Right steps
				motorSteps[1] = (sensor[81]+sensor[82]*256);
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "steps: " << motorSteps[0] << "," << motorSteps[1] << std::endl;

				// Battery
				batteryRaw = (uint8_t)sensor[83]+(uint8_t)sensor[84]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "batteryRaw: " << batteryRaw << std::endl;
				
				// Micro sd state.
				microSdState = sensor[85];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "microSdState: " << (int)microSdState << std::endl;

				// Tv remote.
				irCheck = sensor[86];
				irAddress = sensor[87];
				irData = sensor[88];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "tv remote: " << (int)irCheck << "," << (int)irAddress << "," << (int)irData << std::endl;

				// Selector.
				selector = sensor[89];
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "selector: " << (int)selector << std::endl;

				// Ground sensor proximity.
				groundProx[0] = sensor[90]+sensor[91]*256;
				groundProx[1] = sensor[92]+sensor[93]*256;
				groundProx[2] = sensor[94]+sensor[95]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "groundProx: " << groundProx[0] << "," << groundProx[1] << "," << groundProx[2] << std::endl;

				// Ground sensor ambient light.
				groundAmbient[0] = sensor[96]+sensor[97]*256;
				groundAmbient[1] = sensor[98]+sensor[99]*256;
				groundAmbient[2] = sensor[100]+sensor[101]*256;
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "groundAmbient: " << groundAmbient[0] << "," << groundAmbient[1] << "," << groundAmbient[2] << std::endl;

				// Button state.
				buttonState = sensor[102];			
				if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "buttonState: " << (int)buttonState << std::endl;
				
				break;
			
			case 0x03:
				printf("empty packet\r\n");
				break;
			
			default:
				printf("unexpected packet\r\n");
				break;
		}
		expected_recv_packets--;
	}

	if(camera_enabled) {
		expected_recv_packets = 2;
	} else {
		expected_recv_packets = 1;
	}
	
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
	// Exchange x and y to be compatible with e-puck 1.x
	imuMsg.linear_acceleration.x = (accData[1])/800.0*STANDARD_GRAVITY; // 1 g = about 800, then transforms in m/s^2.
	imuMsg.linear_acceleration.y = (accData[0])/800.0*STANDARD_GRAVITY;
	imuMsg.linear_acceleration.z = (accData[2])/800.0*STANDARD_GRAVITY;
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
	if(DEBUG_IMU)std::cout << "[" << epuckname << "] " << "accel (m/s2): " << imuMsg.linear_acceleration.x << ", " << imuMsg.linear_acceleration.y << ", " << imuMsg.linear_acceleration.z << std::endl;
	imuMsg.angular_velocity.x = (gyroRaw[0] - gyroOffset[0]) * DEG2RAD(GYRO_RAW2DPS); // rad/s
	imuMsg.angular_velocity.y = (gyroRaw[1] - gyroOffset[1]) * DEG2RAD(GYRO_RAW2DPS);
	imuMsg.angular_velocity.z = (gyroRaw[2] - gyroOffset[2]) * DEG2RAD(GYRO_RAW2DPS);
	imuMsg.angular_velocity_covariance[0] = 0.01;
	imuMsg.angular_velocity_covariance[1] = 0.0;
	imuMsg.angular_velocity_covariance[2] = 0.0;
	imuMsg.angular_velocity_covariance[3] = 0.0;
	imuMsg.angular_velocity_covariance[4] = 0.01;
	imuMsg.angular_velocity_covariance[5] = 0.0;
	imuMsg.angular_velocity_covariance[6] = 0.0;
	imuMsg.angular_velocity_covariance[7] = 0.0;
	imuMsg.angular_velocity_covariance[8] = 0.01;
	if(DEBUG_IMU)std::cout << "[" << epuckname << "] " << "gyro raw: " << gyroRaw[0] << ", " << gyroRaw[1] << ", " << gyroRaw[2] << std::endl;
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
	ss.str(std::string());;
	ss << "mic: [" << micVolume[0] << ", " << micVolume[1] << ", " << micVolume[2] << ", " << micVolume[3] << "]";
	microphoneMsg.text = ss.str();
	microphonePublisher.publish(microphoneMsg);
	//#############################################################################################################################################	
	
	//#############################################################################################################################################	
	// Time of flight topic
	distSensMsg.range = (float)distanceMm/1000.0;
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
	// Camera image topic
    if(camera_enabled) {
        if(newImageReceived) {
            newImageReceived = false;
            cv::Mat rgb888;
            cv_bridge::CvImage out_msg;
            out_msg.header.stamp = ros::Time::now();; // Same timestamp and tf frame as input image
			rgb888 = cv::Mat(120, 160, CV_8UC3);
			RGB565toRGB888(160, 120, &image[0], rgb888.data);            
			out_msg.encoding = sensor_msgs::image_encodings::RGB8;
            out_msg.image = rgb888;
            imagePublisher.publish(out_msg.toImageMsg());
        }
    }
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
        ss << "floor: [" << groundProx[0] << ", " << groundProx[1] << ", " << groundProx[2] << ", " << groundProx[3] << ", " << groundProx[4] << "]";
        floorMsg.text = ss.str();
        floorPublisher.publish(floorMsg);
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
	
	command[3] = speedLeft & 0xFF;		// left motor LSB
	command[4] = speedLeft >> 8;		// left motor MSB
	command[5] = speedRight & 0xFF;		// right motor LSB
	command[6] = speedRight >> 8;		// right motor MSB	

    if(DEBUG_SPEED_RECEIVED)std::cout << "[" << epuckname << "] " << "new speed: " << speedLeft << ", " << speedRight << std::endl;
    
}

void handlerLED(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    // Controls the state of each LED on the standard robot
    for (int i = 0; i < LED_NUMBER; i++) {
		if(msg->data[i] == 0) {
			command[7] &= ~(1<<i);
		} else {
			command[7] |= (1<<i);
		}
	}
	
    if(DEBUG_LED_RECEIVED) {
        std::cout << "[" << epuckname << "] " << "new LED status: " << std::endl;
        for (int i = 0; i < LED_NUMBER; i++)
            std::cout << msg->data[i] << ", ";
    }
}

void handlerRgbLeds(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
	command[8] = msg->data[0];		// LED2 red
	command[9] = msg->data[1];		// LED2 green
	command[10] = msg->data[2];		// LED2 blue
	command[11] = msg->data[3];		// LED4 red
	command[12] = msg->data[4];		// LED4 green
	command[13] = msg->data[5];		// LED4 blue
	command[14] = msg->data[6];		// LED6 red
	command[15] = msg->data[7];		// LED6 green
	command[16] = msg->data[8];		// LED6 blue
	command[17] = msg->data[9];		// LED8 red
	command[18] = msg->data[10];	// LED8 green
	command[19] = msg->data[11];	// LED8 blue	
	
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
   
	command[0] = 0x80;
	command[1] = 2;		// Sensors enabled.
	command[2] = 1;		// Calibrate proximity sensors.
	command[3] = 0;		// left motor LSB
	command[4] = 0;		// left motor MSB
	command[5] = 0;		// right motor LSB
	command[6] = 0;		// right motor MSB
	command[7] = 0;		// lEDs
	command[8] = 0;		// LED2 red
	command[9] = 0;		// LED2 green
	command[10] = 0;	// LED2 blue
	command[11] = 0;	// LED4 red
	command[12] = 0;	// LED4 green
	command[13] = 0;	// LED4 blue
	command[14] = 0;	// LED6 red
	command[15] = 0;	// LED6 green
	command[16] = 0;	// LED6 blue
	command[17] = 0;	// LED8 red
	command[18] = 0;	// LED8 green
	command[19] = 0;	// LED8 blue
	command[20] = 0;	// speaker   
	expected_recv_packets = 1;
   
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
	np.param("floor", ground_sensors_enabled, false);
    //np.param("ros_rate", rosRate, 7);    
    
    if(DEBUG_ROS_PARAMS) {
        std::cout << "[" << epuckname << "] " << "epuck id: " << robotId << std::endl;
        std::cout << "[" << epuckname << "] " << "epuck address: " << epuckAddress << std::endl;
        std::cout << "[" << epuckname << "] " << "epuck name: " << epuckname << std::endl;
        std::cout << "[" << epuckname << "] " << "init pose: " << init_xpos << ", " << init_ypos << ", " << theta << std::endl;
        std::cout << "[" << epuckname << "] " << "camera enabled: " << camera_enabled << std::endl;
		std::cout << "[" << epuckname << "] " << "ground sensors enabled: " << ground_sensors_enabled << std::endl;
        //std::cout << "[" << epuckname << "] " << "ros rate: " << rosRate << std::endl;
    }
	
	if(epuckAddress.compare("")==0) {
		std::cerr << "Robot address cannot be empty" << std::endl;
		return -1;
	}
	if(initConnectionWithRobot()<0) {
		std::cerr << "Can't connect to the robot" << std::endl;
		exit(1);
	}	
    
	imuPublisher = n.advertise<sensor_msgs::Imu>("imu", 10);

	for(i=0; i<8; i++) {
		/**
		* The advertise() function is how you tell ROS that you want to
		* publish on a given topic name. This invokes a call to the ROS
		* master node, which keeps a registry of who is publishing and who
		* is subscribing. After this advertise() call is made, the master
		* node will notify anyone who is trying to subscribe to this topic name,
		* and they will in turn negotiate a peer-to-peer connection with this
		* node.  advertise() returns a Publisher object which allows you to
		* publish messages on that topic through a call to publish().  Once
		* all copies of the returned Publisher object are destroyed, the topic
		* will be automatically unadvertised.
		*
		* The second parameter to advertise() is the size of the message queue
		* used for publishing messages.  If messages are published more quickly
		* than we can send them, the number here specifies how many messages to
		* buffer up before throwing some away.
		*/
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
	
	if(camera_enabled) {
		imagePublisher = n.advertise<sensor_msgs::Image>("camera", 1);
		command[1] = 3;		// Camera and sensors enabled.
		expected_recv_packets = 2;
	}
	
	if(ground_sensors_enabled) {
		floorPublisher = n.advertise<visualization_msgs::Marker>("floor", 10);
	}

       
    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called handlerVelocity.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
    cmdVelSubscriber = n.subscribe("mobile_base/cmd_vel", 10, handlerVelocity);
    cmdLedSubscriber = n.subscribe("mobile_base/cmd_led", 10, handlerLED);
	cmdRgbLedsSubscriber = n.subscribe("mobile_base/rgb_leds", 10, handlerRgbLeds);
    
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
    }

    closeConnection();

    
}




