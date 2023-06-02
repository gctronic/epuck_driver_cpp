
#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
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
#include <sensor_msgs/LaserScan.h>
extern "C" {
	#include <stdio.h>
	#include <stdint.h> 
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <linux/i2c-dev.h> /* for I2C_SLAVE */
	//#include <linux/i2c.h>
	#include <sys/ioctl.h>
	#include <stdlib.h>
	#include <unistd.h>
}

#define DEBUG_CONNECTION_INIT 1
#define DEBUG_ROS_PARAMS 1
#define DEBUG_UPDATE_SENSORS_TIMING 0
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_COMMUNICATION_ERROR 1
#define DEBUG_ODOMETRY 0
#define DEBUG_IMU 0
#define DEBUG_SPEED_RECEIVED 0
#define DEBUG_LED_RECEIVED 0

#define I2C_CHANNEL "/dev/i2c-12"
#define LEGACY_I2C_CHANNEL "/dev/i2c-4"

#define READ_TIMEOUT_SEC 10    // 10 seconds, keep it high to avoid desynchronize when there are communication delays due to Bluetooth.
#define READ_TIMEOUT_USEC 0
#define MAX_CONSECUTIVE_TIMEOUT 3

#define SENSORS_NUM 10
#define IMU 0
#define MOTOR_SPEED 1
#define FLOOR 2
#define PROXIMITY 3
#define MOTOR_POSITION 4
#define MICROPHONE 5
#define CAMERA 6
#define TV_REMOTE 7
#define SELECTOR 8
#define TIME_OF_FLIGHT 9

#define ACTUATORS_NUM 4
#define MOTORS 0
#define LEDS 1
#define MOTORS_POS 2
#define SPEAKER 3

#define WHEEL_DIAMETER 4        // cm.
#define WHEEL_SEPARATION 5.3    // Separation between wheels (cm).
#define WHEEL_DISTANCE 0.053    // Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER*M_PI)/100.0)    // Wheel circumference (meters).
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE/1000.0)      // Distance for each motor step (meters); a complete turn is 1000 steps (0.000125 meters per step (m/steps)).
#define ROBOT_RADIUS 0.035 // meters.

#define LEDS_NUM 4 		// Number of LEDs on the robot.
#define RGB_LEDS_NUM 4	// Number of RGB LEDs on the robot.

#define ACTUATORS_SIZE 19
#define SENSORS_SIZE 30

#define NUM_SAMPLES_CALIBRATION 20
#define AK8963_ADDRESS 0x0C		// Address of magnetometer
#define AK8963_XOUT_L 0x03		// data

#define MPU9250_ADDRESS_AD1_0 0x68  // Device address when AD1 = 0
#define MPU9250_ADDRESS_AD1_1 0x69

#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define TEMP_OUT_H         0x41
#define GYRO_XOUT_H        0x43

#define GRAVITY_MPU9250 16384 // 1 g for 16 bits in 2g scale mode
#define DEG2RAD(deg) (deg / 180 * M_PI)
#define STANDARD_GRAVITY 9.80665f
#define ACC_RAW2G (2.0/32768.0f)   //2G scale for int16 raw value
#define GYRO_RAW2DPS (250.0/32768.0f)   //250DPS (degrees per second) scale for int16 raw value

int fh;
char zero_to_epuck_buff[ACTUATORS_SIZE]; 
char epuck_to_zero_buff[SENSORS_SIZE];
bool enabledSensors[SENSORS_NUM];
bool changedActuators[ACTUATORS_NUM];
int speedLeft = 0, speedRight = 0;
unsigned char ledState[LEDS_NUM] = {0, 0, 0, 0};
unsigned char rgbLedState[RGB_LEDS_NUM][3];
int stepsLeft = 0, stepsRight = 0;
std::string epuckname;
struct timeval currentTime2, lastTime2;
struct timeval currentTime3, lastTime3;
int consecutiveReadTimeout = 0;

int proxData[8];
int motorPositionData[2];
int micData[4];
uint8_t selectorData;
uint8_t tvRemoteData;

ros::Publisher proxPublisher[8];
sensor_msgs::Range proxMsg[8];
ros::Publisher laserPublisher;
sensor_msgs::LaserScan laserMsg;
ros::Publisher odomPublisher;
nav_msgs::Odometry odomMsg;
ros::Publisher microphonePublisher;
visualization_msgs::Marker microphoneMsg;
ros::Publisher imuPublisher;
sensor_msgs::Imu imuMsg;
ros::Publisher motorSpeedPublisher;
ros::Publisher floorPublisher;

ros::Subscriber cmdVelSubscriber, cmdLedSubscriber;

double leftStepsDiff = 0, rightStepsDiff = 0;
double leftStepsPrev = 0, rightStepsPrev = 0;
signed long int leftStepsRawPrev = 0, rightStepsRawPrev = 0;
signed long int motorPositionDataCorrect[2];
double xPos, yPos, theta;
double deltaSteps, deltaTheta;
ros::Time currentTime, lastTime, currentTimeMap, lastTimeMap;
int overflowCountLeft = 0, overflowCountRight = 0;

uint8_t imu_addr = MPU9250_ADDRESS_AD1_0;
uint8_t accData[6];
uint8_t gyroData[6];
uint8_t temperatureData;
int16_t accValue[3];
int32_t accSum[3] = {0, 0, 0};
int16_t accOffset[3] = {0, 0, 0};
int16_t gyroValue[3];
int32_t gyroSum[3] = {0, 0, 0};
int16_t gyroOffset[3] = {0, 0, 0};

bool debug_enabled = false;
uint8_t debug_count = 0;

int initConnectionWithRobot(void) {
	fh = open(I2C_CHANNEL, O_RDWR);
	if(fh < 0) { // Try with bus number used in older kernel
		fh = open(LEGACY_I2C_CHANNEL, O_RDWR);	
		if(fh < 0) {
			perror("Cannot open I2C device");
			return -1;
		}
	}
	return 0;
}

void closeConnection() {
	close(fh);
}

void updateActuators() {
    
    char buff[36]; // 6 (initial size) + 3*LED_NUM (=10)
	
    if(changedActuators[MOTORS]) {
        changedActuators[MOTORS] = false;
		zero_to_epuck_buff[0] = speedLeft&0xFF;
		zero_to_epuck_buff[1] = speedLeft>>8;
		zero_to_epuck_buff[2] = speedRight&0xFF;
		zero_to_epuck_buff[3] = speedRight>>8;	
    }
    
    if(changedActuators[LEDS]) {
		changedActuators[LEDS] = false;
        unsigned char i, pos = 0;
		if(ledState[0]) {
			zero_to_epuck_buff[5] |= 1;
		} else {
			zero_to_epuck_buff[5] &= ~(1);
		}
		if(ledState[1]) {
			zero_to_epuck_buff[5] |= 2;
		} else {
			zero_to_epuck_buff[5] &= ~(2);
		}
		if(ledState[2]) {
			zero_to_epuck_buff[5] |= 4;
		} else {
			zero_to_epuck_buff[5] &= ~(4);
		}
		if(ledState[3]) {
			zero_to_epuck_buff[5] |= 8;
		} else {
			zero_to_epuck_buff[5] &= ~(8);
		}		
		//zero_to_epuck_buff[5] = 0x0;	// LED1, LED3, LED5, LED7 on/off flag
		zero_to_epuck_buff[6] = 0;		// LED2 red
		zero_to_epuck_buff[7] = 0;		// LED2 green
		zero_to_epuck_buff[8] = 0;		// LED2 blue
		zero_to_epuck_buff[9] = 0;		// LED4 red
		zero_to_epuck_buff[10] = 0;		// LED4 green
		zero_to_epuck_buff[11] = 0;		// LED4 blue
		zero_to_epuck_buff[12] = 0;		// LED6 red
		zero_to_epuck_buff[13] = 0;		// LED6 green
		zero_to_epuck_buff[14] = 0;		// LED6 blue
		zero_to_epuck_buff[15] = 0;		// LED8 red
		zero_to_epuck_buff[16] = 0;		// LED8 green
		zero_to_epuck_buff[17] = 0;		// LED8 blue
    }

}

// The chip has two alternative addresses based on the AD1 pin.
void mpu9250_change_addr(void) {
	if(imu_addr == MPU9250_ADDRESS_AD1_0) {
		imu_addr = MPU9250_ADDRESS_AD1_1;
	} else {
		imu_addr = MPU9250_ADDRESS_AD1_0;
	}
	ioctl(fh, I2C_SLAVE, imu_addr);
}

int read_reg(int file, uint8_t reg, int count, uint8_t *data) {

	if(write(file, &reg, 1) != 1) {
		mpu9250_change_addr();
		if(write(file, &reg, 1) != 1) {
			perror("imu write error");
			return -1;
		}
	}

	if(read(file, data, count) != count) {
		mpu9250_change_addr();
		if(read(file, data, count) != count) {
			printf("count=%d\n", count);
			perror("imu read error");
			return -1;
		}
	}

	return 0;
}

void calibrateAcc() {
	int samplesCount=0, i=0;
	// reset and send configuration first?
	for(i=0; i<NUM_SAMPLES_CALIBRATION; i++) {
		if(read_reg(fh, ACCEL_XOUT_H, 6, accData) == 0) {	// for MPU9250 set just the address also for a multiple read with autoincrement
			accSum[0] += (int16_t)(accData[1] + (accData[0]<<8)); // MPU9250 big-endian
			accSum[1] += (int16_t)(accData[3] + (accData[2]<<8));
			accSum[2] += (int16_t)(accData[5] + (accData[4]<<8));
			samplesCount++;
			//printf("acc sums: x=%d, y=%d, z=%d (samples=%d)\n", accSum[0], accSum[1], accSum[2], samplesCount);
		}
	}
	accOffset[0] = (int16_t)((float)accSum[0]/(float)samplesCount);
	accOffset[1] = (int16_t)((float)accSum[1]/(float)samplesCount);
	accOffset[2] = (int16_t)((float)accSum[2]/(float)samplesCount);
	printf("acc offsets: x=%d, y=%d, z=%d (samples=%d)\n", accOffset[0], accOffset[1], accOffset[2], samplesCount);

}

void calibrateGyro() {
	int samplesCount=0, i=0;
	// reset and send configuration first?
	for(i=0; i<NUM_SAMPLES_CALIBRATION; i++) {
		if(read_reg(fh, GYRO_XOUT_H, 6, gyroData) == 0) {	// // for MPU9250 set just the address also for a multiple read with autoincrement
			gyroSum[0] += (int16_t)(gyroData[1] + (gyroData[0]<<8)); // MPU9250 big-endian
			gyroSum[1] += (int16_t)(gyroData[3] + (gyroData[2]<<8));
			gyroSum[2] += (int16_t)(gyroData[5] + (gyroData[4]<<8));
			samplesCount++;
			//printf("gyro sums: x=%d, y=%d, z=%d (samples=%d)\n", gyroSum[0], gyroSum[1], gyroSum[2], samplesCount);
		}
	}
	gyroOffset[0] = (int16_t)((float)gyroSum[0]/(float)samplesCount);
	gyroOffset[1] = (int16_t)((float)gyroSum[1]/(float)samplesCount);
	gyroOffset[2] = (int16_t)((float)gyroSum[2]/(float)samplesCount);
	printf("gyro offsets: x=%d, y=%d, z=%d (samples=%d)\n", gyroOffset[0], gyroOffset[1], gyroOffset[2], samplesCount);
}

void updateSensorsData() {

    struct timeval timeout;
    fd_set readfds;
    int retval;
    unsigned int bytesRead = 0;
    
    memset(epuck_to_zero_buff, 0x0, SENSORS_SIZE);
    FD_ZERO(&readfds);
    FD_SET(fh, &readfds);
	ioctl(fh, I2C_SLAVE, 0x1F);			// tell the driver we want the device with address 0x1F (7-bits) on the I2C bus	=> main microcontroller.
    retval = write(fh, zero_to_epuck_buff, ACTUATORS_SIZE);
	if(retval != ACTUATORS_SIZE) {
		perror("i2c write error");
	}	
    bytesRead = 0;
    if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&lastTime3, NULL);
    while(bytesRead < SENSORS_SIZE) {
        timeout.tv_sec=READ_TIMEOUT_SEC; // The timeout need to be set every time because the "select" may modify it.
        timeout.tv_usec=READ_TIMEOUT_USEC;                
	if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&lastTime2, NULL);
        retval = select(fh+1, &readfds, NULL, NULL, &timeout);
        if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&currentTime2, NULL);
        if(DEBUG_UPDATE_SENSORS_TIMING)std::cout << "[" << epuckname << "] " << "sensors data read in " << double((currentTime2.tv_sec*1000000 + currentTime2.tv_usec)-(lastTime2.tv_sec*1000000 + lastTime2.tv_usec))/1000000.0 << " sec" << std::endl;
        if (retval>0) {
            int n = read(fh, &epuck_to_zero_buff[bytesRead], SENSORS_SIZE-bytesRead);
            //if(DEBUG_OTHERS)std::cout << "read " << n << " / " << SENSORS_SIZE << " bytes" << std::endl;
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
    if(bytesRead == SENSORS_SIZE) {       
        if(DEBUG_UPDATE_SENSORS_TIMING)gettimeofday(&currentTime3, NULL);
        if(DEBUG_UPDATE_SENSORS_TIMING)std::cout << "[" << epuckname << "] " << "sensors tot read time " << double((currentTime3.tv_sec*1000000 + currentTime3.tv_usec)-(lastTime3.tv_sec*1000000 + lastTime3.tv_usec))/1000000.0 << " sec" << std::endl; 
        if(enabledSensors[PROXIMITY]) {
            proxData[0] = (unsigned char)epuck_to_zero_buff[0] | epuck_to_zero_buff[1]<<8;
            proxData[1] = (unsigned char)epuck_to_zero_buff[2] | epuck_to_zero_buff[3]<<8;
            proxData[2] = (unsigned char)epuck_to_zero_buff[4] | epuck_to_zero_buff[5]<<8;
            proxData[3] = (unsigned char)epuck_to_zero_buff[6] | epuck_to_zero_buff[7]<<8;
            proxData[4] = (unsigned char)epuck_to_zero_buff[8] | epuck_to_zero_buff[9]<<8;
            proxData[5] = (unsigned char)epuck_to_zero_buff[10] | epuck_to_zero_buff[11]<<8;
            proxData[6] = (unsigned char)epuck_to_zero_buff[12] | epuck_to_zero_buff[13]<<8;
            proxData[7] = (unsigned char)epuck_to_zero_buff[14] | epuck_to_zero_buff[15]<<8;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "prox: " << proxData[0] << "," << proxData[1] << "," << proxData[2] << "," << proxData[3] << "," << proxData[4] << "," << proxData[5] << "," << proxData[6] << "," << proxData[7] << std::endl;
        }
        if(enabledSensors[MICROPHONE]) {
            micData[0] = (unsigned char)epuck_to_zero_buff[16] | epuck_to_zero_buff[17]<<8;
            micData[1] = (unsigned char)epuck_to_zero_buff[18] | epuck_to_zero_buff[19]<<8;
            micData[2] = (unsigned char)epuck_to_zero_buff[20] | epuck_to_zero_buff[21]<<8;
			micData[3] = (unsigned char)epuck_to_zero_buff[22] | epuck_to_zero_buff[23]<<8;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "mic: " << micData[0] << "," << micData[1] << "," << micData[2] << std::endl;
        }
		selectorData = epuck_to_zero_buff[24];
        if(enabledSensors[MOTOR_POSITION]) {
            motorPositionData[0] = (unsigned char)epuck_to_zero_buff[25] | epuck_to_zero_buff[26]<<8;
            motorPositionData[1] = (unsigned char)epuck_to_zero_buff[27] | epuck_to_zero_buff[28]<<8;
            if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "position: " << motorPositionData[0] << "," << motorPositionData[1] << std::endl;
        }  		
		tvRemoteData = epuck_to_zero_buff[29];
		
        // if(enabledSensors[MOTOR_SPEED]) {
            // motorSpeedData[0] = (unsigned char)epuck_to_zero_buff[bufIndex] | epuck_to_zero_buff[bufIndex+1]<<8;
            // motorSpeedData[1] = (unsigned char)epuck_to_zero_buff[bufIndex+2] | epuck_to_zero_buff[bufIndex+3]<<8;
            // bufIndex += 4;
            // if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "speed: " << motorSpeedData[0] << "," << motorSpeedData[1] << std::endl;
        // }
        // if(enabledSensors[FLOOR]) {
            // floorData[0] = (unsigned char)epuck_to_zero_buff[bufIndex] | epuck_to_zero_buff[bufIndex+1]<<8;
            // floorData[1] = (unsigned char)epuck_to_zero_buff[bufIndex+2] | epuck_to_zero_buff[bufIndex+3]<<8;
            // floorData[2] = (unsigned char)epuck_to_zero_buff[bufIndex+4] | epuck_to_zero_buff[bufIndex+5]<<8;
            // floorData[3] = (unsigned char)epuck_to_zero_buff[bufIndex+6] | epuck_to_zero_buff[bufIndex+7]<<8;
            // floorData[4] = (unsigned char)epuck_to_zero_buff[bufIndex+8] | epuck_to_zero_buff[bufIndex+9]<<8;
            // bufIndex += 10;
            // if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "floor: " << floorData[0] << "," << floorData[1] << "," << floorData[2] << "," << floorData[3] << "," << floorData[4] << std::endl;
        // }
      
    } else {
        if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "discard the sensors data" << std::endl;
    }
	
	if(enabledSensors[IMU]) {
		ioctl(fh, I2C_SLAVE, imu_addr);
	
		read_reg(fh, ACCEL_XOUT_H, 6, accData);	
		read_reg(fh, GYRO_XOUT_H, 6, gyroData);		

		accValue[0] = (accData[1] + (accData[0]<<8));// MPU9250 big-endian
		accValue[1] = (accData[3] + (accData[2]<<8));
		accValue[2] = (accData[5] + (accData[4]<<8));

		gyroValue[0] = (gyroData[1] + (gyroData[0]<<8));
		gyroValue[1] = (gyroData[3] + (gyroData[2]<<8));
		gyroValue[2] = (gyroData[5] + (gyroData[4]<<8));
		
		if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "acc: " << accValue[0] << "," << accValue[1] << "," << accValue[2] << std::endl;
		if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "gyro: " << gyroValue[0] << "," << gyroValue[1] << "," << gyroValue[2] << std::endl;		
	}	
	
	if(debug_enabled) {
		debug_count++;
		if(debug_count == 50) {
			debug_count = 0;
			std::cout << "[" << epuckname << "] " << "prox: " << proxData[0] << "," << proxData[1] << "," << proxData[2] << "," << proxData[3] << "," << proxData[4] << "," << proxData[5] << "," << proxData[6] << "," << proxData[7] << std::endl;
			std::cout << "[" << epuckname << "] " << "mic: " << micData[0] << "," << micData[1] << "," << micData[2] << std::endl;
			std::cout << "[" << epuckname << "] " << "position: " << motorPositionData[0] << "," << motorPositionData[1] << std::endl;
		}
	}
	
}

void updateRosInfo() {
    static tf::TransformBroadcaster br;
    
    int i = 0;
    if(enabledSensors[PROXIMITY]) {
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
        std::stringstream parent;
        std::stringstream child;
        tf::Transform transform;
        tf::Quaternion q;
        
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
        
    }
    
    if(enabledSensors[MOTOR_POSITION]) {
        
        // The encoders values coming from the e-puck are 2 bytes signed int thus we need to handle the overflows otherwise the odometry will be wrong after a while (about 4 meters).
        if((leftStepsRawPrev>0) && (motorPositionData[0]<0) && (abs(motorPositionData[0]-leftStepsRawPrev)>30000)) {     // Overflow detected (positive).
            overflowCountLeft++;
        }
        if((leftStepsRawPrev<0) && (motorPositionData[0]>0) && (abs(motorPositionData[0]-leftStepsRawPrev)>30000)) {     // Overflow detected (negative).
            overflowCountLeft--;
        }
        motorPositionDataCorrect[0] = (overflowCountLeft*65536) + motorPositionData[0];
        
        if((rightStepsRawPrev>0) && (motorPositionData[1]<0) && (abs(motorPositionData[1]-rightStepsRawPrev)>30000)) {     // Overflow detected (positive).
            overflowCountRight++;
        }
        if((rightStepsRawPrev<0) && (motorPositionData[1]>0) && (abs(motorPositionData[1]-rightStepsRawPrev)>30000)) {     // Overflow detected (negative).
            overflowCountRight--;
        }
        motorPositionDataCorrect[1] = (overflowCountRight*65536) + motorPositionData[1];        
        
        leftStepsRawPrev = motorPositionData[0];
        rightStepsRawPrev = motorPositionData[1];
        
        if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "left, right raw: " << motorPositionData[0] << ", " << motorPositionData[1] << std::endl;
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
        std::stringstream ss;
        ss << epuckname << "/base_link";
        odomMsg.child_frame_id = ss.str();
        odomMsg.pose.pose.position.x = xPos;       
        odomMsg.pose.pose.position.y = yPos;
        odomMsg.pose.pose.position.z = 0;
        // Since all odometry is 6DOF we'll need a quaternion created from yaw.
        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);
        odomMsg.pose.pose.orientation = odomQuat;
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
        odomTrans.transform.rotation = odomQuat;
        br.sendTransform(odomTrans);
    }
    
    if(enabledSensors[IMU]) {	
        std::stringstream ss;
        ss << epuckname << "/base_link";
        imuMsg.header.frame_id = ss.str();
        imuMsg.header.stamp = ros::Time::now();            
        imuMsg.linear_acceleration.x = (accValue[0]-accOffset[0]) * STANDARD_GRAVITY * ACC_RAW2G; // m/s^2
        imuMsg.linear_acceleration.y = (accValue[1]-accOffset[1]) * STANDARD_GRAVITY * ACC_RAW2G;
        imuMsg.linear_acceleration.z = (accValue[2]-accOffset[2]+GRAVITY_MPU9250) * STANDARD_GRAVITY * ACC_RAW2G;
        imuMsg.linear_acceleration_covariance[0] = 0.01;
        imuMsg.linear_acceleration_covariance[1] = 0.0;
        imuMsg.linear_acceleration_covariance[2] = 0.0;
        imuMsg.linear_acceleration_covariance[3] = 0.0;
        imuMsg.linear_acceleration_covariance[4] = 0.01;
        imuMsg.linear_acceleration_covariance[5] = 0.0;
        imuMsg.linear_acceleration_covariance[6] = 0.0;
        imuMsg.linear_acceleration_covariance[7] = 0.0;
        imuMsg.linear_acceleration_covariance[8] = 0.01;
        if(DEBUG_IMU)std::cout << "[" << epuckname << "] " << "accel raw: " << accValue[0] << ", " << accValue[1] << ", " << accValue[2] << std::endl;
        if(DEBUG_IMU)std::cout << "[" << epuckname << "] " << "accel (m/s2): " << imuMsg.linear_acceleration.x << ", " << imuMsg.linear_acceleration.y << ", " << imuMsg.linear_acceleration.z << std::endl;
        imuMsg.angular_velocity.x = (gyroValue[0] - gyroOffset[0]) * DEG2RAD(GYRO_RAW2DPS); // rad/s
        imuMsg.angular_velocity.y = (gyroValue[1] - gyroOffset[1]) * DEG2RAD(GYRO_RAW2DPS);
        imuMsg.angular_velocity.z = (gyroValue[2] - gyroOffset[2]) * DEG2RAD(GYRO_RAW2DPS);
        imuMsg.angular_velocity_covariance[0] = 0.01;
        imuMsg.angular_velocity_covariance[1] = 0.0;
        imuMsg.angular_velocity_covariance[2] = 0.0;
        imuMsg.angular_velocity_covariance[3] = 0.0;
        imuMsg.angular_velocity_covariance[4] = 0.01;
        imuMsg.angular_velocity_covariance[5] = 0.0;
        imuMsg.angular_velocity_covariance[6] = 0.0;
        imuMsg.angular_velocity_covariance[7] = 0.0;
        imuMsg.angular_velocity_covariance[8] = 0.01;
        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
        imuMsg.orientation = odomQuat;
        imuMsg.orientation_covariance[0] = 0.01;
        imuMsg.orientation_covariance[1] = 0.0;
        imuMsg.orientation_covariance[2] = 0.0;
        imuMsg.orientation_covariance[3] = 0.0;
        imuMsg.orientation_covariance[4] = 0.01;
        imuMsg.orientation_covariance[5] = 0.0;
        imuMsg.orientation_covariance[6] = 0.0;
        imuMsg.orientation_covariance[7] = 0.0;
        imuMsg.orientation_covariance[8] = 0.01;
        imuPublisher.publish(imuMsg);
    }
	
    // if(enabledSensors[MOTOR_SPEED]) {
        // std::stringstream ss;
        // ss << epuckname << "/base_link";
        // motorSpeedMsg.header.frame_id = ss.str();
        // motorSpeedMsg.header.stamp = ros::Time::now();
        // motorSpeedMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        // motorSpeedMsg.pose.position.x = 0.15;
        // motorSpeedMsg.pose.position.y = 0;
        // motorSpeedMsg.pose.position.z = 0.15;
        // geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
        // motorSpeedMsg.pose.orientation = odomQuat;
        // motorSpeedMsg.scale.z = 0.01;
        // motorSpeedMsg.color.a = 1.0;
        // motorSpeedMsg.color.r = 1.0;
        // motorSpeedMsg.color.g = 1.0;
        // motorSpeedMsg.color.b = 1.0;
        // ss.str("");
        // ss << "speed: [" << motorSpeedData[0] << ", " << motorSpeedData[1] << "]";
        // motorSpeedMsg.text = ss.str();
        // motorSpeedPublisher.publish(motorSpeedMsg);
    // }
    
    // if(enabledSensors[FLOOR]) {
        // std::stringstream ss;
        // ss << epuckname << "/base_link";
        // floorMsg.header.frame_id = ss.str();
        // floorMsg.header.stamp = ros::Time::now();
        // floorMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        // floorMsg.pose.position.x = 0.15;
        // floorMsg.pose.position.y = 0;
        // floorMsg.pose.position.z = 0.13;
        // geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
        // floorMsg.pose.orientation = odomQuat;
        // floorMsg.scale.z = 0.01;
        // floorMsg.color.a = 1.0;
        // floorMsg.color.r = 1.0;
        // floorMsg.color.g = 1.0;
        // floorMsg.color.b = 1.0;
        // ss.str("");
        // ss << "floor: [" << floorData[0] << ", " << floorData[1] << ", " << floorData[2] << ", " << floorData[3] << ", " << floorData[4] << "]";
        // floorMsg.text = ss.str();
        // floorPublisher.publish(floorMsg);
    // }
    
    if(enabledSensors[MICROPHONE]) {
        std::stringstream ss;
        ss << epuckname << "/base_link";
        microphoneMsg.header.frame_id = ss.str();
        microphoneMsg.header.stamp = ros::Time::now();
        microphoneMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        microphoneMsg.pose.position.x = 0.15;
        microphoneMsg.pose.position.y = 0;
        microphoneMsg.pose.position.z = 0.11;
        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(0);
        microphoneMsg.pose.orientation = odomQuat;
        microphoneMsg.scale.z = 0.01;
        microphoneMsg.color.a = 1.0;
        microphoneMsg.color.r = 1.0;
        microphoneMsg.color.g = 1.0;
        microphoneMsg.color.b = 1.0;
        ss.str("");
        ss << "mic: [" << micData[0] << ", " << micData[1] << ", " << micData[2] <<  ", " << micData[3] << "]";
        microphoneMsg.text = ss.str();
        microphonePublisher.publish(microphoneMsg);
    }

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
    changedActuators[MOTORS] = true;

    if(DEBUG_SPEED_RECEIVED)std::cout << "[" << epuckname << "] " << "new speed: " << speedLeft << ", " << speedRight << std::endl;
    
}

void handlerLED(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    // Controls the state of each LED on the standard robot
    for (int i = 0; i < LEDS_NUM; i++)
        ledState[i] = msg->data[i];
    changedActuators[LEDS] = true;

    if(DEBUG_LED_RECEIVED) {
        std::cout << "[" << epuckname << "] " << "new LED status: " << std::endl;
        for (int i = 0; i < LEDS_NUM; i++)
            std::cout << ledState[i] << ", ";
    }
}

int main(int argc,char *argv[]) {
   
   double init_xpos, init_ypos, init_theta;   
   int rosRate = 0;
   int i = 0;
   
   	zero_to_epuck_buff[4] = 3; // Speaker => 3 = no sound.
   
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
    
    np.param<std::string>("epuck_name", epuckname, "epuck");
    np.param("xpos", init_xpos, 0.0);
    np.param("ypos", init_ypos, 0.0);
    np.param("theta", init_theta, 0.0);
    np.param("imu", enabledSensors[IMU], false);
    np.param("motor_speed", enabledSensors[MOTOR_SPEED], false);
    np.param("floor", enabledSensors[FLOOR], false);
    np.param("proximity", enabledSensors[PROXIMITY], false);
    np.param("motor_position", enabledSensors[MOTOR_POSITION], false);
    np.param("microphone", enabledSensors[MICROPHONE], false);
    np.param("ros_rate", rosRate, 20);    
	np.param("debug", debug_enabled, false);
	
    if(DEBUG_ROS_PARAMS) {
        std::cout << "[" << epuckname << "] " << "epuck name: " << epuckname << std::endl;
        std::cout << "[" << epuckname << "] " << "init pose: " << init_xpos << ", " << init_ypos << ", " << theta << std::endl;
        std::cout << "[" << epuckname << "] " << "imu enabled: " << enabledSensors[IMU] << std::endl;
        std::cout << "[" << epuckname << "] " << "motor speed enabled: " << enabledSensors[MOTOR_SPEED] << std::endl;
        std::cout << "[" << epuckname << "] " << "floor enabled: " << enabledSensors[FLOOR] << std::endl;
        std::cout << "[" << epuckname << "] " << "proximity enabled: " << enabledSensors[PROXIMITY] << std::endl;
        std::cout << "[" << epuckname << "] " << "motor position enabled: " << enabledSensors[MOTOR_POSITION] << std::endl;
        std::cout << "[" << epuckname << "] " << "microphone enabled: " << enabledSensors[MICROPHONE] << std::endl;
        std::cout << "[" << epuckname << "] " << "ros rate: " << rosRate << std::endl;
		std::cout << "[" << epuckname << "] " << "debug enabled: " << debug_enabled << std::endl;
    }
    

    if(initConnectionWithRobot()<0) {
		return -1;
    }
    
    if(enabledSensors[IMU]) {
		ioctl(fh, I2C_SLAVE, imu_addr);	
		calibrateAcc();
		calibrateGyro();
        imuPublisher = n.advertise<sensor_msgs::Imu>("imu", 10);
    }
    if(enabledSensors[MOTOR_SPEED]) {
        motorSpeedPublisher = n.advertise<visualization_msgs::Marker>("motor_speed", 10);
    }
    if(enabledSensors[FLOOR]) {
        floorPublisher = n.advertise<visualization_msgs::Marker>("floor", 10);
    }
    if(enabledSensors[PROXIMITY]) {
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
    }
    if(enabledSensors[MOTOR_POSITION]) {
        odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 10);
        currentTime = ros::Time::now();
        lastTime = ros::Time::now();        
    }
    if(enabledSensors[MICROPHONE]) {
        microphonePublisher = n.advertise<visualization_msgs::Marker>("microphone", 10);
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
   
    theta = init_theta;
    xPos = init_xpos;
    yPos = init_ypos;

    ros::Rate loop_rate(rosRate);
   
    while (ros::ok()) {
		blabla;
        updateSensorsData();
        updateRosInfo();
        updateActuators();
        ros::spinOnce();
        loop_rate.sleep();    // Do not call "sleep" otherwise the bluetooth communication will hang.
                                // We communicate as fast as possible, this shouldn't be a problem...
        if(consecutiveReadTimeout >= MAX_CONSECUTIVE_TIMEOUT) { // We have connection problems, stop here.
            break;
        }
    }

    closeConnection();
    
}



