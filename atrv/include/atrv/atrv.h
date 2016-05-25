#ifndef ATRV_H
#define ATRV_H

#include <atrv/atrv_config.h>
#include <atrv/atrv_info.h>
#include <pthread.h>
#include <time.h>
#include <ros/ros.h>
//#include <sys/time.h>

//#include <string>
//#include <unistd.h>
/**
 * \brief Driver to handle input and output to ATRV-2.
 *  Modified by John Folkesson from 
 *  RFLEX Driver - 2/2010
 *  Modified from Player code by David Lu!!
 *  Original Input Output code by Bill Smart
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
class ATRV{
 public:
  class comandData {
  public:
    float acceleration;
    float cmdTranslation, cmdRotation;
    bool isSonarOn; 
    pthread_mutex_t readMutex; ///< Mutex around reading 
    comandData(){
      pthread_mutex_init(&readMutex, NULL);
      cmdTranslation = cmdRotation = 0.0;
      acceleration = 0.7;
      isSonarOn  = false;
    }
    ~comandData(){
      pthread_mutex_destroy(&readMutex);
    }
    void setVelocity(double v, double r){
      pthread_mutex_lock(&readMutex);
      cmdTranslation = v;
      cmdRotation = r;
      pthread_mutex_unlock(&readMutex);
    }
    void setAcceleration(double a){
      pthread_mutex_lock(&readMutex);
      acceleration = a;
      pthread_mutex_unlock(&readMutex);
    }
    void setSonarPower(bool s){
      pthread_mutex_lock(&readMutex);
      isSonarOn = s;
      pthread_mutex_unlock(&readMutex);
    }
    void getAll(double &v, double &r, double &a, bool &s ){
      pthread_mutex_lock(&readMutex);
      v=cmdTranslation;
      r=cmdRotation;
      a=acceleration;
      s=isSonarOn;
      pthread_mutex_unlock(&readMutex);
    }
    
  };
  class ATRVState{
  public:
    ros::Time odoTime; 
    ros::Time sonarTime; 
    ros::Time packetTime; 
    ros::Time bumpTimer;
    ///< Raw translational odometry
    //long data[8+SONAR_MAX_COUNT];
    long sonar[SONAR_COUNT];
    // x y bearing velocity ang_vel 
    double x,y, theta, v, omega;		
    ///< Raw voltage reading
    double voltage;
    ///< Brake Status
    unsigned char brake;
    unsigned char bump;
    unsigned char readyToUpdate;
    ///< Raw Sonar readings (including unconnected ports)
    ATRVState(){
      memset(sonar,0, (SONAR_COUNT)*sizeof(long));
      x=y=theta=v=omega=0;
      voltage=0;
      brake=0;
      bump=0;
      readyToUpdate=0;
    }
    void init(){
      odoTime=ros::Time::now();
      packetTime=odoTime;
      sonarTime=odoTime;
    }
    void print(){
	printf("x=%f y=%f yawl=%f v=%f w=%f volts=%f bump=%c brake=%c \n", x, y, theta, v, omega, voltage, bump, brake);      
	//printf("time: %lfs x=%f y=%f yawl=%f v=%f w=%f volts=%f bump=%c brake=%c \n",odoTime.toSec(), x, y, theta, v, omega, voltage, bump, brake);
      //printf("sonars: %ld %ld %ld %ld %ld %ld %ld %ld  %ld %ld %ld %ld \n", sonar[0], sonar[1], sonar[2], sonar[3], sonar[4], sonar[5], sonar[6], sonar[7], sonar[8], sonar[9], sonar[10], sonar[11]);
    }
  };
  ATRV();
  virtual ~ATRV();
  void configure(std::string name, long val);
  virtual void publish(bool sonarpower){};
  /** Opens connection to serial port with specified device name
      \param devname Device name assigned to serial port
      \return -1 on error */

  int initialize(const char* devname);

  /** Configure the sonar parameters and send message to RFlex.
   * \param echoDelay Echo Delay
   * \param pingDelay Ping Delay
   * \param setDelay Set Delay
   * \param val Unknown
   * @todo Figure out unknown value's purpose.
   */
  void configureSonar(const unsigned long echoDelay, 
		      const unsigned long pingDelay,
		      const unsigned long setDelay, const unsigned long val);
  void setSonarUpdate();
  /** Turn Brake on or off
   * Note: Brake on means the controller cannot move the robot
   *    and external forces CAN move it.
   * \param power true for on, false for off */
  void setBrakePower(const bool power);

  /** Set the frequency that the Digital IO devices are checked.
   * \param period Period in milliseconds  
   *(actually I (jf) believe this is micro sec)  
   */
  void setDigitalIoPeriod(const long period);
  void setDIOUpdate();
  /** Set the frequency that the odometry is checked.
   * \param period Period in milliseconds
   *(actually I (jf) believe this is micro sec)  
   */
  void setOdometryPeriod(const long period);

  /** Sends a set motion defaults message to the device. */
  void motionSetDefaults();

  /** Sets the velocity
   * \param transVelocity Translational velocity in arbitrary units
   * \param rotVelocity Rotational velocity in arbitrary units
   * \param acceleration Acceleration (also in arbitrary units) */
  void setVelocity(const long transVelocity, const long rotVelocity,
		   const long acceleration);

  /** Sets the motion of the robot
   * \param tvel Translational velocity (in m/s)
   * \param rvel Rotational velocity (in radian/s)
   * \param acceleration Translational acceleration (in m/s/s) */
  void setMovement(float tvel, float rvel, float acceleration);
  void setSonarPower(bool);
  bool isOdomReady() const {
    return odomReady==3;
  }
  bool isPluggedIn();
  int getNumSonars();
    /** Gets brake power
   * \return True if brake is engaged */
  bool getBrakePower() const {
    return (bool) parserState.brake;
  }
  /** Get readings from the sonar on the base of the ATRV in meters
   * param readings Data structure into which the sonar readings are
   * saved 
   */
  void getSonarReadings(float* readings);  

  /** Sends a system status command to the device.
   * Updates the brake and battery status. */
  void sendSystemStatusCommand();
  comandData com;  
  
 void main_update(); //Written by Erik Sillén
 
 
 protected:
	
  
  
  
  ATRVState parserState;
  bool newSonar;
  int herp;
  long last_distance;
  long last_bearing;
  bool found_distance;
  unsigned short dioData[24];	///< Storage for digital IO values
  unsigned char odomReady;
  bool useSonar;
  bool useDIO;
  int updateTimer;
  bool serialOn;
  //These should be set in a config file:
  long odoDistanceConversion;
  long odoAngleConversion;
  long sonarMaxRange;
  long sonarRangeConversion;
  long sonarEchoDelay;
  long sonarPingDelay;
  long sonarSetDelay;
  long period;
 private:
  void parsePacket(const unsigned char* buffer);
  void parseMotReport(const unsigned char* buffer);
  void parseDioReport(const unsigned char* buffer);
  void parseSysReport(const unsigned char* buffer);
  void parseSonarReport(const unsigned char* buffer);
  
  // IO Stuff
  ///< File descriptor for serial port
  int fd;				
  ///< Thread which reads input upon arrival
  pthread_t thread;	
  ///< Mutex around writing to port
  pthread_mutex_t writeMutex; 

  unsigned char readBuffer[BUFFER_SIZE];
  unsigned char writeBuffer[BUFFER_SIZE];

  bool found;
  int offset;

  static void *readThread(void *ptr); 

  /**
   * Send a command to the serial port
   * \param port Should be one of these: SYS_PORT, MOT_PORT, JSTK_PORT, SONAR_PORT, DIO_PORT, IR_PORT
   * \param id
   * \param opcode See opcodes in atrv_info.h
   * \param length length of the data
   * \param data actual data */
  bool sendCommand(const unsigned char port, const unsigned char id, 
		   const unsigned char opcode, const int length, 
		   unsigned char* data);
  ///< After reading the data, it checks for errors and then parses
  void readPacket();
 ///< Reads in a packet until it finds and end of packet signal 
  int readData();   
///< Writes packet currently in write buffer to device
  bool writePacket(const int length) const; 
 ///< Calculates error checking code for specified buffer
  unsigned char computeCRC(const unsigned char *buffer, const int n);
};
#endif
