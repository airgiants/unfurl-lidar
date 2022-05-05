/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

//#include "lib.h"


#include "rollingAverage.h"

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

bool traceClusters = false;


// must match values from canid.h
#define CAN_MESSAGE_LIDAR_START             0x090000  // lidar data scan start/end

#define CAN_MESSAGE_LIDAR_PERSON            0x0c0000  // angle, range of a person


#define NUM_BACKGROUND 1024 // less than the number of points we get
RollingAverage background[NUM_BACKGROUND];

int canSocket; /* can raw socket */
bool gotCan = false;

int connectCan()
{
 
  gotCan = false;

  int enable_canfd = 1;
	struct sockaddr_can addr;
	
	struct ifreq ifr;

/* open socket */
	if ((canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("CAN failure - socket");
		return 1;
	}


strncpy(ifr.ifr_name, "vcan0", IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
	if (!ifr.ifr_ifindex) {
		perror("CAN failure - if_nametoindex");
		return 1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

  /* disable default receive filter on this RAW socket */
	/* This is obsolete as we do not read from the socket at all, but for */
	/* this reason we can remove the receive list in the Kernel to save a */
	/* little (really a very little!) CPU usage.                          */
	setsockopt(canSocket, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

  gotCan = true;
}



bool sendCanStart( bool isStart )
{

  if( ! gotCan )
    return 1;

  struct canfd_frame frame;

  frame.can_id = CAN_EFF_FLAG | CAN_MESSAGE_LIDAR_START;
  frame.data[0] = isStart;
  frame.len = 1;
  
  int required_mtu = sizeof( canfd_frame );

	/* send frame */
	if (write(canSocket, &frame, required_mtu) != required_mtu) {
		perror("CAN failure - write");
		return 1;
	}

  return 0;
}

bool sendCanPerson( float angle, float width, float range)
{
  if( ! gotCan )
    return 1;

  struct canfd_frame frame;

  frame.can_id = CAN_EFF_FLAG | CAN_MESSAGE_LIDAR_PERSON;
  frame.data[0] = (uint8_t)(angle*256.0/(2.0*M_PI));
  
  
  if( width > 10.0 )
    width = 10.0;

  frame.data[1] = (uint8_t)(width*256.0/(10.0));

  if( range > 30.0 )
    range = 30.0;

  frame.data[1] = (uint8_t)(range*256.0/(30.0));
  frame.len = 3;
  
  int required_mtu = sizeof( canfd_frame );

	/* send frame */
	if (write(canSocket, &frame, required_mtu) != required_mtu) {
		perror("CAN failure - write");
		return 1;
	}

  return 0;
}


float normaliseAngle( float a)
{
  while( a < 0 )
    a = a + 2.0 * M_PI;

   while( a > 2.0 * M_PI )
    a = a- 2.0 * M_PI;

  return a;
}

float normaliseAngleMoreThan( float a, float b)
{
  while( a < b )
    a = a + 2.0 * M_PI;

   while( a > b + 2.0 * M_PI )
    a = a- 2.0 * M_PI;

  return a;
}

bool isRationalAngle( float a )
{

  return a > -5*M_PI && a < 5 * M_PI;
}


int backgroundForAngle(float rads)
{
  int b =  (rads / (2.0 * M_PI))*(float)(NUM_BACKGROUND-1);
  b = min( b, NUM_BACKGROUND-1);
  b = max(b, 0);
  return b;
}

class Cluster
{
  public :

  Cluster(float minR, float maxR, float s, float e) { minRange = minR; maxRange = maxR; startAngle = s; endAngle = e;};
  float minRange;
  float maxRange;
  float startAngle;
  float endAngle;
  float meanRange() { return (minRange+maxRange)/2.0;};
  float angle() { return (startAngle+endAngle) / 2.0; };
  float width() { return (normaliseAngleMoreThan(endAngle,startAngle)-startAngle) * meanRange(); };
  float backgroundRange() {  int b = backgroundForAngle((startAngle+endAngle)/2); return background[b].get();  };
     
};


int numScans = 0;
int numClusters = 0;


std::vector<Cluster> clusters;




// maintain a rolling average at each angle
void processBackground( LaserScan *scan)
{
  for( LaserPoint p : scan->points)
  {
    if( p.range>0.01 && isRationalAngle( p.angle))
    {
      float na = normaliseAngle( p.angle );

      int b = backgroundForAngle(na);
      if( numScans < 10 )
        background[b].force(p.range); // so we start up quickly
      else
        background[b].update(p.range);
    }
  }
}

bool inCluster = false;
float clusterStart = 0.0;
float clusterEnd = 0.0;
float clusterStartRange = 0.0;
float clusterMinRange = 0.0;
float clusterMaxRange = 0.0;
int clusterPoints = 0;


void startCluster(LaserPoint p, float na)
{
  clusterStart = na;
  clusterEnd = na;
  clusterStartRange = p.range;
  clusterMinRange = p.range;
  clusterMaxRange = p.range;
  inCluster = true;
  clusterPoints = 1;
  
  if( traceClusters) printf("\nstarted cluster at %0.4f rad, %0.4f m\n", clusterStart, clusterStartRange);

}

// is this point plausibly part of the current range ?
bool fitsCluster(LaserPoint p)
{
  if( abs(p.range-clusterStartRange) > 1.0 )
    return false;

  return true;
}

void extendCluster(LaserPoint p, float na)
{
  clusterEnd = normaliseAngleMoreThan(na,clusterStart);
  clusterMinRange = min(clusterMinRange, p.range);
  clusterMaxRange = max(clusterMaxRange, p.range);
  
  clusterPoints++;

  if( traceClusters) printf("    extended cluster starting at %0.4f to %0.4f rad %d points\n", clusterStart, clusterEnd,clusterPoints);

}



void endCluster()
{
  inCluster = false;
  if( clusterPoints < 2 ){
    if( traceClusters) printf("    abandoned cluster - only %d points\n", clusterPoints);

    return;
  }

  
  float clusterWidth = clusterStartRange * (clusterEnd-clusterStart);  // in m
  if( clusterWidth < 0.25 || clusterWidth > 2.0 )  // discard clusters with silly sizes
  {
    if( traceClusters) printf("    abandoned cluster %0.4f->%0.4f with %d points - bad width %0.4f m\n", clusterStart, clusterEnd, clusterPoints, clusterWidth);

    return;
  }

  
  numClusters ++;

  
  if( traceClusters) printf("    made cluster - %d points\n", clusterPoints);
  clusters.push_back( Cluster( clusterMinRange, clusterMaxRange, clusterStart, clusterEnd));
}

void processClusters( LaserScan *scan)
{
  clusters.clear();

  int numBackgrounds = 0;
  int numPoints = 0;
  float lastAngle = 0;
  bool inBackground = false;


  for( LaserPoint p : scan->points)
  {

    if( p.range > 0.01 && isRationalAngle( p.angle))
    {
      float na = normaliseAngle( p.angle );

      numPoints ++;
      

      
      int b = backgroundForAngle(na);
      float backgroundLimit = (0.9 * background[b].get() - 0.3);


     // if (numPoints%10 != 0)
      //  continue;

      //printf("    angle %f %f, delta %f, range %f (bg %f) \n", p.angle, na, na-lastAngle, p.range, backgroundLimit);

      //

      //continue;

      if( p.range > backgroundLimit)  
      {
        if( ! inBackground )
          if( traceClusters) printf(" ---\n");

        inBackground = true;

        numBackgrounds ++;
        if( inCluster )
          endCluster();
      }
      else
      {
        inBackground = false;

        if( traceClusters) printf("    angle %f %f, delta %f, range %f (bg %f) intensity %f\n", p.angle, na, na-lastAngle, p.range, backgroundLimit, p.intensity);

        if( ! inCluster )
          startCluster(p, na);
        else
          if( fitsCluster(p))
            extendCluster(p, na);
          else
          {
            endCluster();
            startCluster(p, na);
          }
      }

      lastAngle = na;

    }
  }

  if( inCluster )
        endCluster();

  printf("\n\n\n\n\n\n%d points\n", numPoints);
  printf("%d background points\n", numBackgrounds);
  printf("%d clusters\n", numClusters);
  
  int n = 0;

  

  // Theoretical max at our canbus speed is around 8000 messages/sec. 
  // We scan at 2hz, so 4000 clusters/scan is the max, a sensible limit is more like 1000 clusters/scan
  // I doubt we will ever get there but I'll drop clusters after 1000 anyway
  int numC = 0;
  sendCanStart(true);
  for(Cluster c:clusters)
  {
    if( numC++ < 1000)
      sendCanPerson(c.angle(), c.width(), c.meanRange());

    printf("  angle %0.4f (%f to %f) rad width %0.4fm range %0.2fm background at %0.2fm\n", c.angle(), c.startAngle, c.endAngle, c.width(), c.meanRange(), c.backgroundRange());
    if( n++ > 20 )
    {
      printf("   ..... \n\n");
    }
  }
  sendCanStart(false);
              
}



void processScan( LaserScan *scan)
{
  if( traceClusters) printf("processScan\n");
  numClusters = 0;
  
  processBackground(scan);
  processClusters(scan);
  numScans ++;

}


/**
 * @brief ydlidar test
 * @param argc
 * @param argv
 * @return
 * @par Flow chart
 * Step1: instance CYdLidar.\n
 * Step2: set paramters.\n
 * Step3: initialize SDK and LiDAR.(::CYdLidar::initialize)\n
 * Step4: Start the device scanning routine which runs on a separate thread and enable motor.(::CYdLidar::turnOn)\n
 * Step5: Get the LiDAR Scan Data.(::CYdLidar::doProcessSimple)\n
 * Step6: Stop the device scanning thread and disable motor.(::CYdLidar::turnOff)\n
 * Step7: Uninitialize the SDK and Disconnect the LiDAR.(::CYdLidar::disconnecting)\n
 */

int main(int argc, char *argv[]) {
  
  printf(" unfurl-lidar \n");
  printf("\n");
  fflush(stdout);

  connectCan();

  std::string port;
  ydlidar::os_init();

  std::map<std::string, std::string> ports =
    ydlidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    port = ports.begin()->second;
  } else {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
      printf("%d. %s\n", id, it->first.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::os_isOk()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }

  int baudrate = 230400;

  if( true)
  {
    baudrate = 512000; // for tg30
  }
  else
  {
    
    std::map<int, int> baudrateList;
    baudrateList[0] = 115200;
    baudrateList[1] = 128000;
    baudrateList[2] = 153600;
    baudrateList[3] = 230400;
    baudrateList[4] = 512000;

    printf("Baudrate:\n");

    for (std::map<int, int>::iterator it = baudrateList.begin();
        it != baudrateList.end(); it++) {
      printf("%d. %d\n", it->first, it->second);
    }

    while (ydlidar::os_isOk()) {
      printf("Please select the lidar baudrate:");
      std::string number;
      std::cin >> number;

      if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
        continue;
      }

      baudrate = baudrateList[atoi(number.c_str())];
      break;
    }
  }
  if (!ydlidar::os_isOk()) {
    return 0;
  }


  bool isSingleChannel = false;
  if( true)
  {
    isSingleChannel = false; // for tg30
  }
  else
  {
  
    std::string input_channel;
    printf("Whether the Lidar is one-way communication[yes/no]:");
    std::cin >> input_channel;
    std::transform(input_channel.begin(), input_channel.end(),
                  input_channel.begin(),
    [](unsigned char c) {
      return std::tolower(c);  // correct
    });

    if (input_channel.find("y") != std::string::npos) {
      isSingleChannel = true;
    }
  }

  if (!ydlidar::os_isOk()) {
    return 0;
  }

  std::string input_frequency;

  float frequency = 8.0;


  if( true)
  {
    frequency = 3.0; // actually gets us 2hz on TG30, which is about what we want
  }
  else
  {
  
    while (ydlidar::os_isOk() && !isSingleChannel) {
      printf("Please enter the lidar scan frequency[5-12]:");
      std::cin >> input_frequency;
      frequency = atof(input_frequency.c_str());

      if (frequency <= 12 && frequency >= 5.0) {
        break;
      }

      fprintf(stderr,
              "Invalid scan frequency,The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
    }
  }

  if (!ydlidar::os_isOk()) {
    return 0;
  }




  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  /// tof lidar
  int optval = TYPE_TOF;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = isSingleChannel ? 3 : 4;
  optval=10;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
  /// intensity
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = true;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  /// HeartBeat
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  } else {
    fprintf(stderr, "%s\n", laser.DescribeError());
    fflush(stderr);
  }

  LaserScan scan;


  while (ret && ydlidar::os_isOk()) {
    if (laser.doProcessSimple(scan)) {
      fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
              (long long unsigned int) scan.stamp,
              (unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
      processScan(&scan);
      fflush(stdout);
    } else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }

  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}