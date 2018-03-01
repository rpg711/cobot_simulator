//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    cobot_sim.h
\brief   C++ Interface: CobotSim
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <iostream>
#include <stdio.h>
#include <vector>
#include "proghelp.h"
#include "timer.h"
#include "geometry.h"
#include "hardware/drive.h"
#include "vector_map.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "cobot_msgs/CobotDriveMsg.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#ifndef COBOT_SIM_H
#define COBOT_SIM_H

using namespace std;

class CobotSim{
  vector2d loc;
  double vel;
  double angVel;
  AccelLimits transLimits, rotLimits;

  ros::Subscriber driveSubscriber;

  ros::Publisher odometryTwistPublisher;
  ros::Publisher laserPublisher;
  ros::Publisher mapLinesPublisher;
  ros::Publisher posMarkerPublisher;
  ros::Publisher dirMarkerPublisher;
  tf::TransformBroadcaster *br;

  // wheel orientations
  vector2d w0,w1,w2,w3;
  // Radius of base
  static const double baseRadius;
  // "height" of robot
  static const double robotHeight;

  sensor_msgs::LaserScan scanDataMsg;
  nav_msgs::Odometry odometryTwistMsg;

  VectorMap* currentMap;
  vector<VectorMap> maps;
  int curMapIdx;

  visualization_msgs::Marker lineListMarker;
  visualization_msgs::Marker robotPosMarker;
  visualization_msgs::Marker robotDirMarker;

  static const float startX;
  static const float startY;
  vector2f curLoc;

  static const float startAngle;
  float curAngle;

  double tLastCmd;

  static const float DT;

private:
  void initVizMarker(visualization_msgs::Marker& vizMarker, string ns, int id, string type, geometry_msgs::PoseStamped p, geometry_msgs::Point32 scale, double duration, vector<float> color);
  void initCobotSimVizMarkers();
  void loadAtlas();
  void cobotDriveCallback(const cobot_msgs::CobotDriveMsgConstPtr& msg);
  void publishOdometry();
  void publishLaser();
  void publishVisualizationMarkers();
  void publishTransform();

public:
  CobotSim();
  ~CobotSim();
  void setLimits(AccelLimits _transLimits, AccelLimits _rotLimits){transLimits = _transLimits; rotLimits = _rotLimits;}
  void init(ros::NodeHandle &n);
  void run();
};
#endif //COBOT_SIM_H
