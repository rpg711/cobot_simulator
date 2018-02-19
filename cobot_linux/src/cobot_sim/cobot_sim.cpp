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
\brief   C++ Implementation: CobotSim
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================


#include "cobot_sim.h"

const double CobotSim::baseRadius = 0.18;
const double CobotSim::robotHeight = 0.36;
const float CobotSim::startX = -7.5;
const float CobotSim::startY = 1.0;
const float CobotSim::startAngle = 0.0;

CobotSim::CobotSim() {
  w0.heading(RAD(45.0));
  w1.heading(RAD(135.0));
  w2.heading(RAD(-135.0));
  w3.heading(RAD(-45.0));
  tLastCmd = GetTimeSec();
}

CobotSim::~CobotSim() { }

void CobotSim::init(ros::NodeHandle& n) {
  scanDataMsg.header.seq = 0;
  scanDataMsg.header.frame_id = "base_laser";
  scanDataMsg.angle_min = RAD(-135.0);
  scanDataMsg.angle_max = RAD(135.0);
  scanDataMsg.range_min = 0.02;
  scanDataMsg.range_max = 4.0;
  scanDataMsg.angle_increment = RAD(360.0)/1024.0;
  scanDataMsg.intensities.clear();
  scanDataMsg.time_increment = 0.0;
  scanDataMsg.scan_time = 0.05;

  odometryTwistMsg.header.seq = 0;
  odometryTwistMsg.header.frame_id = "odom";
  odometryTwistMsg.child_frame_id = "base_footprint";

  curLoc.set(CobotSim::startX, CobotSim::startY);
  curAngle = CobotSim::startAngle;

  initCobotSimVizMarkers();
  loadAtlas();

  // ROS_INFO("Init Robot Pose: (%4.3f, %4.3f, %4.3f)", curLoc.x, curLoc.y, curAngle);

  driveSubscriber = n.subscribe("/Cobot/Drive", 1, &CobotSim::cobotDriveCallback, this);
  odometryTwistPublisher = n.advertise<nav_msgs::Odometry>("/odom",1);
  laserPublisher = n.advertise<sensor_msgs::LaserScan>("/Cobot/Laser", 1);
  mapLinesPublisher = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  posMarkerPublisher = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  dirMarkerPublisher = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  br = new tf::TransformBroadcaster();
}

/**
 * Helper method that initializes visualization_msgs::Marker parameters
 * @param vizMarker   pointer to the visualization_msgs::Marker object
 * @param ns          namespace for marker (string)
 * @param id          id of marker (int) - must be unique for each marker;
 *                      0, 1, and 2 are already used
 * @param type        specifies type of marker (string); available options:
 *                      arrow (default), cube, sphere, cylinder, linelist
 * @param p           stamped pose to define location and frame of marker
 * @param scale       scale of the marker; see visualization_msgs::Marker
 *                      documentation for details on the parameters
 * @param duration    lifetime of marker in RViz (double); use duration of 0.0
 *                      for infinite lifetime
 * @param color       vector of 4 float values representing color of marker;
 *                    0: red, 1: green, 2: blue, 3: alpha
 */
void CobotSim::initVizMarker(visualization_msgs::Marker& vizMarker, string ns,
    int id, string type, geometry_msgs::PoseStamped p,
    geometry_msgs::Point32 scale, double duration, vector<float> color) {

  vizMarker.header.frame_id = p.header.frame_id;
  vizMarker.header.stamp = ros::Time::now();

  vizMarker.ns = ns;
  vizMarker.id = id;

  if (type == "arrow") {
    vizMarker.type = visualization_msgs::Marker::ARROW;
  } else if (type == "cube") {
    vizMarker.type = visualization_msgs::Marker::CUBE;
  } else if (type == "sphere") {
    vizMarker.type = visualization_msgs::Marker::SPHERE;
  } else if (type == "cylinder") {
    vizMarker.type = visualization_msgs::Marker::CYLINDER;
  } else if (type == "linelist") {
    vizMarker.type = visualization_msgs::Marker::LINE_LIST;
  } else {
    vizMarker.type = visualization_msgs::Marker::ARROW;
  }

  vizMarker.pose = p.pose;
  vizMarker.points.clear();
  vizMarker.scale.x = scale.x;
  vizMarker.scale.y = scale.y;
  vizMarker.scale.z = scale.z;

  vizMarker.lifetime = ros::Duration(duration);

  vizMarker.color.r = color.at(0);
  vizMarker.color.g = color.at(1);
  vizMarker.color.b = color.at(2);
  vizMarker.color.a = color.at(3);

  vizMarker.action = visualization_msgs::Marker::ADD;
}

void CobotSim::initCobotSimVizMarkers() {
  geometry_msgs::PoseStamped p;
  geometry_msgs::Point32 scale;
  vector<float> color;
  color.resize(4);

  p.header.frame_id = "/map";

  p.pose.orientation.w = 1.0;
  scale.x = 0.1;
  scale.y = 0.0;
  scale.z = 0.0;
  color[0] = 0.0;
  color[1] = 0.0;
  color[2] = 1.0;
  color[3] = 1.0;
  initVizMarker(lineListMarker, "map_lines", 0, "linelist", p, scale, 0.0, color);

  p.pose.position.z = CobotSim::robotHeight / 2.0;
  scale.x = CobotSim::baseRadius * 2.0;
  scale.y = CobotSim::baseRadius * 2.0;
  scale.z = CobotSim::robotHeight;
  color[0] = 1.0;
  color[1] = 0.0;
  color[2] = 0.0;
  color[3] = 1.0;
  initVizMarker(robotPosMarker, "robot_position", 1, "cylinder", p, scale, 0.0, color);

  scale.x = CobotSim::baseRadius * 2.0;
  scale.y = 0.1;
  scale.z = 0.1;
  color[0] = 0.0;
  color[1] = 1.0;
  color[2] = 0.0;
  color[3] = 1.0;
  initVizMarker(robotDirMarker, "robot_direction", 2, "arrow", p, scale, 0.0, color);

}

void CobotSim::loadAtlas() {
  static const bool debug = false;
  string atlas_path = ros::package::getPath("cobot_linux").append("/../maps/atlas.txt");

  FILE* fid = fopen(atlas_path.c_str(),"r");
  if(fid==NULL) {
    TerminalWarning("Unable to load Atlas!");
    return;
  }
  char mapName[4096];
  int mapNum;
  if(debug) printf("Loading Atlas...\n");
  maps.clear();
  while(fscanf(fid,"%d %s\n",&mapNum,mapName)==2) {
    if(debug) printf("Loading map %s\n",mapName);
    maps.push_back(VectorMap(mapName,ros::package::getPath("cobot_linux").append("/../maps").c_str(),false));
  }
  curMapIdx = -1;
  if(maps.size()>0) {
    curMapIdx = 0;
    currentMap = &maps[curMapIdx];
  }

  vector<line2f> map_segments = currentMap->Lines();
  for (size_t i = 0; i < map_segments.size(); i++) {
    // The line list needs two points for each line
    geometry_msgs::Point p0, p1;

    p0.x = map_segments.at(i).P0().x;
    p0.y = map_segments.at(i).P0().y;
    p0.z = 0.01;

    p1.x = map_segments.at(i).P1().x;
    p1.y = map_segments.at(i).P1().y;
    p1.z = 0.01;

    lineListMarker.points.push_back(p0);
    lineListMarker.points.push_back(p1);
  }
}

void CobotSim::cobotDriveCallback(const cobot_msgs::CobotDriveMsgConstPtr& msg) {

  if (!isfinite(msg->v) || !isfinite(msg->w)) {
    printf("Ignoring non-finite drive values: %f %f\n", msg->v, msg->w);
    return;
  }

  double dt = GetTimeSec() - tLastCmd;
  double desiredTransSpeed = msg->v;
  double desiredRotSpeed = msg->w;

  //Abide by limits
  if(fabs(desiredTransSpeed) > transLimits.max_vel) {
    desiredTransSpeed = transLimits.max_vel;
  }
  double dvMax = 0.0;
  if(desiredTransSpeed > vel) {
    dvMax = dt * transLimits.max_accel;
  } else {
    dvMax = dt * transLimits.max_deccel;
  }
  double dv = desiredTransSpeed - vel;
  if(fabs(dv) > dvMax) {
    dv = sign(dv) * dvMax;
  }
  vel = vel + dv;

  if(fabs(desiredRotSpeed) > rotLimits.max_vel) {
    desiredRotSpeed = sign(desiredRotSpeed) * rotLimits.max_vel;
  }
  double drMax = dt * rotLimits.max_accel;
  double dr = desiredRotSpeed - angVel;
  if(fabs(dr) > drMax) {
    dr = sign(dr) * drMax;
  }
  angVel = angVel + dr;

  curLoc.x += vel * cos(curAngle) * dt;
  curLoc.y += vel * sin(curAngle) * dt;
  curAngle = angle_mod(curAngle+angVel*dt);

  // ROS_WARN("Robot pose: (%4.3f, %4.3f, %4.3f)", curLoc.x, curLoc.y, curAngle);

  tLastCmd = GetTimeSec();
}

void CobotSim::publishOdometry() {
  static const double kMaxCommandAge = 0.5;

  if (GetTimeSec() > tLastCmd + kMaxCommandAge) {
    angVel = 0.0;
    vel = 0.0;
  }

  tf::Quaternion robotQ = tf::createQuaternionFromYaw(curAngle);

  odometryTwistMsg.header.stamp = ros::Time::now();
  odometryTwistMsg.pose.pose.position.x = curLoc.x;
  odometryTwistMsg.pose.pose.position.y = curLoc.y;
  odometryTwistMsg.pose.pose.position.z = 0.0;
  odometryTwistMsg.pose.pose.orientation.x = robotQ.x();
  odometryTwistMsg.pose.pose.orientation.y = robotQ.y();
  odometryTwistMsg.pose.pose.orientation.z = robotQ.z();;
  odometryTwistMsg.pose.pose.orientation.w = robotQ.w();
  odometryTwistMsg.twist.twist.angular.x = 0.0;
  odometryTwistMsg.twist.twist.angular.y = 0.0;
  odometryTwistMsg.twist.twist.angular.z = angVel;
  odometryTwistMsg.twist.twist.linear.x = vel * cos(curAngle);
  odometryTwistMsg.twist.twist.linear.y = vel * sin(curAngle);
  odometryTwistMsg.twist.twist.linear.z = 0.0;

  odometryTwistPublisher.publish(odometryTwistMsg);

  robotPosMarker.pose.position.x = curLoc.x;
  robotPosMarker.pose.position.y = curLoc.y;
  robotPosMarker.pose.position.z = CobotSim::robotHeight / 2.0;
  robotPosMarker.pose.orientation.w = 1.0;

  robotDirMarker.pose.position.x = curLoc.x;
  robotDirMarker.pose.position.y = curLoc.y;
  robotDirMarker.pose.position.z = CobotSim::robotHeight / 2.0;
  robotDirMarker.pose.orientation.x = robotQ.x();
  robotDirMarker.pose.orientation.y = robotQ.y();
  robotDirMarker.pose.orientation.z = robotQ.z();
  robotDirMarker.pose.orientation.w = robotQ.w();

}

void CobotSim::publishLaser() {
  if(curMapIdx<0 || curMapIdx>=int(maps.size())) {
    TerminalWarning("Unknown map, unable to generate laser scan!");
    curMapIdx = -1;
    return;
  }

  scanDataMsg.header.stamp = ros::Time::now();
  vector2f laserLoc(0.145,0.0);
  // ROS_INFO("curLoc: (%4.3f, %4.3f)", curLoc.x, curLoc.y);
  laserLoc = curLoc + laserLoc.rotate(curAngle);
  vector<float> ranges = maps[curMapIdx].getRayCast(laserLoc,curAngle,RAD(360.0)/1024.0,769,0.02,4.0);
  scanDataMsg.ranges.resize(ranges.size());
  for(int i=0; i<int(ranges.size()); i++) {
    scanDataMsg.ranges[i] = ranges[i];
    if(ranges[i]>3.5) {
      scanDataMsg.ranges[i] = 0.0;
    }
  }
  laserPublisher.publish(scanDataMsg);
}

void CobotSim::publishTransform() {
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/odom"));

  transform.setOrigin(tf::Vector3(curLoc.x,curLoc.y,0.0));
  q.setRPY(0.0,0.0,curAngle);
  transform.setRotation(q);
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_footprint"));

  transform.setOrigin(tf::Vector3(0.0 ,0.0, 0.0));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_footprint", "/base_link"));

  transform.setOrigin(tf::Vector3(0.145,0.0, 0.23));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/base_laser"));
}

void CobotSim::publishVisualizationMarkers() {
  mapLinesPublisher.publish(lineListMarker);
  posMarkerPublisher.publish(robotPosMarker);
  dirMarkerPublisher.publish(robotDirMarker);
}

void CobotSim::run() {
  //publish odometry and status
  publishOdometry();
  //publish laser rangefinder messages
  publishLaser();
  // publish visualization marker messages
  publishVisualizationMarkers();
  //publish tf
  publishTransform();
}
