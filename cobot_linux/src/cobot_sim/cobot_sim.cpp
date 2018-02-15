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

const double CobotSim::baseRadius = 0.2;

CobotSim::CobotSim()
{
  w0.heading(RAD(45.0));
  w1.heading(RAD(135.0));
  w2.heading(RAD(-135.0));
  w3.heading(RAD(-45.0));
  t_last_cmd = GetTimeSec();
}

CobotSim::~CobotSim()
{

}

void CobotSim::loadAtlas()
{
  static const bool debug = false;
  string atlas_path = ros::package::getPath("cobot_linux").append("/../maps/atlas.txt");

  FILE* fid = fopen(atlas_path.c_str(),"r");
  if(fid==NULL){
    TerminalWarning("Unable to load Atlas!");
    return;
  }
  char mapName[4096];
  int mapNum;
  if(debug) printf("Loading Atlas...\n");
  maps.clear();
  while(fscanf(fid,"%d %s\n",&mapNum,mapName)==2){
    if(debug) printf("Loading map %s\n",mapName);
    maps.push_back(VectorMap(mapName,ros::package::getPath("cobot_linux").append("/../maps").c_str(),true));
  }
  curMapIdx = -1;
  if(maps.size()>0){
    curMapIdx = 0;
    currentMap = &maps[curMapIdx];
  }
}

// void CobotSim::localizationCallback(const cobot_msgs::CobotLocalizationMsgConstPtr& msg)
// {
//   basic_string<char> mapName = msg->map;
//   bool reloadMap = curMapIdx<0;
//   if(!reloadMap)
//     reloadMap = (maps[curMapIdx].mapName.compare(mapName) != 0);
//   if(reloadMap){
//     curMapIdx = -1;
//     for(unsigned int i=0; i<maps.size() && curMapIdx<0; i++){
//       if(maps[i].mapName.compare(mapName) == 0)
//         curMapIdx = i;
//     }
//     if(curMapIdx<0){
//       char buf[1025];
//       snprintf(buf, 1023, "Map %s not found in atlas!",mapName.c_str());
//       TerminalWarning(buf);
//     }
//   }
//   if( (curLoc-vector2f(msg->x,msg->y)).sqlength()>0.2 || angle_diff(curAngle, msg->angle)>RAD(10.0)){
//     printf("CoBot Simulator: reset location to %.3f,%.3f, %.2f\u00b0\n",msg->x,msg->y, DEG(msg->angle));
//     curLoc.set(msg->x,msg->y);
//     curAngle = msg->angle;
//   }
// }

void CobotSim::init(ros::NodeHandle& n)
{
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

//   odometryTwistMsg.header.seq = 0;
//   odometryTwistMsg.header.frame_id = "odom";
//   odometryTwistMsg.child_frame_id = "base_footprint";

  loadAtlas();
  drive_sub = n.subscribe("Cobot/Drive", 1, &CobotSim::cobotDriveCallback, this);
//   odometryPublisher = n.advertise<cobot_msgs::CobotOdometryMsg>("Cobot/Odometry",1);
//   odometryTwistPublisher = n.advertise<nav_msgs::Odometry>("odom",1);
  laserPublisher = n.advertise<sensor_msgs::LaserScan>("Cobot/Laser", 1);
//   localizationSubscriber = n.subscribe("Cobot/Localization",1,&CobotSim::localizationCallback,this);
//   br = new tf::TransformBroadcaster();
}

void CobotSim::cobotDriveCallback(const cobot_msgs::CobotDriveMsgConstPtr& msg)
{
  if (!isfinite(msg->velocity_x) ||
    !isfinite(msg->velocity_y) ||
    !isfinite(msg->velocity_r)) {
    printf("Ignoring non-finite drive values: %f %f %f\n",
           msg->velocity_x, msg->velocity_y, msg->velocity_r);
    return;
  }

  double dt = GetTimeSec() - t_last_cmd;
  vector2d desiredTransSpeed(msg->velocity_x,msg->velocity_y);
  double desiredRotSpeed = msg->velocity_r;

  //Abide by limits
  if(desiredTransSpeed.sqlength()>sq(transLimits.max_vel))
    desiredTransSpeed = desiredTransSpeed.norm(transLimits.max_vel);
  double dvMax = 0.0;
  if(desiredTransSpeed.sqlength()>vel.sqlength())
    dvMax = dt*transLimits.max_accel;
  else
    dvMax = dt*transLimits.max_deccel;
  vector2d dv = desiredTransSpeed - vel;
  if(dv.sqlength()>sq(dvMax)){
    dv = dv.norm(dvMax);
  }
  vel = vel + dv;

  if(fabs(desiredRotSpeed)>rotLimits.max_vel)
    desiredRotSpeed = sign(desiredRotSpeed)*rotLimits.max_vel;
  double drMax = dt*rotLimits.max_accel;
  double dr = desiredRotSpeed - ang_vel;
  if(fabs(dr)>drMax){
    dr = sign(dr)*drMax;
  }
  ang_vel = ang_vel + dr;

  curLoc += vector2f(V2COMP(vel)).rotate(curAngle)*dt;
  curAngle = angle_mod(curAngle+ang_vel*dt);

  t_last_cmd = GetTimeSec();
}

// void CobotSim::publishOdometry()
// {
//   static const double kMaxCommandAge = 0.5;
//   static double tLast = GetTimeSec();
//
//   if (GetTimeSec() > t_last_cmd + kMaxCommandAge) {
//     ang_vel = 0.0;
//     vel.zero();
//   }
//   double dT = GetTimeSec() - tLast;
//   cobot_msgs::CobotOdometryMsg msg;
//   msg.dr = ang_vel*dT;
//   msg.dx = vel.x*dT;
//   msg.dy = vel.y*dT;
//   msg.v0 = vel.dot(w0)+baseRadius*ang_vel;
//   msg.v1 = vel.dot(w1)+baseRadius*ang_vel;
//   msg.v2 = vel.dot(w2)+baseRadius*ang_vel;
//   msg.v3 = vel.dot(w3)+baseRadius*ang_vel;
//   msg.vr = ang_vel;
//   msg.vx = vel.x;
//   msg.vy = vel.y;
//   msg.VBatt = 32.0;
//   msg.status = 0x04;
//
//   odometryPublisher.publish(msg);
//
//   odometryTwistMsg.header.stamp = ros::Time::now();
//   odometryTwistMsg.pose.pose.position.x = curLoc.x;
//   odometryTwistMsg.pose.pose.position.y = curLoc.y;
//   odometryTwistMsg.pose.pose.position.z = 0;
//   odometryTwistMsg.pose.pose.orientation.w = cos(curAngle*0.5);
//   odometryTwistMsg.pose.pose.orientation.x = 0;
//   odometryTwistMsg.pose.pose.orientation.y = 0;
//   odometryTwistMsg.pose.pose.orientation.z = sin(curAngle*0.5);
//   odometryTwistMsg.twist.twist.angular.x = 0;
//   odometryTwistMsg.twist.twist.angular.y = 0;
//   odometryTwistMsg.twist.twist.angular.z = ang_vel;
//   odometryTwistMsg.twist.twist.linear.x = vel.x;
//   odometryTwistMsg.twist.twist.linear.y = vel.y;
//   odometryTwistMsg.twist.twist.linear.z = 0;
//
//   odometryTwistPublisher.publish(odometryTwistMsg);
//
//   tLast = GetTimeSec();
// }

void CobotSim::publishLaser()
{
  if(curMapIdx<0 || curMapIdx>=int(maps.size())){
    TerminalWarning("Unknown map, unable to generate laser scan!");
    curMapIdx = -1;
    return;
  }

  scanDataMsg.header.stamp = ros::Time::now();
  vector2f laserLoc(0.145,0.0);
  laserLoc = curLoc + laserLoc.rotate(curAngle);
  vector<float> ranges = maps[curMapIdx].getRayCast(laserLoc,curAngle,RAD(360.0)/1024.0,769,0.02,4.0);
  scanDataMsg.ranges.resize(ranges.size());
  for(int i=0; i<int(ranges.size()); i++){
    scanDataMsg.ranges[i] = ranges[i];
    if(ranges[i]>3.5)
      scanDataMsg.ranges[i] = 0.0;
  }
  laserPublisher.publish(scanDataMsg);
}

// void CobotSim::publishTransform()
// {
//   tf::Transform transform;
//   transform.setOrigin(tf::Vector3(curLoc.x,curLoc.y, 0.0));
//   transform.setRotation(tf::Quaternion(tf::Vector3(0,0,1),curAngle));
//   br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
//
//   transform.setOrigin(tf::Vector3(0.0 ,0.0, 0.0));
//   transform.setRotation(tf::Quaternion(0, 0, 0, 1));
//   br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
//
//   transform.setOrigin(tf::Vector3(0.145,0.0, 0.23));
//   transform.setRotation(tf::Quaternion(0, 0, 0, 1));
//   br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser"));
//
// }

void CobotSim::run()
{
  //publish odometry and status
//   publishOdometry();
  //publish Laser rangefinder messages
  publishLaser();
  //publish tf
//   publishTransform();
}


