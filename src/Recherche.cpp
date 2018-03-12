/***********************************************************************/
/*                           Recherche.cpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BEAUHAIRE Pierre                                          */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#include "Recherche.hpp"

// Constructor
Recherche::Recherche(ros::NodeHandle nh) {
  this->ar_subscriber_ = nh.subscribe("ar_pose_marker", 1, &Recherche::arCallback, this);
  this->odom_subscriber_ = nh.subscribe("odom", 1, &Recherche::setOdomMessageRecherche, this);
  this->vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  this->sound_publisher_ = nh.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
  this->odom_message_.pose.pose.position.x = 0.0;
  this->odom_message_.pose.pose.position.y = 0.0;
  this->odom_message_.pose.pose.orientation.z = 0.0;
  this->odom_message_.pose.pose.orientation.w = 0.0;
  this->twist_message_.linear.x = 0.0;
  this->twist_message_.linear.y = 0.0;
  this->twist_message_.linear.z = 0.0;
  this->twist_message_.angular.x = 0.0;
  this->twist_message_.angular.y = 0.0;
  this->twist_message_.angular.z = 0.0;
  this->sound_message_.value = 0.0;
  this->Marker_Position_[0] = -1.0;
  this->Marker_Position_[1] = -1.0;
  this->Marker_Position_[2] = -1.0;         // à l'init, ne voit pas d'amers, => x = y = z = -1
  this->etat_ = ETAT_RECHERCHE;
}

// Callback of the odometry topic. Sets the odom message with odometry informations
void Recherche::setOdomMessageRecherche(const nav_msgs::Odometry &message) {
  this->odom_message_.pose.pose.position.x = message.pose.pose.position.x;
  this->odom_message_.pose.pose.position.y = message.pose.pose.position.y;
  this->odom_message_.pose.pose.orientation.z = message.pose.pose.orientation.z;
  this->odom_message_.pose.pose.orientation.w = message.pose.pose.orientation.w;
}

// Callback of the ar_pose_marker topic. Sets the position of the amer
void Recherche::arCallback(ar_track_alvar_msgs::AlvarMarkers req) {
  // si on voit un ar
  if (!req.markers.empty()) {
    this->Marker_Position_[0] = req.markers[0].pose.pose.position.x;
    this->Marker_Position_[1] = req.markers[0].pose.pose.position.y;
    this->Marker_Position_[2] = req.markers[0].pose.pose.position.z;
  }
  else {
    ("Je vois rien\n");
    this->Marker_Position_[0] = -1.0;
    this->Marker_Position_[1] = -1.0;
    this->Marker_Position_[2] = -1.0;
  }
}

// Sets twist message to control speed, and sends the message to /mobile_base/commands/velocity topic
void Recherche::sendTwistMessage(float length, float angle) {
  this->twist_message_.linear.x = length;
  this->twist_message_.linear.y = 0.0;
  this->twist_message_.linear.z = 0.0;
  this->twist_message_.angular.x = 0.0;
  this->twist_message_.angular.y = 0.0;
  this->twist_message_.angular.z = angle;
  this->vel_publisher_.publish(this->twist_message_);
}

// Returns the state of the research
int Recherche::getEtat() {
  return this->etat_;
}

// Returns the position (x,y,z) of the amer in sight
double* Recherche::getMarkerPosition() {
  return this->Marker_Position_;
}

// Calculates the orientated angle of the robot thanks to odometry
double Recherche::calculateAngle(float x, float y, float z, float w) {
  double cosy, siny, yaw;
  cosy = 1.0 - 2.0 * (y * y + z * z);  
  siny = 2.0 * (w * z + x * y);
  yaw = atan2(siny, cosy);          // yaw : rotation around Z axis
  return (yaw + M_PI);              // angle between [0 ; 2Pi]
}

// Makes the TurtleBot turns on itself until it sees an amer
void Recherche::rechercherAmer() {
  printf("... RECHERCHE AMER ...\n");
  double angle_depart = calculateAngle(this->odom_message_.pose.pose.orientation.x, this->odom_message_.pose.pose.orientation.y, this->odom_message_.pose.pose.orientation.z, this->odom_message_.pose.pose.orientation.w);
  double angle_courant = angle_depart;
  double seuil = 0.2;
  int cpt = 0;
  bool found = true;
  // tant qu'on ne voit pas d'amer
  while (((this->Marker_Position_[0] == -1) || (this->Marker_Position_[1] == -1) || (this->Marker_Position_[2] == -1)) && (found)) {
    // on tourne avec pour maximum 2Pi
    ros::spinOnce();
    angle_courant = calculateAngle(this->odom_message_.pose.pose.orientation.x, this->odom_message_.pose.pose.orientation.y, this->odom_message_.pose.pose.orientation.z, this->odom_message_.pose.pose.orientation.w);
    if (((cpt < 100000) && (cpt > 200)) && ((angle_courant < angle_depart) || (angle_courant > (angle_depart + seuil)))) {
      found = false;
      //exit(0);
    }
    else {
      sendTwistMessage(0.0, 1.0);
    }
    cpt++;
    ros::Duration(0.05).sleep();
  }
  if (found) {
    this->sound_message_.value = 0.0;
    this->sound_publisher_.publish(this->sound_message_);
    this->etat_ = ETAT_ASSERVISSEMENT;
  } else {
    this->sound_message_.value = 1.0;
    this->sound_publisher_.publish(this->sound_message_);
    this->etat_ = ETAT_FIN;
  }
}

// Makes the turtlebot move to the front of the amer
void Recherche::asservissementAmer() {
  printf("... ASSERVISSEMENT SUR L'AMER ...\n");
  ros::Duration(0.05).sleep();
  double vitesse = V_LIN_MAX;
  double pente = (V_LIN_MAX - V_LIN_MIN) / (SEUIL_MAX - SEUIL_MIN);
  // on avance de V_LIN_MAX m/s jusqu'à distance_amer = SEUIL_MAX
  while ((this->Marker_Position_[2]-BASE_ROBOT) > SEUIL_MAX) {
    if (this->Marker_Position_[0] < 0) {
      sendTwistMessage(V_LIN_MAX, V_ANG);
    } else if (this->Marker_Position_[0] > 0) {
      sendTwistMessage(V_LIN_MAX, -V_ANG);
    } else {
      sendTwistMessage(V_LIN_MAX, 0.0);
    }
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  // on avance avec une vitesse dégressive jusqu'à (vitesse = V_LIN_MIN || distance_amer <= SEUIL_MIN)
  while ((vitesse > V_LIN_MIN) && ((this->Marker_Position_[2]-BASE_ROBOT) > SEUIL_MIN)) {
    // on met à jour la vitesse
    vitesse = pente * (this->Marker_Position_[2]-BASE_ROBOT);
    vitesse = V_LIN_MIN;
    if (this->Marker_Position_[0] < 0) {
      sendTwistMessage(vitesse, V_ANG);
    } else if (this->Marker_Position_[0] > 0) {
      sendTwistMessage(vitesse, -V_ANG);
    } else {
      sendTwistMessage(vitesse, 0.0);
    }
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  // on avance à vitesse = V_LIN_MIN jusqu'à distance_amer = SEUIL_CRIT
  while ((this->Marker_Position_[2]-BASE_ROBOT) > SEUIL_CRIT) {
    if (this->Marker_Position_[0] < 0) {
      sendTwistMessage(V_LIN_MIN, V_ANG);
    } else if (this->Marker_Position_[0] > 0) {
      sendTwistMessage(V_LIN_MIN, -V_ANG);
    } else {
      sendTwistMessage(V_LIN_MIN, 0.0);
    }
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  // si on a perdu l'amer de vue pendant l'asservissement
  if ((this->Marker_Position_[0] == -1) && (this->Marker_Position_[1] == -1) && (this->Marker_Position_[2] == -1)) {
    this->sound_message_.value = 1.0;
    this->sound_publisher_.publish(this->sound_message_);
    this->etat_ = ETAT_RECHERCHE;
  } else {
    this->sound_message_.value = 0.0;
    this->sound_publisher_.publish(this->sound_message_);
    this->etat_ = ETAT_FIN;
  }
}

// Describes the TurtleBot moves sequence
void Recherche::start() {
  double * amer;
  while (getEtat() != ETAT_FIN) {
    switch(getEtat()) {
      case ETAT_RECHERCHE:
        rechercherAmer();
        amer = getMarkerPosition();
        printf("( %lf , %lf , %lf )\n", amer[0], amer[1], amer[2]);
        break;
      case ETAT_ASSERVISSEMENT:
        asservissementAmer();
        amer = getMarkerPosition();
        printf("( %lf , %lf , %lf )\n", amer[0], amer[1], amer[2]);
        break;
      default:
        rechercherAmer();
        amer = getMarkerPosition();
        printf("( %lf , %lf , %lf )\n", amer[0], amer[1], amer[2]);
        break;
    }
    ros::Duration(1).sleep();
  }
}

