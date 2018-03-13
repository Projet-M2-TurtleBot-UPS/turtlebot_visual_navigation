/***********************************************************************/
/*                           Recherche.hpp                             */
/***********************************************************************/
/* -VERSION: ROS_Ubuntu 14.04                                          */
/* -AUTHOR:  BEAUHAIRE Pierre                                          */
/* -LAST_MODIFICATION: 03/2018                                         */
/***********************************************************************/

#ifndef RECHERCHE_H
#define RECHERCHE_H

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/Sound.h>

#define V_LIN_MAX 0.5                       // Seuil maximum pour la vitesse linéaire
#define V_LIN_MIN 0.3                       // Seuil minimum pour la vitesse linéaire
#define V_ANG_MAX 1.4                       // Vitesse angulaire du turtlebot
#define V_ANG_MIN 0.7                       // Vitesse angulaire du turtlebot
#define SEUIL_LIN_MAX 4.0                   // Seuil à partir duquel on fait baisser la vitesse linéaire
#define SEUIL_LIN_MIN 3.0                   // Seuil à partir duquel la vitesse linéaire du robot doit être minimale
#define SEUIL_ANG_MAX 0.1                   // Seuil à partir duquel on fait baisser la vitesse angulaire
#define SEUIL_ANG_MIN 0.02                  // Seuil à partir duquel la vitesse angulaire du robot doit être minimale
#define SEUIL_CRIT 0.5                      // Seuil critique pour atteindre l'amer (on ne doit pas aller plus loin)
#define BASE_ROBOT 0.075                    // Distance camera - centre du robot
#define ETAT_RECHERCHE 0                    // Etat dans lequel le robot recherche un amer
#define ETAT_ASSERVISSEMENT 1               // Etat dans lequel le robot s'asservit sur un amer
#define ETAT_FIN 2                          // Etat final

class Recherche
{
	private:
    ros::Subscriber ar_subscriber_;         // Publisher to /ar_pose_marker
    ros::Subscriber odom_subscriber_;       // Publisher to /odom
    ros::Publisher vel_publisher_;          // Publisher to /mobile_base/commands/velocity
    ros::Publisher sound_publisher_;        // Publisher to /mobile_base/commands/sound
    nav_msgs::Odometry odom_message_;       // The message to save odometry informations
    geometry_msgs::Twist twist_message_;    // The message to make the turtlebot move
    kobuki_msgs::Sound sound_message_;      // The message to make a sound
    double Marker_Position_[2];             // Marker_Position_[0] = x; Marker_Position_[1] = y; Marker_Position_[2] = z
    int etat_;                              // The id of the state we are in (Looking for an amer, enslavement on the amer, and final state)

	public:

		// Constructors
    Recherche(ros::NodeHandle nh);
    Recherche(){};

		// Destructor
		~Recherche(){};

    // Setters
    void setOdomMessageRecherche(const nav_msgs::Odometry &message);
    void arCallback(ar_track_alvar_msgs::AlvarMarkers req);
    void sendTwistMessage(float length, float angle);

		// Getters
    int getEtat();
    double * getMarkerPosition();

    // Operators
    double calculateAngle(float x, float y, float z, float w);
    int signe(double val);
    double calculVitesseAngulaire();
    void rechercherAmer();
    void asservissementAmer();
    void start();

};

#endif //RECHERCHE_H
