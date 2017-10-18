
#ifndef DIVNCONQ_LOCAL_PLANNER_ROS_H_
#define DIVNCONQ_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>

// time
#include <time.h>

//files
#include <fstream>
#include <iostream>
using namespace std;

// msgs
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// boost classes  TODO do I need?
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// other
#include <array>
#include <vector>

// definitions
#define PI 3.14159265
#define D2R 0.0174532925      // = 3.14159265/180

namespace divnconq_local_planner{

  /**
   * @class DivnConqPlannerROS
   * @brief Plugin to the ros base_local_planner. Implements a wrapper for the Elastic Band Method
   */

   struct pos {

	double x, y, az;	

   };

   struct region {

	int begin, end;	
	bool checked; //determines whether navigability has been checked

   };

   struct gaps {

	int desc; //is it descending? = 1 or -1
	int deg; //degree of the gap

   };


  class DivnConqPlannerROS : public nav_core::BaseLocalPlanner{

    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      DivnConqPlannerROS();

      /**
       * @brief Constructs the ros wrapper
       * @param name The name to give this instance of the elastic band local planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      DivnConqPlannerROS(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~DivnConqPlannerROS();

      /**
       * @brief Initializes the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Set the plan that the controller is following; also reset DivnConq-planner
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

    private:

      //Pointer to external objects (do NOT delete object)
      costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap  TODO: not used yet
      tf::TransformListener* tf_; ///<@brief pointer to Transform Listener  TODO: not used yet

      // Topics & Services
      ros::Subscriber amcl_sub; ///<@brief subscribes to the amcl topic
      ros::Subscriber laser_sub; ///<@brief subscribes to the laser topic
      ros::Publisher path_pub; ///<@brief publishes to the bubble shape to visualize on rviz 

      // Data
      pos now; // present frame
      pos next; // next frame
      pos nError; // error between present and next frames
      double distance_;
      int length; // number of frames in the global plan 
      int count; // keeps track of the number for the next frame in the global plan
      std::vector<geometry_msgs::PoseStamped> plan; // contains the global plan
      geometry_msgs::Twist cmd; // contains the velocity
      visualization_msgs::Marker points;

      double yawR;
      sensor_msgs::LaserScan laserData;

      double PND[180]; //pose nearness diagram data
      double RND[180]; //robot nearness diagram data
      gaps gap[135]; //135 because you can have maximum 180*3/4 (3 gaps per 4 degrees)
      region reg[68]; //68 because you can have maximum 135/2 regions (one per 2 gaps)
      int numberGaps;
      int numberRegions;

      int intermGoalA; //angle to intermediate goal
      double intermGoalD; //distance to intermediate goal
      int intermGoal; //the value of the intermediate goal's place in the global path

      double dmax; //maximum laser range
      double R; //robot's radius
      double ds; //security distance

      int clostRegion; //closest region
      int side; //what side of the goal the closest region is
      int situation; //contains the obstacle situation
      int Srd; //angle of the gap in the closest region closes to the intermediate goal

      bool inside[180]; //indicates if an obstacle is inside the security zone

     //measuring
      double average;
      int num;
      ofstream file;
      double stopTime, startTime;
      bool firstTime, hasStarted;
      double pathLength;


      // Flags
      bool goal_reached_;
      bool initialized_;
      bool flag, flag2;

      // Velocity methods
      /**
      * @brief Set Vel: function that sets linear speed 
      */
      void setVel();

      /**
      * @brief Set Rot: function that sets angular speed 
      */
      void setRot();

      /**
      * @brief Set Vel Z: function that sets linear and angular speed to ZERO
      */
      void setVelZ();


      // Methods
      /**
       * @brief Amcl Callback: function is called whenever a new amcl msg is published on that topic 
       * @param Pointer to the received message
       */
      void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg);
       /**
       * @brief Laser Callback: function that retrieves the data coming from the laser 
       * @param Pointer to the received message
       */
      void laserCallback(const sensor_msgs::LaserScan::Ptr& msg);
      /**
      * @brief getYaw: function calculates the Yaw angle from the position message that amcl sent 
      * @param msg: passes the amcl position info to the function
      */
      double getYaw(geometry_msgs::PoseWithCovarianceStamped msg);

      /**
      * @brief setNowError: calculates the error between the next and present frame
      */
      void setNowError();

      /**
      * @brief getNext: uses count to set the next goal frame 
      */
      void setNext();


      ///////// SPECIFIC TO DIVIDE AND CONQUER

      /**
      * @brief blockedPath: indicates whether the global path is blocked by an obstacle 
      */

      bool blockedPath();

      /**
      * @brief setPND: sets the Pose Nearness Diagram
      */

      void setPND();

      /**
      * @brief setRND: sets the Robot Nearness Diagram
      * @param msg: passes the amcl position info to the function
      */

      void setRND();

      /**
      * @brief setIntermGoal: sets the intermediate goal, which is needed to start the algorithm
      */

      void setIntermGoal();

      /**
      * @brief createRegions: looks for gaps and uses them to create regions
      */

      void createRegions();

      /**
      * @brief checkSafety: checks if any obstacles are inside the security zone
      */

      bool checkSafety();

      /**
      * @brief regionNavigable: checks if a region is wide enough for the robot to fit through it
      */

      bool regionNavigable();

      /**
      * @brief chooseRegion: searches the regions until it finds the closest navigable one
      */

      bool chooseRegion();

      /**
      * @brief selectSituation: selects the situation of the robot in it's environment 
      * following the divide and conquer paper
      */

      void selectSituation();

      /**
      * @brief calculateAngle: calculates an angle using a certain formula depending on the situation
      */

      double calculateAngle();

      /**
      * @brief distance: calculates the distance between 2 points when we know their angle and distance from the robot
      */

      double distance(int angleA, double distA, int angleB, double distB);

      /**
      * @brief theAlgorithm: follows all the steps of the algorithm iin order
      */

      double theAlgorithm();

      /**
      * @brief goalVisible: checks if the intermediate goal is closer than a certain distance, so 
      * we can continue on the global path
      */

      bool goalVisible();

      /**
      * @brief setRotR: sets the angular speed when we are avoiding an obstacle
      */
   
      void setRotR();

      /**
      * @brief setRotR: sets the linear speed when we are avoiding an obstacle
      */

      void setVelR();

      /**
      * @brief setRotR: sets the error when we are avoiding an obstacle
      */

      void setNowErrorR();


      /**
      * @brief setRotR: chooses as intermediate goal first point of the global path that is finds and is close enough
      */

      int findIntermGoal();

      void pathVisualization();



  };
};

#endif

