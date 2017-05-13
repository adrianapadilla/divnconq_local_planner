
#include "divnconq_local_planner/divnconq_local_planner.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(divnconq_local_planner, DivnConqPlannerROS, divnconq_local_planner::DivnConqPlannerROS, nav_core::BaseLocalPlanner)

namespace divnconq_local_planner{

	DivnConqPlannerROS::DivnConqPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

	DivnConqPlannerROS::DivnConqPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false)
         {
		// initialize planner
		initialize(name, tf, costmap_ros);
         }

	DivnConqPlannerROS::~DivnConqPlannerROS() {}

	void DivnConqPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
	{

		// check if the plugin is already initialized
		if(!initialized_)
		{
		// copy adress of costmap and Transform Listener (handed over from move_base)
		costmap_ros_ = costmap_ros;
		tf_ = tf;


		// subscribe to topics (to get odometry information, we need to get a handle to the topic in the global namespace)
		ros::NodeHandle gn;
		amcl_sub = gn.subscribe("amcl_pose", 100, &DivnConqPlannerROS::amclCallback, this);
		laser_sub = gn.subscribe("laser", 100, &DivnConqPlannerROS::laserCallback, this);

		// set initialized flag
		initialized_ = true;
		flag = 1;
		flag2 = 1;

		//initializing robot info
		R = 0.5; //robot's radius: I'm using 0.5 
		ds = R*2; //security distance
		dmax =3; //maximum laser range: 20

		numberGaps = 0;
		numberRegions = 0;
		clostRegion = 0;
		side =  0;
		situation = 0;

		cmd.linear.x= 0.0;
		cmd.linear.y= 0.0;
		cmd.angular.z= 0.0;


		// this is only here to make this process visible in the rxlogger right from the start
		ROS_DEBUG("DivnConq Local Planner plugin initialized.");
		}
		else
		{
		ROS_WARN("This planner has already been initialized, doing nothing.");
		}

	}

	bool DivnConqPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
	{

		// check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

		//reset next counter
		count = 1; //20

		//set plan, length and next goal
		plan = orig_global_plan; 
		length = (plan).size();  
		setNext(); 

		// set goal as not reached
		goal_reached_ = false;

		return true;

	}

	bool DivnConqPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{

		// check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

		if(length != 0){ 

			setNowError();

			if(!blockedPath() & flag){ 

				if(distance_ < 0.3){ 


					if(count<(length-1)){

						if((length - 1 - count) < 11){ 
							count = length - 1;
						}else{
							count += 10; 
						}
						setNext();
					}else{
	
						setVelZ();
						goal_reached_ = true;

					}


				}else{

					if(fabs(nError.az) > 25*D2R){ 

						setRot();

					}else{
	
						setVel();

					}

				}

			}else{

				ROS_INFO("found an obstacle");
				flag = 0;// this flag blocks the simple_local_planner when an obstacle is found
				
				if(flag2){
		
					intermGoal = count + 200;
					if(intermGoal >= length) intermGoal = length-1;

					flag2 = 0;
				}

				setIntermGoal();

				if(goalVisible()){ 

					ROS_INFO("goal is visible again!");
							
					count = intermGoal;
					setNext();

					ROS_INFO("intermgoal is: %d", intermGoal);
					
					flag = 1; //once the goal is visible, we keep following the global path
					flag2 = 1; //this flag is meant to only allow computeReboundAngle() to go once
				
					 
				}else{


					ROS_INFO("found an obstacle!");

					yawR = theAlgorithm();

					ROS_INFO("clostRegion: %d", clostRegion);
					ROS_INFO("reg[clostRegion].begin: %d", reg[clostRegion].begin);
					ROS_INFO("reg[clostRegion].end: %d", reg[clostRegion].end);
					ROS_INFO("intermGoal: %d", intermGoal);
					ROS_INFO("intermGoalA: %d", intermGoalA);
					ROS_INFO("intermGoalD: %f", intermGoalD);
					ROS_INFO("angle: %f", (yawR/D2R + 90 - now.az/D2R));
					ROS_INFO("side = %d", side);
					ROS_INFO("situation = %d", situation);


					//if the goal isn't reachable or there is no navigable path, theAlgorithm() = 11:
					if(yawR == 11){
						setVelZ();
						if(intermGoalD < 0.5){
							ROS_INFO("Goal Reached");
							goal_reached_ = true;
							return true; //the robot shouldn't move until a new goal is given

						}else{
							ROS_INFO("Goal not reachable... :(");
							goal_reached_ = true;
							return true; //the robot shouldn't move until a new goal is given
						}
					}



					ROS_INFO("the new angle is: %.4f", yawR/D2R);
					setNowErrorR();

					if(fabs(nError.az) > 5*D2R){ 

						setRotR();

					}else{
						flag2 = 1; 
						setVelR(); 
						
					}


				}
			
			}

			ROS_INFO("Length: [%d]", length);
			ROS_INFO("count = %d", count);

			ROS_INFO("vel -> linear.x = %.4f", cmd.linear.x);
			ROS_INFO("vel -> angular.z=%.4f", cmd.angular.z);


		}

		// set retrieved commands to reference variable
		ROS_DEBUG("Retrieving velocity command: (%f, %f, %f)", cmd.linear.x, cmd.linear.y, cmd.angular.z);
		cmd_vel = cmd;  

		return true;

	}

	bool DivnConqPlannerROS::isGoalReached()
	{
		// check if plugin initialized
		if(!initialized_)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}

		// this info comes from compute velocity commands:
		return goal_reached_;

	}

	void DivnConqPlannerROS::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
	{

		ROS_INFO("Seq: [%d]", msg->header.seq);

		now.x = msg->pose.pose.position.x;
		now.y = msg->pose.pose.position.y;
		now.az = getYaw(*msg); 
		setNowError(); 

	}

	void DivnConqPlannerROS::laserCallback(const sensor_msgs::LaserScan::Ptr& msg)
	{
		ROS_INFO("Seq. laser: [%d]", msg->header.seq);

		laserData = *msg;

		for(int i = 0; i<180; i++){ 

			if(laserData.ranges[i]>3) laserData.ranges[i] = 3;

		}
 
	}


	double DivnConqPlannerROS::getYaw(geometry_msgs::PoseWithCovarianceStamped msg)
	{

		double q[4];
		q[0]= msg.pose.pose.orientation.x;
		q[1]= msg.pose.pose.orientation.y;
		q[2]= msg.pose.pose.orientation.z;
		q[3]= msg.pose.pose.orientation.w;

		double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
		double t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);  

		return std::atan2(t3, t4);

	}

	void DivnConqPlannerROS::setVel()
	{

		ROS_INFO("normal linear speed");
		// the output speed has been adjusted with a P regulator, that depends on how close we are to our current goal
		cmd.linear.x= 0.5*distance_;

		// keeping a small angular speed so that the movement is smooth
		cmd.angular.z= 0.75*(nError.az);

	}

	void DivnConqPlannerROS::setRot()
	{



		// the angular speed has been adjusted with a P regulator, that depends on how close we are to pointing at our current goal
		if (fabs(nError.az) > 50*D2R){

			ROS_INFO("normal angular speed");
			cmd.angular.z=(nError.az)*0.3;
			cmd.linear.x= 0.0;

		}else{

			cmd.angular.z=(nError.az)*0.5;
			cmd.linear.x= 0.05;
		}

	}

	void DivnConqPlannerROS::setVelZ()
	{

		cmd.linear.x= 0;
		cmd.linear.y= 0;
		cmd.angular.z=0;

	}

	void DivnConqPlannerROS::setNext()
	{

		next.x = plan[count].pose.position.x;
		next.y = plan[count].pose.position.y;

	}

	void DivnConqPlannerROS::setNowError()
	{

		double d;

		nError.x = (next.x - now.x);
		nError.y = (next.y - now.y);

		// if these two variables are null, the tangent doesn't exist 
		// plus, the angle error is irrelevant because we have arrived at our destination

		if (nError.y == 0 & nError.x == 0){  
			d = now.az;
		}else{	
			d = std::atan2(nError.y, nError.x);
		}

		distance_ = std::sqrt(nError.x*nError.x +nError.y*nError.y);
		nError.az = d - now.az;

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( nError.az > 180*D2R ) { nError.az -= 360*D2R; }
		if ( nError.az < -180*D2R ) { nError.az += 360*D2R;  }

	}


	void DivnConqPlannerROS::setVelR()
	{
		ROS_INFO("R linear speed");
		cmd.linear.x= 0.3;
		cmd.angular.z= 0.75*(nError.az);


	}

	void DivnConqPlannerROS::setRotR()
	{

		ROS_INFO("R angular speed");
		// the angular speed has been adjusted with a P regulator, that depends on how close we are to pointing at our current goal
		if (fabs(nError.az) > 50*D2R){

			cmd.angular.z=(nError.az)*0.3;

		}else{

			cmd.angular.z=(nError.az)*0.5;
		}
		
		// linear speed is zero while the angle is too big
		cmd.linear.x= 0.1; 

	}

	void DivnConqPlannerROS::setNowErrorR()
	{

		nError.az = yawR - now.az;

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( nError.az > 180*D2R ) { nError.az -= 360*D2R; }
		if ( nError.az < -180*D2R ) { nError.az += 360*D2R;  }

	}


	bool DivnConqPlannerROS::blockedPath() //boundary & angle depend on speed
	{

		double speed = std::sqrt(cmd.linear.x*cmd.linear.x );
		double boundary[180];

		if(speed){

			int min = 90 - 30/(speed*9); //more speed equals a smaller angle amplitud that needs to be examined because 
			int max = 90 + 30/(speed*9); //the distance at which the obstacle will be detected (see below) is bigger


			if( min<90 ) min = 0;
			if( max>180) max = 180;

			//sets a boundary in front of the robot for obstacle detection. Depends on speed of the robot.
			for(int i = min; i<90; i++){
				boundary[i] =(i/60)*6*speed; 
			}
			for(int i = 90; i<max; i++){
				boundary[i] =((180-i)/60)*6*speed;
			}


			// checks if obstacle in middle of path (inside boundary) in order to start the local path planner
			for(int i = min; i<max; i++){

				if(laserData.ranges[i]<=boundary[i]) return true;	
		
			}

		}

		return false;

	}



	//represents nearness of obstacles from robot center
	void DivnConqPlannerROS::setPND()
	{

		for(int i = 0; i<180; i++){
			if(laserData.ranges[i]<=dmax) PND[i] = dmax + 2*R -laserData.ranges[i];
			else PND[i] = 0;
		}

	}

	//represents nearness of obstacles from robot boundary
	void DivnConqPlannerROS::setRND()
	{

		for(int i = 0; i<180; i++){
			if(laserData.ranges[i]<=dmax) RND[i] = dmax + R -laserData.ranges[i];
			else RND[i] = 0;
		}

	}

	void DivnConqPlannerROS::setIntermGoal()
	{

		pos iG;
		double d;

		iG.x = plan[intermGoal].pose.position.x - now.x;
		iG.y = plan[intermGoal].pose.position.y - now.y;

		if (iG.y == 0 & iG.x == 0){  
			d = now.az;
		}else{	
			d = std::atan2(iG.y, iG.x); //angle in global frame (radians)
		}

		intermGoalD = std::sqrt(iG.x*iG.x + iG.y*iG.y);
		intermGoalA = (1/D2R)*(d + 90*D2R - now.az); //angle in robot's frame (degrees)

		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( intermGoalA > 180 )  intermGoalA -= 360;
		if ( intermGoalA < -180 )  intermGoalA += 360;

	}


	bool DivnConqPlannerROS::goalVisible() 
	{		

		int intermGoal_ = findIntermGoal();
		
		if(intermGoal_ == 0){ return false; }

		else{ 

			intermGoal = intermGoal_;

		}

		return true;
	}

	int DivnConqPlannerROS::findIntermGoal() 
	{
		int nextPoint = 0;

		for(int m = length-1; m > intermGoal; m--){  
				
				int allClear = 1;
				double alpha;	
				double dist;	
				pos iError;

				iError.x = plan[m].pose.position.x - now.x;
				iError.y = plan[m].pose.position.y - now.y;

				dist = std::sqrt(iError.x*iError.x +iError.y*iError.y);

				if(dist>0.5){
					allClear = 0;
				}else{

					alpha = std::atan2(iError.y,iError.x); //this is the angle to the goal (straight line)

					alpha = alpha +90*D2R -now.az; // passing to robot's frame

					alpha = alpha/D2R; //passing radians to degrees


					int min = alpha - 15;
					int max = alpha + 15;

					if(min < 0 | max >= 180) allClear = 0;

					for(int i = min; i < max; i++)
					{

						if(laserData.ranges[i] < 0.5) allClear = 0; 

					}

					if (allClear){ nextPoint = m;}

				}
			
		}

		ROS_INFO("intermediate goal is:=%d", nextPoint);

		return nextPoint;
	}

	void DivnConqPlannerROS::createRegions()
	{
		int j = 1;
		int k = 0;
		int m;

	//first we look for gaps: j gaps

		for(int i = 0; i<179; i++){

			if(fabs(PND[i]-PND[i+1]) > 2*R) {
				if(PND[i]>PND[i+1]) {gap[j].desc = 1; gap[j].deg = i+1;} //descending into valley
				else{gap[j].desc = -1; gap[j].deg = i;} //ascending from valley
				j++;
			}
		}

		gap[0].desc = -gap[1].desc;
		gap[0].deg = 0;

		gap[j].desc = -gap[j-1].desc;
		gap[j].deg = 179;

		
		j++;


	//now we look for regions, using the gaps found: k regions

		for(int i = 0; i<(j-1); i++){
			
			if(gap[i].desc == 1) {reg[k].begin = gap[i].deg; reg[k].end = gap[i+1].deg; reg[k].checked = 0; k++;}
			if(gap[i].desc == -1) {
				if(gap[i+1].desc == -1){reg[k].begin = gap[i].deg; reg[k].end = gap[i+1].deg; reg[k].checked = 0; k++;}
			}

		}

		numberGaps = j;
		numberRegions = k;
	}

	bool DivnConqPlannerROS::checkSafety()
	{

		int j = 0;
		double ns = dmax - ds; //ns is the security zone nearness

		for(int i = 0; i<180; i++){

			if(RND[i] >= ns) {inside[i] = 1; j= 1;}
			else inside[i] = 0;

		}

		return j; //if any obstacle is inside the security region, return 1

	}



	bool DivnConqPlannerROS::regionNavigable()
	{

		double dist1; // vertical distance on same angle of one gap
		double dist2; // horizontal distance between the two gaps that form the region

		//side = 0 --> region is on the left of the goal
		//side = 1 --> region is on the right of the goal
		//side = 2 --> goal is inside the region

		reg[clostRegion].checked = 1;
		ROS_INFO("checking: %d", clostRegion);

		if(side == 2){

			dist2 = distance(reg[clostRegion].begin - 1, laserData.ranges[reg[clostRegion].begin - 1], reg[clostRegion].end + 1, laserData.ranges[reg[clostRegion].end + 1]); 

			
		}

		//when side = 0, me must use reg[clostRegion].end (closest to goal)

		if(side == 0){

			dist2 = distance(reg[clostRegion].begin - 1, laserData.ranges[reg[clostRegion].begin - 1], reg[clostRegion].end + 1, laserData.ranges[reg[clostRegion].end + 1]); 

		}

		//when side = 1, me must use reg[clostRegion].begin (closest to goal)

		if(side == 1){

			dist2 = distance(reg[clostRegion].begin - 1, laserData.ranges[reg[clostRegion].begin - 1], reg[clostRegion].end + 1, laserData.ranges[reg[clostRegion].end + 1]); 

		}

		if(dist2 < 2*R){ return false;} //not navigable

		return true;

	}

	double DivnConqPlannerROS::distance(int angleA, double distA, int angleB, double distB){
		
		double a, b, c;

		if (angleA >= 180) {angleA = 179; distA = laserData.ranges[179];}
		if (angleB >= 180) {angleB = 179; distB = laserData.ranges[179];}
		if (angleA <0) {angleA = 0; distA = laserData.ranges[0];}
		if (angleB <0) {angleB = 0; distB = laserData.ranges[0];}

		a = distA * std::cos(D2R*fabs(angleA-angleB));
		b = distA * std::sin(D2R*fabs(angleA-angleB));
		c = fabs(distB - a);

		ROS_INFO("distA = %f", distA);
		ROS_INFO("distB = %f", distB);
		ROS_INFO("angleA = %d", angleA);
		ROS_INFO("angleB = %d", angleB);

		return std::sqrt(c*c + b*b);

	}

	bool DivnConqPlannerROS::chooseRegion()
	{

		clostRegion = 0;
		int m;

		for(int i = 0; i < numberRegions; i++){
			if(!reg[i].checked){
				clostRegion = i;
				break;
			}
		}

		if(((reg[clostRegion].end + reg[clostRegion].begin)/2)<intermGoalA){
			side = 1;
			m = reg[clostRegion].end;
		}else{
			side = 0;
			m = reg[clostRegion].begin;
		}

		//is goal inside a region?: if so, check if that region is navigable

		for(int i = 0; i < numberRegions; i++){
			if((reg[i].begin<intermGoalA) & (reg[i].end>intermGoalA) & (!reg[i].checked)){
				clostRegion = i;
				side = 2; // this means goal is in region
				return regionNavigable();
			}
		}

		//if not in a region, find the region with the closest gap to the goal

		for(int i = 0; i < numberRegions; i++){
			if(!reg[i].checked){
				if(((reg[i].begin + reg[i].end)/2)<intermGoalA){ 
				
					if(fabs(intermGoalA-reg[i].end)<fabs(intermGoalA-m)){ 
						clostRegion = i; side = 1; m = reg[clostRegion].end;
					}

				}else{
				
					if(fabs(intermGoalA-reg[i].begin)<fabs(intermGoalA-m)){
						clostRegion = i; side = 0; m = reg[clostRegion].begin;
					}
				}
			}
		}


		return regionNavigable();
		
	
	}


	void DivnConqPlannerROS::selectSituation()
	{

		//side = 0 --> region is on the right of the goal
		//side = 1 --> region is on the left of the goal
		//side = 2 --> goal is inside the region
		
		// values of situation:

		//1: High Safety Goal in Region -> HSGR
		//2: High Safety Wide Region (>=30º) -> HSWR
		//3: High Safety Narrow Region (<30º) -> HSNR

		//4: Low Safety Goal in Region -> LSGR
		//5: Low Safety 1 Side -> LS1
		//6: Low Safety 2 Sides -> LS2

		bool goalInRegion = 0; 
		bool Safety1 = 0;
		bool Safety2 = 0;

		if(side == 0) Srd = reg[clostRegion].begin;
		if(side == 1) Srd = reg[clostRegion].end;
		if(side == 2) goalInRegion = 1;


		if(!checkSafety()){ // if checkSafety() == 0 : there are NO obstacles inside the security area: HIGH SAFETY
		
			if( fabs(reg[clostRegion].end - reg[clostRegion].begin) > 30 ) situation = 2;
			else situation = 3;

			if(goalInRegion) situation = 1;

		}else{ // if checkSafety() == 1 : there is an obstacle inside the security area: LOW SAFETY

			for(int i = 0; i < Srd ; i++){
				if(inside[i] == 1) Safety1 = 1;
			}

			for(int i = Srd; i < 180 ; i++){
				if(inside[i] == 1) Safety2 = 1;
			}

			if(Safety1 & Safety2)  situation = 6; 
			else situation = 5;

			if(goalInRegion) situation = 4;


		}



	}

	double DivnConqPlannerROS::calculateAngle()
	{

		// values of situation:

		//1: High Safety Goal in Region -> HSGR
		//2: High Safety Wide Region (>=30º) -> HSWR
		//3: High Safety Narrow Region (<30º) -> HSNR

		//4: Low Safety Goal in Region -> LSGR
		//5: Low Safety 1 Side -> LS1
		//6: Low Safety 2 Sides -> LS2

		double angle;
		int m;


		if(situation == 1){

			angle = intermGoalA;

		}

		if (situation == 2){

			double thetaDisc, distDisc, alpha;

			if(side == 1){
				thetaDisc = reg[clostRegion].end;
				distDisc = laserData.ranges[reg[clostRegion].end + 1];
				m = -1;
			}else{
				thetaDisc = reg[clostRegion].begin;
				distDisc = laserData.ranges[reg[clostRegion].begin - 1];
				m = 1;
			}

			alpha = (1/D2R)*std::asin((R+ds)/distDisc);

			angle = thetaDisc + m*alpha;

		}

		if (situation == 3){

			angle = (reg[clostRegion].begin + reg[clostRegion].end)/2;

		}

		if (situation == 4){

			double thetaObs, distObs, beta;
			double n;

			thetaObs = 0;
			distObs = laserData.ranges[0];

			for(int i = 0; i < 180; i++){
				if(inside[i] & (laserData.ranges[i]<distObs)){		
					thetaObs = i;
					distObs = laserData.ranges[i];
				}
			}

			if(thetaObs<intermGoalA) m = 1;
			else m = -1;

			beta = ((ds-distObs)/ds)*fabs((m*180 + thetaObs)- intermGoalA);
			n = distObs/(ds+R);
			angle = (intermGoalA*n + (m*180 + thetaObs)*(1-n));

		}

		if (situation == 5){

			double thetaDisc, distDisc, alpha;
			double thetaObs, distObs, gamma;
			double n;

			thetaObs = 0;
			distObs = laserData.ranges[0];

			for(int i = 0; i < 180; i++){
				if(inside[i] & (laserData.ranges[i]<distObs)){		
					thetaObs = i;
					distObs = laserData.ranges[i];
				}
			}


			if(side == 1){
				thetaDisc = reg[clostRegion].end;
				distDisc = laserData.ranges[reg[clostRegion].end];

			}else{
				thetaDisc = reg[clostRegion].begin;
				distDisc = laserData.ranges[reg[clostRegion].begin];
			}

			if(thetaObs<thetaDisc) m = 1;
			else m = -1;

			n = distObs/(ds+R); //the closer I am to an obstacle, the heavier it's avoidance angle will be

			//weighted arithmetic mean between robot avoidance angle and the discontinuity angle
			angle = (thetaDisc*n + (m*180 + thetaObs)*(1-n));


		}

		if (situation == 6){

			double thetaObsR, distObsR, thetaObsL, distObsL, gammaL, gammaR;
			double thetaDisc, distDisc, alpha;
			double Smed1, Smed2;
			double n, p, q;

			if(side == 1){
				thetaDisc = reg[clostRegion].end;
				distDisc = laserData.ranges[reg[clostRegion].end];
			}else{
				thetaDisc = reg[clostRegion].begin;
				distDisc = laserData.ranges[reg[clostRegion].begin];
			}

			thetaObsL = 0;
			distObsL = laserData.ranges[0];

			thetaObsR = thetaDisc;
			distObsR = distDisc; 

			
			for(int i = 0; i < thetaDisc; i++){
				if(inside[i] == 1 & (laserData.ranges[i]<distObsL)){		
					thetaObsL = i;
					distObsL = laserData.ranges[i];
				}
			}

			for(int i = thetaDisc; i < 180; i++){
				if(inside[i] == 1 & (laserData.ranges[i]<distObsR)){		
					thetaObsR = i;
					distObsR = laserData.ranges[i];
				}
			}


			n = distObsR/(ds+R); //the closer I am to an obstacle, the heavier it's avoidance angle will be

			p = distObsL/(ds+R); //the closer I am to an obstacle, the heavier it's avoidance angle will be

			q = distObsL/(distObsL + distObsR); //the closer I am to obstacle L, the heavier it's avoidance angle will be
			
		angle = q*(thetaDisc*n + (-180 + thetaObsR)*(1-n)) + (1-q)*(thetaDisc*p + (180 + thetaObsL)*(1-p));
			

		}


		// make sure that we chose the smallest angle, so that the robot takes the shortest turn
		if ( angle > 180 )  angle -= 360;
		if ( angle < -180 )  angle += 360;

		return (angle - 90  + now.az/D2R)*D2R;
		
	}	

	double DivnConqPlannerROS::theAlgorithm(){

		bool goalReachable = 1;
      		int regionsChecked = 0;
		bool thereIsRegion = 0;

		setPND();
		setRND();
		createRegions();

		//search for a region that is navigable:
		while(regionsChecked < numberRegions){
			if(chooseRegion()) { thereIsRegion = 1; break; }
			regionsChecked++;
		}


		if(thereIsRegion){

			selectSituation();
			return calculateAngle();

		}

		return 11;

	}
	


}




















