#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/callback_queue.h>

#include <dynamic_reconfigure/server.h>
#include <ork_morse_interface/ORKMorseConfig.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "object_recognition_msgs/RecognizedObject.h"
#include "object_recognition_msgs/ObjectType.h"
#include "object_recognition_msgs/ObjectInformation.h"

#include <geometry_msgs/Pose.h>

#include <sstream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>



 //Defines
#define DEBUG 0

#define N_OBJECT 3
#define N_ELEM 10

#define MUG 0
#define MEDIC 1
#define PENHOLDER 2


#define ID_MUG "eab1dbc9af00dc2368eb07a8f9000894"
#define ID_MEDIC "987654321"
#define ID_PENHOLDER "123456789"

 //Globals
geometry_msgs::PoseStamped PoseMugOrk[N_ELEM];	//Variable containing the position of the object detected by ORK callback 
unsigned int nNewPoseMugOrk = 0;

geometry_msgs::PoseStamped PoseMedicOrk[N_ELEM];	//Variable containing the position of the object detected by ORK callback 
unsigned int nNewPoseMedicOrk= 0;

geometry_msgs::PoseStamped PosePenholderOrk[N_ELEM];	//Variable containing the position of the object detected by ORK callback 
unsigned int nNewPosePenholderOrk= 0;


unsigned int timeoutStart = 100; // Number of iteration within the main ros::loop before reseting the Pose of an objet
unsigned int rosFreq = 50; 			// Frequency of the main ros::loop in Hz

std::string idMug = ID_MUG;
std::string idMedic = ID_MEDIC;
std::string idPenholder = ID_PENHOLDER;
 
/*==============================FUNCTIONS======================================*/

/////////////////////////////////////////////////////////////////////////////////
/*-------------------------------isPoseNew-------------------------------------*/
// Description : compare the newPose variable to the current active PoseArray  //
// If newPose is located within a cube of 5cm³ centered by one of the pose of  //
// the array, it returns the indice within the array corresponding to this pose//
// indicating that this pose has already been displayed and needs to be refresh//
//                                                                             // 
// Input:  -> geometry_msgs::PoseStamped& newPose                              //
//         -> geometry_msgs::PoseStamped::ConstPtr& ptrAllPoseArray            //
//         -> unsigned int* ptrTimeoutPoseArray: array with timeout variables  //
//                                                                             //
// OutPut: => Indice 'i' of ptrAllPoseArray corresponding to newPose           //
//                                                                             //
/////////////////////////////////////////////////////////////////////////////////

int isPoseNew(geometry_msgs::PoseStamped& newPose, geometry_msgs::PoseStamped ptrAllPoseArray[N_ELEM],unsigned int ptrTimeoutPoseArray[N_ELEM]){

	for(int i = 0 ; i < N_ELEM ; ++i){

		if(ptrTimeoutPoseArray[i] <= 0)		
			continue;
	
		//If newPose is located inside a cube of 10cm³ centered on ptrAllPoseArray
		if( (newPose.pose.position.x >= ((ptrAllPoseArray[i].pose.position.x) - 0.05)) && (newPose.pose.position.x <= ((ptrAllPoseArray[i].pose.position.x) + 0.05)) &&
		    (newPose.pose.position.y >= ((ptrAllPoseArray[i].pose.position.y) - 0.05)) && (newPose.pose.position.y <= ((ptrAllPoseArray[i].pose.position.y) + 0.05)) &&
		    (newPose.pose.position.z >= ((ptrAllPoseArray[i].pose.position.z) - 0.05)) && (newPose.pose.position.z <= ((ptrAllPoseArray[i].pose.position.z) + 0.05)) ){

			tf::Quaternion newPoseQuaterion;
			tf::Quaternion poseArrayQuaterion;

			tf::quaternionMsgToTF (newPose.pose.orientation,  newPoseQuaterion);
			tf::quaternionMsgToTF (ptrAllPoseArray[i].pose.orientation, poseArrayQuaterion);
		
			tf::Vector3 newPoseVector = newPoseQuaterion.getAxis();
			tf::Vector3 newPoseArrayVector = poseArrayQuaterion.getAxis();
			//Test angle between two Pose (10 degrees max)
			if((newPoseVector.angle(newPoseArrayVector) <= 0.174) && //Angle between two vector inferior to 10 degrees
			   ((newPoseQuaterion.getAngle() - poseArrayQuaterion.getAngle()) <= 0.174) &&
			   ((newPoseQuaterion.getAngle() - poseArrayQuaterion.getAngle()) >= -0.174)
				 ){			
				if(DEBUG){
					ROS_INFO("[ORK_MORSE node][DEBUG] NEW OBJECT DETECTED ALREADY EXIST: %d", i);
					ROS_INFO("[ORK_MORSE node][DEBUG] newPose :");
					ROS_INFO("[ORK_MORSE node][DEBUG] -> position.x = %f",newPose.pose.position.x);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> position.y = %f",newPose.pose.position.y);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> position.z = %f",newPose.pose.position.z);
					ROS_INFO("[ORK_MORSE node][DEBUG] -------------------------------------------");
					ROS_INFO("[ORK_MORSE node][DEBUG] -> orientation.x = %f",newPose.pose.orientation.x);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> orientation.y = %f",newPose.pose.orientation.y);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> orientation.z = %f",newPose.pose.orientation.z);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> orientation.w = %f",newPose.pose.orientation.w);
					ROS_INFO("[ORK_MORSE node][DEBUG] ===========================================");
					ROS_INFO("[ORK_MORSE node][DEBUG] poseArray :");
					ROS_INFO("[ORK_MORSE node][DEBUG] -> position.x = %f",ptrAllPoseArray[i].pose.position.x);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> position.y = %f",ptrAllPoseArray[i].pose.position.y);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> position.z = %f",ptrAllPoseArray[i].pose.position.z);
					ROS_INFO("[ORK_MORSE node][DEBUG] -------------------------------------------");
					ROS_INFO("[ORK_MORSE node][DEBUG] -> orientation.x = %f",ptrAllPoseArray[i].pose.orientation.x);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> orientation.y = %f",ptrAllPoseArray[i].pose.orientation.y);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> orientation.z = %f",ptrAllPoseArray[i].pose.orientation.z);
					ROS_INFO("[ORK_MORSE node][DEBUG] -> orientation.w = %f",ptrAllPoseArray[i].pose.orientation.w);
					ROS_INFO("[ORK_MORSE node][DEBUG] ===========================================");
				}
				return i;
			}
		}
	}
	if(DEBUG)
		ROS_INFO("[ORK_MORSE node][DEBUG] NEW OBJECT POSE DETECTED !");
	return -1;
}

/////////////////////////////////////////////////////////////////////////////////
/*----------------------------mySigintHandler----------------------------------*/
// Description : Shutdown routine                                              //
/////////////////////////////////////////////////////////////////////////////////
void mySigintHandler(int sig)
{
	//Do plenty of cool stuff ...

	// ...

	// ... nevermind, nothing to do (-_-')

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}




/*==============================CALLBACKS======================================*/
/*--------------------------callback_detection---------------------------------*/
void callback_detection(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& objectDetected){
	ROS_INFO("[ORK_MORSE node] NEW OBJECT DETECTED: %d",(unsigned int)objectDetected->objects.size());
	
	geometry_msgs::PoseStamped tmpPoseOrk;

	const char* tmpIdMug = idMug.c_str();
	const char* tmpIdMedic = idMedic.c_str();
	const char* tmpIdPenHolder = idPenholder.c_str();

		//For all object detected by ORK
	for(int i = 0 ; i < objectDetected->objects.size() ; ++i){
		//ROS_INFO("[ORK_MORSE node] OBJECT KEY: %s",objectDetected->objects[i].type.key.c_str());


		if(objectDetected->objects[i].type.key == idMug){
			// MUG ? //	

			ROS_INFO("[ORK_MORSE node] OBJECT KEY: %s = MUG | SCORE = %f",objectDetected->objects[i].type.key.c_str(),objectDetected->objects[i].confidence);
				//Update array PoseMugOrk with new detection
			tmpPoseOrk.header = objectDetected->objects[i].pose.header;
			tmpPoseOrk.pose = objectDetected->objects[i].pose.pose.pose;
			if(nNewPoseMugOrk < N_ELEM){
				PoseMugOrk[nNewPoseMugOrk] = tmpPoseOrk;			
				++nNewPoseMugOrk;
			}
		
		}else 
			if(objectDetected->objects[i].type.key == idMedic){
			// MEDIC ? //	

			ROS_INFO("[ORK_MORSE node] OBJECT KEY: %s = MEDIC | SCORE = %f",objectDetected->objects[i].type.key.c_str(),objectDetected->objects[i].confidence);
				//Update array PoseMedicOrk with new detection
			tmpPoseOrk.header = objectDetected->objects[i].pose.header;
			tmpPoseOrk.pose = objectDetected->objects[i].pose.pose.pose;
			if(nNewPoseMedicOrk < N_ELEM){
				PoseMedicOrk[nNewPoseMedicOrk] = tmpPoseOrk;
				++nNewPoseMedicOrk;
			}			

		}else 
			if(objectDetected->objects[i].type.key == idPenholder){
			// PENHOLDER ? //	

			ROS_INFO("[ORK_MORSE node] OBJECT KEY: %s = PENHOLDER | SCORE = %f",objectDetected->objects[i].type.key.c_str(),objectDetected->objects[i].confidence);
				//Update array PosePenholderOrk with new detection
			tmpPoseOrk.header = objectDetected->objects[i].pose.header;
			tmpPoseOrk.pose = objectDetected->objects[i].pose.pose.pose;
			if(nNewPosePenholderOrk < N_ELEM){
				PosePenholderOrk[nNewPosePenholderOrk] = tmpPoseOrk;
				++nNewPosePenholderOrk;
			}	
		
		}else 
			// DEFAULT //
			ROS_INFO("[ORK_MORSE node] OBJECT KEY: %s = UNKNOW !!",objectDetected->objects[i].type.key.c_str());
		}
}
/*--------------------------callback_detection-END-----------------------------*/



/*--------------------------callback_chgParams---------------------------------*/
void callback_chgParams(ork_morse_interface::ORKMorseConfig &config, uint32_t level) {
  ROS_INFO("[ORK_MORSE node] Reconfigure Request of ID:");

	if(idMug != config.id_mug.c_str()){
  	ROS_INFO("[ORK_MORSE node] idMug changed from %s to: %s",idMug.c_str() , config.id_mug.c_str());
		idMug = config.id_mug.c_str();
	}

	if(idMedic != config.id_medic.c_str()){
  	ROS_INFO("[ORK_MORSE node] idMedic changed from %s to: %s",idMedic.c_str(), config.id_medic.c_str());
		idMedic = config.id_medic.c_str();
	}

	if(idPenholder != config.id_penholder.c_str()){
  	ROS_INFO("[ORK_MORSE node] idPenholder changed from %s to: %s",idPenholder.c_str(), config.id_penholder.c_str());
		idPenholder = config.id_penholder.c_str();
	}

	if(timeoutStart != config.start_timeout){
  	ROS_INFO("[ORK_MORSE node] timeoutStart changed from %d to: %d",timeoutStart, config.start_timeout);
		timeoutStart = config.start_timeout;
	}
}
/*--------------------------callback_chgParams-END-----------------------------*/

/*===================================MAIN======================================*/
int main(int argc, char *argv[]){

	//Variables
	geometry_msgs::Pose initPose[N_OBJECT][N_ELEM];			//Variable containing the initial position of objects
	geometry_msgs::PoseStamped Pose[N_OBJECT][N_ELEM];	//Variable containing the position of the object detected by ORK 
	unsigned int PoseTimeout[N_OBJECT][N_ELEM];					//Timeout variable for each objects
	bool PoseIsInit[N_OBJECT][N_ELEM];									//Variable indicating if the object has been initialized
	//Example: Use of defines : initPose[MUG][1] = initial position of mug_1 ...
	geometry_msgs::PoseStamped tfPose;

	unsigned int* nNewPoseOrk= NULL;
	geometry_msgs::PoseStamped* PoseOrk = NULL; 
	ros::Publisher* MORSEPub = NULL;



	ROS_INFO("[ORK_MORSE node] Initialization of node : ORK_MORSE_node");
	ros::init(argc, argv, "ORK_MORSE_node");

	ros::NodeHandle r;
		
	//Shutdown signal Ctrl-C
	signal(SIGINT, mySigintHandler);

	//Dynamic reconfigure parameters (to make id_params dynamic with rqt_reconfigure ...)
	dynamic_reconfigure::Server<ork_morse_interface::ORKMorseConfig> server;
  dynamic_reconfigure::Server<ork_morse_interface::ORKMorseConfig>::CallbackType f;

  f = boost::bind(&callback_chgParams, _1, _2);
  server.setCallback(f);

	//Subscribe to Linemod and/or Tabletop topics from ORK nodes
		//LINEMOD

	ros::Subscriber ORKLinemod = r.subscribe("/linemod_recognized_object_array", 50, callback_detection);
	if(!ORKLinemod)
		ROS_INFO("[ORK_MORSE node] Subscribing to topic /linemod_recognized_object_array: FAILED");
	else
		ROS_INFO("[ORK_MORSE node] Subscribing to topic /linemod_recognized_object_array: OK");

		//TABLETOP
	ros::Subscriber ORKTabletop = r.subscribe("/tabletop_recognized_object_array", 50, callback_detection);
	if(!ORKTabletop)
		ROS_INFO("[ORK_MORSE node] Subscribing to topic /tabletop_recognized_object_array: FAILED");
	else
		ROS_INFO("[ORK_MORSE node] Subscribing to topic /tabletop_recognized_object_array: OK");

	if(!ORKTabletop && !ORKLinemod){
		ROS_INFO("[ORK_MORSE node] All subscribing failed ! Start ORK nodes before starting this one.");
		ROS_INFO("[ORK_MORSE node] ShutDown of node : ORK_MORSE_node");
		return -1;
	}

	//Publish to Morses topics for all objects
	ros::Publisher MORSEPubMug[N_ELEM];
	ros::Publisher MORSEPubMedic[N_ELEM];
	ros::Publisher MORSEPubPenholder[N_ELEM];

		//MUG
	for(int i = 0 ; i < N_ELEM ; ++i){

		std::stringstream topicName;
		topicName << "/morse_TP_mug_" << i+1;	
		std::string tmpTopicName = topicName.str();

		MORSEPubMug[i] = r.advertise<geometry_msgs::Pose>(tmpTopicName,100);	

		if(MORSEPubMug[i])	
			ROS_INFO("[ORK_MORSE node] Publishing to %s: OK", tmpTopicName.c_str());
		else
			ROS_INFO("[ORK_MORSE node] Publishing to %s: FAILED", tmpTopicName.c_str());			
	}

		//MEDIC
	for(int i = 0 ; i < N_ELEM ; ++i){

		std::stringstream topicName;
		topicName << "/morse_TP_medic_" << i+1;		
		std::string tmpTopicName = topicName.str();

		MORSEPubMedic[i] = r.advertise<geometry_msgs::Pose>(tmpTopicName,100);	

		if(MORSEPubMug[i])	
			ROS_INFO("[ORK_MORSE node] Publishing to %s: OK", tmpTopicName.c_str());
		else
			ROS_INFO("[ORK_MORSE node] Publishing to %s: FAILED", tmpTopicName.c_str());
	}

		//PENHOLDER
	for(int i = 0 ; i < N_ELEM ; ++i){

		std::stringstream topicName;
		topicName << "/morse_TP_penholder_" << i+1;		
		std::string tmpTopicName = topicName.str();

		MORSEPubPenholder[i] = r.advertise<geometry_msgs::Pose>(tmpTopicName,100);	

		if(MORSEPubPenholder[i])	
			ROS_INFO("[ORK_MORSE node] Publishing to %s: OK", tmpTopicName.c_str());
		else
			ROS_INFO("[ORK_MORSE node] Publishing to %s: FAILED", tmpTopicName.c_str());
	}

	//Create initPose msgs
	for(int i = 0 ; i < N_OBJECT ; ++i){
		for(int j = 0 ; j < N_ELEM ; ++j){
			initPose[i][j].position.x = j;
			initPose[i][j].position.y = i;			
			initPose[i][j].position.z = -1.0;

			initPose[i][j].orientation.x = initPose[i][j].orientation.y = initPose[i][j].orientation.z = 0.0;
			initPose[i][j].orientation.w = 1.0;			

			PoseTimeout[i][j] = 0;
		}		 			
	}


	ROS_INFO("[ORK_MORSE node] SENDING INIT POSE: ...");

		//Wait for morse to initialize connections
	ros::Duration(2.0).sleep();

	for(int j = 0 ; j < N_ELEM ; ++j){
		MORSEPubMug[j].publish(initPose[MUG][j]);
		PoseIsInit[MUG][j] = true;
		MORSEPubMedic[j].publish(initPose[MEDIC][j]);
		PoseIsInit[MEDIC][j] = true;
		MORSEPubPenholder[j].publish(initPose[PENHOLDER][j]);
		PoseIsInit[PENHOLDER][j] = true;
	}	
	ROS_INFO("[ORK_MORSE node] SENDING INIT POSE: DONE");

	tf::TransformListener listener(ros::Duration(60.0));
	ros::Rate loop_rate(rosFreq);
	std::string errMsg;

	//=> MAIN ROS LOOP <=//

	while(ros::ok()){
		
		if(!listener.canTransform("/odom","/head_mount_kinect_rgb_optical_frame",ros::Time(0),&errMsg)){
			ROS_ERROR("[ORK_MORSE node] Unable to transform frame");
			ROS_ERROR("[ORK_MORSE node] %s",errMsg.c_str());
			ros::Duration(1.0).sleep();
			continue;
		}

		//Update detection based on ROS ORK CALLBACKS
		for(int i = 0 ; i < N_OBJECT ; ++i){
			switch(i){
				case MUG:
					nNewPoseOrk = &nNewPoseMugOrk;
					PoseOrk = PoseMugOrk; 
				break;

				case MEDIC:
					nNewPoseOrk = &nNewPoseMedicOrk;
					PoseOrk = PoseMedicOrk; 
				break;

				case PENHOLDER:
					nNewPoseOrk = &nNewPosePenholderOrk;
					PoseOrk = PosePenholderOrk; 
				break;

				default: // MUG
					nNewPoseOrk = &nNewPoseMugOrk;
					PoseOrk = PoseMugOrk; 
			}//switch(OBJECTS)
			
			if(*(nNewPoseOrk) > 0){
					//For all new detection
				for(int k = 0 ; k < (*nNewPoseOrk) ; ++k){

					//ATTENTION TRICHE INCOMING !!!!! Tmp change, because \camera_rgb_optical_frame isn't connected to the virtual tf tree right now ...
					PoseOrk[k].header.frame_id = "head_mount_kinect_rgb_optical_frame";
							
						//Transform the position of the detection from the camera to the absolute coordinate of morse			
					try{
						listener.transformPose("/odom", *(PoseOrk + k), tfPose);
					}catch(tf::TransformException &ex){
						ROS_ERROR("[ORK_MORSE node] %s",ex.what());
						//ros::Duration(1.0).sleep();
						continue;						
					}



						//Check if the detection already exists within MORSE /map frame
					int indice = isPoseNew(tfPose,
					                       *(Pose + i),
					                       *(PoseTimeout + i)
					                       );
						
					if(indice >= 0){
						//If the detection already exists : reset timeout array only
						PoseTimeout[i][indice] = timeoutStart;
					}else{
						//If the detection is new : set new timeout array and insert newPose in PoseArray
							//Search for free space within PoseArray
						for(int j = 0 ; j < N_ELEM ; ++j){

							if(PoseTimeout[i][j] <= 0){
									//Free Space found
								PoseTimeout[i][j] = timeoutStart;
								Pose[i][j] = tfPose;
								break;

							}else{
									//Free Space not found
								if(j == (N_ELEM-1)){
									//TODO :Search the older one and replace it ?
									ROS_INFO("[ORK_MORSE node] NEW OBJECT INSERT FAILED: NOT ENOUGH SPACE WITHIN PoseTimeout/PoseArray");
								}
							}								
						}						
					}
				}//for all nNewPose
				(*nNewPoseOrk) = 0;
			}//if nNewPoseOrk > 0
		}//for all OBJECTS

		//Publish detection if timeout > 1 | Init Pose if timeout == 1 | do nothing else
		for(int i = 0 ; i < N_OBJECT ; ++i){
			switch(i){
				case MUG:
					MORSEPub = MORSEPubMug;
				break;

				case MEDIC:
					MORSEPub = MORSEPubMedic;
				break;

				case PENHOLDER:
					MORSEPub = MORSEPubPenholder;
				break;

				default: // MUG 
					MORSEPub = MORSEPubMug;
			}//switch(OBJECTS)

			for(int j = 0 ; j < N_ELEM ; ++j){
				if(PoseTimeout[i][j] > 1){
					//ROS_INFO("PUBLISH");				
					MORSEPub[j].publish(Pose[i][j].pose);
					PoseIsInit[i][j] = false;
				}else{
					if((PoseTimeout[i][j] <= 1) && (!PoseIsInit[i][j])){
						//ROS_INFO("PUBLISH INIT");
						MORSEPub[j].publish(initPose[i][j]);
						PoseIsInit[i][j] = true;
						PoseTimeout[i][j] = 0;
					}					
				}				
			}
		}

		//Decrease timeout
		for(int i = 0 ; i < N_OBJECT ; ++i){
			for(int j = 0 ; j < N_ELEM ; ++j){
				if(PoseTimeout[i][j] > 0)
					PoseTimeout[i][j]--;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
/*		if(DEBUG)
			ROS_INFO("[ORK_MORSE node][DEBUG] LOOP !");*/

	};//while ros::ok

	ROS_INFO("[ORK_MORSE_node] Shutdown node ORK_MORSE_node received");
	ROS_INFO("[ORK_MORSE_node] Reseting all pose on Morse");
	//shutdown : initPose
	for(int i = 0 ; i < N_OBJECT ; ++i){
		switch(i){
			case MUG:
				MORSEPub = MORSEPubMug;
			break;
			case MEDIC:
				MORSEPub = MORSEPubMedic;
			break;
			case PENHOLDER:
				MORSEPub = MORSEPubPenholder;
			break;
			default: // MUG 
				MORSEPub = MORSEPubMug;
		}//switch(OBJECTS)
		for(int j = 0 ; j < N_ELEM ; ++j){
			MORSEPub[j].publish(initPose[i][j]);		
		}
	}	
}

