#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ds4_driver/Status.h"
#include "joystick_ecat_demo/flexSlave1.h"
//#include "akd_ethercat_lib/akd_fixedPDO_1725.h"
//#include "akd_ethercat_lib/akd_fixedPDO_1B20.h"

#include "AKDEcatController.h"
#include "pthread.h"

AKDController master1;
int targetPos1, targetPos2, targetVel1, targetVel2, prevTargetPos1, prevTargetPos2;
AKDController::ecat_OpModes mode, prevMode;



pthread_mutex_t callbackLock; 


  struct __attribute__((__packed__)) slaveStruct{
    //rxPDOs
    uint16_t  ctrlWord;     // 0x6040, 0
    uint16_t  maxTorque;    // 0x6072, 0
    int32_t  targetPos;    // 0x607A, 0
    int32_t  targetVel;    // 0x60FF, 0
    
    //txPDOs
    uint16_t  coeStatus;    // 0x6041, 0
    int32_t  actualPos;    // 0x6064, 0
    int32_t  actualVel;    // 0x606C, 0

  } s1, s2;

  AKDController::ecat_pdoEntry_t tx[3] = {{0x6041,0}, {0x6064,0}, {0x606C,0}};
  AKDController::ecat_pdoEntry_t rx[4] = {{0x6040,0}, {0x6072,0},{0x607A,0},{0x60FF,0}};

  joystick_ecat_demo::flexSlave1 ext_s1, ext_s2;



void readJoystick(const ds4_driver::Status::ConstPtr& joystick)
{

  pthread_mutex_lock(&callbackLock);
  if(joystick->button_cross) {
    targetPos1 = 0;
  } else if(joystick->button_circle){
    targetPos1 = 90;
  } else if(joystick->button_triangle){
    targetPos1 = 180;
  } else if(joystick->button_square){
    targetPos1 = 270;
  }

  targetVel1 = joystick->axis_right_y * 600;

  if(joystick->button_dpad_down) {
    targetPos2 = 0;
  } else if(joystick->button_dpad_right){
    targetPos2 = 90;
  } else if(joystick->button_dpad_up){
    targetPos2 = 180;
  } else if(joystick->button_dpad_left){
    targetPos2 = 270;
  }

  targetVel2 = joystick->axis_left_y * 500;

  if(joystick->button_l1)
    mode = AKDController::ecat_OpModes::profPos;
  else if(joystick->button_r1)
    mode = AKDController::ecat_OpModes::profVel;

  pthread_mutex_unlock(&callbackLock);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_demo");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("status", 1000, readJoystick);

  ros::Publisher  pub1 = n.advertise<joystick_ecat_demo::flexSlave1>("slave1", 1000);
  ros::Publisher  pub2 = n.advertise<joystick_ecat_demo::flexSlave1>("slave2", 1000);

  ros::AsyncSpinner spinner(1);

  int err;
  ros::Rate r(100);
  std::string ifname;
  bool updateMove;

  pthread_mutex_init(&callbackLock, NULL);


  if(!n.getParam("ifname", ifname)){
    ROS_FATAL("No interface parameter specified.");
    return -1;
  }
  

  if(!master1.ecat_Init(ifname.c_str())) {
    ROS_FATAL("Couldn't init ecat master on %s", ifname.c_str());
    return -2;
  }

  master1.confSlavePDOs(1, &s1, sizeof(s1), 0x1600, 0x1601, 0x1602, 0x1603, 0x1a00, 0x1a01, 0x1a02, 0x1a03);
  master1.confSlavePDOs(2, &s2, sizeof(s2), 0x1600, 0x1601, 0x1602, 0x1603, 0x1a00, 0x1a01, 0x1a02, 0x1a03);

  if(!master1.confSlaveEntries(0, rx, sizeof(rx)/sizeof(AKDController::ecat_pdoEntry_t), tx, sizeof(tx)/sizeof(AKDController::ecat_pdoEntry_t))){
      printf("Couldn't confSlaveEntries.\n");
      return -1;
  }

  if(!master1.confUnits(1, 30, 360)){
    ROS_ERROR("Couldn't configure slave1 units.");
  }

  if(!master1.confUnits(2, 7, 360)){
    ROS_ERROR("Couldn't configure slave2 units.");
  }

  if(!master1.confMotionTask(0, 500, 6000, 6000)){
    ROS_ERROR("Couldn't configure motion task settings.");
  }
  
  if(!master1.setOpMode(0, AKDController::ecat_OpModes::profPos)){
    ROS_ERROR("\nMode switch failed\n");
    return -5;
  }
  ROS_INFO("\nMode switched!\n");
  mode = AKDController::ecat_OpModes::profPos;
  prevMode = AKDController::ecat_OpModes::profPos;

  if(!master1.confProfPos(0, true, false)){
    ROS_ERROR("Couldn't configure profile position mode settings.");
  }


  if(!master1.ecat_Start()){
    ROS_FATAL("Couldn't start ecat master");
    return -3;
  }

  ROS_INFO("\nReading fault(s)...\n");
  if(master1.readFault(0)){
    ROS_INFO("\nWaiting for fault(s) to clear!\n");
    do{
      master1.clearFault(0,false);

    }while(master1.readFault(0));
    ROS_INFO("\nFault(s) cleared!\n");
  }

  ROS_INFO("\nEnabling...\n");
  if(!master1.Enable()){
    ROS_INFO("\nEnable failed\n");
    return -4;
  }
  ROS_INFO("\nEnabled!\n");

  s1.maxTorque = 1000;
  s2.maxTorque = 1000;

  

  ROS_INFO("\nHoming!\n");
  err = master1.Home(0, 0, 0, 500, 1000, 500, 0, 0);
  if(err != true){
    ROS_INFO("\nFailed to home. %i\n", err);
    return -6;
  }
  ROS_INFO("\nHomed\n");

  spinner.start();

  while(ros::ok()){
    

    pthread_mutex_lock(&callbackLock);
      updateMove = false;
      if(prevTargetPos1 != targetPos1){
        s1.targetPos = targetPos1;
        updateMove = true;
        prevTargetPos1 = targetPos1;
      }
      s1.targetVel = targetVel1;

      if(prevTargetPos2 != targetPos2){
        s2.targetPos = targetPos2;
        updateMove = true;
        prevTargetPos2 = targetPos2;
      }
      s2.targetVel = targetVel2;
      //ROS_INFO("prevTargetPos: %i  targetPos: %i\n", prevTargetPos, targetPos);
      if(prevMode != mode){
        if(!master1.setOpMode(0, mode)){
          ROS_ERROR("\nMode switch failed\n");
          mode = prevMode;
        }
        ROS_INFO("\nMode switched!\n");
        prevMode = mode;
      }
    pthread_mutex_unlock(&callbackLock);
    
    if(master1.Update(0, updateMove, 5000) != 0)
    {
      ROS_ERROR("Update Failed");
    }

    ext_s1.targetPos = s1.targetPos;
    ext_s1.targetVel = s1.targetVel;
    ext_s1.actualPos = s1.actualPos;
    ext_s1.actualVel = s1.actualVel;
    ext_s1.coeStatus = s1.coeStatus;
    ext_s1.ctrlWord  = s1.ctrlWord;
    ext_s1.maxTorque = s1.maxTorque;

    ext_s2.targetPos = s2.targetPos;
    ext_s2.targetVel = s2.targetVel;
    ext_s2.actualPos = s2.actualPos;
    ext_s2.actualVel = s2.actualVel;
    ext_s2.coeStatus = s2.coeStatus;
    ext_s2.ctrlWord  = s2.ctrlWord;
    ext_s2.maxTorque = s2.maxTorque;

    pub1.publish(ext_s1);
    pub2.publish(ext_s2);


    if(master1.readFault(0)){
      ROS_INFO("\nWaiting for fault(s) to clear! Status1: 0x%4x Status2: 0x%4x\n", s1.coeStatus, s2.coeStatus);
    do{
      master1.clearFault(0,false);

    }while(master1.readFault(0));
      ROS_INFO("\nFault(s) cleared!\n");
    }

    r.sleep();
  }

  pthread_mutex_destroy(&callbackLock);

  ros::waitForShutdown();

  return 0;
}