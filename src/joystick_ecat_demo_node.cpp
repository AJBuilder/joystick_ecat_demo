#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ds4_driver/Status.h"
#include "joystick_ecat_demo/slave1.h"
//#include "akd_ethercat_lib/akd_fixedPDO_1725.h"
//#include "akd_ethercat_lib/akd_fixedPDO_1B20.h"

#include "AKDEcatController.h"
#include "pthread.h"

AKDController master1;
int targetPos, prevTargetPos;
ecat_OpModes mode, prevMode;



pthread_mutex_t callbackLock; 


struct __attribute__((__packed__)) slaveStruct{
    //0x1725
    //rxPDOs
    uint16_t  ctrlWord; 
    uint32_t  targetPos;
    uint32_t  digOutputs;
    uint16_t  tqFdFwd;         
    uint16_t  maxTorque;
    
    //0x1B20
    //txPDOs
    int32_t   posActual;
    int32_t   posFdback2;
    int32_t   velActual;
    uint32_t  digInputs;
    int32_t   followErr;
    uint32_t  latchPos;
    uint16_t  coeStatus;
    int16_t   tqActual;
    uint16_t  latchStatus;
    int16_t   analogInput;
  } s1, s2;

  joystick_ecat_demo::slave1 ext_s1, ext_s2;



void readJoystick(const ds4_driver::Status::ConstPtr& joystick)
{

  pthread_mutex_lock(&callbackLock);
  if(joystick->button_cross) {
    targetPos = 0;
  } else if(joystick->button_circle){
    targetPos = 90;
  } else if(joystick->button_triangle){
    targetPos = 180;
  } else if(joystick->button_square){
    targetPos = 270;
  }

  if(joystick->button_l1)
    mode = profPos;
  else if(joystick->button_r1)
    mode = profVel;

  pthread_mutex_unlock(&callbackLock);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_demo");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("status", 1000, readJoystick);

  ros::Publisher  pub1 = n.advertise<joystick_ecat_demo::slave1>("slave1", 1000);
  ros::Publisher  pub2 = n.advertise<joystick_ecat_demo::slave1>("slave2", 1000);

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

  master1.confSlavePDOs(1, &s1, sizeof(s1), 0x1725, 0,0,0, 0x1B20, 0,0,0);
  master1.confSlavePDOs(2, &s2, sizeof(s2), 0x1725, 0,0,0, 0x1B20, 0,0,0);

  if(master1.confUnits(1, 30, 360) && master1.confUnits(2, 7, 360)){
    ROS_ERROR("Couldn't configure units.");
  }

  if(!master1.confMotionTask(0, 2000, 10000, 10000)){
    ROS_ERROR("Couldn't configure motion task settings.");
  }
  
  if(!master1.setOpMode(0, profPos)){
    ROS_ERROR("\nMode switch failed\n");
    return -5;
  }
  ROS_INFO("\nMode switched!\n");
  mode = profPos;
  prevMode = profPos;

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
  err = master1.Home(0, 0, 0, 6000, 1000, 500, 0, 0);
  if(err != true){
    ROS_INFO("\nFailed to home. %i\n", err);
    return -6;
  }
  ROS_INFO("\nHomed\n");

  spinner.start();

  while(ros::ok()){
    

    pthread_mutex_lock(&callbackLock);
      updateMove = false;
      if(prevTargetPos != targetPos){
        s1.targetPos = targetPos;
        s2.targetPos = targetPos;
        updateMove = true;
        prevTargetPos = targetPos;
      }
      //ROS_INFO("prevTargetPos: %i  targetPos: %i\n", prevTargetPos, targetPos);
      if(prevMode != mode){
        if(!master1.setOpMode(0, profPos)){
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

    for(int i = 0; i < 2 ; i++){
    struct slaveStruct *temp;
    joystick_ecat_demo::slave1 *ext_temp;
    if(i == 0) {
      temp = &s1;
      ext_temp = &ext_s1;
    }
    else {
      temp = &s2;
      ext_temp = &ext_s2; 
    }

    ext_temp->outputs.ctrlWord = temp->ctrlWord;
    ext_temp->outputs.targetPos = temp->targetPos;
    ext_temp->outputs.digOutputs = temp->digOutputs;
    ext_temp->outputs.tqFdFwd = temp->tqFdFwd;
    ext_temp->outputs.maxTorque = temp->maxTorque;

    ext_temp->inputs.posActual = temp->posActual;
    ext_temp->inputs.posFdback2 = temp->posFdback2;
    ext_temp->inputs.velActual = temp->velActual;
    ext_temp->inputs.digInputs = temp->digInputs;
    ext_temp->inputs.followErr = temp->followErr;
    ext_temp->inputs.latchPos = temp->latchPos;
    ext_temp->inputs.coeStatus = temp->coeStatus;
    ext_temp->inputs.tqActual = temp->tqActual;
    ext_temp->inputs.latchStatus = temp->latchStatus;
    ext_temp->inputs.analogInput = temp->analogInput;

  }
  
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