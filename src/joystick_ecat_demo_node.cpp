#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ds4_driver/Status.h"
#include "joystick_ecat_demo/slave1.h"
//#include "akd_ethercat_lib/akd_fixedPDO_1725.h"
//#include "akd_ethercat_lib/akd_fixedPDO_1B20.h"

#include "AKDEcatController.h"

AKDController master1;


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
  ROS_INFO("I heard: [%i]", joystick->button_circle);
  
  if(joystick->button_circle) {
    s1.targetPos = 180;
    s2.targetPos = 180;
  } else{
    s1.targetPos = 0;
    s2.targetPos = 0;
  }

  master1.Update(0, true, 1000);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_demo");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("status", 1000, readJoystick);

  ros::Publisher  pub1 = n.advertise<joystick_ecat_demo::slave1>("slave1", 1000);
  ros::Publisher  pub2 = n.advertise<joystick_ecat_demo::slave1>("slave1", 1000);


  int err;
  ros::Rate r(10);
  char ifname[10] = "ens33";
  

  if(!master1.ecat_Init(ifname)) {
    ROS_FATAL("Couldn't init ecat master");
    return -1;
  }

  master1.confSlavePDOs(1, &s1, sizeof(s1), 0x1725, 0,0,0, 0x1B20, 0,0,0);
  master1.confSlavePDOs(2, &s2, sizeof(s2), 0x1725, 0,0,0, 0x1B20, 0,0,0);

  master1.confUnits(1, 1, 360);
  master1.confUnits(2, 1, 360);

  master1.confMotionTask(0, 2000, 10000, 10000);

  master1.confProfPos(0, true, false);

  if(!master1.ecat_Start()){
    ROS_FATAL("Couldn't start ecat master");
    return -2;
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
    return -3;
  }
  ROS_INFO("\nEnabled!\n");

  s1.maxTorque = 1000;
  s2.maxTorque = 1000;

  if(!master1.setOpMode(0, profPos)){
    ROS_INFO("\nMode switch failed\n");
    return -4;
  }
  ROS_INFO("\nMode switched!\n");

  err = master1.Home(0, 0, 0, 6000, 1000, 500, 0, 0);
  if(err != true){
    ROS_INFO("\nFailed to home. %i\n", err);
    return -5;
  }
  ROS_INFO("\nHomed\n");


  while(ros::ok()){
    
    master1.Update(0, false, 1000);

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


    ros::spinOnce();
    r.sleep();
  }

  

  return 0;
}