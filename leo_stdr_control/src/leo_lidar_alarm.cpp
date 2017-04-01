// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 


const double MIN_SAFE_DISTANCE = 1; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
float ping_dist_in_left_=3.0;
float ping_dist_in_right_=3.0;
float ping_dist_in_slightly_right_=3.0;
float ping_dist_in_slightly_left_=3.0;
float ping_dist_in_add1_=3.0;
float ping_dist_in_add2_=3.0;
float ping_dist_in_add3_=3.0;
float ping_dist_in_add4_=3.0;
float ping_dist_in_add5_=3.0;
float ping_dist_in_add6_=3.0;


int ping_index_1= -1; // NOT real; callback will have to find this
int ping_index_2= -1;
int ping_index_3= -1;  
int ping_index_4= -1;  
int ping_index_5= -1;  
int ping_index_6= -1;  
int ping_index_7= -1; 
int ping_index_8= -1; 
int ping_index_9= -1; 
int ping_index_10= -1; 
int ping_index_11= -1; 

double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_1&&ping_index_2&&ping_index_3&&ping_index_4&&ping_index_5&&ping_index_6&&ping_index_7&&ping_index_8&&ping_index_9&&ping_index_10&&ping_index_11<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_1 = (int) ((0.0 -angle_min_)/angle_increment_);
        ping_index_2 = (int) ((-1.57 -angle_min_)/angle_increment_);
        ping_index_3 = (int) ((1.57 -angle_min_)/angle_increment_);
        ping_index_4 = (int) ((0.78 -angle_min_)/angle_increment_);
        ping_index_5 = (int) ((-0.78-angle_min_)/angle_increment_);
  ping_index_6 = (int) ((0.39-angle_min_)/angle_increment_);
  ping_index_7 = (int) ((-0.39-angle_min_)/angle_increment_);
  ping_index_8 = (int) ((1.175-angle_min_)/angle_increment_);
  ping_index_9 = (int) ((-1.175-angle_min_)/angle_increment_);
  ping_index_10 = (int) ((0.2-angle_min_)/angle_increment_);
  ping_index_11 = (int) ((-0.2-angle_min_)/angle_increment_);



        ROS_INFO("LIDAR setup: ping_index_1 = %d",ping_index_1);
        ROS_INFO("LIDAR setup: ping_index_2 = %d",ping_index_2);
        ROS_INFO("LIDAR setup: ping_index_3 = %d",ping_index_3);
        ROS_INFO("LIDAR setup: ping_index_4 = %d",ping_index_4);
        ROS_INFO("LIDAR setup: ping_index_5 = %d",ping_index_5);
 ROS_INFO("LIDAR setup: ping_index_6 = %d",ping_index_6);
 ROS_INFO("LIDAR setup: ping_index_7 = %d",ping_index_7);
 ROS_INFO("LIDAR setup: ping_index_8 = %d",ping_index_8);
 ROS_INFO("LIDAR setup: ping_index_9 = %d",ping_index_9);
 ROS_INFO("LIDAR setup: ping_index_10 = %d",ping_index_10);
 ROS_INFO("LIDAR setup: ping_index_11 = %d",ping_index_11);
     } 
   ping_dist_in_front_ = laser_scan.ranges[ping_index_1];
   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   ping_dist_in_right_ = laser_scan.ranges[ping_index_2];
   ROS_INFO("ping dist in right = %f",ping_dist_in_right_);
   ping_dist_in_left_ = laser_scan.ranges[ping_index_3];
   ROS_INFO("ping dist in left = %f",ping_dist_in_left_);
   ping_dist_in_slightly_left_ = laser_scan.ranges[ping_index_4];
   ROS_INFO("ping dist in slight left = %f",ping_dist_in_slightly_left_);
   ping_dist_in_slightly_right_ = laser_scan.ranges[ping_index_5];
   ROS_INFO("ping dist in slight right = %f",ping_dist_in_slightly_right_);
ping_dist_in_add1_ = laser_scan.ranges[ping_index_6];
   ROS_INFO("ping dist in front add1 = %f",ping_dist_in_add1_);
ping_dist_in_add2_ = laser_scan.ranges[ping_index_7];
   ROS_INFO("ping dist in front add2 = %f",ping_dist_in_add2_);
ping_dist_in_add3_ = laser_scan.ranges[ping_index_8];
   ROS_INFO("ping dist in front add3 = %f",ping_dist_in_add3_);
ping_dist_in_add4_ = laser_scan.ranges[ping_index_9];
   ROS_INFO("ping dist in front add4 = %f",ping_dist_in_add4_);
ping_dist_in_add5_ = laser_scan.ranges[ping_index_10];
   ROS_INFO("ping dist in front add5 = %f",ping_dist_in_add5_);
ping_dist_in_add6_ = laser_scan.ranges[ping_index_11];
   ROS_INFO("ping dist in front add6 = %f",ping_dist_in_add6_);


   if ((ping_dist_in_front_<MIN_SAFE_DISTANCE)||(ping_dist_in_left_<MIN_SAFE_DISTANCE)||(ping_dist_in_right_<MIN_SAFE_DISTANCE)||(ping_dist_in_slightly_left_<MIN_SAFE_DISTANCE)||(ping_dist_in_slightly_right_<MIN_SAFE_DISTANCE)||(ping_dist_in_add1_<MIN_SAFE_DISTANCE)||(ping_dist_in_add2_<MIN_SAFE_DISTANCE)||(ping_dist_in_add3_<MIN_SAFE_DISTANCE)||(ping_dist_in_add4_<MIN_SAFE_DISTANCE)||(ping_dist_in_add5_<MIN_SAFE_DISTANCE)||(ping_dist_in_add6_<MIN_SAFE_DISTANCE)) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
   }
   else {
       laser_alarm_=false;
   }
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);
   
   lidar_dist_msg.data = ping_dist_in_left_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
   lidar_dist_msg.data = ping_dist_in_right_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
   lidar_dist_msg.data = ping_dist_in_slightly_right_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
   lidar_dist_msg.data = ping_dist_in_slightly_left_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

