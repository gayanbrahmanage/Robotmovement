//-----------------------------------------Parameters and headers----------------------------------------------------------//

//ROS
#include "ros/ros.h"

//Odometry information
#include <drrobot_jaguar4x4_player/MotorInfoArray.h>

//Message types
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sstream>

#define LOOPRATE         10                 //Looprate

#define WHEELRAD         0.0835             //Radius of the wheel
#define WHEELDIST        0.305              //Distance between the two wheels

#define ENCODERCOUNT     400                //Number of encoder steps per full revolution
#define MAXENCODER       32767              //Maximum encoder value 
#define ENCROLL          1000               //Encoder rollover threshold (just set it to something high)

//-------------------------------------Node Class--------------------------------------------------------------------------//

class TrajectoryPlotterNode
{
  private:
  //--------------------------------------Initializing stuff-------------------------------------------------------------//
    ros::NodeHandle n;

    ros::Publisher velocity_pub;
    ros::Publisher odom_pub;
    ros::Subscriber path_sub;
    ros::Subscriber MotorInfo_sub;

   //2D Pose struct
    typedef struct Pose2D
    {
      double x;
      double y;
      double theta;

      Pose2D():x(0), y(0), theta(0){}
      Pose2D(double a, double b): x(a), y(b), theta(0) {}
      Pose2D(double a, double b, double c): x(a), y(b), theta(c) {}
      Pose2D(Pose2D & rhs): x(rhs.x), y(rhs.y), theta(rhs.theta){}
      Pose2D(Pose2D && rhs): x(rhs.x), y(rhs.y), theta(rhs.theta){}
      Pose2D & operator=(Pose2D & rhs){
        if(this != &rhs)
        {
          x = rhs.x;
          y = rhs.y;
          theta = rhs.theta;
        }
        return *this;
      }
      Pose2D & operator=(Pose2D && rhs){
        if(this != &rhs)
        {
          x = rhs.x;
          y = rhs.y;
          theta = rhs.theta;
        }
        return *this;
      }
    }Pose2D;

    //Encoder values
    typedef struct Encoder
    {
      int left;
      int right;

      Encoder():left(0), right(0){}
    
    }Encoder;

    //2D Twist struct, for differential drive
    typedef struct Twist2D
    {
      double linear;
      double angular;

      Twist2D():linear(0), angular(0){}
      Twist2D(Twist2D & rhs)
      {
        linear = rhs.linear;
        angular = rhs.angular;
      }
       Twist2D & operator=(Twist2D & rhs)
       {
        if(this != &rhs)
        {
          linear = rhs.linear;
          angular = rhs.angular;
        }
        return *this;
      }
    }Twist2D;

    Pose2D poseCur;    //Current position
    Pose2D Goal; //Goal position
    std::vector<Pose2D> path;
    std::vector<Pose2D>::iterator it;
    Pose2D ManGoal; //Manual goal position

    ros::Time currentTime = ros::Time::now(); //Time
  //--------------------------------------Movement Math Functions---------------------------------------------------//

    //Returns scalar length from A to B
    double lengthAB(Pose2D & A, Pose2D & B)
    {
      return hypot((B.x-A.x), (B.y-A.y));
    }

    //Returns difference in angle between current pose vs. angle from A to B
    double angleAB(Pose2D & A, Pose2D & B)
    {
      return atan2((B.y-A.y),(B.x-A.x))-A.theta;
    }


  //--------------------------------------Movement Functions-------------------------------------------------------//
 
  //Movement
  void Mov()
    {
      geometry_msgs::Twist msg;
      ROS_INFO("Current Pose: [%f], [%f]", poseCur.x, poseCur.y);
      ROS_INFO("Distance from Goal: [%f], point: [%f]", lengthAB(Goal,poseCur),Goal.x);
      if(lengthAB(Goal, poseCur)>0.5)
      {
        msg.linear.x = 0.1/4; //Linear velocity being sent 
        if (fabs(angleAB(poseCur, Goal))>0)
        {
        ROS_INFO("Angle Difference:[%f]: ",angleAB(poseCur, Goal));
        msg.angular.z = angleAB(poseCur, Goal)/4;  //Angular velocity being sent
        }
        else 
        {
          ROS_INFO ("else");
          msg.angular.z = 0;
        }
      }
      else
      {
        if(it!=path.end())
        {
          Goal = (*it);
          it++;
        }
        else{
          msg.linear.x = 0;
          msg.angular.z = 0;
        }
      }
      velocity_pub.publish(msg);
      ////ROS_INFO("Velocity message sent: [%f]", msg.linear.x);
    }

  void ManMov()
    {

    }

  //Regular movement

  //--------------------------------------Encoder Conversion Functions---------------------------------------------------//

    //Eliminates rollover
    int RollRemover(int encoderValue)
    {
      if (abs(encoderValue)>=ENCROLL)
      {
        if (encoderValue < 0) 
          return MAXENCODER+encoderValue;
        else 
          return -MAXENCODER+encoderValue;
      }
      else 
        return encoderValue;
    }

    //Returns conversion from encoder odometry to values in meters
    double EncToMeters(int EncoderVal)
    {
      return EncoderVal*2*M_PI*(WHEELRAD)/(ENCODERCOUNT);
    }

    //Takes encoder movement and converts it to forward + angular rotation (i.e. 2D twist)
    Twist2D EncToTwist(Encoder & dEncoder)
    {
      Twist2D vel;

      vel.linear = (EncToMeters(dEncoder.right)+EncToMeters(dEncoder.left))/2;
      vel.angular = (EncToMeters(dEncoder.right)-EncToMeters(dEncoder.left))/WHEELDIST;

         ////ROS_INFO("Twist Velocity: [%f] cm/s, [%f] degrees", vel.linear*100, vel.angular*(180/M_PI));
      return vel;
    }

    //Adds Twist to a current pose 
    void AddTwistTo(Pose2D & Pose, Twist2D & Twist)
    {
      Pose.x += Twist.linear*cos(Pose.theta);
      Pose.y += Twist.linear*sin(Pose.theta);
      Pose.theta += Twist.angular;
    }



//-------------------------------------------Transform functions---------------------------------------------------------//
    //Publishes odometry to topic "odom"
    void odomPublisher(Twist2D && twistVel) 
  {
    static tf::TransformBroadcaster odom_broadcaster; //The broadcaster for the odometry->baselink transform
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = poseCur.x;
    odom_trans.transform.translation.y = poseCur.y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(poseCur.theta);

    odom_broadcaster.sendTransform(odom_trans);

    //publishing the odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";
    
    //position
    odom.pose.pose.position.x = poseCur.x;
    odom.pose.pose.position.y = poseCur.y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(poseCur.theta);

    //velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = twistVel.linear;;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = twistVel.angular;

    odom_pub.publish(odom);	
    AddTwistTo(poseCur, twistVel);
  }

  public:
    TrajectoryPlotterNode(){
      path.push_back(Pose2D(1,0,0));
      path.push_back(Pose2D(2,0,0));
      it = path.begin();
      if(it!=path.end())
      {
        Goal = (*it); 
        it++;
      }
      velocity_pub = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 10);
      odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
      path_sub = n.subscribe<nav_msgs::Path>("path", 1, boost::bind(&TrajectoryPlotterNode::pathCallback, this, _1));
      MotorInfo_sub = n.subscribe<drrobot_jaguar4x4_player::MotorInfoArray>("drrobot_motor", 1, boost::bind(&TrajectoryPlotterNode::odomCallback, this, _1));
    }
  //--------------------------------------Callback and publisher functions-------------------------------------------------//
    //Callback function from "path"
    void pathCallback(const nav_msgs::Path::ConstPtr& pathmsg)
    {
      ROS_INFO("hello");
    }

    //Callback function for odometry from "drrobot_motor"; converts encoder movement to Twist as "twistVel"
    void odomCallback(const drrobot_jaguar4x4_player::MotorInfoArray::ConstPtr& odommsg)
    {
      static Encoder prev;
      Encoder Delta;
      
            ////ROS_INFO("Raw wheel movement: [%d] left, [%d] right", odommsg->motorInfos[0].encoder_pos, odommsg->motorInfos[1].encoder_pos);

      Delta.left = RollRemover((odommsg->motorInfos[0].encoder_pos-prev.left));
      Delta.right = -RollRemover((odommsg->motorInfos[1].encoder_pos-prev.right)); //Negative to account for right encoder being reversed

            ////ROS_INFO("Wheel movement: [%d] left, [%d] right", Delta.left, Delta.right);

      prev.left = odommsg->motorInfos[0].encoder_pos;
      prev.right = odommsg->motorInfos[1].encoder_pos;

      odomPublisher(EncToTwist(Delta));
      Mov();
    }
};
//--------------------------------------Main-------------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  //Initializing
  ros::init(argc, argv, "drrobot_trajectory_plotter");
  TrajectoryPlotterNode DrRobotPlotterNode;
  ros::spin();
  return 0;
}
//By Noam and Thomas