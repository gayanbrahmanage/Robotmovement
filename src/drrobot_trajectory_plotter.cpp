//-----------------------------------------Parameters and headers----------------------------------------------------------//

//ROS
#include "ros/ros.h"

//Odometry information
#include <drrobot_jaguar4x4_player/MotorInfoArray.h>

//Message types
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sstream>

#define LOOPRATE         10                 //Looprate

#define WHEELRAD         0.083              //Radius of the wheel
#define WHEELDIST        0.42               //Distance between the two wheels
#define WHEELWIDTH       0.036              //Width of individual wheels

#define LOOKAHEAD        0.05             //Lookahead distance

#define ENCODERCOUNT     376                //Number of encoder steps per full revolution
#define MAXENCODER       32767              //Maximum encoder value
#define ENCROLL          1000               //Encoder rollover threshold (just set it to something high)

//-------------------------------------Node Class--------------------------------------------------------------------------//

class TrajectoryPlotterNode
{
  private: //private structures
    //2D Pose struct
     typedef struct Pose2D
     {
       double x;
       double y;
       double theta;

       //Constructors
       Pose2D():x(0), y(0), theta(0){}
       Pose2D(double a, double b): x(a), y(b), theta(0) {}
       Pose2D(double a, double b, double c): x(a), y(b), theta(c) {}
       Pose2D(const Pose2D & rhs): x(rhs.x), y(rhs.y), theta(rhs.theta){}
       Pose2D(Pose2D && rhs): x(rhs.x), y(rhs.y), theta(rhs.theta){}

       //Overloading assignment operator
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

       //Constructor
       Encoder():left(0), right(0){}
       Encoder(int a, int b): left(a), right(b){}

     }Encoder;

     //2D Twist struct, for differential drive
     typedef struct Twist2D
     {
       double linear;
       double angular;

       //Constructors
       Twist2D():linear(0), angular(0){}
       Twist2D(Twist2D & rhs)
       {
         linear = rhs.linear;
         angular = rhs.angular;
       }
       //Overloading assignment operator
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

  private: //attributes
  //--------------------------------------Initializing stuff-------------------------------------------------------------//
    ros::NodeHandle n;
    ros::Publisher velocity_pub;
    ros::Publisher odom_pub;
    ros::Subscriber path_sub;
    ros::Subscriber MotorInfo_sub;
    bool pathRecieved;
    std::vector<Pose2D> path;
    Pose2D CurPose;                          //Current position
    Pose2D GoalPose;                         //Goal position
    std::vector<Pose2D>::iterator it;        //Iterator
    ros::Time currentTime = ros::Time::now(); //Time
  private: // private methods
  //--------------------------------------Geometry Functions---------------------------------------------------//

    //Returns scalar length from A to B
    double lengthAB(Pose2D & A, Pose2D & B)
    {
      return hypot((B.x-A.x), (B.y-A.y));
    }

    //Returns shortest difference in angle between current pose vs. angle from A to B
    double angleAB(Pose2D & A, Pose2D & B)
    {
        double goalangle = atan2((B.y-A.y),(B.x-A.x));
        if(goalangle<0)
			goalangle += 2*M_PI;
        double onedir = goalangle - A.theta;
        if(onedir >M_PI)
			onedir -= 2*M_PI;
		else if (onedir<-M_PI)
			onedir += 2*M_PI;
        return onedir;
    }
    //--------------------------------------Movement Function-------------------------------------------------------//

    //Movement command publisher from A to B using 'Follow the Carrot' algorithm
    void cmd_velPublisher(Pose2D & A, Pose2D & B)
      {
        if(pathRecieved == true)
        {
            geometry_msgs::Twist msg;
               ROS_INFO("Distance from GoalPose - [%f], GoalPose Coordinates - X: [%f], Y: [%f]", lengthAB(A, B), B.x, B.y);
            if(lengthAB(A, B)>LOOKAHEAD) //If the look-ahead distance hasn't been reached...
            {
              double angle = angleAB(A, B);
              if (fabs(angle)>0)   //If the robot isn't facing the next point, i.e. the angle error isn't 0...
              {
                    ROS_INFO("Angle Difference:[%f]: ",angleAB(A, B));
                if (fabs(angle)>M_PI/4)           //If the angle is greater than 45 degrees, rotate but don't move forward
                {
                   ROS_INFO("Rotating");
                   msg.linear.x = 0;
                   msg.angular.z = angle/3;
                }
                else                                      //Else, rotate and move forward
                {
                 ROS_INFO("Forward and rotating");
                 msg.linear.x = 0.1/2;
                 msg.angular.z = angle;
                }
              }
              else                         //Else, don't rotate
              {
                ROS_INFO("Forward");
                msg.linear.x = 0.1/2;
                msg.angular.z = 0;
              }
            }
        else                         //Else...
        {
          if(it!=path.end())  //If you're not at the end of the path, go the next point
          {
            GoalPose = (*it);
            it++;
          }
          else                //Else, stop
          {
            ROS_INFO("Goal Reached");
            pathRecieved = false;
            path.clear();
            msg.angular.z = 0;
            msg.linear.x = 0;
          }
        }
        velocity_pub.publish(msg);
      }
        ////ROS_INFO("Velocity message sent: [%f]", msg.linear.x);
    }

    //--------------------------------------Encoder Conversion Functions---------------------------------------------------//

      //Eliminates rollover
      int RollRemover(int encoderValue)
      {
        if (abs(encoderValue)>=ENCROLL) //If the encoder value is larger than the rollover threshold...
        {
          if (encoderValue < 0)               //If the encoder value is negative, add it to the max. encoder
            return MAXENCODER+encoderValue;
          else                                //Else, subtract the max. encoder from the encoder value
            return -MAXENCODER+encoderValue;
        }
        else                           //Else, just return it as it is
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
        if (Pose.theta < 0)                           //Keeps rotational pose positive
          Pose.theta += 2*M_PI;

        else if (Pose.theta >= 2*M_PI) //Keeps rotational pose below 360 degrees
           Pose.theta -= 2*M_PI;

        ROS_INFO("CurPose - X:[%f], Y:[%f], Theta:[%f]", Pose.x, Pose.y, Pose.theta);
      }



  //-------------------------------------------Transform functions---------------------------------------------------------//
      //Publishes odometry to topic "odom"
      void odomPublisher(Twist2D && twistVel)
    {
      static tf::TransformBroadcaster odom_broadcaster; //The broadcaster for the odometry->baselink transform
      currentTime = ros::Time::now(); 
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = currentTime;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = CurPose.x;
      odom_trans.transform.translation.y = CurPose.y;
      odom_trans.transform.translation.z = 0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(CurPose.theta);

      odom_broadcaster.sendTransform(odom_trans);

      //publishing the odometry message
      nav_msgs::Odometry odom;
      odom.header.stamp = currentTime;
      odom.header.frame_id = "odom";

      //position
      odom.pose.pose.position.x = CurPose.x;
      odom.pose.pose.position.y = CurPose.y;
      odom.pose.pose.position.z = 0;
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(CurPose.theta);

      //velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = twistVel.linear;;
      odom.twist.twist.linear.y = 0;
      odom.twist.twist.angular.z = twistVel.angular;

      odom_pub.publish(odom);
      AddTwistTo(CurPose, twistVel);
      sendTransform();
    }
    void sendTransform ()
    {
      static tf::TransformBroadcaster broadcaster;
      broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.03,0,0.1)), currentTime, "base_link", "laser"));
      broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.03,0,0.1)), currentTime, "laser", "scan"));

    }

  public:
    TrajectoryPlotterNode(){
      pathRecieved =false;
      velocity_pub = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 10);
      odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
      path_sub = n.subscribe<nav_msgs::Path>("path", 1, boost::bind(&TrajectoryPlotterNode::pathCallback, this, _1));
      MotorInfo_sub = n.subscribe<drrobot_jaguar4x4_player::MotorInfoArray>("drrobot_motor", 1, boost::bind(&TrajectoryPlotterNode::motorCallback, this, _1));
    }
  //--------------------------------------Callback and publisher functions-------------------------------------------------//
    //Callback function from "path"
    void pathCallback(const nav_msgs::Path::ConstPtr& pathmsg)
    {
        static tf::TransformListener listener;
        if(pathRecieved == false)
        {
          tf::StampedTransform transform;
          try
          {
            listener.lookupTransform("/base_link", "/map", ros::Time(0), transform); //get the robot position relative to the map
          }
          catch (tf::TransformException &ex)
          {
            ROS_ERROR("%s",ex.what());
            return;
          }
          for(int i =0; i<path.size();i++)
          {
            Pose2D temp;
            temp.x = pathmsg->poses[i].pose.position.x - transform.getOrigin().x();
            temp.y = pathmsg->poses[i].pose.position.y - transform.getOrigin().y();
            path.push_back(temp);
          }
          it = path.begin();
          if(it!=path.end())
          {
            GoalPose = (*it);
            it++;
          }
          pathRecieved = true;
        }
    }

    //Callback function for odometry from "drrobot_motor"; converts encoder movement to Twist as "twistVel"
    void motorCallback(const drrobot_jaguar4x4_player::MotorInfoArray::ConstPtr& odommsg)
    {
      static Encoder prev(odommsg->motorInfos[0].encoder_pos,odommsg->motorInfos[1].encoder_pos);
      Encoder Delta;

            ////ROS_INFO("Raw wheel movement: [%d] left, [%d] right", odommsg->motorInfos[0].encoder_pos, odommsg->motorInfos[1].encoder_pos);

      Delta.left = RollRemover((odommsg->motorInfos[0].encoder_pos-prev.left));
      Delta.right = -RollRemover((odommsg->motorInfos[1].encoder_pos-prev.right)); //Negative to account for right encoder being reversed

            ////ROS_INFO("Wheel movement: [%d] left, [%d] right", Delta.left, Delta.right);

      prev.left = odommsg->motorInfos[0].encoder_pos;
      prev.right = odommsg->motorInfos[1].encoder_pos;

      //Publish function for odometry; publishes to "odom"
      odomPublisher(EncToTwist(Delta));

      //Publish function for movement command; publishes to "drrobot_cmd_vel"
      cmd_velPublisher(CurPose, GoalPose);
    }
};
//--------------------------------------Main-------------------------------------------------------------------------------//

int main(int argc, char **argv)
{
  //Initializing
  ros::init(argc, argv, "drrobot_trajectory_plotter");
  TrajectoryPlotterNode DrRobotPlotterNode;
  ros::spin();
  while(ros::ok())
  {
  }
  return 0;
}
//By Noam and Thomas
