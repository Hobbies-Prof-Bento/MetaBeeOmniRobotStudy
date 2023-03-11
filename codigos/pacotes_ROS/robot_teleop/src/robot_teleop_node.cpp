#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robot_teleop/kbhit.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv,"robot_teleop_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

    geometry_msgs::Twist move;

    float vel_linear = 0.2;
    float vel_angular = 0.26;

    ROS_INFO("Aperte W/X/A/D = frente/tras/esquerda/direita");
    ROS_INFO("Aperte H/J = antihorario/horario");
    ROS_INFO("Aperte S = parar");
    ROS_INFO("Aperte 1/2 = + vel_linear/- vel_linear");
    ROS_INFO("Aperte 3/4 = + vel_angular/- vel_angular");
    ROS_INFO("Aperte P = sair");

    while(ros::ok){
        if(kbhit()){
            char key = getchar();            
            switch (key)
            {
            case 'w':
                move.linear.x = vel_linear;
                move.linear.y = 0.0;
                move.angular.z = 0.0;
                pub.publish(move);
                break;

            case 'x':
                move.linear.x = -vel_linear;
                move.linear.y = 0.0;
                move.angular.z = 0.0;
                pub.publish(move);
                break;
            
            case 'a':
                move.linear.x = 0.0;
                move.linear.y = vel_linear;
                move.angular.z = 0.0;
                pub.publish(move);
                break;
            
            case 'd':
                move.linear.x = 0.0;
                move.linear.y = -vel_linear;
                move.angular.z = 0.0;
                pub.publish(move);
                break;

            case 'h':
                move.linear.x = 0.0;
                move.linear.y = 0.0;
                move.angular.z = vel_angular;
                pub.publish(move);
                break;
            
            case 'j':
                move.linear.x = 0.0;
                move.linear.y = 0.0;
                move.angular.z = -vel_angular;
                pub.publish(move);
                break;

            case 's':
                move.linear.x = 0.0;
                move.linear.y = 0.0;
                move.angular.z = 0.0;
                pub.publish(move);
                break;
            
            case 'p':
                return 0;
                break;

            case 'q':
                move.linear.x = vel_linear;
                move.linear.y = vel_linear;
                move.angular.z = 0.0;
                pub.publish(move);
                break;
            
            case 'e':
                move.linear.x = vel_linear;
                move.linear.y = -vel_linear;
                move.angular.z = 0.0;
                pub.publish(move);
                break;
            
            case 'z':
                move.linear.x = -vel_linear;
                move.linear.y = vel_linear;
                move.angular.z = 0.0;
                pub.publish(move);
                break;

            case 'c':
                move.linear.x = -vel_linear;
                move.linear.y = -vel_linear;
                move.angular.z = 0.0;
                pub.publish(move);
                break;
            
            case '1':
                vel_linear += 0.2;
                ROS_INFO("vel_linear: %f", vel_linear);
                break;
            
            case '2':
                vel_linear -=0.2;
                ROS_INFO("vel_linear: %f", vel_linear);
                break;
            
            case '3':
                vel_angular +=0.26;
                ROS_INFO("vel_angular: %f", vel_angular);
                break;
            
            case '4':
                vel_angular -=0.26;
                ROS_INFO("vel_angular: %f", vel_angular);
                break;

            default:
                break;
            }
        }
    }

    return 0;
}