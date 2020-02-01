#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <arpa/inet.h> 

#include <netinet/in.h> 
#include <string.h> 

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>


#define PORT 9999

int sock = 0, valread; 
struct sockaddr_in serv_addr; 
// char *hello = "Hello from client"; 
char buffer[1024] = {0};
// char *hello = "Hello from server"; 
   


void sendUdpMsg(const mav_msgs::ActuatorsPtr msg){
  std::vector<double> velocities = msg->angular_velocities;

  velocities[0] = velocities[0] + 1000;
  velocities[1] = velocities[1] + 1000;
  velocities[2] = velocities[2] + 1000;
  velocities[3] = velocities[3] + 1000;

  ROS_INFO_STREAM("Callback");


  std::stringstream velocity_ss;
  std::copy(velocities.begin(), velocities.end(), std::ostream_iterator<double>(velocity_ss, " "));
  std:: string v = velocity_ss.str();
  const char* velocity_str = v.c_str();

  ROS_INFO_STREAM(velocity_str);
  // char const * velocity_str = reinterpret_cast<char const *>(data);
  send(sock , velocity_str , strlen(velocity_str) , 0 ); 
}


int main(int argc, char** argv)
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "udp_publisher");

  // Create a ROS node handle
  ros::NodeHandle nh;

  ROS_INFO("Udp node active");

  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
        return -1; 
    } 
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(PORT); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, "192.168.1.255", &serv_addr.sin_addr)<=0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
        return -1; 
    } 
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n"); 
        return -1; 
    }

  ros::Subscriber sub = nh.subscribe("/pelican/command/motor_speed", 1000, sendUdpMsg);
  // mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);



  // Don't exit the program.
  ros::spin();
}