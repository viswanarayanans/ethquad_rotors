import socket
import rospy
from mav_msgs.msg import Actuators
# from mav_msgs.msg import default_topics


def UdpCallback(data):
	# print(type(data.angular_velocities[0]))
	send_data = str(tuple(map(sum, zip(data.angular_velocities, (1000,1000,1000,1000)))))
	# print(tuple(map(sum, zip(data.angular_velocities, (1000,1000,1000,1000)))))
	str_data = str(send_data.split('(')[-1].split(')')[0])
	print(str_data)


	client.sendto(str_data, ('192.168.1.255', 9999))


if __name__ == '__main__':
	rospy.init_node('udp_python')
	rospy.Subscriber("/pelican/command/motor_speed", Actuators, UdpCallback)

	client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
	client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	# client.sendto('1300,1300,1300,1300,0', ('192.168.1.255', 9999))



	rospy.spin()
