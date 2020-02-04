#!/usr/bin/env python
import rospy
import serial
import sys
from math import sin, cos
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3


def euler_to_quaternion(yaw, pitch, roll):
	w = cos(yaw/2.0) * cos(pitch/2.0) * cos(roll/2.0) + sin(yaw/2.0) * sin(pitch/2.0) * sin(roll/2.0)
	x = cos(yaw/2.0) * cos(pitch/2.0) * cos(roll/2.0) - sin(yaw/2.0) * sin(pitch/2.0) * cos(roll/2.0)
	y = sin(yaw/2.0) * cos(pitch/2.0) * sin(roll/2.0) + cos(yaw/2.0) * sin(pitch/2.0) * cos(roll/2.0)
	z = sin(yaw/2.0) * cos(pitch/2.0) * cos(roll/2.0) - cos(yaw/2.0) * sin(pitch/2.0) * sin(roll/2.0)
	return w, x, y, z


def parse_imu_data(imu_data):
	rospy.loginfo(imu_data)
	imu_msg = Imu()
	mag_msg = MagneticField()
	mag_field = Vector3()
	orient_field = Quaternion()
	omega_dot_field = Vector3()
	v_dot_field = Vector3()

	imu_msg.header.stamp = rospy.Time.now()
	mag_msg.header.stamp = rospy.Time.now()

	sections = line.split(',')
	yaw = float(sections[1])
	pitch = float(sections[2])
	roll = float(sections[3])
	w, x, y, z = euler_to_quaternion(yaw, pitch, roll)

	orient_field.x = x
	orient_field.y = y
	orient_field.z = z
	orient_field.w = w

	mag_field.x = float(sections[4])
	mag_field.y = float(sections[5])
	mag_field.z = float(sections[6])

	v_dot_field.x = float(sections[7])
	v_dot_field.y = float(sections[8])
	v_dot_field.z = float(sections[9])

	omega_dot_field.x = float(sections[10])
	omega_dot_field.y = float(sections[11])
	raw_omega_dot_Z   = sections[12]
	omega_dot_field.z = float(raw_omega_dot_Z[:raw_omega_dot_Z.find("*")])

	imu_msg.orientation = orient_field
	imu_msg.angular_velocity = omega_dot_field
	imu_msg.linear_acceleration = v_dot_field
	mag_msg.magnetic_field = mag_field
	return imu_msg, mag_msg

if __name__ == "__main__":
	rospy.init_node("imu_driver")
	imu_out = rospy.Publisher("/imu_out", Imu, queue_size=10)
	mag_out = rospy.Publisher("/mag_out", MagneticField, queue_size=10)
	port_handle = rospy.get_param("~port","/dev/ttyUSB0")
	baud_rate = rospy.get_param("~baudrate", 115200)
	# try:
	# 	port = serial.Serial(port_handle, baud_rate, timeout=3.0)
	# except serial.serialutil.SerialException:
	# 	rospy.loginfo("Serial port exception, shutting down imu_driver node...")
	# 	sys.exit()
	file_handle = open("test_imu_data.txt", "r")
	rospy.logdebug("Setting up serial port for IMU device, port = %s, baud rate = %s" % (port_handle, baud_rate))
	try:
		while not rospy.is_shutdown():
			# line = port.readline()
			line = file_handle.readline()
			if line == "":
				rospy.logwarn("IMU: no data")
			else:
				if line.startswith("$VNYMR"):
					rospy.loginfo(line)
					imu_msg, mag_msg = parse_imu_data(line)
					imu_out.publish(imu_msg)
					mag_out.publish(mag_msg)
			rospy.sleep(0.001)
	except rospy.ROSInterruptException:
		port.close()
	except serial.serialutil.SerialException:
		rospy.loginfo("Serial port exception, shutting down imu_driver node...")
