#!/usr/bin/env python
import rospy
import utm
import serial
import sys
from std_msgs.msg import Float64
from imu_pkg.msg import GNSS

# $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx 

# $GPGGA,181132.000,4220.3207,N,07105.3779,W,1,08,1.1,14.4,M,-33.8,M,,0000*51

# hhmmss.ss = UTC of position
# llll.ll = latitude of position
# a = N or S
# yyyyy.yy = Longitude of position
# a = E or W
# x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix)
# xx = number of satellites in use
# x.x = horizontal dilution of precision
# x.x = Antenna altitude above mean-sea-level
# M = units of antenna altitude, meters
# x.x = Geoidal separation
# M = units of geoidal separation, meters
# x.x = Age of Differential GPS data (seconds)
# xxxx = Differential reference station ID 
UTC = 1
LAT = 2
LAT_DIR = 3
LON = 4
LON_DIR = 5
ALT = 9
EASTING = 0
NORTHING = 1
ZONE_NUM = 2
ZONE_LET = 3

def parse_gps_data(gps_data):
	msg = GNSS()
	msg.lat = None
	sections = line.split(',')
	try:
		if sections[LAT_DIR] == 'S':
			sign = -1.0
		else:
			sign = 1.0
		raw_lat = sections[LAT]
		idx = raw_lat.find('.') - 2
		degree_part = float(raw_lat[:idx])
		minutes_part = float(raw_lat[idx:])
		dec_degrees = minutes_part / 60.0
		lat = sign*(degree_part + dec_degrees)
	except ValueError:
		return msg
	try:
		if sections[LON_DIR] == 'W':
			sign = -1.0
		else:
			sign = 1.0
		raw_lon = sections[LON]
		idx = raw_lon.find('.') - 2
		degree_part = float(raw_lon[:idx])
		minutes_part = float(raw_lon[idx:])
		dec_degrees = minutes_part / 60.0
		lon = sign*(degree_part + dec_degrees)
	except ValueError:
		return msg
	try:
		altitude = float(sections[ALT])
	except ValueError:
		return msg

	utm_out = utm.from_latlon(lat, lon)
	easting = utm_out[EASTING]
	northing = utm_out[NORTHING]
	try:
		zone_num = int(utm_out[ZONE_NUM])
	except ValueError:
		return msg
	zone_let = utm_out[ZONE_LET]
	msg.header.stamp = rospy.Time.now()
	msg.lat = lat
	msg.lon = lon
	msg.alt = altitude
	msg.utm_east = easting
	msg.utm_north = northing
	msg.zone_num = zone_num
	msg.zone_letter = zone_let
	return msg

if __name__ == "__main__":
	rospy.init_node("gps_driver")
	out = rospy.Publisher("/gps_out", GNSS, queue_size=10)
	port_handle = rospy.get_param("~port","/dev/ttyUSB1")
	baud_rate = rospy.get_param("~baudrate", 4800)
	try:
		port = serial.Serial(port_handle, baud_rate, timeout=3.0)
	except serial.serialutil.SerialException:
		rospy.loginfo("Serial port exception, shutting down gps_driver node...")
		sys.exit()
	# file_handle = open("data.txt", "r")
	rospy.logdebug("Setting up serial port for GPS device, port = %s, baud rate = %s" % (port_handle, baud_rate))
	rate = rospy.Rate(5)
	try:
		while not rospy.is_shutdown():
			line = port.readline()
			# line = file_handle.readline()
			if line == "":
				rospy.logwarn("GPS: no data")
			else:
				if line.startswith("$GPGGA"):
					rospy.loginfo(line)
					msg = parse_gps_data(line)
					if msg.lat != None:
						out.publish(msg)
			rate.sleep()
	except rospy.ROSInterruptException:
		port.close()
	except serial.serialutil.SerialException:
		rospy.loginfo("Serial port exception, shutting down gps_driver node...")
