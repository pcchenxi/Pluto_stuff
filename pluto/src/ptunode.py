#!/usr/bin/env python

import time
import serial,struct,time,rospy,threading
from geometry_msgs.msg import Vector3Stamped, Vector3

#TODO: make these a setting
mintilt =-1.0
maxtilt = 1.0
minpan =-1.0
maxpan = 1.0


#A simplified wrist program previously for testing purposes but more and more fully working
def init():
	ser.write(b'\x02') 
	ser.write(b'\x04') 
	ser.write(b'\x81') 
	ser.write(b'\x01') 
	ser.write(b'\x28') 
	ser.write(b'\x03') 
	
	#Send tilt home command
	ser.write(b'\x02') 
	ser.write(b'\x04') 
	ser.write(b'\x61') 
	ser.write(b'\x01') 
	ser.write(b'\x01') 
	ser.write(b'\x03') 

	#ser.write(b'\x02') #stx
	#ser.write(b'\x04') #telid
	#ser.write(b'\x66') #telid
	#ser.write(b'\x08') #command
	#ser.write(b'\x4F') #ramp
	#ser.write(b'\x0A') # data
	#ser.write(b'\xD7') # data
	#ser.write(b'\xA3') # data
	#ser.write(b'\x3D') # data
	#ser.write(b'\x44') #end
	#ser.write(b'\x03') #end
	
def send_pan_to(position):
	fourbytes = struct.pack('f', float(position))

	ser.write(b'\x02') #stx
	ser.write(b'\x04') #telid
	ser.write(b'\x86') #telid
	ser.write(b'\x0B') #command
	ser.write(b'\x04') #ramp

	for byte in fourbytes:
		if byte == b'\x02' or byte == b'\x03' or byte == b'\x10':
			ser.write(b'\x10')
			ser.write(chr(ord(byte) | ord(b'\x80')))
		else:
			ser.write(byte)
	
	ser.write(b'\x8F') #BCC (this doesn't seem to matter at all)
	ser.write(b'\x03') #end

def send_tilt_to(position):
	ser.write(b'\x02') #stx
	ser.write(b'\x04') #telid
	ser.write(b'\x66') #telid
	ser.write(b'\x0B') #command
	ser.write(b'\x04') #ramp
	fourbytes = struct.pack('f', float(position))
	for byte in fourbytes:
		if byte == b'\x02' or byte == b'\x03' or byte == b'\x10':
			ser.write(b'\x10')
			ser.write(bytes(ord(byte) | ord(b'\x80')))
		else:
			ser.write(byte)
	ser.write(b'\x8F') #BCC (this doesn't seem to matter at all)
	ser.write(b'\x03') #end

def request_pan_position():
	ser.write(b'\x02') #stx
	ser.write(b'\x04') #telid
	ser.write(b'\x82') #telid
	ser.write(b'\x0A') #command
	ser.write(b'\x3C') # idk
	ser.write(b'\x36') #bcc
	ser.write(b'\x03') #end
	
def request_tilt_position():
	ser.write(b'\x02') #stx
	ser.write(b'\x04') #telid
	ser.write(b'\x62') #telid
	ser.write(b'\x0A') #command
	ser.write(b'\x3C') # idk
	ser.write(b'\x36') #bcc
	ser.write(b'\x03') #end

def control_callback(msg):
	global panpos,tiltpos,newCommand
	pp=msg.x
	tp=msg.y
	panpos = min(max(pp,minpan),maxpan)
	tiltpos = min(max(tp,mintilt),maxtilt)
	newCommand = True	

def interpret_bytes(td,dt):
	global actualPanPos,actualTiltPos,tiltmsgnumber,panmsgnumber,pan_speed,tilt_speed,last_time_pan,last_time_tilt,last_pos_pan,last_pos_tilt, send_time, send_tilt
	try:
		if dt[0] == b'\x0a' and dt[1]==b'\x3c': #get extended, current position
			posbytes = []
			last = None
			for byte in dt[2:-1]:
				if byte == b'\x10':
					last = byte
				elif last == b'\x10':
					if byte == b'\x82':
						posbytes.append(b'\x02')
					elif byte == b'\x83':
						posbytes.append(b'\x03')
					elif byte == b'\x90':
						posbytes.append(b'\x10')
					last = None
				else:
					posbytes.append(byte)
			pos = struct.unpack('f', ''.join(posbytes))[0]
		
			if td == [b'\x08',b'\x86']: #telid for pan position				
				current_time = time.time()
				time_diff = current_time - last_time_pan
				pose_diff = pos - last_pos_pan
				speed = pose_diff/time_diff			
				last_time_pan = current_time
				last_pos_pan = pos
				pan_speed = speed
				#print "pan speed: ", pan_speed

				actualPanPos = pos
				panmsgnumber = panmsgnumber + 1

			elif td == [b'\x08',b'\x66']: #telid for tilt position
				current_time = time.time()
				time_diff = current_time - last_time_tilt
				pose_diff = pos - last_pos_tilt
				speed = pose_diff/time_diff
				last_time_tilt = current_time
				last_pos_tilt = pos
				tilt_speed = speed

				send_time = current_time
				send_tilt = pos
				#print "tilt speed: ", tilt_speed, "time: ", current_time, "pos:", pos

				tiltmsgnumber = tiltmsgnumber + 1
				actualTiltPos = pos
	except:
		pass



def read_serial():
	telid = []
	data = []
	lastByte = None
	startedCommand = False
	STX = b'\x02'
	ETX = b'\x03'
	while not rospy.is_shutdown():
		byte = ser.read(1) #I want this to be non-blocking...
		if byte == ETX:
			interpret_bytes(telid,data)
			telid = []
			data = []
		elif byte == STX:
			data = []
			telid = []
			lastByte = byte
		elif lastByte == STX:
			lastByte = 'telid'
			telid.append(byte)
		elif lastByte == 'telid':
			telid.append(byte)
			lastByte ='telid2'
		elif lastByte == 'telid2':
			data.append(byte)
		elif data:
			data.append(byte)
	print "data:", data

panpos = 0.0
tiltpos = 0.0
actualPanPos = 0.0
actualTiltPos = 0.0
newCommand = False
panmsgnumber = 0
tiltmsgnumber = 0

tilt_speed = 0.0
pan_speed = 0.0
last_time_pan = time.time()
last_time_tilt = time.time()
last_pos_pan = 0.0
last_pos_tilt = 0.0

send_time = time.time()
send_tilt = 0.0

ser = serial.Serial("/dev/ttyUSB0")
	
init() #makes sure home has been done for pan and tilt parts.
time.sleep(1) #waits a second before continuing. We want the home command to really be sent with no interference

thread = threading.Thread(target=read_serial)
thread.start()

rospy.init_node("ptunode",anonymous = True)
rospy.Subscriber("/ptu/cmd_pos",Vector3,control_callback)

pub = rospy.Publisher('/ptu/ptupos', Vector3, queue_size=10)
pubPan = rospy.Publisher('/ptu/panpos', Vector3Stamped, queue_size=10)
pubTilt = rospy.Publisher('/ptu/tiltpos', Vector3Stamped, queue_size=10)
rate = rospy.Rate(9)
lastpanmsgnumber = -1
lasttiltmsgnumber = -1

while not rospy.is_shutdown():

	request_pan_position()
	request_tilt_position()
	
	if newCommand:
		send_tilt_to(tiltpos)
		send_pan_to(panpos)
		newCommand = False

	vec = Vector3()
	vec.x = actualPanPos
	vec.y = actualTiltPos
	pub.publish(vec);

	vec = Vector3Stamped()
	vec.header.stamp.secs = time.time()
	vec.vector.x = actualPanPos
	pubPan.publish(vec)
	lastpanmsgnumber = panmsgnumber

	vec = Vector3Stamped()
	vec.header.stamp.secs = time.time()
	#vec.vector.y = actualTiltPos
	tm = time.time()
	time_diff = tm - send_time
	vec.vector.y = send_tilt + tilt_speed*time_diff

	send_time = tm
	send_tilt = vec.vector.y

	#print "speed:", tilt_speed, "pos:", vec.vector.y, "time_diff:", time_diff, "time:", tm
	pubTilt.publish(vec)
	lasttiltmsgnumber = tiltmsgnumber

	rate.sleep()
		
		

thread.join()
ser.close()
