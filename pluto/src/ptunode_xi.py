#!/usr/bin/env python

import serial,struct,time,rospy,threading
from geometry_msgs.msg import Vector3

#TODO: make these a setting
mintilt =-1.0
maxtilt = 0.3
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
	global actualPanPos,actualTiltPos,tiltmsgnumber,panmsgnumber
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
						posbytes.append(b'\x90')
					last = None
				else:
					posbytes.append(byte)
			pos = struct.unpack('f', ''.join(posbytes))[0]
			if td == [b'\x08',b'\x86']: #telid for pan position
				if (abs(actualPanPos - pos) < 0.1):
					actualPanPos = pos
					panmsgnumber = panmsgnumber + 1
			elif td == [b'\x08',b'\x66']: #telid for tilt position
				if (abs(actualTiltPos - pos) < 0.1):
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


panpos = 0.0
tiltpos = 0.0
actualPanPos = 0.0
actualTiltPos = 0.0
newCommand = False
panmsgnumber = 0
tiltmsgnumber = 0

ser = serial.Serial("/dev/ttyUSB2")
	
init() #makes sure home has been done for pan and tilt parts.
time.sleep(1) #waits a second before continuing. We want the home command to really be sent with no interference

thread = threading.Thread(target=read_serial)
thread.start()

rospy.init_node("ptunode",anonymous = True)
rospy.Subscriber("/ptu/cmd_pos",Vector3,control_callback)

pub = rospy.Publisher('/ptu/ptupos', Vector3, queue_size=10)
pubPan = rospy.Publisher('/ptu/panpos', Vector3, queue_size=10)
pubTilt = rospy.Publisher('/ptu/tiltpos', Vector3, queue_size=10)
rate = rospy.Rate(10)
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

	if (panmsgnumber > lastpanmsgnumber):
		vec = Vector3()
		vec.x = actualPanPos
		pubPan.publish(vec)
		lastpanmsgnumber = panmsgnumber
	if (tiltmsgnumber > lasttiltmsgnumber):
		vec = Vector3()
		vec.y = actualTiltPos
		pubTilt.publish(vec)
		lasttiltmsgnumber = tiltmsgnumber

	rate.sleep()
		
		

thread.join()
ser.close()
