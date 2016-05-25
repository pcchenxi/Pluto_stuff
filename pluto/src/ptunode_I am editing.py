#!/usr/bin/env python

import time
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
	global actualPanPos,actualTiltPos
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
					#print "posbytes in if: ", posbytes
				else:
					posbytes.append(byte)
					#print "posbytes in else: ", posbytes
			pos = struct.unpack('f', ''.join(posbytes))[0]
			if td == [b'\x08',b'\x86']: #telid for pan position
				actualPanPos = pos
			elif td == [b'\x08',b'\x66']: #telid for tilt position
				actualTiltPos = pos
				#print "tail: ", td
				print "posbytes: ", posbytes
				#print "join: ", repr(''.join(posbytes))
				#print "unpack: ", struct.unpack('f', ''.join(posbytes))
				print "pos: ", pos
				#print time.time()
	except:
		pass



def read_serial():
	telid = []
	data = []
	posbytes = []

	lastByte = None
	startedCommand = False
	STX = b'\x02'
	ETX = b'\x03'
	while not rospy.is_shutdown():
		byte = ser.read(1)		
		if byte == STX:
			telid = []
			data = []
			posbytes = []

			print "STX: ", repr(byte)

			telid = ser.read(2)
			print "TELID: ", repr(telid)

			if (telid != b'\x08\x83') & (telid != b'\x08\x86'):
				print "TELID error"
				continue
			
			byte = ser.read(1)
			byte_pre = None
			while (byte!=ETX):
				if (byte_pre == b'\x10') & (byte == b'\x82'):
					data[len(data)-1] = b'\x02'
				elif (byte_pre == b'\x10') & (byte == b'\x83'):
					data[len(data)-1] = b'\x03'
				elif (byte_pre == b'\x10') & (byte == b'\x90'):
					data[len(data)-1] = b'\x10'
				else:
					data.append(byte)
				
				byte_pre = byte
				byte = ser.read(1)
			
			print "Data: ", repr(data)
			print "EXT: ", repr(byte)

			lens = len(data)
			for byte in data[0:lens-1]:
				posbytes.append(byte)

			print "posbytes: ", repr(posbytes)

			lens = len(posbytes)
			#if lens >= 4:
			#	pos = struct.unpack('f', ''.join(posbytes))
			#	print "pos: ", repr(pos)
				


panpos = 0.0
tiltpos = 0.0
actualPanPos = 0.0
actualTiltPos = 0.0
newCommand = False

ser = serial.Serial("/dev/ttyUSB0")
	
init() #makes sure home has been done for pan and tilt parts.
time.sleep(1) #waits a second before continuing. We want the home command to really be sent with no interference

thread = threading.Thread(target=read_serial)
thread.start()

rospy.init_node("ptunode",anonymous = True)
rospy.Subscriber("/ptu/cmd_pos",Vector3,control_callback)

pub = rospy.Publisher('/ptu/ptupos', Vector3, queue_size=10)
rate = rospy.Rate(10)
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
	pub.publish(vec)

	rate.sleep()
		
		

thread.join()
ser.close()
