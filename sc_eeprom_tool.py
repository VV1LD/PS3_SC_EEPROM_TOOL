#************************************************************************
#sc_eeprom_tool.py (v0.01) - Teensy++ 2.0 Compatible PS3 Syscon EEPROM utility
#Modified from SPIway from judges <judges@eEcho.com>
#Hacked together for Original Model Syscons on Fat PS3 consoles by
#WildCard https://twitter.com/VVildCard777 2019
#
#This code is licensed to you under the terms of the GNU GPL, version 2;
#see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
#*************************************************************************

import serial, time, datetime, sys, struct

class TeensySerialError(Exception):
	pass

class TeensySerial(object):
	BUFSIZE = 32768

	def __init__(self, port):
		self.ser = serial.Serial(port, 9600, timeout=300, rtscts=False, dsrdtr=False, xonxoff=False, writeTimeout=120)
		if self.ser is None:
			raise TeensySerialError("could not open serial %s")%port
		self.ser.flushInput()
		self.ser.flushOutput()
		self.obuf = ""

	def write(self, s):
		if isinstance(s,int):
			s = chr(s)
		elif isinstance(s,tuple) or isinstance(s,list):
			s = ''.join([chr(c) for c in s])
		self.obuf += s
		while len(self.obuf) > self.BUFSIZE:
			self.ser.write(self.obuf[:self.BUFSIZE])
			self.obuf = self.obuf[self.BUFSIZE:]

	def flush(self):
		if len(self.obuf):
			self.ser.write(self.obuf)
			self.ser.flush()
			self.obuf = ""

	def read(self, size):
		self.flush()
		data = self.ser.read(size)
		return data

	def readbyte(self):
		return ord(self.read(1))

	def close(self):
		print
		print "Closing serial device..."
		if self.ser is None:
			print "Device already closed."
		else:
			self.ser.close()
			print "Done."

class SPIError(Exception):
	pass

class SPIFlasher(TeensySerial):
	VERSION_MAJOR = 0
	VERSION_MINOR = 0
	SPI_DISABLE_PULLUPS = 0
	MF_ID = 0
	DEVICE_ID = 0
	SPI_SECTOR_SIZE = 0
	SPI_TOTAL_SECTORS = 0
	SPI_BLOCK_COUNT = 0
	SPI_SECTORS_PER_BLOCK = 0
	SPI_BLOCK_SIZE = 0
	SPI_ADDRESS_LENGTH = 0

	# Teensy commands
	CMD_PING1 = 0
	CMD_PING2 = 1
	CMD_GET_STATUS = 2
	CMD_DUMP_EEPROM = 3
	CMD_UNLOCK_EEPROM = 4
	CMD_WRITE_EEPROM = 5
	
	def __init__(self, port, ver_major, ver_minor):
		if port:
			TeensySerial.__init__(self, port)
		self.SPI_DISABLE_PULLUPS = 0
		self.VERSION_MAJOR = ver_major
		self.VERSION_MINOR = ver_minor

	def ping(self):
		self.write(self.CMD_PING1)
		self.write(self.CMD_PING2)
		ver_major = self.readbyte()
		ver_minor = self.readbyte()
		freeram = (self.readbyte() << 8) | self.readbyte()
		if (ver_major != self.VERSION_MAJOR) or (ver_minor != self.VERSION_MINOR):
			print "Ping failed (expected v%d.%02d, got v%d.%02d)"%(self.VERSION_MAJOR, self.VERSION_MINOR, ver_major, ver_minor)
			self.close()
			sys.exit(1)

		return freeram

	def get_status_eeprom(self):
		self.write(self.CMD_GET_STATUS)
		status = 0
		bytes = bytearray(4)
		for x in range(0, 4):
			bytes[x] = self.readbyte()
		print

		status = struct.unpack('>L', bytes)
		sys.stdout.flush()	

		print hex(status[0])
		if(status[0] == 0xffffffff):
			print "Status: OK"
			return 0
		else:
			print "Status: ???"

			return 1


	def dump_eeprom(self,filename):
		status = 1
		while status:
			status = self.get_status_eeprom()
			#time.sleep(10)
			
		self.write(self.CMD_DUMP_EEPROM)
		 
		fo = open(filename,"wb")
		data = self.read(0x8000)
		fo.write(data)
		sys.stdout.flush()	
		return 0



	def write_eeprom_data(self, data, offset):

		data_array = bytearray.fromhex(data)
		offset = int(offset, 16)
		size = int(len(data_array))
		
		if(offset >= 0x8000):
			print "out of range!"
			return 1

		if(size+offset >= 0x8000):
			print "size to large!"
			return 1
	
		status = 1
		while status:
			status = self.get_status_eeprom()
	#		time.sleep(10)
			
		self.unlock_eeprom()

		blocks = (size/32)
		if (size%32) > 0:
			blocks = blocks + 1
				
		for x in range(0,int(blocks)):
			if (x == int(blocks)-1) and (size%32) > 0:
				indexed_offset = offset + (32 * x)
				
				self.write(self.CMD_WRITE_EEPROM)
				self.write((indexed_offset >> 8) & 0xff)
				self.write(indexed_offset & 0xff)
				self.write(size%32)

				self.write(data_array[x*32:(x*32)+(size%32)])				
			
			else:
				indexed_offset = offset + (32 * x)
				
				self.write(self.CMD_WRITE_EEPROM)
				self.write((indexed_offset >> 8) & 0xff)
				self.write(indexed_offset & 0xff)
				self.write(32)

				self.write(data_array[x*32:(x*32)+32])

			sys.stdout.flush()	
	
	
		return 0

	def write_eeprom_file(self,filename, offset, size):

		offset = int(offset, 16)
		size = int(size,16)
	
		if(offset >= 0x8000):
			print "out of range!"
			return 1
	
		if(size+offset >= 0x8000):
			print "size to large!"
			return 1

	
		status = 1
		while status:
			status = self.get_status_eeprom()
	#		time.sleep(10)
			
		self.unlock_eeprom()	
		

		blocks = (size/32)
		if (size%32) > 0:
			blocks = blocks + 1
		
		file = open(filename,"rb")
		data = file.read()
		
		for x in range(0,int(blocks)):
			if (x == int(blocks)-1) and (size%32) > 0:
				indexed_offset = offset + (32 * x)
				
				self.write(self.CMD_WRITE_EEPROM)
				self.write((indexed_offset >> 8) & 0xff)
				self.write(indexed_offset & 0xff)
				self.write(size%32)

				self.write(data[x*32:x*32+(size%32)])				
			
			else:
				indexed_offset = offset + (32 * x)
				
				self.write(self.CMD_WRITE_EEPROM)
				self.write((indexed_offset >> 8) & 0xff)
				self.write(indexed_offset & 0xff)
				self.write(32)

				self.write(data[x*32:(x*32)+32])

			sys.stdout.flush()	
		file.close()
		return 0

	def unlock_eeprom(self):
		self.write(self.CMD_UNLOCK_EEPROM)
		
		print "EEPROM Unlocked"
		sys.stdout.flush()	
		return 0




if __name__ == "__main__":
	VERSION_MAJOR = 0
	VERSION_MINOR = 1

	print "FAT PS3 Syscon EEPROM utility by WildCard"
	print "Based on SPIway from PS3 developer judges <judges@eEcho.com>"
	print "Just a simple tool to make dumping/writing to syscon eeprom easier"
	print

	if len(sys.argv) == 1:
		print "USAGE:"
		print "		dump: dumps eeprom (0x8000) in size to filename"
		print "		status: gets eeprom status, 0xffffffff if all is OK"
		print "		writefile: writes up to 0x3fff bytes from binary file to eeprom"
		print "					sc_eeprom_tool.py COM1 writefile <filename> <offset> <size>"
		print "		writedata: writes up to 0x3fff bytes from cmdline to eeprom"
		print "					sc_eeprom_tool.py COM1 writedata <data> <offset>"
		print "		unlock: unlocks eeprom for writing"
		print "Examples:"
		print "  sc_eeprom_tool.py COM1 dump ./myeeprom.bin"		
		print "  sc_eeprom_tool.py COM1 status"		
		print "  sc_eeprom_tool.py COM1 writefile dump.bin 0 0x3fff"		
		print "  sc_eeprom_tool.py COM1 writedata 99d9662bb3d761 0"		

		sys.exit(0)

	n = SPIFlasher(sys.argv[1], VERSION_MAJOR, VERSION_MINOR)
	print "Pinging Teensy..."
	freeram = n.ping()
	print "Available memory: %d bytes"%(freeram)
	print
	
	tStart = time.time()
	
	if len(sys.argv) == 4 and sys.argv[2] == "dump":
		print "Dumping eeprom..."
		n.dump_eeprom(sys.argv[3])
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))

	elif len(sys.argv) == 6 and sys.argv[2] == "writefile":
		print "Writing to eeprom..."
		n.write_eeprom_file(sys.argv[3], sys.argv[4], sys.argv[5])
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))

	elif len(sys.argv) == 5 and sys.argv[2] == "writedata":
		print "Writing to eeprom..."
		n.write_eeprom_data(sys.argv[3], sys.argv[4])
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))


	elif len(sys.argv) == 3 and sys.argv[2] == "status":
		print "Getting chip status..."
		n.get_status_eeprom()
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))

	elif len(sys.argv) == 3 and sys.argv[2] == "bootloader":
		print
		print "Entering Teensy's bootloader mode... Goodbye!"
		#n.bootloader()
		sys.exit(0)

	n.ping()
