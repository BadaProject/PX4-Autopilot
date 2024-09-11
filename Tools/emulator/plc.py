import struct

# PLCEmulators class 생성
   * write_command와 read_commnad packet이 있다.
   * write_command packet 구조는 다음과 같다.
   * read_command packet 구조는 다음과 같다.

```python
write_command_head_buffer = struct.pack('40B',
	0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54,
	0x00, 0x00, # Reserved[2]
	0x00, 0x00, # PLC Info[2]
	0xA0, # CPU Info[1]
	0x33, # Source of Frame[1]
	0x01, 0x00, # Invoke ID[2]
	0x14, 0x00, # Length[2]
	0x00, # Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module
	0xFF, # checksum
	REQUEST_WRITE, 0x00, # Request[2]
	0x14, 0x00, # Data Type[2]
	0x00, 0x00, # Reserved[2]
	0x01, 0x00, # block length[2]
	0x08, 0x00, # block name length[2]
	0x25, 0x44, 0x42, 0x30, 0x36, 0x30, 0x30, 0x30, # %DB06000
	0x00, 0x00 # PX4 to PLC Data Size
)

write_command = struct.pack('40B',
	0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54,
	0x00, 0x00, # Reserved[2]
	0x00, 0x00, # PLC Info[2]
	0xA0, # CPU Info[1]
	0x33, # Source of Frame[1]
	0x01, 0x00, # Invoke ID[2]
	0x14, 0x00, # Length[2]
	0x00, # Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module
	0xFF, # checksum
	REQUEST_WRITE, 0x00, # Request[2]
	0x14, 0x00, # Data Type[2]
	0x00, 0x00, # Reserved[2]
	0x01, 0x00, # block length[2]
	0x08, 0x00, # block name length[2]
	0x25, 0x44, 0x42, 0x30, 0x36, 0x30, 0x30, 0x30, # %DB06000
	0x00, 0x00 # PX4 to PLC Data Size
)

def getCheckSum(data, start, length):
	sum = 0
	for i in range(length):
		sum += data[start + i]
	sum = sum & 0x00FF
	return sum

class PLCPacket:
	def __init__(self):
		self.CompanyID = struct.pack('8B', *[0]*8)
		self.Reserved1= struct.pack('2B', *[0]*2)
		self.PLCInfo = struct.pack('2B', *[0]*2)
		self.CPUInfo = 0
		self.SourceOfFrame = 0
		self.InvoikeID = struct.pack('2B', *[0]*2)
		self.SizeOfApplicationInstruction = struct.pack('2B', *[0]*2)
		self.SlotFEnetIFModule = 0
		self.checksum = 0
		self.request = struct.pack('2B', *[0]*2)
		self.DataType = struct.pack('2B', *[0]*2)
		self.Reserved2 = struct.pack('2B', *[0]*2)
		self.BlockLength = struct.pack('2B', *[0]*2)
		self.BlockNameLength = struct.pack('2B', *[0]*2)
		self.BlockName = struct.pack('8B', *[0]*8)
		self.DataSize = struct.pack('2B', *[0]*2)


	def getCheckSum(self, data, start, length):
		sum = 0
		for i in range(start, start + length):
			sum += data[i]
		sum = sum & 0x00FF
		return sum

	def getReadCommandPacket(self):
		return struct.pack('40B',
			self.CompanyID[0], self.CompanyID[1], self.CompanyID[2], self.CompanyID[3], self.CompanyID[4], self.CompanyID[5], self.CompanyID[6], self.CompanyID[7],
			self.Reserved1[0], self.Reserved1[1],
			self.PLCInfo[0], self.PLCInfo[1],
			self.CPUInfo,
			self.SourceOfFrame,
			self.InvoikeID[0], self.InvoikeID[1],
			self.SizeOfApplicationInstruction[0], self.SizeOfApplicationInstruction[1],
			self.SlotFEnetIFModule,
			self.checksum,
			self.request[0], self.request[1],
			self.DataType[0], self.DataType[1],
			self.Reserved2[0], self.Reserved2[1],
			self.BlockLength[0], self.BlockLength[1],
			self.BlockNameLength[0], self.BlockNameLength[1],
			self.BlockName[0], self.BlockName[1], self.BlockName[2], self.BlockName[3], self.BlockName[4], self.BlockName[5], self.BlockName[6], self.BlockName[7],
			self.DataSize[0], self.DataSize[1]
		)


	def getWriteCommandPacket(self):
		return struct.pack('40B',
		     self.CompanyID[0], self.CompanyID[1], self.CompanyID[2], self.CompanyID[3], self.CompanyID[4], self.CompanyID[5], self.CompanyID[6], self.CompanyID[7],
		     self.Reserved1[0], self.Reserved1[1],
		     self.PLCInfo[0], self.PLCInfo[1],
		     self.CPUInfo,
		     self.SourceOfFrame,
		     self.InvoikeID[0], self.InvoikeID[1],
		     self.SizeOfApplicationInstruction[0], self.SizeOfApplicationInstruction[1],
		     self.SlotFEnetIFModule,
		     self.checksum,
		     self.request[0], self.request[1],
		     self.DataType[0], self.DataType[1],
		     self.Reserved2[0], self.Reserved2[1],
		     self.BlockLength[0], self.BlockLength[1],
		     self.BlockNameLength[0], self.BlockNameLength[1],
		     self.BlockName[0], self.BlockName[1], self.BlockName[2], self.BlockName[3], self.BlockName[4], self.BlockName[5], self.BlockName[6], self.BlockName[7],
		     self.DataSize[0], self.DataSize[1]
		)

	def parseCommandPacket(self, data):
		self.data = data
		self.CompanyID = data[0:8]
		self.Reserved1= data[8:10]
		self.PLCInfo = data[10:12]
		self.CPUInfo = data[12]
		self.SourceOfFrame = data[13]
		self.InvoikeID = data[14:16]
		self.SizeOfApplicationInstruction = data[16:18]
		self.SlotFEnetIFModule = data[18]
		self.checksum = data[19]
		self.request = data[20:22]
		self.DataType = data[22:24]
		self.Reserved2 = data[24:26]
		self.BlockLength = data[26:28]
		self.BlockNameLength = data[28:30]
		self.BlockName = data[30:38]
		self.DataSize = data[38:40]

	def checkReadCommandPacket(self):
		if self.CompanyID != struct.pack('8B', 0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54):
			return False
		if self.Reserved1 != struct.pack('2B', 0, 0) :
			return False
		if self.PLCInfo != struct.pack('2B', 0, 0):
			return False
		if self.CPUInfo != 0xA0:
			return False
		if self.SourceOfFrame != 0x33:
			return False
		if self.InvoikeID != struct.pack('2B', 0, 0):
			return False
		if self.SizeOfApplicationInstruction != struct.pack('2B', 0x14, 0x00) :
			return False
		if self.SlotFEnetIFModule != 0 :
			return False
		if self.checksum != getCheckSum(self.data, 0, 19):
			return False
		if self.request != struct.pack('2B', 0x54, 0x00) :
			return False
		if self.DataType != struct.pack('2B', 0x14, 0x00):
			return False
		if self.Reserved2 != struct.pack('2B', 0, 0):
			return False
		if self.BlockLength != struct.pack('2B', 0x10, 0):
			return False
		if self.BlockNameLength != struct.pack('2B', 0x08, 0):
			return False
		if self.BlockName != struct.pack('8B', 0x25, 0x44, 0x42, 0x30, 0x37, 0x30, 0x30, 0x30): # '%DB07000':
			return False
		if self.DataSize != '\x00\x00':
			return False
		return True

	def checkWriteCommandPacket(self):
		if self.CompanyID != struct.pack('8B', 0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54):
			return False
		if self.Reserved1 != struct.pack('2B', 0, 0) :
			return False
		if self.PLCInfo != struct.pack('2B', 0, 0):
			return False
		if self.CPUInfo != 0xA0:
			return False
		if self.SourceOfFrame != 0x33:
			return False
		if self.InvoikeID != struct.pack('2B', 1, 0):
			return False
		if self.SizeOfApplicationInstruction != struct.pack('2B', 0x14, 0x00) :
			return False
		if self.SlotFEnetIFModule != 0 :
			return False
		if self.checksum != getCheckSum(self.data, 0, 19):
			return False
		if self.request != struct.pack('2B', 0x58, 0x00) :
			return False
		if self.DataType != struct.pack('2B', 0x14, 0x00):
			return False
		if self.Reserved2 != struct.pack('2B', 0, 0):
			return False
		if self.BlockLength != struct.pack('2B', 1, 0):
			return False
		if self.BlockNameLength != struct.pack('2B', 8, 0):
			return False
		if self.BlockName != struct.pack('8B'0x25, 0x44, 0x42, 0x30, 0x36, 0x30, 0x30, 0x30):#'%DB06000':
			return False
		if self.DataSize != '\x00\x00':
			return False
		return True

	def getDataSize(self):
		return self.DataSize[0] + (self.DataSize[1]<<8)
```

*
