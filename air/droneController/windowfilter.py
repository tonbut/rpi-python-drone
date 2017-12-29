#FIR filter with given size and decay rate
class WindowFilter:
	def __init__(self,size,factor):
		self.window=[0]*size
		self.index=0
		self.factor=factor
		
	def add(self,value):
		if self.window[0]==None:
			for i in range(0,len(self.window)):
				self.window[i]=value
		else:
			self.index=(self.index+1)%len(self.window)
			self.window[self.index]=value
			
	def get(self):
		total=0
		m=1
		d=0
		j=self.index
		for i in range(0,len(self.window)):
			total+=self.window[j]*m
			d=d+m
			m=m-self.factor
			j=j-1
			if j<0:
				j=len(self.window)-1
		return total/d
		
#calculate average over a fixed window size of samples
class WindowAverage:
	def __init__(self,size):
		self.len=size
		self.window=[0]*size
		self.index=0
		self.total=0
		
	def add(self,value):
		if self.window[0]==None:
			for i in range(0,self.len):
				self.window[i]=value
				self.total=value*self.len
		else:
			self.index=(self.index+1)%len(self.window)
			self.total+=value-self.window[self.index]
			self.window[self.index]=value
			
	def get(self):
		return self.total/self.len