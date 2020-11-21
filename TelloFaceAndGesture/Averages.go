package main

func Abs( number int ) int {
	if number < 0 {
		return -number
	}
	return number
}

type AverageTaker interface {
   InitializeAverage( desiredSampleCount int )
   CalculateAverage() int
   AddSample( sample int )
   Clear()
   AtDesiredSampleCount() bool
}

/*For averaging measurments, it would be nice if 
Go had generics so the sample type could be a generic*/
type Average struct {
	buffer, samples, desiredSampleCount int 
}

func ( self *Average ) InitializeAverage( desiredSampleCount int ) {
   self.desiredSampleCount = desiredSampleCount
}

func ( self *Average ) CalculateAverage() int {
   if self.samples == 0 {
	   return 0
   }
   return self.buffer / self.samples
}

func ( self *Average ) AddSample( sample int ) {
   self.buffer += sample
   self.samples += 1
}

func ( self *Average ) Clear() {
   self.buffer = 0
   self.samples = 0
}

func ( self *Average ) AtDesiredSampleCount() bool {
   if self.desiredSampleCount == 0 {
	   return true
   }
   return self.samples >= self.desiredSampleCount
}
