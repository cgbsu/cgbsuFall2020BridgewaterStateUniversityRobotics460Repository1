package main

import (
	"math"
	"gocv.io/x/gocv"
)
func ToHsv( sourceImage gocv.Mat ) gocv.Mat {
	// destinationImage := gocv.NewMat()
	gocv.CvtColor( sourceImage, &sourceImage, gocv.ColorRGBToHSV )
	// return destinationImage
	return sourceImage
}

const HsvOffsetLowThresholdConstant = 80;
const HsvOffsetHighThresholdConstant = 30;

func CalculateThresholds( hsvSample0, hsvSample1 gocv.Mat ) ( gocv.Scalar, gocv.Scalar ) {
	hsvMean0 := hsvSample0.Mean()
	hsvMean1 := hsvSample1.Mean()
	hChannelLowThreashold := math.Min( hsvMean0.Val1, hsvMean1.Val1 ) - HsvOffsetLowThresholdConstant
	hChannelHighThreashold := math.Max( hsvMean0.Val1, hsvMean1.Val1 ) + HsvOffsetHighThresholdConstant
	sChannelLowThreashold := math.Min( hsvMean0.Val2, hsvMean1.Val2 ) - HsvOffsetLowThresholdConstant
	sChannelHighThreashold := math.Max( hsvMean0.Val2, hsvMean1.Val2 ) + HsvOffsetHighThresholdConstant
	vChannelLowThreashold := math.Min( hsvMean0.Val3, hsvMean1.Val3 ) - HsvOffsetLowThresholdConstant
	vChannelHighThreashold := math.Max( hsvMean0.Val3, hsvMean1.Val3 ) + HsvOffsetHighThresholdConstant
	return gocv.Scalar{ Val1: hChannelLowThreashold, Val2: sChannelLowThreashold, 
					Val3: vChannelLowThreashold, Val4: 0  }, 
			gocv.Scalar{ Val1: hChannelHighThreashold, Val2: sChannelHighThreashold, 
					Val3: vChannelHighThreashold, Val4: 0  }
}

func AverageSkinColor( samples []gocv.Mat ) ( gocv.Scalar, gocv.Scalar ) {
	//Need at least 2 samples to take an average//
	if len( samples ) >= 2 {
		// reader := bufio.NewReader( os.Stdin )
		var lowerBound, upperBound ScalarAverage
		lowerBound.InitializeAverage( len( samples ) )
		upperBound.InitializeAverage( len( samples ) )
		lastSample := ToHsv( samples[ 0 ] )
		for i := 1; i < len( samples ); i += 1 {
			currentImage := ToHsv( samples[ i ] )
			// DebugShowImage( &currentImage, &gocv.CascadeClassifier{} )
			// reader.ReadString( '\n' )
			lowerBoundSample, upperBoundSample := CalculateThresholds( lastSample, currentImage )
			lowerBound.AddScalarSample( lowerBoundSample )
			upperBound.AddScalarSample( upperBoundSample )
			lastSample = currentImage
		}
		return lowerBound.ConstructScalar(), upperBound.ConstructScalar()
	}
	return gocv.Scalar{}, gocv.Scalar{}
}
