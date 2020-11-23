package main
import (
	"math"
	"image"
	"gocv.io/x/gocv"
)

/*
* The next few functions use:
* https://medium.com/@muehler.v/simple-hand-gesture-recognition-using-opencv-and-javascript-eb3d6ced28a0
* https://medium.com/@soffritti.pierfrancesco/handy-hands-detection-with-opencv-ac6e9fb3cec1
* https://www.codepasta.com/computer-vision/2019/04/26/background-segmentation-removal-with-opencv-take-2.html
* as a sources.
*/

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

func SaltAndPepperNoiseRemoval( toDenoise gocv.Mat, maxIterations int ) gocv.Mat {
	lastMedian := toDenoise
	numberOfIterations := 0
	denoised := gocv.NewMat()
	gocv.MedianBlur( toDenoise, &denoised, 3 )
	for MatsAreEqual( denoised, lastMedian ) == false {
		toDenoisePointer := toDenoise.DataPtrUint8()
		denoisedPointer := denoised.DataPtrUint8()
		for i, _ := range toDenoisePointer {
			if ( ^( denoisedPointer[ i ] & toDenoisePointer[ i ] ) ) == 0 {
				denoisedPointer[ i ] = 0
			}
			numberOfIterations += 1
			if numberOfIterations > maxIterations {
				break
			}
			lastMedian = denoised
			gocv.MedianBlur( toDenoise, &denoised, 3 )			
		}
	}
	return denoised
}

func RemoveBackground( toRemoveBackgroundFrom gocv.Mat ) ( gocv.Mat, [][]image.Point ) {
	gocv.GaussianBlur( toRemoveBackgroundFrom, &toRemoveBackgroundFrom, image.Point{ 5, 5 }, 0.0, 0.0, gocv.BorderReflect101 )
	edgeImage := gocv.NewMat()
	gocv.Canny( toRemoveBackgroundFrom, &edgeImage, 0, 100 )
	// edgeImage = SaltAndPepperNoiseRemoval( edgeImage, 30 )
	contours := gocv.FindContours( edgeImage, gocv.RetrievalExternal, gocv.ChainApproxSimple )
	return edgeImage, contours
}
