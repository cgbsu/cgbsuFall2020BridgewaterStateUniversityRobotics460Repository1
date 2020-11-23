package main

import (
	// "math"
	"fmt"
	"image"
	"gocv.io/x/gocv"
	// "golang.org/x/image/colornames"
)

func CompactOnNeighborhoodMedian( points []image.Point, maxNeighborDistance float64 ) []image.Point {
	var medianPoints []image.Point
	if len( points ) == 0 || maxNeighborDistance <= 0.0 {
		return medianPoints
	}
	refrence := points[ 0 ]
	median := points[ 0 ]
	for _, currentPoint := range points {
		if PointDistance( refrence, currentPoint ) > maxNeighborDistance {
			medianPoints = append( medianPoints, median )
			refrence = currentPoint
			median = currentPoint
		} else {
			median = currentPoint.Add( median ).Div( 2 )
		}
	}
	return append( medianPoints, median )
}



func FindFingersCount( inputImage gocv.Mat, window *gocv.Window ) gocv.Mat {
	contoursImage := gocv.NewMat()
	// window.IMShow( inputImage )
	// window.WaitKey( 2000 )
	contours := gocv.FindContours( inputImage, gocv.RetrievalExternal, gocv.ChainApproxSimple )
	if len( contours ) <= 0 {
		fmt.Println( "No contours" )
		return contoursImage
	}
	biggestContorIndex := -1
	biggestArea := 0.0
	i := 0
	for i, _ = range contours {
		currentArea := gocv.ContourArea( contours[ i ] )
		if currentArea > biggestArea {
			biggestArea = currentArea
			biggestContorIndex = i
		}
	}
	if biggestContorIndex < 0 {
		fmt.Println( "Couldnet find the biggest contour" )
		return contoursImage
	}
	hullContours := gocv.NewMat()
	gocv.ConvexHull( contours[ i ], &hullContours, true, true )
	return hullContours
}

type HandFinder struct {
	net gocv.Net
	lastKnownLocation, imageSize, kernelSize image.Point
	searchedForHand bool
}

func ( this *HandFinder ) InitializeHandFinder( pbFile string, kernelSize image.Point ) {
	this.net = gocv.ReadNetFromTensorflow( pbFile )
	this.searchedForHand = false
	if kernelSize.X == 0 && kernelSize.Y == 0 {
		this.kernelSize = kernelSize
	} else {
		this.kernelSize = image.Pt( 150, 150 )
	}
}

func ( this *HandFinder ) FindHand( inputImage gocv.Mat ) image.Point {
	this.imageSize = image.Point{ X: inputImage.Size()[ 0 ], Y: inputImage.Size()[ 1 ] }
	heatMap := gocv.BlobFromImage( inputImage, 1.0, this.kernelSize, gocv.NewScalar( 0.0, 0.0, 0.0, 0.0 ), true, false )
	this.net.SetInput( heatMap, "" )
	probabilities := this.net.Forward( "" )
	fmt.Println( "r: ", probabilities.Rows(), " c: ", probabilities.Cols() )
	// probabilities.Reshape()
	return image.Point{ X: 0, Y: 0 }
}

const HandTotalPointsConstant = 22
const ThumbIndexConstant = 0
const IndexFingerIndexConstant = 1
const MiddleFingerIndexConstant = 2
const RingFingerIndexConstant = 3
const PinkyIndexConstant = 4

type Hand struct {
	fingers [ 5 ][ 4 ]image.Point
	allPoints [ 22 ]image.Point
	writst, imageSize image.Point
	lastDetection gocv.Mat
	net gocv.Net
}

func ( this *Hand ) InitializeHand( protoFile, weightsFile string ) {
	this.net = gocv.ReadNetFromCaffe( protoFile, weightsFile )
}

func ( this *Hand ) MakeHandDataFromImage( inputImage gocv.Mat ) {
	this.imageSize = image.Point{ X: inputImage.Size()[ 0 ], Y: inputImage.Size()[ 1 ] }
	heatMap := gocv.BlobFromImage( inputImage, 1.0 / 255.0, this.imageSize, gocv.NewScalar( 0.0, 0.0, 0.0, 0.0 ), true, false )
	this.net.SetInput( heatMap, "" )
	this.lastDetection = this.net.Forward( "" )
}

//https://github.com/Sandeep-Sthapit/HandGestureDetection
//https://github.com/Balaje/OpenCV/tree/master/haarcascades
//https://medium.com/ai-innovation/hand-tracking-in-unity3d-f741a5e21a92
