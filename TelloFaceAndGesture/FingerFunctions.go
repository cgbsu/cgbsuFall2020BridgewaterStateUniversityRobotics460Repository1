package main

import (
	"math"
	"fmt"
	"image"
	"gocv.io/x/gocv"
	"golang.org/x/image/colornames"
	"image/color"
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

func ( this *Hand ) MakeHandDataFromImage( inputImage gocv.Mat, window *gocv.Window ) {
	// test := inputImage.Clone()
	this.imageSize = image.Point{ X: inputImage.Size()[ 0 ], Y: inputImage.Size()[ 1 ] }
	heatMap := gocv.BlobFromImage( inputImage, 1.0 / 255.0, image.Point{ 300, 300 }, gocv.NewScalar( 0.0, 0.0, 0.0, 0.0 ), true, false )
	this.net.SetInput( heatMap, "" )
	this.lastDetection = this.net.Forward( "" )
	fmt.Println( this.lastDetection.Cols(), " ", colornames.Red )
	/*var points []image.Point
	for i := 0; i < HandTotalPointsConstant; i += 1 {
		probabilityMap := this.lastDetection.RowRange( 0, i )
		gocv.
		gocv.Resize( probabilityMap, &probabilityMap, this.imageSize, 0.0, 0.0, gocv.InterpolationDefault )
		_, maximumValue, _, maximumLocation := gocv.MinMaxLoc( probabilityMap )
		if maximumValue > .8 {
			gocv.Circle( &test, maximumLocation, 8, colornames.Red, -1 )
			points = append( points, maximumLocation )
		}
	}*/
	// window.IMShow( test )
	// if window.WaitKey( 2000 ) != -1 {
		// window.Close()
	// }
}

//https://github.com/Sandeep-Sthapit/HandGestureDetection
//https://github.com/Balaje/OpenCV/tree/master/haarcascades
//https://medium.com/ai-innovation/hand-tracking-in-unity3d-f741a5e21a92
//https://www.learnopencv.com/hand-keypoint-detection-using-deep-learning-and-opencv/
//https://www.youtube.com/watch?v=v-XcmsYlzjA

func FingerCount( img gocv.Mat, debug *gocv.Window ) ( int, image.Rectangle, gocv.Mat ) {

	// imgGrey := gocv.NewMat()
	// defer imgGrey.Close()

	// imgBlur := gocv.NewMat()
	// defer imgBlur.Close()

	// imgThresh := gocv.NewMat()
	// defer imgThresh.Close()

	hull := gocv.NewMat()
	defer hull.Close()

	defects := gocv.NewMat()
	defer defects.Close()

	green := color.RGBA{0, 255, 0, 0}
	// gocv.CvtColor(img, &imgGrey, gocv.ColorBGRToGray)
	// gocv.GaussianBlur(imgGrey, &imgBlur, image.Pt(35, 35), 0, 0, gocv.BorderDefault)
	// gocv.Threshold(imgBlur, &imgThresh, 0, 255, gocv.ThresholdBinaryInv+gocv.ThresholdOtsu)
	// debug.IMShow( imgThresh )
	imgThresh := img
	// now find biggest contour
	// debug.IMShow( img )
	// debug.WaitKey( 2000 )
	contours := gocv.FindContours(imgThresh, gocv.RetrievalExternal, gocv.ChainApproxSimple)
	c := getBiggestContour(contours)

	gocv.ConvexHull(c, &hull, true, false)
	gocv.ConvexityDefects(c, hull, &defects)

	var angle float64
	defectCount := 0
	for i := 0; i < defects.Rows(); i++ {
		start := c[defects.GetIntAt(i, 0)]
		end := c[defects.GetIntAt(i, 1)]
		far := c[defects.GetIntAt(i, 2)]

		a := math.Sqrt(math.Pow(float64(end.X-start.X), 2) + math.Pow(float64(end.Y-start.Y), 2))
		b := math.Sqrt(math.Pow(float64(far.X-start.X), 2) + math.Pow(float64(far.Y-start.Y), 2))
		c := math.Sqrt(math.Pow(float64(end.X-far.X), 2) + math.Pow(float64(end.Y-far.Y), 2))

		// apply cosine rule here
		angle = math.Acos((math.Pow(b, 2)+math.Pow(c, 2)-math.Pow(a, 2))/(2*b*c)) * 57

		// ignore angles > 90 and highlight rest with dots
		if angle <= 90 {
			defectCount++
			gocv.Circle(&img, far, 1, green, 2)
		}
	}

	// status := fmt.Sprintf("defectCount: %d", defectCount+1)

	rect := gocv.BoundingRect(c)
	// gocv.Rectangle(&img, rect, color.RGBA{255, 255, 255, 0}, 2)

	// gocv.PutText(&img, status, image.Pt(10, 20), gocv.FontHersheyPlain, 1.2, green, 2)
	return ( defectCount + 1 ), rect, img
}

func getBiggestContour( contours [][]image.Point ) []image.Point {
	var area float64
	index := 0
	for i, c := range contours {
		newArea := gocv.ContourArea(c)
		if newArea > area {
			area = newArea
			index = i
		}
	}
	return contours[index]
}