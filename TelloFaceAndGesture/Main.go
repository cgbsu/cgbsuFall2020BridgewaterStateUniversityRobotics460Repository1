/*
	<comment>

	You must have ffmpeg and OpenCV installed in order to run this code. It will connect to the Tello
	and then open a window using OpenCV showing the streaming video.

	How to run

			go run examples/tello_opencv.go

	this is based on merging two existing tutorials:
	https://medium.com/@fonseka.live/detect-faces-using-golang-and-opencv-fbe7a48db055
	and
	https://gobot.io/documentation/examples/tello_opencv/

	and of course the classifier

	https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml

	added updates to make windows and mac friendly
	https://medium.com/tarkalabs/automating-dji-tello-drone-using-gobot-2b711bf42af6 

*/
package main

import (
	"fmt"
	"io"
	"log"
	"os/exec"
	"time"
	"math"
	"image"
	"strconv"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
	"golang.org/x/image/colornames"
	//"image/color"
)
/*
const (
	frameSize = 960 * 720 * 3
)*/

type FfmpegStream struct {
	ffmpeg *exec.Cmd
	in io.WriteCloser
	out io.ReadCloser
}

func ( this *FfmpegStream ) InitializeFfmpegStream( telloCameraImageSize image.Point ) {
	this.ffmpeg = exec.Command( "ffmpeg", "-i", "pipe:0", "-pix_fmt", "bgr24", "-vcodec", "rawvideo",
	"-an", "-sn", "-s", ( strconv.Itoa( telloCameraImageSize.Y ) + "x" + strconv.Itoa( telloCameraImageSize.X ) ), "-f", "rawvideo", "pipe:1" )
	var diagnostic error
	this.in, diagnostic = this.ffmpeg.StdinPipe()
	if diagnostic != nil {
		fmt.Println( diagnostic )
	}
	this.out, diagnostic = this.ffmpeg.StdoutPipe()
	if diagnostic != nil {
		fmt.Println( diagnostic )
	}
}

func NewFfmpegStream( telloCameraImageSize image.Point ) *FfmpegStream {
	ffmpegMediaStream := new( FfmpegStream )
	ffmpegMediaStream.InitializeFfmpegStream( telloCameraImageSize )
	return ffmpegMediaStream
}

func ConnectedEvent( drone *tello.Driver, data interface{} ) {
	fmt.Println( "Connected" )
	drone.StartVideo()
	drone.SetVideoEncoderRate( tello.VideoBitRateAuto )
	drone.SetExposure( 0 )
	gobot.Every( 100 * time.Millisecond, func() {
		drone.StartVideo()
	} )
}

func SendVideoData( data interface{}, ffmpegMediaStream *FfmpegStream ) {
	pkt := data.( []byte )
	if _, err := ffmpegMediaStream.in.Write( pkt ); err != nil {
		fmt.Println( err )
	}
}

func RobotWork( drone *tello.Driver, ffmpegMediaStream *FfmpegStream ) {
	if err := ffmpegMediaStream.ffmpeg.Start(); err != nil {
		fmt.Println( err )
		return
	}
	drone.On( tello.ConnectedEvent, func( data interface{} ) { 
		ConnectedEvent( drone, data )
	} )
	drone.On( tello.VideoFrameEvent,func( data interface{} ) {
		SendVideoData( data, ffmpegMediaStream )
	} )
}

func ReadTelloCameraImage( telloCameraImageSize image.Point, numberOfChannels int, ffmpegMediaStream *FfmpegStream ) ( gocv.Mat, error ) {
	buffer := make( []byte, telloCameraImageSize.X * telloCameraImageSize.Y * numberOfChannels )
	if _, diagnostic := io.ReadFull( ffmpegMediaStream.out, buffer ); diagnostic != nil {
		fmt.Println( diagnostic )
		return gocv.NewMat(), diagnostic
	}
	cameraMedia, diagnostic := gocv.NewMatFromBytes( telloCameraImageSize.X, telloCameraImageSize.Y, gocv.MatTypeCV8UC3, buffer )
	if diagnostic != nil {
		log.Print( diagnostic )
		return gocv.NewMat(), diagnostic
	}
	if cameraMedia.Empty() {
		fmt.Println( "No image data" )
		return gocv.NewMat(), diagnostic
	}
	return cameraMedia, nil
}

func ResizeImage( cameraMedia gocv.Mat, newSize image.Point ) gocv.Mat {
	destinationImage := gocv.NewMatWithSize( newSize.X, newSize.Y, gocv.MatTypeCV8UC3 )
	gocv.Resize( cameraMedia, &destinationImage, image.Pt( newSize.X, newSize.Y ), 0, 0, gocv.InterpolationNearestNeighbor )
	return destinationImage
}

func DetectFeatures( classifier *gocv.CascadeClassifier, detectionImage gocv.Mat, onDetect func( instance image.Rectangle ) ) {
	imageRectangles := classifier.DetectMultiScale( detectionImage )
	for _, currentRectangle := range imageRectangles {
		onDetect( currentRectangle )
		gocv.Rectangle( &detectionImage, currentRectangle, colornames.Cadetblue, 3 )
	}
}

/*
* The next few functions use:
* https://medium.com/@muehler.v/simple-hand-gesture-recognition-using-opencv-and-javascript-eb3d6ced28a0
* https://medium.com/@soffritti.pierfrancesco/handy-hands-detection-with-opencv-ac6e9fb3cec1
* as a sources.
*/

var window *gocv.Window

func ToHsv( sourceImage gocv.Mat ) gocv.Mat {
	destinationImage := gocv.NewMat()
	gocv.CvtColor( sourceImage, &destinationImage, gocv.ColorBGRToHSV )
	return destinationImage
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

func MakeHandMask( skinColorLowerBound, skinColorUpperBound gocv.Scalar, detectionImage gocv.Mat ) gocv.Mat {
	colorConvertedImage := gocv.NewMatWithSize( 
			detectionImage.Size()[ 0 ], detectionImage.Size()[ 1 ], gocv.MatTypeCV8UC3 )
	gocv.CvtColor( detectionImage, &colorConvertedImage, gocv.ColorBGRToHLS )
	gocv.InRangeWithScalar( colorConvertedImage, skinColorLowerBound, skinColorUpperBound, &colorConvertedImage )
	return colorConvertedImage
}

func AverageSkinColor( samples []gocv.Mat ) ( gocv.Scalar, gocv.Scalar ) {
	//Need at least 2 samples to take an average//
	if len( samples ) >= 2 {
		var lowerBound, upperBound ScalarAverage
		lowerBound.InitializeAverage( len( samples ) )
		upperBound.InitializeAverage( len( samples ) )
		lastSample := ToHsv( samples[ 0 ] )
		for i := 1; i < len( samples ); i += 1 {
			currentImage := ToHsv( samples[ i ] )
			lowerBoundSample, upperBoundSample := CalculateThresholds( lastSample, currentImage )
			lowerBound.AddScalarSample( lowerBoundSample )
			upperBound.AddScalarSample( upperBoundSample )
			lastSample = currentImage
		}
		return lowerBound.ConstructScalar(), upperBound.ConstructScalar()
	}
	return gocv.Scalar{}, gocv.Scalar{}
}

type ImageProcessor struct {
	detectionImageSize image.Point
	frame, maxSkinColorSampleFrames, numberOfSkinColorSampleFrames, skinSampleWaitFrames int
	foundSkinColor bool
	skinColorLowerBound, skinColorUpperBound gocv.Scalar
	skinColorSampleFrames []gocv.Mat
	classifier gocv.CascadeClassifier
}

func ( this *ImageProcessor ) InitializeImageProcessor( detectionImageSize image.Point, skinSampleWaitFrames int, maxSkinColorSampleFrames int, haarCascade string ) {
	this.detectionImageSize = detectionImageSize
	this.numberOfSkinColorSampleFrames = maxSkinColorSampleFrames
	this.maxSkinColorSampleFrames = this.numberOfSkinColorSampleFrames
	this.skinSampleWaitFrames = skinSampleWaitFrames
	this.classifier = gocv.NewCascadeClassifier()
	this.classifier.Load( haarCascade )
	fmt.Println( "Cascade loaded" )
	this.foundSkinColor = false
}

func ( this *ImageProcessor ) SampleSkinColor( image *gocv.Mat ) ( bool, gocv.Scalar, gocv.Scalar ) {
	if this.frame < this.skinSampleWaitFrames { 
		return false, gocv.Scalar{}, gocv.Scalar{}
	}
	if this.numberOfSkinColorSampleFrames >= 0 {
		this.skinColorSampleFrames = append( this.skinColorSampleFrames, image.Clone() )
		this.numberOfSkinColorSampleFrames -= 1
		return false, gocv.Scalar{}, gocv.Scalar{}
	}
	this.skinColorLowerBound, this.skinColorUpperBound = AverageSkinColor( this.skinColorSampleFrames )
	this.foundSkinColor = true
	return false, this.skinColorLowerBound, this.skinColorUpperBound
}

func ( this *ImageProcessor ) ResetSkinColor() {
	this.numberOfSkinColorSampleFrames = this.maxSkinColorSampleFrames
	this.foundSkinColor = false
}

func ( this *ImageProcessor ) ProcessImage( image *gocv.Mat ) {
	if this.foundSkinColor == false {
		this.SampleSkinColor( image )
	}
	this.frame += 1
}

func ( this *ImageProcessor ) CleanUp() {
	this.classifier.Close()
}

const TelloPortConstant = "8890"
const NumberOfTelloImageColorChannels = 3

func main() {
	telloCameraImageSize := image.Pt( 720, 960 )
	drone := tello.NewDriver( TelloPortConstant )
	window = gocv.NewWindow( "Demo" )
	ffmpegMediaStream := NewFfmpegStream( telloCameraImageSize )
	var imageProcessor ImageProcessor
	imageProcessor.InitializeImageProcessor( image.Pt( 90, 120 ), 10, 5, "HaarCascades/haarcascade_frontalface_default.xml" )
	robot := gobot.NewRobot( "tello",
		[]gobot.Connection{},
		[]gobot.Device{ drone },
		func() {
			RobotWork( drone, ffmpegMediaStream )
	} )
	robot.Start( false )
	// webcam, _ := gocv.VideoCaptureDevice( 0 )
	cameraMedia := gocv.NewMat()
	var diagnostic error
	for {
		cameraMedia, diagnostic = ReadTelloCameraImage( telloCameraImageSize, NumberOfTelloImageColorChannels, ffmpegMediaStream )	
		if diagnostic == nil {
			// webcam.Read( &cameraMedia )
			imageProcessor.ProcessImage( &cameraMedia )
			window.IMShow( cameraMedia )
			window.WaitKey( 1 )	
		}
	}
}

/*
	// result := MakeHandMask( gocv.Scalar{ Val1: 0, Val2: 26, Val3: 13, Val4: 255 }, 
		// gocv.Scalar{ Val1: 15, Val2: 204, Val3: 153, Val4: 255 }, cameraMedia )
	// result := MakeHandMask( gocv.Scalar{ Val1: 0, Val2: 20, Val3: 70, Val4: 255 }, 
			// gocv.Scalar{ Val1: 20, Val2: 255, Val3: 255, Val4: 255 }, cameraMedia )
	// result := MakeHandMask( gocv.Scalar{ Val1: 0, Val2: 26, Val3: 13, Val4: 255 }, 
			// gocv.Scalar{ Val1: 20, Val2: 255, Val3: 200, Val4: 255 }, cameraMedia )
	// result := MakeHandMask( lowerBound, upperBound, cameraMedia )
	// detectionImage := ResizeImage( result, detectionImageSize )
	// DetectFeatures( &classifier, detectionImage, func( instance image.Rectangle ) {
		// fmt.Println( "Found hand at ", instance )
	// } )``
	//Trying different ranges
*/

/*
	drone.On( tello.FlightDataEvent, func( data interface{} ) {
		// TODO: protect flight data from race condition
		flightData := data.( *tello.FlightData )
		fmt.Println( "battery power:", flightData.BatteryPercentage )
	} )

	drone.On( tello.WifiDataEvent, func( data interface{} ) {
		fmt.Println( "WifiDataEvent" )
		fmt.Println( data )
	} )

	drone.On( tello.LogEvent, func( data interface{} ) {
		fmt.Println( "Log Event" )
		fmt.Println( data )
	} )
*/
