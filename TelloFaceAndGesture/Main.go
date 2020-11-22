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
	// "math"
	// "bufio"
	// "os"
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
var closeWindow bool
var webcam *gocv.VideoCapture
func DebugShowImage( toShow *gocv.Mat, classifier *gocv.CascadeClassifier ) {
	if closeWindow == false {
		window.IMShow( *toShow )
		closeWindow = ( window.WaitKey( 1 ) != -1 )
		if closeWindow == true {
			webcam.Close()
			classifier.Close()
			window.Close()
		}
	}
}

const NumberOfFeatureSamplesConstant = 3
/*const NoResultConstant = 0
const StartOverConstant = 1
const GotResultConstant = 2

type FeatureData {
	gocv.Scalar
}*/


const DroneSpeedConstant = .5
//Impirically derived//
const FeatureSizeToimageSizeRatioConstant = .18

type ImageProcessor struct {
	detectionImageSize image.Point
	frame, maxSkinColorSampleFrames, numberOfSkinColorSampleFrames, skinSampleWaitFrames, samplesDispached int
	foundSkinColor bool
	skinColorLowerBound, skinColorUpperBound gocv.Scalar
	skinColorSampleFrames []gocv.Mat
	classifier gocv.CascadeClassifier
	featureRectangle ScalarAverage
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
	this.featureRectangle.InitializeAverage( NumberOfFeatureSamplesConstant )
}

func ( this *ImageProcessor ) SampleSkinColor( image *gocv.Mat ) ( bool, gocv.Scalar, gocv.Scalar ) {
	if this.frame < this.skinSampleWaitFrames { 
		return false, gocv.Scalar{}, gocv.Scalar{}
	}
	if this.numberOfSkinColorSampleFrames >= 0 {
		this.skinColorSampleFrames = append( this.skinColorSampleFrames, image.Clone() )
		test := ToHsv( *image )
		DebugShowImage( &test, &this.classifier )
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

func ( this *ImageProcessor ) MakeSkinMask( toMask *gocv.Mat ) gocv.Mat {
	// binaryImage := gocv.NewMat()
	skinMask := gocv.NewMat()
	gocv.InRangeWithScalar( ToHsv( toMask.Clone() ), this.skinColorLowerBound, this.skinColorUpperBound, &skinMask )//&binaryImage )
	// structuringElement := gocv.GetStructuringElement( gocv.MorphEllipse, image.Point{ X: 3, Y: 3 } ) 
	// morphed := binaryImage.Clone()
	// gocv.MorphologyEx( binaryImage, &morphed, gocv.MorphOpen, structuringElement )
	// skinMask := gocv.NewMat()
	// gocv.Dilate( morphed, &skinMask, gocv.NewMat() )
	return skinMask
}
/*
func ( this *ImageProcessor ) WaitForFeatureThreads( sample *gocv.Mat ) ( bool, gocv.Scalar ) {
}
*/

func ( this *ImageProcessor ) SampleFeature( sample *gocv.Mat ) ( bool, bool, gocv.Scalar ) {
	detectionImage := ResizeImage( *sample, this.detectionImageSize )
	if this.featureRectangle.AtDesiredSampleCount() == false {
		detectionImage := ResizeImage( *sample, this.detectionImageSize )
		DebugShowImage( &detectionImage, &this.classifier )
		largestFeatureBound := image.Rectangle{ Min: image.Point{ X: 0, Y: 0 }, Max: image.Point{ X: 0, Y: 0 } }
		largestFeatureBoundArea := 0
		numberOfFeatures := 0
		samplingFailure := false
		DetectFeatures( &this.classifier, detectionImage, func( instance image.Rectangle ) {
			numberOfFeatures += 1
			currentFeatureSize := instance.Size()
			currentFeatureArea := currentFeatureSize.X
			currentFeatureArea  *= currentFeatureSize.Y
			if largestFeatureBoundArea < currentFeatureArea {
				largestFeatureBoundArea = currentFeatureArea
				largestFeatureBound = instance
			}
		} )
		if numberOfFeatures > 0 {
			this.featureRectangle.AddScalarSample( RectangleToScalar( largestFeatureBound ) )
			samplingFailure = true
		} else {
			this.featureRectangle.Clear()
		}
		return false, samplingFailure, RectangleToScalar( largestFeatureBound )
	}
	//This mean it throws away a frame for detection.//
	averagedScalar := this.featureRectangle.ConstructScalar()
	this.featureRectangle.Clear()
	gocv.Rectangle( &detectionImage, ScalarToRectangle( averagedScalar ), colornames.Cadetblue, 3 )
	DebugShowImage( &detectionImage, &this.classifier )
	return true, false, averagedScalar
}

					/*featureCenter := image.Point{ X: ( featureRectangle.Min.X + ( featureRectangle.Size().X / 2 ) ), 
						Y: ( featureRectangle.Min.Y + ( featureRectangle.Size().Y / 2 ) ) }
					horizontalVelocity := ( featureDetectImageSize.X / 2 ) - featureCenter.X
					verticalVelocity := ( featureDetectImageSize.Y / 2 ) - featureCenter.Y
					magnitude := math.Sqrt( ( horizontalVelocity * horizontalVelocity ) + ( verticalVelocity * verticalVelocity ) )
					horizontalVelocity = ( horizontalVelocity / magnitude ) * SpeedConstant
					verticalVelocity = ( verticalVelocity / magnitude ) * SpeedConstant
					drone.SetVector( SpeedConstant, horizontalVelocity, verticalVelocity, 0.0 )*/


func ( this *ImageProcessor ) RecommendMovementVector( featureScalar gocv.Scalar ) Vector3 {
	featureRectangle := ScalarToRectangle( featureScalar )
	imageRatio := float64( RectangleArea( featureRectangle ) ) / float64( PointArea( this.detectionImageSize ) )
	if imageRatio < FeatureSizeToimageSizeRatioConstant {
		// featureCenter := Vector3{ x: ( featureRectangle.Min.X + ( featureRectangle.Size().X / 2 ) ), 
			// Y: ( featureRectangle.Min.Y + ( featureRectangle.Size().Y / 2 ) ) }
		featureCenter := PointToVector3( featureRectangle.Size() )
		featureCenter.MultiplyByScalar( .5, false )
		featureCenter.AddVector( PointToVector3( featureRectangle.Min ), false )
		// horizontalVelocity := ( featureDetectImageSize.X / 2 ) - featureCenter.X
		// verticalVelocity := ( featureDetectImageSize.Y / 2 ) - featureCenter.Y
		moveVector := PointToVector3( this.detectionImageSize )
		moveVector.MultiplyByScalar( .5, false )
		moveVector = featureCenter.AddVector( moveVector.Negate( true ), false )
		moveVector = moveVector.Normalize()
		moveVector.MultiplyByScalar( DroneSpeedConstant, false )
		return Vector3{ DroneSpeedConstant, moveVector.x, -moveVector.y }
	}
	return Vector3{ x: 0.0, y: 0.0, z: 0.0 }
}

func ( this *ImageProcessor ) ProcessImage( image *gocv.Mat ) {
	if this.foundSkinColor == false {
		this.SampleSkinColor( image )
		if this.foundSkinColor == true {
			fmt.Println( this.skinColorLowerBound )
			fmt.Println( this.skinColorUpperBound )
		}
	} else {
		img := this.MakeSkinMask( image )
		DebugShowImage( &img, &this.classifier )
	}
	// doneSampling, featureRectangle := this.SampleFeature( image )
	// fmt.Println( doneSampling, ", ", featureRectangle )
	this.frame += 1
}

func ( this *ImageProcessor ) CleanUp() {
	this.classifier.Close()
}


	//X forward
	//Y right
	//Z up

const TelloPortConstant = "8890"
const NumberOfTelloImageColorChannels = 3


func TestMain0() {
	closeWindow = false
	// telloCameraImageSize := image.Pt( 720, 960 )
	featureDetectImageSize := image.Pt( 90, 120 )
	window = gocv.NewWindow( "Demo" )
	var imageProcessor ImageProcessor
	imageProcessor.InitializeImageProcessor( featureDetectImageSize, 10, 100, "HaarCascades/haarcascade_frontalface_default.xml" )
	webcam, _ = gocv.VideoCaptureDevice( 0 )
	cameraMedia := gocv.NewMat()
	for {
		webcam.Read( &cameraMedia )
		imageProcessor.ProcessImage( &cameraMedia )
	}
	defer webcam.Close()
}

func TestMain1() {
	closeWindow = false
	telloCameraImageSize := image.Pt( 720, 960 )
	featureDetectImageSize := image.Pt( 90, 120 )
	drone := tello.NewDriver( TelloPortConstant )
	window = gocv.NewWindow( "Demo" )
	ffmpegMediaStream := NewFfmpegStream( telloCameraImageSize )
	var imageProcessor ImageProcessor
	imageProcessor.InitializeImageProcessor( featureDetectImageSize, 10, 5, "HaarCascades/haarcascade_frontalface_default.xml" )
	robot := gobot.NewRobot( "tello",
		[]gobot.Connection{},
		[]gobot.Device{ drone },
		func() {
			RobotWork( drone, ffmpegMediaStream )
	} )
	robot.Start( false )
	fmt.Println( "Takeoff" )
	drone.TakeOff()
	fmt.Println( "1" )
	time.Sleep( 3 * time.Second )
	fmt.Println( "2" )
	// defer func() {
		// time.Sleep( 5 * time.Second )
		// drone.Land()
	// }()
	previousMoveVector := Vector3{ x: 0.0, y: 0.0, z: 0.0 }
	cameraMedia := gocv.NewMat()
	var diagnostic error
	frames := 0
	for {
		cameraMedia, diagnostic = ReadTelloCameraImage( telloCameraImageSize, NumberOfTelloImageColorChannels, ffmpegMediaStream )	
		if diagnostic == nil {
			doneSampling, samplingFailure, featureScalar := imageProcessor.SampleFeature( &cameraMedia )
			// featureRectangle := ScalarToRectangle( featureScalar )
			frames += 1
			if doneSampling == true {
				previousMoveVector := imageProcessor.RecommendMovementVector( featureScalar )
				drone.SetVector( float32( previousMoveVector.x ), float32( previousMoveVector.y ), float32( previousMoveVector.z ), 0.0 )
				fmt.Println( "Move ", previousMoveVector.x, ", ", previousMoveVector.y, ", ", previousMoveVector.z )
			} else if samplingFailure == true {
				fmt.Println( "Stop" )
				drone.SetVector( 0.0, 0.0, 0.0, float32( previousMoveVector.y ) )
			}
			window.IMShow( cameraMedia )
			window.WaitKey( 1 )	
		}
	}
	// drone.Land()
}

func main() {
	TestMain1()
}
/*				imageRatio := float64( RectangleArea( featureRectangle ) ) / float64( PointArea( featureDetectImageSize ) )
				fmt.Println( "Image ratio ", imageRatio )
				if imageRatio < FeatureSizeToimageSizeRatioConstant {
					drone.SetVector( DroneSpeedConstant, 0.0, 0.0, 0.0 )
					fmt.Println( "Forward" )
				}
			} else if ( frames % 10 ) == 0 { //imageProcessor.featureRectangle.numberOfSamples == 0 {
				fmt.Println( "Stop" )
				drone.SetVector( 0.0, 0.0, 0.0, 0.0 )
			}*/

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
