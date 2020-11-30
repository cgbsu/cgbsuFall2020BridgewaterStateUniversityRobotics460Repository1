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

func FindLargestFeature( classifier *gocv.CascadeClassifier, sample gocv.Mat ) ( image.Rectangle, int ) {
	largestFeatureBound := image.Rectangle{ Min: image.Point{ X: 0, Y: 0 }, Max: image.Point{ X: 0, Y: 0 } }
	largestFeatureBoundArea := 0
	numberOfFeatures := 0
	DetectFeatures( classifier, sample, func( instance image.Rectangle ) {
		numberOfFeatures += 1
		currentFeatureSize := instance.Size()
		currentFeatureArea := currentFeatureSize.X
		currentFeatureArea  *= currentFeatureSize.Y
		if largestFeatureBoundArea < currentFeatureArea {
			largestFeatureBoundArea = currentFeatureArea
			largestFeatureBound = instance
		}
	} )
	return largestFeatureBound, numberOfFeatures
}

/*
* The next few functions use:
* https://medium.com/@muehler.v/simple-hand-gesture-recognition-using-opencv-and-javascript-eb3d6ced28a0
* https://medium.com/@soffritti.pierfrancesco/handy-hands-detection-with-opencv-ac6e9fb3cec1
* https://www.codepasta.com/computer-vision/2019/04/26/background-segmentation-removal-with-opencv-take-2.html
* https://www.youtube.com/watch?v=v-XcmsYlzjA
* as a sources.
*/

// var window *gocv.Window
// var closeWindow bool
// var webcam *gocv.VideoCapture
/*func DebugShowImage( toShow *gocv.Mat, classifier *gocv.CascadeClassifier ) {
	if closeWindow == false {
		window.IMShow( *toShow )
		closeWindow = ( window.WaitKey( 1 ) != -1 )
		if closeWindow == true {
			webcam.Close()
			classifier.Close()
			window.Close()
		}
	}
}*/

const NumberOfFeatureSamplesConstant = 3

const DroneSpeedConstant = .5
//Impirically derived//
const FeatureSizeToimageSizeRatioConstant = .18 //Try: .1
const NumberOfSamplesToUpdateSkinColorParametersConstant = 10

type ImageProcessor struct {
	detectionImageSize, gestureDetectionImageSize image.Point
	frame, maxSkinColorSampleFrames, numberOfSkinColorSampleFrames, skinSampleWaitFrames, samplesDispached int
	foundSkinColor, initilizedWaitTime bool
	skinColorLowerBound, skinColorUpperBound gocv.Scalar
	skinColorSampleFrames []gocv.Mat
	faceClassifier, gestureClassifier gocv.CascadeClassifier
	faceFeatureRectangle, handFeatureRectangle ScalarAverage
	timeFoundHandFeature time.Time
	lastHandBoundingRectangle image.Rectangle
	fingerCount Average
}

func ( this *ImageProcessor ) InitializeImageProcessor( detectionImageSize, gestureDetectionImageSize image.Point, skinSampleWaitFrames int, 
			maxSkinColorSampleFrames int, faceHaarCascade, gestureHaarCascade string ) {
	this.fingerCount.InitializeAverage( 10 )
	this.detectionImageSize = detectionImageSize
	this.gestureDetectionImageSize = gestureDetectionImageSize
	this.numberOfSkinColorSampleFrames = maxSkinColorSampleFrames
	this.maxSkinColorSampleFrames = this.numberOfSkinColorSampleFrames
	this.skinSampleWaitFrames = skinSampleWaitFrames
	this.faceClassifier = gocv.NewCascadeClassifier()
	this.gestureClassifier = gocv.NewCascadeClassifier()
	this.faceClassifier.Load( faceHaarCascade )
	this.gestureClassifier.Load( gestureHaarCascade )
	fmt.Println( "Cascades loaded" )
	this.foundSkinColor = false
	this.faceFeatureRectangle.InitializeAverage( NumberOfFeatureSamplesConstant )
	this.handFeatureRectangle.InitializeAverage( NumberOfFeatureSamplesConstant )
}

func ( this *ImageProcessor ) SampleSkinColor( image *gocv.Mat ) ( bool, gocv.Scalar, gocv.Scalar ) {
	if this.frame < this.skinSampleWaitFrames { 
		return false, gocv.Scalar{}, gocv.Scalar{}
	}
	this.skinColorSampleFrames = append( this.skinColorSampleFrames, image.Clone() )
	//Such bad code writing, such bad code writing such bad code writing suchbadcodewriting suchbadcodewriting suchbadcodewritingsuchbadcodewriting//
	// if this.foundSkinColor == false {
		// test := ToHsv( *image )
		// DebugShowImage( &test, &this.faceClassifier )
	// }
	this.numberOfSkinColorSampleFrames -= 1
	if this.numberOfSkinColorSampleFrames >= 0 {
		return false, gocv.Scalar{}, gocv.Scalar{}
	}
	this.skinColorLowerBound, this.skinColorUpperBound = AverageSkinColor( this.skinColorSampleFrames )
	this.maxSkinColorSampleFrames = NumberOfSamplesToUpdateSkinColorParametersConstant
	this.numberOfSkinColorSampleFrames = this.maxSkinColorSampleFrames
	// for i, _ := range this.skinColorSampleFrames {
		// this.skinColorSampleFrames[ i ].Close()
	// }
	this.skinColorSampleFrames = []gocv.Mat{}
	this.foundSkinColor = true
	return true, this.skinColorLowerBound, this.skinColorUpperBound
}

func ( this *ImageProcessor ) ResetSkinColor() {
	this.numberOfSkinColorSampleFrames = this.maxSkinColorSampleFrames
	this.foundSkinColor = false
}

func ( this *ImageProcessor ) MakeSkinMask( toMask gocv.Mat ) gocv.Mat {
	skinMask := gocv.NewMat()
	gocv.InRangeWithScalar( ToHsv( toMask ), this.skinColorLowerBound, this.skinColorUpperBound, &skinMask )
	structuringElement := gocv.GetStructuringElement( gocv.MorphEllipse, image.Point{ X: 3, Y: 3 } ) 
	gocv.MorphologyEx( skinMask, &skinMask, gocv.MorphOpen, structuringElement )
	gocv.Dilate( skinMask, &skinMask, gocv.NewMat() )
	return skinMask
}

func ( this *ImageProcessor ) SampleFeature( classifier *gocv.CascadeClassifier, 
			sample gocv.Mat, featureRectangle *ScalarAverage, detectionImageSize image.Point ) ( bool, bool, gocv.Scalar, gocv.Mat, int ) {
	detectionImage := ResizeImage( sample, detectionImageSize )
	if featureRectangle.AtDesiredSampleCount() == false {
		samplingFailure := false
		largestFeatureBound, numberOfFeatures := FindLargestFeature( classifier, detectionImage )
		// DebugShowImage( &detectionImage, &this.classifier )
		if numberOfFeatures > 0 {
			featureRectangle.AddScalarSample( RectangleToScalar( largestFeatureBound ) )
		} else {
			featureRectangle.Clear()
			samplingFailure = true
		}
		return false, samplingFailure, RectangleToScalar( largestFeatureBound ), detectionImage, numberOfFeatures
	}
	//This mean it throws away a frame for detection.//
	averagedScalar := featureRectangle.ConstructScalar()
	featureRectangle.Clear()
	gocv.Rectangle( &detectionImage, ScalarToRectangle( averagedScalar ), colornames.Cadetblue, 3 )
	// DebugShowImage( &detectionImage, &this.classifier )
	return true, false, averagedScalar, gocv.NewMat(), 0
}

func ( this *ImageProcessor ) RecommendMovementVector( featureScalar gocv.Scalar ) ( Vector3, bool ) {
	featureRectangle := ScalarToRectangle( featureScalar )
	imageRatio := float64( RectangleArea( featureRectangle ) ) / float64( PointArea( this.detectionImageSize ) )
	if imageRatio < FeatureSizeToimageSizeRatioConstant {
		featureCenter := PointToVector3( featureRectangle.Size() )
		featureCenter.MultiplyByScalar( .5, false )
		featureCenter.AddVector( PointToVector3( featureRectangle.Min ), false )
		moveVector := PointToVector3( this.detectionImageSize )
		moveVector.MultiplyByScalar( .5, false )
		moveVector = featureCenter.AddVector( moveVector.Negate( true ), false )
		moveVector = moveVector.Normalize()
		moveVector.MultiplyByScalar( DroneSpeedConstant, false )
		return Vector3{ DroneSpeedConstant, moveVector.x, -moveVector.y }, false
	}
	return Vector3{ x: 0.0, y: 0.0, z: 0.0 }, true
}

func ( this *ImageProcessor ) BlockFace( featureScalar gocv.Scalar, image *gocv.Mat ) {
	gocv.Rectangle( image, ScaleRectangleToFitImage( 
		featureScalar, this.detectionImageSize, MatSize( image ) ), colornames.Black, -1 )
}

func ( this *ImageProcessor ) ResampleSkinColor( featureRectangle image.Rectangle, imageWithSkinFeature *gocv.Mat ) {
	faceSample := imageWithSkinFeature.Region( featureRectangle )
	this.SampleSkinColor( &faceSample )
}

func ( this *ImageProcessor ) PrapareImageForHandDetection( inputImage gocv.Mat ) ( bool, bool, gocv.Scalar, gocv.Mat ) {
	didNotAddSample, featureDetectFailure, featureScalar, resizedImage, _ := this.SampleFeature( &this.faceClassifier, inputImage, &this.faceFeatureRectangle, this.detectionImageSize )
	skinMaskImage := this.MakeSkinMask( inputImage.Clone() )
	//This part helps improve skin color parameters//
	if featureDetectFailure == false && didNotAddSample == false && IsZeroScalar( featureScalar ) == false {
		this.ResampleSkinColor( ScalarToRectangle( featureScalar ), &resizedImage )
		this.BlockFace( featureScalar, &skinMaskImage )
	}
	return didNotAddSample, featureDetectFailure, featureScalar, skinMaskImage
	// return false, false, gocv.NewScalar( 0.0, 0.0, 0.0, 0.0 ), gocv.NewMat()
}


// var testWindow2 *gocv.Window

func ( this *ImageProcessor ) DetectGestureExperimental( skinMask, inputImage gocv.Mat, debug *gocv.Window ) ( int, image.Rectangle ) {
	didNotAddSample, featureDetectFailure, featureScalar, _, _ := this.SampleFeature( &this.gestureClassifier, inputImage, &this.handFeatureRectangle, this.gestureDetectionImageSize )
	if didNotAddSample == true && featureDetectFailure == false && IsZeroScalar( featureScalar ) == false {
		//Make a rectangle bigger than a fist so the robot can look inside it for features.//
		// this.ResampleSkinColor( ScalarToRectangle( featureScalar ), &resizedImage )
		// this.lastHandBoundingRectangle = ScaleRectangleToFitImage( featureScalar, this.detectionImageSize, image.Pt( 480, 640 ) )//MatSize( &inputImage ) )
		// gocv.Rectangle( &inputImage, this.lastHandBoundingRectangle, colornames.Cadetblue, 3 )
		
		yFactor := ( featureScalar.Val4 - featureScalar.Val2 )
		featureScalar.Val2 -= yFactor * 1.5
		featureScalar.Val4 -= yFactor
		xFactor := ( featureScalar.Val3 - featureScalar.Val1 ) / 4
		featureScalar.Val1 -= xFactor
		featureScalar.Val3 += xFactor
		
		skinMaskSize := MatSize( &skinMask )
		test := RectangleToScalar( ScaleRectangleToFitImage( featureScalar, this.gestureDetectionImageSize, skinMaskSize ) )
		test = ToBoundedScalar( test, skinMaskSize ) //MatSize( &inputImage ) )
		if test.Val1 != test.Val3 && test.Val2 != test.Val4 {
			if test.Val3 != 0 && test.Val4 != 0 && test.Val3 > test.Val1 && test.Val4 > test.Val2 {
				// fmt.Println( test, ", ", skinMaskSize, " ", detectImage.Size() )
				region := MatRegion( skinMask, test )
				if region.Empty() == false {
					// gocv.Rectangle( &skinMask, ScalarToRectangle( test ), colornames.Black, 3 )
					// debug.IMShow( region )
					// testWindow2.IMShow( detectImage )
					// if testWindow2.WaitKey( 20 ) != -1 {
						// testWindow2.Close()
					// }
					this.timeFoundHandFeature = time.Now()
					this.initilizedWaitTime = true
					this.fingerCount.Clear()
					this.lastHandBoundingRectangle = ScalarToRectangle( test )
					fmt.Println( "Found hand" )
					return -1, ScalarToRectangle( test )
				} else {
					region.Close()
				}
			}
		} else {
			fmt.Println( "False positive" )
		}
	}
	debug.IMShow( skinMask )
	if this.initilizedWaitTime == true && time.Since( this.timeFoundHandFeature ) < 2 * time.Second {
		region := MatRegion( skinMask, RectangleToScalar( this.lastHandBoundingRectangle ) )
		if region.Empty() == false {
			count, handBound, _ := FingerCount( region, debug )
			this.fingerCount.AddSample( count )
			if this.fingerCount.AtDesiredSampleCount() == true {
				// fmt.Println( "I SEE ", this.fingerCount.CalculateAverage(), " FINGERS" )
				return count, handBound
			}
		}
	}
	return 0, image.Rect( 0, 0, 0, 0 )
}

func ( this *ImageProcessor ) DetectGesture( inputImage gocv.Mat ) bool {
	//didNotAddSample, featureDetectFailure, featureScalar, resizedImage, numberOfHands 
	didNotAddSample, _, averagedScalar, _, _ := this.SampleFeature( &this.gestureClassifier, inputImage, &this.handFeatureRectangle, this.gestureDetectionImageSize )
	if IsZeroScalar( averagedScalar ) == false {
		return didNotAddSample
	}
	return false
}

func ( this *ImageProcessor ) ProcessImage( media *gocv.Mat, debug *gocv.Window ) ( int, image.Rectangle, bool, gocv.Scalar ) {
	if this.foundSkinColor == false {
		fmt.Println( "Skin color frames ", this.numberOfSkinColorSampleFrames )
		this.SampleSkinColor( media )
		if this.foundSkinColor == true {
			fmt.Println( this.skinColorLowerBound )
			fmt.Println( this.skinColorUpperBound )
		}
	} else {
		// doneSampling, featureDetectFailure, faceScalar 
		doneSampling, _, faceScalar, skinMask := this.PrapareImageForHandDetection( *media )
		// debug.IMShow( skinMask )
		count, handBound := this.DetectGestureExperimental( skinMask, *media, debug )
		return count, handBound, doneSampling, faceScalar
		// DebugShowImage( &skinMaskImage, &this.faceClassifier )
	}
	this.frame += 1
	return 0, image.Rect( 0, 0, 0, 0 ), false, gocv.NewScalar( 0.0, 0.0, 0.0, 0.0 )
}
	
func ( this *ImageProcessor ) CleanUp() {
	this.faceClassifier.Close()
	this.gestureClassifier.Close()
}

/*************************
* Note for drone vectors *
* X forward **************
* Y right ****************
* Z up *******************
*************************/

const TelloPortConstant = "8890"
const NumberOfTelloImageColorChannels = 3


func TestMain0() {
	// closeWindow = false
	// telloCameraImageSize := image.Pt( 720, 960 )
	featureDetectImageSize := image.Pt( 90 * 2, 120 * 2 )
	var imageProcessor ImageProcessor
	window := gocv.NewWindow( "Demo" )
	imageProcessor.InitializeImageProcessor( featureDetectImageSize, image.Pt( 720, 960 ), 10, 10, "HaarCascades/haarcascade_frontalface_default.xml", 
		"HaarCascades/aGest.xml" )
	webcam, _ := gocv.VideoCaptureDevice( 0 )
	cameraMedia := gocv.NewMat()
	for {
		webcam.Read( &cameraMedia )
		// DebugShowImage( &cameraMedia, nil )
		window.IMShow( cameraMedia )
		if window.WaitKey( 1 ) != -1 {
			break
		}
		imageProcessor.ProcessImage( &cameraMedia, window )
	}
	defer func() {
		webcam.Close()
		imageProcessor.CleanUp()
		window.Close()
	}()
}

func TestMain1() {
	// closeWindow = false
	telloCameraImageSize := image.Pt( 720, 960 )
	featureDetectImageSize := image.Pt( 90 * 2, 120 * 2 )
	drone := tello.NewDriver( TelloPortConstant )
	window := gocv.NewWindow( "Demo" )
	ffmpegMediaStream := NewFfmpegStream( telloCameraImageSize )
	var imageProcessor ImageProcessor
	// image.Pt( 320, 240 )
	/*image.Pt( 320 / 2, 240 / 2 )*/
	imageProcessor.InitializeImageProcessor( featureDetectImageSize, featureDetectImageSize, 10, 5, "HaarCascades/haarcascade_frontalface_default.xml", 
			"HaarCascades/aGest.xml" )
	robot := gobot.NewRobot( "tello",
		[]gobot.Connection{},
		[]gobot.Device{ drone },
		func() {
			RobotWork( drone, ffmpegMediaStream )
	} )
	robot.Start( false )
	fmt.Println( "Takeoff" )
	drone.TakeOff()
	fmt.Println( "Waiting" )
	time.Sleep( 3 * time.Second )
	fmt.Println( "Waiting" )
	defer func() {
		drone.Land()
		time.Sleep( 5 * time.Second )
		window.Close()
		imageProcessor.CleanUp()
	}()
	previousMoveVector := Vector3{ x: 0.0, y: 0.0, z: 0.0 }
	cameraMedia := gocv.NewMat()
	var diagnostic error
	frames := 0
	framesSinceDetect := 0
	// gotClose := false
	detectedHand := make( chan bool )
	for {
		cameraMedia, diagnostic = ReadTelloCameraImage( telloCameraImageSize, NumberOfTelloImageColorChannels, ffmpegMediaStream )	
		if diagnostic == nil {
			secondImage := cameraMedia.Clone()
			cameraMedia = cameraMedia.Clone()
			go func() { detectedHand <- imageProcessor.DetectGesture( secondImage ) }()
			doneSampling, _, featureScalar, _, _ := imageProcessor.SampleFeature( &imageProcessor.faceClassifier, cameraMedia, &imageProcessor.faceFeatureRectangle, imageProcessor.detectionImageSize )
			// featureRectangle := ScalarToRectangle( featureScalar )
			frames += 1
			framesSinceDetect += 1
			if doneSampling == true {
				previousMoveVector, _ = imageProcessor.RecommendMovementVector( featureScalar )
				drone.SetVector( float32( previousMoveVector.x ), float32( previousMoveVector.y ), float32( previousMoveVector.z ), 0.0 )
				framesSinceDetect = 0
				fmt.Println( "Move ", previousMoveVector.x, ", ", previousMoveVector.y, ", ", previousMoveVector.z )
			} else {//if samplingFailure == true || framesSinceDetect > 10 {
				// fmt.Println( "Stop sample failure ", samplingFailure, " got close ", gotClose )
				if framesSinceDetect > 100 {
					drone.SetVector( 0.0, 0.0, 0.0, float32( previousMoveVector.y ) )
				} else {
					drone.SetVector( 0.0, 0.0, 0.0, 0.0 )
				}
			}

			if ( <- detectedHand ) == true {
				fmt.Println( "TIME TO LAND" )
				drone.Land()
				break
			}
			window.IMShow( cameraMedia )
			window.WaitKey( 1 )
			// DebugShowImage( &cameraMedia, &imageProcessor.faceClassifier )
		} else {
			fmt.Println( "Error reading from camera" )
			drone.Land()
			break
		}
	}
	// drone.Land()
}
func TestMain2() {
	var hand Hand
	hand.InitializeHand( "CaffeModels/pose_deploy.prototxt", "CaffeModels/pose_iter_102000.caffemodel" )
	window := gocv.NewWindow( "Demo" )
	webcam, _ := gocv.VideoCaptureDevice( 0 )
	cameraMedia := gocv.NewMat()
	// for {
		webcam.Read( &cameraMedia )
		hand.MakeHandDataFromImage( cameraMedia, window )
	// }
	webcam.Close()
	window.Close()
	// TestMain1()
}

func TestMain3() {
	var imageProcessor ImageProcessor
	imageProcessor.InitializeImageProcessor( image.Pt( 90, 120 ), image.Pt( 240, 320 ), 10, 100, "HaarCascades/haarcascade_frontalface_default.xml", 
		"HaarCascades/aGest.xml" )
	window := gocv.NewWindow( "Demo" )
	// testWindow2 = gocv.NewWindow( "Demo2" )
	webcam, err := gocv.VideoCaptureDevice( 0 )
	cameraMedia := gocv.NewMat()
	clone := gocv.NewMat()
	fmt.Println( "GOCV: ", err )
	lastRectangle := image.Rect( 0, 0, 10, 10 )
	for {
		webcam.Read( &cameraMedia )
		cameraMedia.CopyTo( &clone )

		// count, rectangle, _ := 

		count, handRectangle, _, _ := imageProcessor.ProcessImage( &clone, window )
		if count == -1 {
			lastRectangle = handRectangle
			gocv.Rectangle( &cameraMedia, handRectangle, colornames.Cadetblue, 3 )
		}
		if count != 0 {
			fmt.Println( "Found ", count, "fingers" )
			gocv.Rectangle( &cameraMedia, lastRectangle, colornames.Cadetblue, 3 )
		}

		// count, rectangle, img := FingerCount( cameraMedia, window )
		// window.IMShow( img )
		// fmt.Println( count, rectangle )
		// window.IMShow( cameraMedia )
		if window.WaitKey( 1 ) != -1 {
			break
		}
		// clone.Close()
	}
	defer func() {
		webcam.Close()
		imageProcessor.CleanUp()
		window.Close()
	}()
}

func TestMain4() {
	// closeWindow = false
	telloCameraImageSize := image.Pt( 720, 960 )
	featureDetectImageSize := image.Pt( 90 * 2, 120 * 2 )
	drone := tello.NewDriver( TelloPortConstant )
	window := gocv.NewWindow( "Demo" )
	ffmpegMediaStream := NewFfmpegStream( telloCameraImageSize )
	var imageProcessor ImageProcessor
	// image.Pt( 320, 240 )
	/*image.Pt( 320 / 2, 240 / 2 )*/
	imageProcessor.InitializeImageProcessor( featureDetectImageSize, featureDetectImageSize, 100, 5, "HaarCascades/haarcascade_frontalface_default.xml", 
			"HaarCascades/aGest.xml" )
	robot := gobot.NewRobot( "tello",
		[]gobot.Connection{},
		[]gobot.Device{ drone },
		func() {
			RobotWork( drone, ffmpegMediaStream )
	} )
	robot.Start( false )
	fmt.Println( "Takeoff" )
	drone.TakeOff()
	fmt.Println( "Waiting" )
	time.Sleep( 3 * time.Second )
	fmt.Println( "Waiting" )
	defer func() {
		drone.Land()
		time.Sleep( 5 * time.Second )
		window.Close()
		imageProcessor.CleanUp()
	}()
	previousMoveVector := Vector3{ x: 0.0, y: 0.0, z: 0.0 }
	cameraMedia := gocv.NewMat()
	var diagnostic error
	frames := 0
	framesSinceDetect := 0
	// gotClose := false
	for {
		cameraMedia, diagnostic = ReadTelloCameraImage( telloCameraImageSize, NumberOfTelloImageColorChannels, ffmpegMediaStream )	
		if diagnostic == nil {
			fingerCount, _, doneSampling, featureScalar := imageProcessor.ProcessImage( &cameraMedia, window )//imageProcessor.SampleFeature( &imageProcessor.faceClassifier, cameraMedia, &imageProcessor.faceFeatureRectangle, imageProcessor.detectionImageSize )
			// featureRectangle := ScalarToRectangle( featureScalar )
			frames += 1
			framesSinceDetect += 1
			if doneSampling == true {
				previousMoveVector, _ = imageProcessor.RecommendMovementVector( featureScalar )
				drone.SetVector( float32( previousMoveVector.x ), float32( previousMoveVector.y ), float32( previousMoveVector.z ), 0.0 )
				framesSinceDetect = 0
				fmt.Println( "Move ", previousMoveVector.x, ", ", previousMoveVector.y, ", ", previousMoveVector.z )
			} else {//if samplingFailure == true || framesSinceDetect > 10 {
				// fmt.Println( "Stop sample failure ", samplingFailure, " got close ", gotClose )
				if framesSinceDetect > 100 {
					drone.SetVector( 0.0, 0.0, 0.0, float32( previousMoveVector.y ) )
				} else {
					drone.SetVector( 0.0, 0.0, 0.0, 0.0 )
				}
			}

			if fingerCount == 4 {
				fmt.Println( "TIME TO LAND" )
				drone.Land()
				break
			}
			window.IMShow( cameraMedia )
			window.WaitKey( 1 )
			// DebugShowImage( &cameraMedia, &imageProcessor.faceClassifier )
		} else {
			fmt.Println( "Error reading from camera" )
			drone.Land()
			break
		}
	}
	// drone.Land()
}


func main() {
	TestMain3()
}
