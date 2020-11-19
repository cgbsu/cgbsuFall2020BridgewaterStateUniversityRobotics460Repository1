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
	"image"
	"strconv"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
	"golang.org/x/image/colornames"
	//"image/color"
)

const (
	frameSize = 960 * 720 * 3
)


/***************************************************************************
* Tried to make a struct to hold these things, but couldent figure out the *
* reason the streams kept closing, I think it has something to do with *****
* reallocation but I couldent figure out where it was happening and I ******
* have bigger fish to fry. *************************************************
***************************************************************************/
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

func ReadTelloCameraImage( telloCameraImageSize image.Point, ffmpegMediaStream *FfmpegStream ) ( gocv.Mat, error ) {
	buffer := make( []byte, frameSize )
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

const TelloPortConstant = "8890"

func main() {
	telloCameraImageSize := image.Pt( 720, 960 )
	detectionImageSize := image.Pt( 90, 120 )
	drone := tello.NewDriver( TelloPortConstant )
	window := gocv.NewWindow( "Demo" )
	classifier := gocv.NewCascadeClassifier()
	classifier.Load( "haarcascade_frontalface_default.xml" )
	ffmpegMediaStream := NewFfmpegStream( telloCameraImageSize )
	fmt.Println( "Cascade loaded" )
	defer classifier.Close()

	robot := gobot.NewRobot( "tello",
		[]gobot.Connection{},
		[]gobot.Device{ drone },
		func() {
			RobotWork( drone, ffmpegMediaStream )
	} )
	robot.Start( false )
	for {
		cameraMedia, diagnostic := ReadTelloCameraImage( telloCameraImageSize, ffmpegMediaStream )
		if diagnostic == nil {
			detectionImage := ResizeImage( cameraMedia, detectionImageSize )
			DetectFeatures( &classifier, detectionImage, func( instance image.Rectangle ) {
				fmt.Println( "Found face at ", instance )
			} )
			window.IMShow( detectionImage )
			window.WaitKey( 1 )
		}
	}
}


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
