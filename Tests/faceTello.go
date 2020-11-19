/*
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
	"os"
	"time"
	"image"
	"image/jpeg"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
	"golang.org/x/image/colornames"
	//"image/color"
)

const (
	frameSize = 960 * 720 * 3
)

func main() {
	drone := tello.NewDriver( "8890" )
	window := gocv.NewWindow( "Demo" )
	//camera := gocv.NewWindowDriver()
	classifier := gocv.NewCascadeClassifier()
	classifier.Load( "haarcascade_frontalface_default.xml" )
	fmt.Println( "Cascade loaded" )
	defer classifier.Close()
	ffmpeg := exec.Command( "ffmpeg", "-i", "pipe:0", "-pix_fmt", "bgr24", "-vcodec", "rawvideo",
		"-an", "-sn", "-s", "960x720", "-f", "rawvideo", "pipe:1" )
	ffmpegIn, _ := ffmpeg.StdinPipe()
	ffmpegOut, _ := ffmpeg.StdoutPipe()
	work := func() {

		if err := ffmpeg.Start(); err != nil {
			fmt.Println( err )
			return
		}
		//count:=0

		drone.On( tello.ConnectedEvent, func( data interface{} ) {
			fmt.Println( "Connected" )
			drone.StartVideo()
			drone.SetVideoEncoderRate( 4 )//tello.VideoBitRateAuto )
			drone.SetExposure( 0 )
			gobot.Every( 100*time.Millisecond, func() {
				drone.StartVideo()
			} )
		})

		drone.On( tello.VideoFrameEvent, func( data interface{} ) {
			fmt.Println( "Got video data" )
			pkt := data.([]byte)
			fmt.Println( "Frame size ", len( pkt ) )//, frameSize )
			if _, err := ffmpegIn.Write( pkt ); err != nil {
				fmt.Println( err )
			}
		} )

		/*drone.On( tello.FlightDataEvent, func( data interface{} ) {
			// TODO: protect flight data from race condition
			flightData := data.( *tello.FlightData )
			fmt.Println( "battery power:", flightData.BatteryPercentage )
		} )*/

		/*drone.On( tello.WifiDataEvent, func( data interface{} ) {
			fmt.Println( "WifiDataEvent" )
			fmt.Println( data )
		} )*/

		/*drone.On( tello.LogEvent, func( data interface{} ) {
			fmt.Println( "Log Event" )
			fmt.Println( data )
		} )*/

	}

	robot := gobot.NewRobot( "tello",
		[]gobot.Connection{},
		[]gobot.Device{ drone },
		work,
	)
	f, err := os.Create("test.jpg")
    if err != nil {
        fmt.Println(err)
        return
	}
	wroteImg := false
	robot.Start( false )
	for {
		fmt.Println( "TEST" )
		buf := make( []byte, frameSize )
		if _, err := io.ReadFull( ffmpegOut, buf ); err != nil {
			fmt.Println(err)
			continue
		}
		if wroteImg == false {
			i := image.NewRGBA( image.Rect( 0, 0, 720, 480 ) )
			i.Pix = buf
			if errr := jpeg.Encode( f, i, nil ); errr != nil {
				fmt.Println( errr )
			}
			f.Close()
			wroteImg = true
		}
		fmt.Println( "Buflen ", len( buf ), " framesize ", frameSize )
		fmt.Println( "Making image from bytes" )
		img, err := gocv.NewMatFromBytes( 720, 960, gocv.MatTypeCV8UC3, buf )
		if err != nil {
			fmt.Println( "ERROR MAKING IMAGE FROM BYTES" )
			log.Print( err )
			continue
		}
		if img.Empty() {
			fmt.Println( "No image data" )
			continue
		}
		fmt.Println( "Looking for rectangles" )
		imageRectangles := classifier.DetectMultiScale(img)
		
		for _, rect := range imageRectangles {
			log.Println( "found a face,", rect )
			gocv.Rectangle( &img, rect, colornames.Cadetblue, 3 )
		}
		fmt.Println( "Show image" )
		// ptr := img.DataPtrInt8()
		// fmt.Println( ptr )
		window.IMShow( img )
		window.WaitKey( 1 )
		//if window.WaitKey( 1 ) >= 0 {
			//	break
			//}
			//if count < 1000 {
				//
				//}
				//window.WaitKey( 1 )
				//count +=1
			}
}
