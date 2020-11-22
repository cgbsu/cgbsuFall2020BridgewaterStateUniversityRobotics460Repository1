package main

import (
	"fmt"
	"time"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
)

const TelloPortConstant = "8890"

func main() {
	drone := tello.NewDriver( TelloPortConstant )
	robot := gobot.NewRobot( "tello",
		[]gobot.Connection{},
		[]gobot.Device{ drone },
		func() {
			drone.On( tello.ConnectedEvent, func( data interface{} ) { 
				fmt.Println( "Connected" )
			} )
	} )
	robot.Start( false )
	time.Sleep( 5 * time.Second )
	drone.Land()
}
