package main

import ( 
	"fmt"
	"bufio"
	"os"
    "time"
    "gobot.io/x/gobot"
    "gobot.io/x/gobot/platforms/dji/tello"
)

func main() {
    drone := tello.NewDriver("8888")
    work := func() {
        drone.On( tello.ConnectedEvent, func( data interface{} ) {
            fmt.Println( "Connected" )
            gobot.Every( 100*time.Millisecond, func() {
            } )
        })
        drone.On( tello.FlightDataEvent, func( data interface{} ) {
            // TODO: protect flight data from race condition
            flightData := data.( *tello.FlightData )
            fmt.Println( "Battery power:", flightData.BatteryPercentage, "%" )
        } )
    }
    robot := gobot.NewRobot( "tello",
		[]gobot.Connection{},
		[]gobot.Device{ drone },
		work,
	)
	reader := bufio.NewReader( os.Stdin )
	robot.Start( false )
	for {
		fmt.Println( "-----" )
		reader.ReadString( '\n' )
	}
}
