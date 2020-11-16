// +build example
package main

import (
	"gocv.io/x/gocv"
)

func main() {
	window := gocv.NewWindow("Hello")
	img := gocv.IMRead( "gopher.png", gocv.IMReadUnchanged )

	for {
		window.IMShow(img)
		window.WaitKey(1)
	}
}
