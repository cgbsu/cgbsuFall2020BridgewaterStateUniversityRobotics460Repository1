package main

import (
	"image"
	"gocv.io/x/gocv"
)

func RectangleToScalar( rectangle image.Rectangle ) gocv.Scalar {
	return gocv.Scalar{ 
		Val1: float64( rectangle.Min.X ), Val2: float64( rectangle.Min.Y ), 
		Val3: float64( rectangle.Max.X ), Val4: float64( rectangle.Max.Y ) }
}

func ScalarToRectangle( scalar gocv.Scalar ) image.Rectangle {
	return image.Rectangle{ Min: image.Point{ X: int( scalar.Val1 ), Y: int( scalar.Val2 ) }, 
			Max: image.Point{ X: int( scalar.Val3 ), Y: int( scalar.Val4 ) } }
}

func PointToVector3( point image.Point ) Vector3 {
	return Vector3{ x: float64( point.X ), y: float64( point.Y ), z: 0.0 }
}

func RectangleArea( rectangle image.Rectangle ) int {
	return rectangle.Size().X * rectangle.Size().Y
}

func PointArea( point image.Point ) int {
	return point.X * point.Y
}
