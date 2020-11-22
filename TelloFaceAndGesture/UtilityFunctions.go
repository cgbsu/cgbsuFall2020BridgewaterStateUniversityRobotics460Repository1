package main

import (
	"image"
	"gocv.io/x/gocv"
)

func IsZeroScalar( scalar gocv.Scalar ) bool {
	return ( scalar.Val1 == 0 ) && ( scalar.Val2 == scalar.Val1 && ( scalar.Val2 == scalar.Val3 && ( scalar.Val3 == scalar.Val4 ) ) )
}

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

func MatSize( mat *gocv.Mat ) image.Point {
	return image.Point{ mat.Size()[ 0 ], mat.Size()[ 1 ] }
}

func ScaleRectangleToFitImage( rectangle gocv.Scalar, from, to image.Point ) image.Rectangle {
	xFactor := float64( to.X ) / float64( from.X )
	yFactor := float64( to.Y ) / float64( from.Y )
	rectangle.Val1 *= xFactor
	rectangle.Val2 *= yFactor
	rectangle.Val3 *= xFactor
	rectangle.Val4 *= yFactor
	xScalar := ( 1.05 + ( .5 * rectangle.Val1 / float64( to.X ) ) )
	yScalar0 := ( 2.0 * ( rectangle.Val2 / float64( to.Y ) ) )
	yScalar1 := ( 2.0 * ( rectangle.Val4 / float64( to.Y ) ) )
	rectangle.Val1 *= xScalar
	rectangle.Val2 *= yScalar0
	rectangle.Val3 *= xScalar
	rectangle.Val4 *= yScalar1
	return ScalarToRectangle( rectangle )
}
