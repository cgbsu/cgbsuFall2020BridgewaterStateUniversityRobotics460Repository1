package main

import (
	// "fmt"
	"image"
	"math"
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

const HashMatDefaultMersennePrimeConstant = 2147483647
const HashMatDefaultInitialValueConstant = 0
const HashMatDefaultAlphaConstant = 31

func HashMat( toHash gocv.Mat, initialValue, alpha, mersennePrime int ) int {
	hash := initialValue
	for _, value := range toHash.DataPtrUint8() {
		hash = ( ( ( hash * alpha ) + int( value ) ) % mersennePrime )
	}
	return hash
}

func DefaultHashMat( toHash gocv.Mat ) int {
	return HashMat( toHash, HashMatDefaultInitialValueConstant, 
			HashMatDefaultAlphaConstant, HashMatDefaultMersennePrimeConstant )
}

func MatsAreEqual( first, second gocv.Mat ) bool {
	if first.Size()[ 0 ] == second.Size()[ 0 ] && first.Size()[ 1 ] == second.Size()[ 1 ] {
		return ( DefaultHashMat( first ) == DefaultHashMat( second ) )
	}
	return false
}

func PointDistance( first, second image.Point ) float64 {
	first.X -= second.X
	first.Y -= second.Y
	return math.Sqrt( float64( ( first.X * first.X ) + ( second.X * second.X ) ) )
}

func ToBoundedScalar( toBound gocv.Scalar, imageSize image.Point ) gocv.Scalar {
	bound := func( canidate *float64, bound float64 ) {
		if *canidate < 0.0 {
			*canidate = 0.0
		} else if *canidate >= bound {
			*canidate = ( bound - 1 )
		}
	}
	bound( &toBound.Val1, float64( imageSize.X ) )
	bound( &toBound.Val2, float64( imageSize.Y ) )
	bound( &toBound.Val3, float64( imageSize.X ) )
	bound( &toBound.Val4, float64( imageSize.Y ) )
	return toBound
}

func MatRegion( toCut gocv.Mat, region gocv.Scalar ) gocv.Mat {
	copy := gocv.NewMat()
	// fmt.Println( toCut.Rows(), " :: ", toCut.Cols() )
	minimumDimention := math.Min( float64( toCut.Rows() ), float64( toCut.Cols() ) )
	if region.Val1 <= minimumDimention && region.Val2 <= minimumDimention && region.Val3 <= minimumDimention && region.Val4 <= minimumDimention {
		toCopy := toCut.RowRange( int( region.Val2 ), int( region.Val4 ) )
		toCopy = toCopy.ColRange( int( region.Val1 ), int( region.Val3 ) )
		toCopy.CopyTo( &copy )
		return copy
	}
	return gocv.NewMat()
}
