package main

import ( 
	"math"
)

type Vector3 struct {
	x, y, z float64
}

func ( this *Vector3 ) Magnitude() float64 {
	return math.Sqrt( ( this.x * this.x ) + ( this.y * this.y ) +( this.z * this.z ) )
}

func ( this *Vector3 ) Normalize() Vector3 {
	normal := *this
	magnitude := this.Magnitude()
	normal.x /= magnitude
	normal.y /= magnitude
	normal.z /= magnitude
	return normal
}

func ( this *Vector3 ) MultiplyByScalar( scalar float64, copy bool ) Vector3 {
	toMultiply := this
	if copy == true {
		*toMultiply = *this
	}
	toMultiply.x *= scalar
	toMultiply.y *= scalar
	toMultiply.z *= scalar
	return *toMultiply
}

func ( this *Vector3 ) AddScalar( scalar float64, copy bool ) Vector3 {
	toAdd := this
	if copy == true {
		*toAdd = *this
	}
	toAdd.x += scalar
	toAdd.y += scalar
	toAdd.z += scalar
	return *toAdd
}

func ( this *Vector3 ) AddVector( other Vector3, copy bool ) Vector3 {
	toAdd := this
	if copy == true {
		*toAdd = *this
	}
	toAdd.x += other.x
	toAdd.y += other.y
	toAdd.z += other.z
	return *toAdd
}

func ( this *Vector3 ) Negate( copy bool ) Vector3 {
	toNegate := this
	if copy == true {
		*toNegate = *this
	}
	toNegate.x = -toNegate.x
	toNegate.y = -toNegate.y
	toNegate.z = -toNegate.z
	return *toNegate
}
