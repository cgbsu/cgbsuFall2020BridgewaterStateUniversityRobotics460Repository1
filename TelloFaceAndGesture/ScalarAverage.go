package main

import (
	"gocv.io/x/gocv"
)

//User beware, this class treats Scalars as sacars of int!//
type ScalarAverage struct {
	averageScalarComponents [ 4 ]Average
	desiredSampleCount int
}

func ( this* ScalarAverage ) ApplyToAll( operation func( *Average, int ) ) {
	for i, _ := range this.averageScalarComponents {
		operation( &this.averageScalarComponents[ i ], i )
	}
}

func ( this *ScalarAverage ) InitializeAverage( desiredSampleCount int ) {
	this.desiredSampleCount = desiredSampleCount
	this.ApplyToAll( func ( component *Average, componentIndex int ) {
		component.InitializeAverage( desiredSampleCount )
	} )
}

func ( this *ScalarAverage ) CalculateAverage() int {
	var results [ 4 ]int
	this.ApplyToAll( func ( component *Average, componentIndex int ) {
		results[ componentIndex ] = component.CalculateAverage()
	} )
	total := 0
	for currentAverage, _ := range results {
		total += currentAverage
	}
	return total / 4
}

func ( this *ScalarAverage ) CalculateAverages() [ 4 ]int {
	var results [ 4 ]int
	this.ApplyToAll( func ( component *Average, componentIndex int ) {
		results[ componentIndex ] = component.CalculateAverage()
	} )
	return results
}

func ( this *ScalarAverage ) AddSample( sample int ) {
	this.ApplyToAll( func ( component *Average, componentIndex int ) {
		component.AddSample( sample )
	} )
}

func ( this *ScalarAverage ) AddScalarSample( sample gocv.Scalar ) {
	scalarSample := [ 4 ]int{ int( sample.Val1 ), int( sample.Val2 ), int( sample.Val3 ), int( sample.Val4 ) }
	this.ApplyToAll( func ( component *Average, componentIndex int ) {
		component.AddSample( scalarSample[ componentIndex ] )
	} )
}

func ( this *ScalarAverage ) Clear() {
	this.ApplyToAll( func ( component *Average, componentIndex int ) {
		component.Clear()
	} )
}

func ( this *ScalarAverage ) AtDesiredSampleCount() bool {
	atDesiredSampleCount := true
	this.ApplyToAll( func ( component *Average, componentIndex int ) {
		atDesiredSampleCount = component.AtDesiredSampleCount() && atDesiredSampleCount
	} )
	return atDesiredSampleCount
}

func ( this *ScalarAverage ) ConstructScalar() gocv.Scalar {
	return gocv.Scalar{ Val1: float64( this.averageScalarComponents[ 0 ].CalculateAverage() ), 
			Val2: float64( this.averageScalarComponents[ 1 ].CalculateAverage() ), 
			Val3: float64( this.averageScalarComponents[ 2 ].CalculateAverage() ), 
			Val4: float64( this.averageScalarComponents[ 3 ].CalculateAverage() )  }
}
