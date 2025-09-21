package jolt;

enum abstract MotorState(Int) from Int to Int {
	var Off;			    ///< Motor is off
	var Velocity;			///< Motor will drive to target velocity
	var Position;			///< Motor will drive to target position
}