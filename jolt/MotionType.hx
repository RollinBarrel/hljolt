package jolt;

enum abstract MotionType(Int) from Int to Int {
	var Static;
	var Kinematic;
	var Dynamic;
}