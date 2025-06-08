package jolt;

enum abstract AllowedDOFs(Int) from Int to Int {
	var None		 = 0b000000;
	var All			 = 0b111111;
	var TranslationX = 0b000001;
	var TranslationY = 0b000010;
	var TranslationZ = 0b000100;
	var RotationX	 = 0b001000;
	var RotationY	 = 0b010000;
	var RotationZ	 = 0b100000;
	var Plane2D		 = 0b100011;
}