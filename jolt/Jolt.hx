package jolt;

import h3d.Vector4;

@:hlNative("jolt")
class Jolt {
    public static function initialize():Void {}
    public static function uninitialize():Void {}
    public static function instanceCreate():JoltInstance return null;

    public static function boxShapeCreate(halfExtent:Vector4, convexRadius:Float, material:PhysicsMaterial = null):ShapeRef return null;
    public static function sphereShapeCreate(radius:Float, material:PhysicsMaterial = null):ShapeRef return null;
    public static function capsuleShapeCreate(halfHeightOfCylinder:Float, radius:Float, material:PhysicsMaterial = null):ShapeRef return null;
    public static function convexHullShapeCreate(verts:hl.NativeArray<hl.F32>, convexRadius:Float, material:PhysicsMaterial = null):ShapeRef return null;
    public static function scaledShapeCreate(shape:ShapeRef, scale:Vector4):ShapeRef return null;
    public static function rotatedTranslatedShapeCreate(shape:ShapeRef, pos:Vector4, rot:Vector4):ShapeRef return null;

    public static function bodyCreationSettingsCreate(shape:ShapeRef, pos:Vector4, rot:Vector4, motionType:MotionType, objectLayer:Int):BodyCreationSettings return null;
}
