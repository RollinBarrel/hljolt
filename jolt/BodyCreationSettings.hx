package jolt;

import h3d.Vector4;

@:hlNative("jolt", "body_creation_settings_")
abstract BodyCreationSettings(hl.Abstract<"BodyCreationSettings">) {
    public static function create(shape:ShapeRef, pos:Vector4, rot:Vector4, motionType:MotionType, objectLayer:Int):BodyCreationSettings return null;

}