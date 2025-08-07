package jolt;

import h3d.Vector4;

@:hlNative("jolt", "body_")
abstract Body(hl.Abstract<"Body">) {
    public function resetForce():Void {}
    public function resetTorque():Void {}
    public function getShape():ShapeRef return null;
    public function getMotionProperties():MotionProperties return null;
    public function getWorldSpaceSurfaceNormal(subShapeID:Int, pos:Vector4):Vector4 return null;
}