package jolt;

import h3d.Vector4;

@:hlNative("jolt", "motion_properties_")
abstract MotionProperties(hl.Abstract<"MotionProperties">) {
    public function getLinearDamping():Float return 0.;
    public function setLinearDamping(damping:Float):Void {}

    public function getAngularDamping():Float return 0.;
    public function setAngularDamping(damping:Float):Void {}

    public function getInverseMass():Float return -1.;

    public function scaleToMass(mass:Float):Void {}

    public function multiplyWorldSpaceInverseInertiaByVector(rotation:Vector4, vec:Vector4):Vector4 return null;
}