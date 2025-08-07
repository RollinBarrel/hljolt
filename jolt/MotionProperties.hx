package jolt;

@:hlNative("jolt", "motion_properties_")
abstract MotionProperties(hl.Abstract<"MotionProperties">) {
    public function getLinearDamping():Float return 0.;
    public function setLinearDamping(damping:Float):Void {}

    public function getAngularDamping():Float return 0.;
    public function setAngularDamping(damping:Float):Void {}

    public function getInverseMass():Float return -1.;

    public function scaleToMass(mass:Float):Void {}
}