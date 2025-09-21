package jolt;

import h3d.Vector4;

@:hlNative("jolt", "constraint_")
abstract ConstraintRef(hl.Abstract<"_ConstraintRef">) {
    public function setEnabled(toggle:Bool) {}

    public function notifyShapeChanged(bodyId:Int, deltaCOM:Vector4) {}

    public function distanceSetDistance(min:Float, max:Float) {}
    public function distanceGetLimitsSpringSettings():SpringSettings return null;

    public function sliderGetCurrentPosition():Float return -1.;
    public function sliderSetMaxFrictionForce(friction:Float) {}
    public function sliderGetMotorSettings():MotorSettings return null;
    public function sliderSetMotorState(state:MotorState) {}
    public function sliderSetTargetVelocity(vel:Float) {}
    public function sliderSetTargetPosition(pos:Float) {}
    public function sliderSetLimits(min:Float, max:Float) {}
}