package jolt;

import h3d.Vector4;

class SliderConstraint extends Constraint {
    public inline function getCurrentPosition() return ref.sliderGetCurrentPosition();

    public inline function setMaxFrictionForce(friction) ref.sliderSetMaxFrictionForce(friction);

    public inline function getMotorSettings() return ref.sliderGetMotorSettings();

    public inline function setMotorState(state) ref.sliderSetMotorState(state);
    public inline function setTargetVelocity(vel) ref.sliderSetTargetVelocity(vel);
    public inline function setTargetPosition(pos) ref.sliderSetTargetPosition(pos);

    public inline function setLimits(min, max) ref.sliderSetLimits(min, max);
}