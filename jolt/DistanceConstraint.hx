package jolt;

import h3d.Vector4;

class DistanceConstraint extends Constraint {
    public inline function setDistance(min, max) ref.distanceSetDistance(min, max);
    public inline function getLimitsSpringSettings() return ref.distanceGetLimitsSpringSettings();
}