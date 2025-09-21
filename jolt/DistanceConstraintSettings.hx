package jolt;

import h3d.Vector4;

@:hlNative("jolt", "distance_constraint_settings_")
abstract DistanceConstraintSettings(hl.Abstract<"DistanceConstraintSettings">) {
    public static function alloc():DistanceConstraintSettings return null;
    
    public function setPoint1(point:Vector4) {}
    public function setPoint2(point:Vector4) {}
    public function create(body1:Body, body2:Body):DistanceConstraint {
        return new DistanceConstraint(_create(body1, body2));
    }

    @:hlNative("jolt", "distance_constraint_settings_create") function _create(body1:Body, body2:Body):ConstraintRef return null;
}