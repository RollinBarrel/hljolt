package jolt;

import h3d.Vector4;

@:hlNative("jolt", "slider_constraint_settings_")
abstract SliderConstraintSettings(hl.Abstract<"SliderConstraintSettings">) {
    public static function alloc():SliderConstraintSettings return null;
    
    public function setSliderAxis(axis:Vector4) {}
    public function setAutoDetectPoint(toggle:Bool) {}
    public function setPoint1(point:Vector4) {}
    public function setPoint2(point:Vector4) {}
    public function create(body1:Body, body2:Body):SliderConstraint {
        return new SliderConstraint(_create(body1, body2));
    }

    @:hlNative("jolt", "slider_constraint_settings_create") function _create(body1:Body, body2:Body):ConstraintRef return null;
}