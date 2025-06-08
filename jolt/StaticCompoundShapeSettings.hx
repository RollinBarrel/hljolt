package jolt;

import h3d.Vector4;

@:hlNative("jolt", "static_compound_shape_settings_")
abstract StaticCompoundShapeSettings(hl.Abstract<"StaticCompoundShapeSettings">) {
    public static function alloc():StaticCompoundShapeSettings return null;

    public function addShape(position:Vector4, rotation:Vector4, shape:ShapeRef, userData:Int):Void {}
    public function create():ShapeRef return null;
}