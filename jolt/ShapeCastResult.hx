package jolt;

import h3d.Vector4;

@:hlNative("jolt", "shape_cast_result_")
abstract ShapeCastResult(hl.Abstract<"ShapeCastResult">) {
    public function getContactPointOn1():Vector4 return null;
    public function getContactPointOn2():Vector4 return null;
    public function getPenetrationAxis():Vector4 return null;
    public function getPenetrationDepth():Float return -1.;
    public function getSubShapeID1():Int return -1;
    public function getSubShapeID2():Int return -1;
    public function getBodyID2():Int return -1;
    public function getFraction():Float return -1.;
    public function isBackface():Bool return false;
}