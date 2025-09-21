package jolt;

import h3d.Vector4;

@:hlNative("jolt", "shape_")
abstract ShapeRef(hl.Abstract<"_ShapeRef">) {
    public function getSubType():ShapeSubType return -1;
    public function castRay(origin:Vector4, direction:Vector4):RayCastResult return null;
    public function scale(scale:Vector4):ShapeRef return null;
    public function scaledGetInnerShape():ShapeRef return null;
}