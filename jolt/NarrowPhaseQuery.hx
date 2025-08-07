package jolt;

import h3d.Vector4;

@:hlNative("jolt", "narrow_phase_query_")
abstract NarrowPhaseQuery(hl.Abstract<"NarrowPhaseQuery">) {
    public function castRay(
        origin:Vector4, direction:Vector4,
        broadPhaseLayerFilter:(broadPhaseLayer:Int)->Bool, objectLayerFilter:(objectLayer:Int)->Bool, bodyFilter:(bodyID:Int)->Bool
    ):RayCastResult return null;

    public function castShape(
        shape:ShapeRef, origin:Vector4, rotation:Vector4, direction:Vector4,
        broadPhaseLayerFilter:(broadPhaseLayer:Int)->Bool, objectLayerFilter:(objectLayer:Int)->Bool, bodyFilter:(bodyID:Int)->Bool, //shapeFilter
        callback:(hits:hl.NativeArray<ShapeCastResult>)->Void
    ):Void {}
}