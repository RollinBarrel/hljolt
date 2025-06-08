package jolt;

import h3d.Vector4;

@:hlNative("jolt", "instance_")
abstract JoltInstance(hl.Abstract<"JoltInstance">) {
    public function setBroadphaseLayers(layers:hl.NativeArray<Int>):Void {}
    public function setObjectVsBroadphaseLayerFilter(callback:(oLayer:Int, bpLayer:Int)->Bool):Void {}
    public function setObjectVsObjectLayerFilter(callback:(layer1:Int, layer2:Int)->Bool):Void {}
    public function setOnContactValidate(callback:(body1:Int, body2:Int, baseOffset:Vector4, collisionResult:CollideShapeResult)->Int):Void {}
    public function setOnContactAdded(callback:(body1:Int, body2:Int, manifold:ContactManifold, settings:ContactSettings)->Void):Void {}
    public function setOnContactPersisted(callback:(body1:Int, body2:Int, manifold:ContactManifold, settings:ContactSettings)->Void):Void {}
    public function setOnContactRemoved(callback:(body1:Int, body2:Int, shape1:Int, shape2:Int)->Void):Void {}
    
    public function setGravity(v:Vector4):Void {}
    public function getGravity():Vector4 return null;

    public function getBodyInterface(callback:BodyInterface->Void):Void {}
    public function update(dt:Float, collisionSteps:Int):Int return -1;
}