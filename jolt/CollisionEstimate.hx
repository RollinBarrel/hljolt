package jolt;

import h3d.Vector4;

@:hlNative("jolt", "collision_estimate_")
abstract CollisionEstimate(hl.Abstract<"CollisionEstimationResult">) {
    public static function estimate(body1:Body, body2:Body, contactManifold:ContactManifold, combinedFriction:hl.F32, combinedRestitution:hl.F32, minVelocityForRestitution:hl.F32, numIterations:Int, callback:(estimate:CollisionEstimate)->Void):Void {}
    public function getLinearVelocity1():Vector4 return null;
    public function getLinearVelocity2():Vector4 return null;
}