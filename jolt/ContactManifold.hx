package jolt;

import h3d.Vector4;

@:hlNative("jolt", "contact_manifold_")
abstract ContactManifold(hl.Abstract<"ContactManifold">) {
    public function getWorldSpaceNormal():Vector4 return null;
    public function getPenetrationDepth():hl.F32 return -1.;
}
