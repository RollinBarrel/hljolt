package jolt;

import h3d.Vector4;

@:hlNative("jolt", "contact_settings_")
abstract ContactSettings(hl.Abstract<"ContactSettings">) {
    public function setCombinedFriction(friction:Float):Void {}
    public function setCombinedRestitution(restitution:Float):Void {}
}
