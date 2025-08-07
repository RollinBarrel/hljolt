package jolt;

@:hlNative("jolt", "body_lock_interface_")
abstract BodyLockInterface(hl.Abstract<"BodyLockInterface">) {
    public function lockWrite(bodyID:Int, callback:Body->Void) {}
    public function lockRead(bodyID:Int, callback:Body->Void) {}
}