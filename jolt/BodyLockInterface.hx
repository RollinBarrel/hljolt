package jolt;

@:hlNative("jolt", "body_lock_interface_")
abstract BodyLockInterface(hl.Abstract<"BodyLockInterface">) {
    public function lockWrite(bodyID:Int, callback:Body->Void) {}
    public function lockRead(bodyID:Int, callback:Body->Void) {}
    public function lockMultiWrite(bodyIDs:hl.NativeArray<Int>, callback:hl.NativeArray<Body>->Void) {}
    public function lockMultiRead(bodyIDs:hl.NativeArray<Int>, callback:hl.NativeArray<Body>->Void) {}
}