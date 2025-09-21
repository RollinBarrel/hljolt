package jolt;

@:hlNative("jolt", "state_recorder_")
abstract StateRecorder(hl.Abstract<"_StateRecorder">) {
    public static function create():StateRecorder return null;
    
    public function rewind() {}
    public function clear() {}
}