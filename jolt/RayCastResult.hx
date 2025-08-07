package jolt;

@:struct class RayCastResult {
    public var bodyID:Int;
    public var fraction:Single;
    public var subShapeID:Int;
    // public inline function getBodyID() return bodyID & 0x7fffff;

    public inline function hasHit() return fraction <= 1.;
}