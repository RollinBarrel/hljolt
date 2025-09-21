package jolt;

@:struct class MotorSettings {
    @:packed public var springSettings(default,never):SpringSettings;
    public var minForceLimit:Single;
    public var maxForceLimit:Single;
    public var minTorqueLimit:Single;
    public var maxTorqueLimit:Single;
}