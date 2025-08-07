package jolt;

import h3d.Vector4;

@:hlNative("jolt", "body_interface_")
abstract BodyInterface(hl.Abstract<"BodyInterface">) {
    public function createBody(settings:BodyCreationSettings):Int return -1;
    public function addBody(bodyID:Int, activate:Bool):Void {}
    public function removeBody(bodyID:Int):Void {}
    public function destroyBody(bodyID:Int):Void {}

    public function activateBody(bodyID:Int):Void {}
    public function deactivateBody(bodyID:Int):Void {}
    public function isActive(bodyID:Int):Bool return false;

    public function getShape(bodyID:Int):ShapeRef return null;
    public function setShape(bodyID:Int, shape:ShapeRef, recalcMass:Bool, activate:Bool):Void {}

    public function setPosition(bodyID:Int, pos:Vector4, activate:Bool):Void {}
    public function getPosition(bodyID:Int):Vector4 return null;
    public function getCenterOfMassPosition(bodyID:Int):Vector4 return null;

    public function setRotation(bodyID:Int, rot:Vector4, activate:Bool):Void {}
    public function getRotation(bodyID:Int):Vector4 return null;

    public function setLinearVelocity(bodyID:Int, vel:Vector4):Void {}
    public function getLinearVelocity(bodyID:Int):Vector4 return null;
    public function setAngularVelocity(bodyID:Int, vel:Vector4):Void {}
    public function getAngularVelocity(bodyID:Int):Vector4 return null;
    public function getPointVelocity(bodyID:Int, pos:Vector4):Vector4 return null;

    public function addForce(bodyID:Int, force:Vector4, activate:Bool):Void {}
    public function addForceAtPosition(bodyID:Int, force:Vector4, pos:Vector4, activate:Bool):Void {}
    public function addTorque(bodyID:Int, torque:Vector4, activate:Bool):Void {}
    public function addImpulse(bodyID:Int, impulse:Vector4):Void {}
    public function addImpulseAtPosition(bodyID:Int, impulse:Vector4, pos:Vector4):Void {}
    public function addAngularImpulse(bodyID:Int, impulse:Vector4):Void {}

    public function setRestitution(bodyID:Int, v:Float):Void {}
    public function getRestitution(bodyID:Int):Float return -1.;
    public function setFriction(bodyID:Int, v:Float):Void {}
    public function getFriction(bodyID:Int):Float return -1.;
    public function setGravityFactor(bodyID:Int, v:Float):Void {}
    public function getGravityFactor(bodyID:Int):Float return -1.;

    public function getUserData(bodyID:Int):Dynamic return 0;
    public function setUserData(bodyID:Int, v:Dynamic):Void {}
}