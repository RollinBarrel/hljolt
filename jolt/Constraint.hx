package jolt;

class Constraint {
    var ref:ConstraintRef;
    public function new(ref) {
        this.ref = ref;
    }

    public inline function setEnabled(enabled) ref.setEnabled(enabled);

    public inline function notifyShapeChanged(bodyId, deltaCOM) ref.notifyShapeChanged(bodyId, deltaCOM);
}