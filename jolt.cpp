#define HL_NAME(n) jolt_##n
#include <hl.h>

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

#include <cstdint>
#include <cstring>

using namespace JPH;
using namespace literals;

#define JOLTINST _ABSTRACT(JoltInstance)
#define BODYIF _ABSTRACT(BodyInterface)
#define BODY _ABSTRACT(Body)
#define SHAPE _ABSTRACT(_ShapeRef)
#define SHAPESETTINGS _ABSTRACT(ShapeSettings)
#define PHYSMAT _ABSTRACT(PhysicsMaterial)
#define COLSHAPERES _ABSTRACT(CollideShapeResult)
#define CONTACTMANIFOLD _ABSTRACT(ContactManifold)
#define CONTACTSETTINGS _ABSTRACT(ContactSettings)

typedef struct __ShapeRef _ShapeRef;
struct __ShapeRef {
    void (*finalise)(_ShapeRef*);
    Shape* ref;
};
void finalize_shape_ref(_ShapeRef* ref) {
    ref->ref->Release();
}

class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface {
public:
    uint numBroadPhaseLayers = 1;
	BroadPhaseLayer mObjectToBroadPhase[128] = {};
    BPLayerInterfaceImpl() {}

	virtual uint GetNumBroadPhaseLayers() const override {
		return numBroadPhaseLayers;
	}

	virtual BroadPhaseLayer	GetBroadPhaseLayer(ObjectLayer inLayer) const override {
		return mObjectToBroadPhase[inLayer];
	}
};

class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter {
public:
	vclosure* shouldCollide;

	virtual bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override {
		if(!shouldCollide) return true;
        if(shouldCollide->hasValue) {
            return ((bool(*)(void*, int, int))shouldCollide->fun)(shouldCollide->value, inLayer1, inLayer2.GetValue());
        } else {
            return ((bool(*)(int, int))shouldCollide->fun)(inLayer1, inLayer2.GetValue());
        }
	}

	ObjectVsBroadPhaseLayerFilterImpl() {
		shouldCollide = NULL;
	}

	~ObjectVsBroadPhaseLayerFilterImpl() {
        if (shouldCollide)
            hl_remove_root(&shouldCollide);
    }
};

class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter {
public:
    vclosure* shouldCollide;

	virtual bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override {
		if(!shouldCollide) return true;
        if(shouldCollide->hasValue) {
            return ((bool(*)(void*, int, int))shouldCollide->fun)(shouldCollide->value, inObject1, inObject2);
        } else {
            return ((bool(*)(int, int))shouldCollide->fun)(inObject1, inObject2);
        }
	}
    
    ObjectLayerPairFilterImpl() {
        shouldCollide = NULL;
    }

    ~ObjectLayerPairFilterImpl() {
        if (shouldCollide)
            hl_remove_root(&shouldCollide);
    }
};

class ContactListenerImpl : public ContactListener {
public:
	vclosure* onContactValidate;
	vclosure* onContactAdded;
	vclosure* onContactPersisted;
	vclosure* onContactRemoved;

	virtual ValidateResult OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inCollisionResult) override {
		if(!onContactValidate) return ValidateResult::AcceptAllContactsForThisBodyPair;

		int body1 = inBody1.GetID().GetIndexAndSequenceNumber();
		int body2 = inBody2.GetID().GetIndexAndSequenceNumber();
		DVec3* baseOffset = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
		baseOffset->Set(inBaseOffset.GetX(), inBaseOffset.GetY(), inBaseOffset.GetZ());

        if(onContactValidate->hasValue) {
            return (ValidateResult)((int(*)(void*, int, int, DVec3*, const CollideShapeResult*))onContactValidate->fun)(onContactValidate->value, body1, body2, baseOffset, &inCollisionResult);
        } else {
            return (ValidateResult)((int(*)(int, int, DVec3*, const CollideShapeResult*))onContactValidate->fun)(body1, body2, baseOffset, &inCollisionResult);
        }
	}

	virtual void OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override {
		if(!onContactAdded) return;

		int body1 = inBody1.GetID().GetIndexAndSequenceNumber();
		int body2 = inBody2.GetID().GetIndexAndSequenceNumber();

        if(onContactAdded->hasValue) {
            return ((void(*)(void*, int, int, const ContactManifold*, ContactSettings*))onContactAdded->fun)(onContactAdded->value, body1, body2, &inManifold, &ioSettings);
        } else {
            return ((void(*)(int, int, const ContactManifold*, ContactSettings*))onContactAdded->fun)(body1, body2, &inManifold, &ioSettings);
        }
	}

	virtual void OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override {
		if(!onContactPersisted) return;

		int body1 = inBody1.GetID().GetIndexAndSequenceNumber();
		int body2 = inBody2.GetID().GetIndexAndSequenceNumber();

        if(onContactPersisted->hasValue) {
            return ((void(*)(void*, int, int, const ContactManifold*, ContactSettings*))onContactPersisted->fun)(onContactPersisted->value, body1, body2, &inManifold, &ioSettings);
        } else {
            return ((void(*)(int, int, const ContactManifold*, ContactSettings*))onContactPersisted->fun)(body1, body2, &inManifold, &ioSettings);
        }
	}

	virtual void OnContactRemoved(const SubShapeIDPair &inSubShapePair) override {
		if(!onContactRemoved) return;

		int body1 = inSubShapePair.GetBody1ID().GetIndexAndSequenceNumber();
		int body2 = inSubShapePair.GetBody2ID().GetIndexAndSequenceNumber();
		int shape1 = inSubShapePair.GetSubShapeID1().GetValue();
		int shape2 = inSubShapePair.GetSubShapeID2().GetValue();

        if(onContactRemoved->hasValue) {
            return ((void(*)(void*, int, int, int, int))onContactRemoved->fun)(onContactRemoved->value, body1, body2, shape1, shape2);
        } else {
            return ((void(*)(int, int, int, int))onContactRemoved->fun)(body1, body2, shape1, shape2);
        }
	}
};

class BodyActivationListenerImpl : public BodyActivationListener {
public:
	virtual void OnBodyActivated(const BodyID &inBodyID, std::uint64_t inBodyUserData) override {
	}

	virtual void OnBodyDeactivated(const BodyID &inBodyID, std::uint64_t inBodyUserData) override {
	}
};

typedef struct __JoltInstance _JoltInstance;
class JoltInstance {
	static void threadInit(int threadIndex) {
		hl_register_thread(&threadIndex);
		hl_blocking(true);
	}
	static void threadExit(int threadIndex) {
		hl_blocking(false);
		hl_unregister_thread();
	}
public:
	_JoltInstance* wrapper;

	TempAllocatorImpl temp_allocator;
	JobSystemThreadPool job_system;
	BPLayerInterfaceImpl broad_phase_layer_interface;
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
	ObjectLayerPairFilterImpl object_vs_object_layer_filter;
	PhysicsSystem physics_system;
	BodyActivationListenerImpl body_activation_listener;
	ContactListenerImpl contact_listener;

	JoltInstance() :
		temp_allocator(10 * 1024 * 1024),
		job_system(),
		broad_phase_layer_interface(),
		object_vs_broadphase_layer_filter(),
		object_vs_object_layer_filter(),
		physics_system(),
		body_activation_listener(),
		contact_listener()
	{
		job_system.SetThreadInitFunction(threadInit);
		job_system.SetThreadExitFunction(threadExit);
		job_system.Init(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);

		physics_system.Init(
			65536,
			0,
			65536,
			10240,
			broad_phase_layer_interface,
			object_vs_broadphase_layer_filter,
			object_vs_object_layer_filter
		);

		physics_system.SetBodyActivationListener(&body_activation_listener);
		physics_system.SetContactListener(&contact_listener);
	}
};

struct __JoltInstance {
    void (*finalise)(_JoltInstance*);
    JoltInstance* jolt;
};
void finalize_jolt_instance(_JoltInstance* jolt) {
    delete jolt->jolt;
}

HL_PRIM void HL_NAME(initialize)() {
    RegisterDefaultAllocator();

    // Trace = TraceImpl;
	// JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)

    Factory::sInstance = new Factory();
    RegisterTypes();
}
DEFINE_PRIM(_VOID, initialize, _NO_ARG);

HL_PRIM void HL_NAME(uninitialize)() {
	UnregisterTypes();

	delete Factory::sInstance;
	Factory::sInstance = nullptr;
}
DEFINE_PRIM(_VOID, uninitialize, _NO_ARG);

HL_PRIM _JoltInstance* HL_NAME(instance_create)() {
	 _JoltInstance* jolt = (_JoltInstance*)hl_gc_alloc_finalizer(sizeof(_JoltInstance));
    jolt->finalise = finalize_jolt_instance;
    jolt->jolt = new JoltInstance();
    jolt->jolt->wrapper = jolt;
    return jolt;
}
DEFINE_PRIM(JOLTINST, instance_create, _NO_ARG);

HL_PRIM void HL_NAME(instance_set_broadphase_layers)(_JoltInstance* jolt, varray* layers) {
	int* l = hl_aptr(layers, int);
	int size = layers->size;

	BPLayerInterfaceImpl& broad_phase_layer_interface = jolt->jolt->broad_phase_layer_interface;

	broad_phase_layer_interface.numBroadPhaseLayers = size;
	for (int i = 0; i < size; i++) {
		broad_phase_layer_interface.mObjectToBroadPhase[i] = (BroadPhaseLayer)l[i];
	}
}
DEFINE_PRIM(_VOID, instance_set_broadphase_layers, JOLTINST _ARR);

HL_PRIM void HL_NAME(instance_set_object_vs_broadphase_layer_filter)(_JoltInstance* jolt, _vclosure* callback) {
	vclosure* &shouldCollide = jolt->jolt->object_vs_broadphase_layer_filter.shouldCollide;
	if (shouldCollide) {
        hl_remove_root(&shouldCollide);
    }
	shouldCollide = callback;
    if (shouldCollide) {
        hl_add_root(&shouldCollide);
    }
}
DEFINE_PRIM(_VOID, instance_set_object_vs_broadphase_layer_filter, JOLTINST _FUN(_BOOL, _I32 _I32));

HL_PRIM void HL_NAME(instance_set_object_vs_object_layer_filter)(_JoltInstance* jolt, _vclosure* callback) {
	vclosure* &shouldCollide = jolt->jolt->object_vs_object_layer_filter.shouldCollide;
	if (shouldCollide) {
        hl_remove_root(&shouldCollide);
    }
	shouldCollide = callback;
    if (shouldCollide) {
        hl_add_root(&shouldCollide);
    }
}
DEFINE_PRIM(_VOID, instance_set_object_vs_object_layer_filter, JOLTINST _FUN(_BOOL, _I32 _I32));

HL_PRIM void HL_NAME(instance_set_on_contact_validate)(_JoltInstance* jolt, _vclosure* callback) {
	vclosure* &onContactValidate = jolt->jolt->contact_listener.onContactValidate;
	if (onContactValidate) {
        hl_remove_root(&onContactValidate);
    }
	onContactValidate = callback;
    if (onContactValidate) {
        hl_add_root(&onContactValidate);
    }
}
DEFINE_PRIM(_VOID, instance_set_on_contact_validate, JOLTINST _FUN(_I32, _I32 _I32 _STRUCT COLSHAPERES));

HL_PRIM void HL_NAME(instance_set_on_contact_added)(_JoltInstance* jolt, _vclosure* callback) {
	vclosure* &onContactAdded = jolt->jolt->contact_listener.onContactAdded;
	if (onContactAdded) {
        hl_remove_root(&onContactAdded);
    }
	onContactAdded = callback;
    if (onContactAdded) {
        hl_add_root(&onContactAdded);
    }
}
DEFINE_PRIM(_VOID, instance_set_on_contact_added, JOLTINST _FUN(_VOID, _I32 _I32 CONTACTMANIFOLD CONTACTSETTINGS));

HL_PRIM void HL_NAME(instance_set_on_contact_persisted)(_JoltInstance* jolt, _vclosure* callback) {
	vclosure* &onContactPersisted = jolt->jolt->contact_listener.onContactPersisted;
	if (onContactPersisted) {
        hl_remove_root(&onContactPersisted);
    }
	onContactPersisted = callback;
    if (onContactPersisted) {
        hl_add_root(&onContactPersisted);
    }
}
DEFINE_PRIM(_VOID, instance_set_on_contact_persisted, JOLTINST _FUN(_VOID, _I32 _I32 CONTACTMANIFOLD CONTACTSETTINGS));

HL_PRIM void HL_NAME(instance_set_on_contact_removed)(_JoltInstance* jolt, _vclosure* callback) {
	vclosure* &onContactRemoved = jolt->jolt->contact_listener.onContactRemoved;
	if (onContactRemoved) {
        hl_remove_root(&onContactRemoved);
    }
	onContactRemoved = callback;
    if (onContactRemoved) {
        hl_add_root(&onContactRemoved);
    }
}
DEFINE_PRIM(_VOID, instance_set_on_contact_removed, JOLTINST _FUN(_VOID, _I32 _I32 _I32 _I32));

HL_PRIM void HL_NAME(instance_get_body_interface)(_JoltInstance* jolt, vclosure* callback) {
	BodyInterface& body_interface = jolt->jolt->physics_system.GetBodyInterface();

	if(callback->hasValue) {
		((void(*)(void*, BodyInterface*))callback->fun)(callback->value, &body_interface);
	} else {
		((void(*)(BodyInterface*))callback->fun)(&body_interface);
	}
}
DEFINE_PRIM(_VOID, instance_get_body_interface, JOLTINST _FUN(_VOID, BODYIF));

HL_PRIM int HL_NAME(instance_update)(_JoltInstance* jolt, double inDeltaTime, int inCollisionSteps) {
	return (int)jolt->jolt->physics_system.Update((float)inDeltaTime, inCollisionSteps, &jolt->jolt->temp_allocator, &jolt->jolt->job_system);
}
DEFINE_PRIM(_I32, instance_update, JOLTINST _F64 _I32);

HL_PRIM _ShapeRef* HL_NAME(box_shape_create)(DVec3* inHalfExtent, double inConvexRadius, PhysicsMaterial* inMaterial) {
	// Jolt does refcounts on this and will release it when there's no references
	BoxShapeSettings* settings = new BoxShapeSettings(inHalfExtent->ToVec3RoundDown(), (float)inConvexRadius, inMaterial);

	Shape* r = settings->Create().Get().GetPtr();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = r;
    return ref;
}
DEFINE_PRIM(SHAPE, box_shape_create, _STRUCT _F64 PHYSMAT);

// inMaterial = nullptr
HL_PRIM _ShapeRef* HL_NAME(sphere_shape_create)(double inRadius, PhysicsMaterial* inMaterial) {
	// Jolt does refcounts on this and will release it when there's no references
	SphereShapeSettings* settings = new SphereShapeSettings((float)inRadius, inMaterial);

	Shape* r = settings->Create().Get().GetPtr();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = r;
    return ref;
}
DEFINE_PRIM(SHAPE, sphere_shape_create, _F64 PHYSMAT);

HL_PRIM BodyCreationSettings* HL_NAME(body_creation_settings_create)(_ShapeRef* inShape, DVec3* inPosition, DVec3* inRotation, EMotionType inMotionType, int inObjectLayer) {
	BodyCreationSettings* settings = new BodyCreationSettings(
		inShape->ref,
		RVec3(inPosition->mF64[0], inPosition->mF64[1], inPosition->mF64[2]),
		QuatArg(inRotation->mF64[0],inRotation->mF64[1], inRotation->mF64[2], inRotation->mF64[3]),
		inMotionType,
		inObjectLayer
	);
	return settings;
}
DEFINE_PRIM(_STRUCT, body_creation_settings_create, SHAPE _STRUCT _STRUCT _I32 _I32);

HL_PRIM int HL_NAME(body_interface_create_body)(BodyInterface* body_interface, BodyCreationSettings* settings) {
	int id = body_interface->CreateBody(*settings)->GetID().GetIndexAndSequenceNumber();
	delete settings;
	return id;
}
DEFINE_PRIM(_I32, body_interface_create_body, BODYIF _STRUCT);

HL_PRIM void HL_NAME(body_interface_add_body)(BodyInterface* body_interface, uint32 bodyID, bool activate) {
	body_interface->AddBody(BodyID(bodyID), (EActivation)!activate);
}
DEFINE_PRIM(_VOID, body_interface_add_body, BODYIF _I32 _BOOL);

HL_PRIM void HL_NAME(body_interface_remove_body)(BodyInterface* body_interface, uint32 bodyID) {
	body_interface->RemoveBody(BodyID(bodyID));
}
DEFINE_PRIM(_VOID, body_interface_remove_body, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_destroy_body)(BodyInterface* body_interface, uint32 bodyID) {
	body_interface->DestroyBody(BodyID(bodyID));
}
DEFINE_PRIM(_VOID, body_interface_destroy_body, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_position)(BodyInterface* body_interface, uint32 bodyID, DVec3* inPosition, bool activate) {
	body_interface->SetPosition(
		BodyID(bodyID),
		RVec3((Real)inPosition->mF64[0], (Real)inPosition->mF64[1], (Real)inPosition->mF64[2]),
		(EActivation)!activate
	);
}
DEFINE_PRIM(_VOID, body_interface_set_position, BODYIF _I32 _STRUCT _BOOL);

HL_PRIM void HL_NAME(body_interface_activate_body)(BodyInterface* body_interface, uint32 bodyID) {
	body_interface->ActivateBody(BodyID(bodyID));
}
DEFINE_PRIM(_VOID, body_interface_activate_body, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_deactivate_body)(BodyInterface* body_interface, uint32 bodyID) {
	body_interface->DeactivateBody(BodyID(bodyID));
}
DEFINE_PRIM(_VOID, body_interface_deactivate_body, BODYIF _I32);

HL_PRIM bool HL_NAME(body_interface_is_active)(BodyInterface* body_interface, uint32 bodyID) {
	return body_interface->IsActive(BodyID(bodyID));
}
DEFINE_PRIM(_BOOL, body_interface_is_active, BODYIF _I32);

HL_PRIM DVec3* HL_NAME(body_interface_get_position)(BodyInterface* body_interface, uint32 bodyID) {
	RVec3 r = body_interface->GetPosition(BodyID(bodyID));

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_position, BODYIF _I32);

HL_PRIM DVec3* HL_NAME(body_interface_get_center_of_mass_position)(BodyInterface* body_interface, uint32 bodyID) {
	RVec3 r = body_interface->GetCenterOfMassPosition(BodyID(bodyID));

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_center_of_mass_position, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_rotation)(BodyInterface* body_interface, uint32 bodyID, DVec3* inRotation, bool activate) {
	body_interface->SetRotation(
		BodyID(bodyID),
		Quat((float)inRotation->mF64[0],(float)inRotation->mF64[1], (float)inRotation->mF64[2], (float)inRotation->mF64[3]),
		(EActivation)!activate
	);
}
DEFINE_PRIM(_VOID, body_interface_set_rotation, BODYIF _I32 _STRUCT _BOOL);

HL_PRIM DVec3* HL_NAME(body_interface_get_rotation)(BodyInterface* body_interface, uint32 bodyID) {
	Quat r = body_interface->GetRotation(BodyID(bodyID));

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
	d->mF64[3] = r.GetW();
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_rotation, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_linear_velocity)(BodyInterface* body_interface, uint32 bodyID, DVec3* inLinearVelocity) {
	body_interface->SetLinearVelocity(BodyID(bodyID), inLinearVelocity->ToVec3RoundDown());
}
DEFINE_PRIM(_VOID, body_interface_set_linear_velocity, BODYIF _I32 _STRUCT);

HL_PRIM DVec3* HL_NAME(body_interface_get_linear_velocity)(BodyInterface* body_interface, uint32 bodyID) {
	Vec3 r = body_interface->GetLinearVelocity(BodyID(bodyID));

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_linear_velocity, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_angular_velocity)(BodyInterface* body_interface, uint32 bodyID, DVec3* inAngularVelocity) {
	body_interface->SetAngularVelocity(BodyID(bodyID), inAngularVelocity->ToVec3RoundDown());
}
DEFINE_PRIM(_VOID, body_interface_set_angular_velocity, BODYIF _I32 _STRUCT);

HL_PRIM DVec3* HL_NAME(body_interface_get_angular_velocity)(BodyInterface* body_interface, uint32 bodyID) {
	Vec3 r = body_interface->GetAngularVelocity(BodyID(bodyID));

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_angular_velocity, BODYIF _I32);

HL_PRIM DVec3* HL_NAME(body_interface_get_point_velocity)(BodyInterface* body_interface, uint32 bodyID, DVec3* inPoint) {
	Vec3 r = body_interface->GetPointVelocity(
		BodyID(bodyID),
		RVec3(inPoint->mF64[0], inPoint->mF64[1], inPoint->mF64[2])
	);

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_point_velocity, BODYIF _I32 _STRUCT);

HL_PRIM void HL_NAME(body_interface_add_force)(BodyInterface* body_interface, uint32 bodyID, DVec3* inForce, bool activate) {
	body_interface->AddForce(
		BodyID(bodyID),
		Vec3Arg(inForce->mF64[0], inForce->mF64[1], inForce->mF64[2]),
		(EActivation)!activate
	);
}
DEFINE_PRIM(_VOID, body_interface_add_force, BODYIF _I32 _STRUCT _BOOL);

HL_PRIM void HL_NAME(body_interface_add_force_at_position)(BodyInterface* body_interface, uint32 bodyID, DVec3* inForce, DVec3* inPoint, bool activate) {
	body_interface->AddForce(
		BodyID(bodyID),
		Vec3Arg(inForce->mF64[0], inForce->mF64[1], inForce->mF64[2]),
		RVec3(inPoint->mF64[0], inPoint->mF64[1], inPoint->mF64[2]),
		(EActivation)!activate
	);
}
DEFINE_PRIM(_VOID, body_interface_add_force_at_position, BODYIF _I32 _STRUCT _STRUCT _BOOL);

HL_PRIM void HL_NAME(body_interface_add_torque)(BodyInterface* body_interface, uint32 bodyID, DVec3* inTorque, bool activate) {
	body_interface->AddTorque(
		BodyID(bodyID),
		Vec3Arg(inTorque->mF64[0], inTorque->mF64[1], inTorque->mF64[2]),
		(EActivation)!activate
	);
}
DEFINE_PRIM(_VOID, body_interface_add_torque, BODYIF _I32 _STRUCT _BOOL);

HL_PRIM void HL_NAME(body_interface_add_impulse)(BodyInterface* body_interface, uint32 bodyID, DVec3* inImpulse) {
	body_interface->AddImpulse(
		BodyID(bodyID),
		Vec3Arg(inImpulse->mF64[0], inImpulse->mF64[1], inImpulse->mF64[2])
	);
}
DEFINE_PRIM(_VOID, body_interface_add_impulse, BODYIF _I32 _STRUCT);

HL_PRIM void HL_NAME(body_interface_add_impulse_at_position)(BodyInterface* body_interface, uint32 bodyID, DVec3* inImpulse, DVec3* inPoint) {
	body_interface->AddImpulse(
		BodyID(bodyID),
		Vec3Arg(inImpulse->mF64[0], inImpulse->mF64[1], inImpulse->mF64[2]),
		RVec3(inPoint->mF64[0], inPoint->mF64[1], inPoint->mF64[2])
	);
}
DEFINE_PRIM(_VOID, body_interface_add_impulse_at_position, BODYIF _I32 _STRUCT _STRUCT);

HL_PRIM void HL_NAME(body_interface_add_angular_impulse)(BodyInterface* body_interface, uint32 bodyID, DVec3* inAngularImpulse) {
	body_interface->AddAngularImpulse(
		BodyID(bodyID),
		Vec3Arg(inAngularImpulse->mF64[0], inAngularImpulse->mF64[1], inAngularImpulse->mF64[2])
	);
}
DEFINE_PRIM(_VOID, body_interface_add_angular_impulse, BODYIF _I32 _STRUCT);

HL_PRIM void HL_NAME(body_interface_set_restitution)(BodyInterface* body_interface, uint32 bodyID, double inRestitution) {
	body_interface->SetRestitution(BodyID(bodyID), (float)inRestitution);
}
DEFINE_PRIM(_VOID, body_interface_set_restitution, BODYIF _I32 _F64);

HL_PRIM double HL_NAME(body_interface_get_restitution)(BodyInterface* body_interface, uint32 bodyID) {
	return body_interface->GetRestitution(BodyID(bodyID));
}
DEFINE_PRIM(_F64, body_interface_get_restitution, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_friction)(BodyInterface* body_interface, uint32 bodyID, double inFriction) {
	body_interface->SetFriction(BodyID(bodyID), (float)inFriction);
}
DEFINE_PRIM(_VOID, body_interface_set_friction, BODYIF _I32 _F64);

HL_PRIM double HL_NAME(body_interface_get_friction)(BodyInterface* body_interface, uint32 bodyID) {
	return body_interface->GetFriction(BodyID(bodyID));
}
DEFINE_PRIM(_F64, body_interface_get_friction, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_gravity_factor)(BodyInterface* body_interface, uint32 bodyID, double inGravityFactor) {
	body_interface->SetGravityFactor(BodyID(bodyID), (float)inGravityFactor);
}
DEFINE_PRIM(_VOID, body_interface_set_gravity_factor, BODYIF _I32 _F64);

HL_PRIM double HL_NAME(body_interface_get_gravity_factor)(BodyInterface* body_interface, uint32 bodyID) {
	return body_interface->GetGravityFactor(BodyID(bodyID));
}
DEFINE_PRIM(_F64, body_interface_get_gravity_factor, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_user_data)(BodyInterface* body_interface, uint32 bodyID, void* inUserData) {
	body_interface->SetUserData(BodyID(bodyID), (unsigned long)inUserData);
}
DEFINE_PRIM(_VOID, body_interface_set_user_data, BODYIF _I32 _DYN);

HL_PRIM void* HL_NAME(body_interface_get_user_data)(BodyInterface* body_interface, uint32 bodyID) {
	return (void*)body_interface->GetUserData(BodyID(bodyID));
}
DEFINE_PRIM(_DYN, body_interface_get_user_data, BODYIF _I32);


// BodyInterface::AddBodiesPrepare
// BodyInterface::AddBodiesFinalize
// BodyInterface::AddBodiesAbort
// BodyInterface::RemoveBodies
