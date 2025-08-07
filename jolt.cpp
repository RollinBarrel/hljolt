#define HL_NAME(n) jolt_##n
#include <hl.h>

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include "Jolt/Physics/Collision/ShapeCast.h"
#include "Jolt/Physics/Collision/CollisionCollectorImpl.h"
#include <Jolt/Physics/Collision/CastResult.h>
#include "Jolt/Physics/Collision/NarrowPhaseQuery.h"
#include "Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h"
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

#include <cstdint>
#include <cstring>

using namespace JPH;
using namespace literals;

#define JOLTINST _ABSTRACT(JoltInstance)
#define BODYLOCKIF _ABSTRACT(BodyLockInterface)
#define BODYIF _ABSTRACT(BodyInterface)
#define NARROWQUERY _ABSTRACT(NarrowPhaseQuery)
#define SHAPECASTRES _ABSTRACT(ShapeCastResult)
#define BODY _ABSTRACT(Body)
#define BODYCREATIONSETTINGS _ABSTRACT(BodyCreationSettings)
#define MOTIONPROPS _ABSTRACT(MotionProperties)
#define SHAPE _ABSTRACT(_ShapeRef)
#define SHAPESETTINGS _ABSTRACT(ShapeSettings)
#define STATICCOMPOUNDSHAPESETTINGS _ABSTRACT(StaticCompoundShapeSettings)
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
		
		bool res;
		hl_blocking(false);
        if(shouldCollide->hasValue) {
            res = ((bool(*)(void*, int, int))shouldCollide->fun)(shouldCollide->value, inLayer1, inLayer2.GetValue());
        } else {
            res = ((bool(*)(int, int))shouldCollide->fun)(inLayer1, inLayer2.GetValue());
        }
		hl_blocking(true);
		return res;
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

		bool res;
		hl_blocking(false);
        if(shouldCollide->hasValue) {
            res = ((bool(*)(void*, int, int))shouldCollide->fun)(shouldCollide->value, inObject1, inObject2);
        } else {
            res = ((bool(*)(int, int))shouldCollide->fun)(inObject1, inObject2);
        }
		hl_blocking(true);
		return res;
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

		int res;
		hl_blocking(false);
        if(onContactValidate->hasValue) {
            res = ((int(*)(void*, int, int, DVec3*, const CollideShapeResult*))onContactValidate->fun)(onContactValidate->value, body1, body2, baseOffset, &inCollisionResult);
        } else {
            res = ((int(*)(int, int, DVec3*, const CollideShapeResult*))onContactValidate->fun)(body1, body2, baseOffset, &inCollisionResult);
        }
		hl_blocking(true);
		return (ValidateResult)res;
	}

	virtual void OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override {
		if(!onContactAdded) return;

		int body1 = inBody1.GetID().GetIndexAndSequenceNumber();
		int body2 = inBody2.GetID().GetIndexAndSequenceNumber();

		hl_blocking(false);
        if(onContactAdded->hasValue) {
            ((void(*)(void*, int, int, const ContactManifold*, ContactSettings*))onContactAdded->fun)(onContactAdded->value, body1, body2, &inManifold, &ioSettings);
        } else {
            ((void(*)(int, int, const ContactManifold*, ContactSettings*))onContactAdded->fun)(body1, body2, &inManifold, &ioSettings);
        }
		hl_blocking(true);
	}

	virtual void OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override {
		if(!onContactPersisted) return;

		int body1 = inBody1.GetID().GetIndexAndSequenceNumber();
		int body2 = inBody2.GetID().GetIndexAndSequenceNumber();

		hl_blocking(false);
        if(onContactPersisted->hasValue) {
            ((void(*)(void*, int, int, const ContactManifold*, ContactSettings*))onContactPersisted->fun)(onContactPersisted->value, body1, body2, &inManifold, &ioSettings);
        } else {
            ((void(*)(int, int, const ContactManifold*, ContactSettings*))onContactPersisted->fun)(body1, body2, &inManifold, &ioSettings);
        }
		hl_blocking(true);
	}

	virtual void OnContactRemoved(const SubShapeIDPair &inSubShapePair) override {
		if(!onContactRemoved) return;

		int body1 = inSubShapePair.GetBody1ID().GetIndexAndSequenceNumber();
		int body2 = inSubShapePair.GetBody2ID().GetIndexAndSequenceNumber();
		int shape1 = inSubShapePair.GetSubShapeID1().GetValue();
		int shape2 = inSubShapePair.GetSubShapeID2().GetValue();

		hl_blocking(false);
        if(onContactRemoved->hasValue) {
            ((void(*)(void*, int, int, int, int))onContactRemoved->fun)(onContactRemoved->value, body1, body2, shape1, shape2);
        } else {
            ((void(*)(int, int, int, int))onContactRemoved->fun)(body1, body2, shape1, shape2);
        }
		hl_blocking(true);
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

class BroadPhaseLayerFilterHL : public BroadPhaseLayerFilter {
public:
	vclosure* shouldCollide;
	bool ShouldCollide(BroadPhaseLayer inLayer) const override {
		if (!shouldCollide)
			return true;

		bool res;
		hl_blocking(false);
        if(shouldCollide->hasValue) {
            res = ((bool(*)(void*, int))shouldCollide->fun)(shouldCollide->value, inLayer.GetValue());
        } else {
            res = ((bool(*)(int))shouldCollide->fun)(inLayer.GetValue());
        }
		hl_blocking(true);
		return res;
	}
};

class ObjectLayerFilterHL : public ObjectLayerFilter {
public:
	vclosure* shouldCollide;
	bool ShouldCollide(ObjectLayer inLayer) const override {
		if (!shouldCollide)
			return true;

		bool res;
		hl_blocking(false);
        if(shouldCollide->hasValue) {
            res = ((bool(*)(void*, int))shouldCollide->fun)(shouldCollide->value, inLayer);
        } else {
            res = ((bool(*)(int))shouldCollide->fun)(inLayer);
        }
		hl_blocking(true);
		return res;
	}
};

class BodyFilterHL : public BodyFilter {
public:
	vclosure* shouldCollide;
	vclosure* shouldCollideLocked;

	bool ShouldCollide(const BodyID &inBodyID) const override {
		if (!shouldCollide)
			return true;

		bool res;
		hl_blocking(false);
        if(shouldCollide->hasValue) {
            res = ((bool(*)(void*, int))shouldCollide->fun)(shouldCollide->value, inBodyID.GetIndexAndSequenceNumber());
        } else {
            res = ((bool(*)(int))shouldCollide->fun)(inBodyID.GetIndexAndSequenceNumber());
        }
		hl_blocking(true);
		return res;
	}

	bool ShouldCollideLocked(const Body &inBody) const override {
		if (!shouldCollideLocked)
			return true;

		bool res;
		hl_blocking(false);
        if(shouldCollideLocked->hasValue) {
            res = ((bool(*)(void*, const Body*))shouldCollideLocked->fun)(shouldCollideLocked->value, &inBody);
        } else {
            res = ((bool(*)(const Body*))shouldCollideLocked->fun)(&inBody);
        }
		hl_blocking(true);
		return res;
	}
};

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

HL_PRIM void HL_NAME(instance_set_gravity)(_JoltInstance* jolt, DVec3* inGravity) {
	jolt->jolt->physics_system.SetGravity(Vec3Arg(inGravity->mF64[0], inGravity->mF64[1], inGravity->mF64[2]));
}
DEFINE_PRIM(_VOID, instance_set_gravity, JOLTINST _STRUCT);

HL_PRIM DVec3* HL_NAME(instance_get_gravity)(_JoltInstance* jolt) {
	Vec3 r = jolt->jolt->physics_system.GetGravity();

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, instance_get_gravity, JOLTINST);

HL_PRIM DVec3* HL_NAME(contact_manifold_get_world_space_normal)(ContactManifold* manifold) {
	Vec3 r = manifold->mWorldSpaceNormal;

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, contact_manifold_get_world_space_normal, CONTACTMANIFOLD);

HL_PRIM void HL_NAME(contact_settings_set_combined_friction)(ContactSettings* settings, double friction) {
	settings->mCombinedFriction = friction;
}
DEFINE_PRIM(_VOID, contact_settings_set_combined_friction, CONTACTSETTINGS _F64);

HL_PRIM void HL_NAME(contact_settings_set_combined_restitution)(ContactSettings* settings, double restitution) {
	settings->mCombinedRestitution = restitution;
}
DEFINE_PRIM(_VOID, contact_settings_set_combined_restitution, CONTACTSETTINGS _F64);

HL_PRIM void HL_NAME(instance_get_body_lock_interface)(_JoltInstance* jolt, vclosure* callback) {
	const BodyLockInterface& body_lock_interface = jolt->jolt->physics_system.GetBodyLockInterface();

	if(callback->hasValue) {
		((void(*)(void*, const BodyLockInterface*))callback->fun)(callback->value, &body_lock_interface);
	} else {
		((void(*)(const BodyLockInterface*))callback->fun)(&body_lock_interface);
	}
}
DEFINE_PRIM(_VOID, instance_get_body_lock_interface, JOLTINST _FUN(_VOID, BODYLOCKIF));

HL_PRIM void HL_NAME(instance_get_body_lock_interface_no_lock)(_JoltInstance* jolt, vclosure* callback) {
	const BodyLockInterfaceNoLock& body_lock_interface = jolt->jolt->physics_system.GetBodyLockInterfaceNoLock();

	if(callback->hasValue) {
		((void(*)(void*, const BodyLockInterface*))callback->fun)(callback->value, &body_lock_interface);
	} else {
		((void(*)(const BodyLockInterface*))callback->fun)(&body_lock_interface);
	}
}
DEFINE_PRIM(_VOID, instance_get_body_lock_interface_no_lock, JOLTINST _FUN(_VOID, BODYLOCKIF));

HL_PRIM void HL_NAME(instance_get_body_interface)(_JoltInstance* jolt, vclosure* callback) {
	BodyInterface& body_interface = jolt->jolt->physics_system.GetBodyInterface();

	if(callback->hasValue) {
		((void(*)(void*, BodyInterface*))callback->fun)(callback->value, &body_interface);
	} else {
		((void(*)(BodyInterface*))callback->fun)(&body_interface);
	}
}
DEFINE_PRIM(_VOID, instance_get_body_interface, JOLTINST _FUN(_VOID, BODYIF));

HL_PRIM void HL_NAME(instance_get_body_interface_no_lock)(_JoltInstance* jolt, vclosure* callback) {
	BodyInterface& body_interface = jolt->jolt->physics_system.GetBodyInterfaceNoLock();

	if(callback->hasValue) {
		((void(*)(void*, BodyInterface*))callback->fun)(callback->value, &body_interface);
	} else {
		((void(*)(BodyInterface*))callback->fun)(&body_interface);
	}
}
DEFINE_PRIM(_VOID, instance_get_body_interface_no_lock, JOLTINST _FUN(_VOID, BODYIF));

HL_PRIM void HL_NAME(instance_get_narrow_phase_query)(_JoltInstance* jolt, vclosure* callback) {
	const NarrowPhaseQuery& narrow_phase_query = jolt->jolt->physics_system.GetNarrowPhaseQuery();

	if(callback->hasValue) {
		((void(*)(void*, const NarrowPhaseQuery*))callback->fun)(callback->value, &narrow_phase_query);
	} else {
		((void(*)(const NarrowPhaseQuery*))callback->fun)(&narrow_phase_query);
	}
}
DEFINE_PRIM(_VOID, instance_get_narrow_phase_query, JOLTINST _FUN(_VOID, NARROWQUERY));

HL_PRIM void HL_NAME(instance_get_narrow_phase_query_no_lock)(_JoltInstance* jolt, vclosure* callback) {
	const NarrowPhaseQuery& narrow_phase_query = jolt->jolt->physics_system.GetNarrowPhaseQueryNoLock();

	if(callback->hasValue) {
		((void(*)(void*, const NarrowPhaseQuery*))callback->fun)(callback->value, &narrow_phase_query);
	} else {
		((void(*)(const NarrowPhaseQuery*))callback->fun)(&narrow_phase_query);
	}
}
DEFINE_PRIM(_VOID, instance_get_narrow_phase_query_no_lock, JOLTINST _FUN(_VOID, NARROWQUERY));

HL_PRIM int HL_NAME(instance_update)(_JoltInstance* jolt, double inDeltaTime, int inCollisionSteps) {
	hl_blocking(true);
	EPhysicsUpdateError err = jolt->jolt->physics_system.Update((float)inDeltaTime, inCollisionSteps, &jolt->jolt->temp_allocator, &jolt->jolt->job_system);
	hl_blocking(false);
	return (int)err;
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

HL_PRIM _ShapeRef* HL_NAME(capsule_shape_create)(double inHalfHeightOfCylinder, double inRadius, PhysicsMaterial* inMaterial) {
	// Jolt does refcounts on this and will release it when there's no references
	CapsuleShapeSettings* settings = new CapsuleShapeSettings(inHalfHeightOfCylinder, inRadius, inMaterial);

	Shape* r = settings->Create().Get().GetPtr();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = r;
    return ref;
}
DEFINE_PRIM(SHAPE, capsule_shape_create, _F64 _F64 PHYSMAT);

HL_PRIM _ShapeRef* HL_NAME(convex_hull_shape_create)(varray* inPoints, double inMaxConvexRadius, PhysicsMaterial* inMaterial) {
	float* pts = hl_aptr(inPoints, float);
	int size = inPoints->size;
	
	int vecCount = size / 3;
	Vec3* points = new Vec3[vecCount];
	int pos = 0;
	for (int i = 0; i < vecCount; i++) {
		float x = pts[pos++];
		float y = pts[pos++];
		float z = pts[pos++];
		points[i].Set(x, y, z);
	}

	ConvexHullShapeSettings* settings = new ConvexHullShapeSettings(points, vecCount, inMaxConvexRadius, inMaterial);

	Shape* r = settings->Create().Get().GetPtr();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = r;
    return ref;
}
DEFINE_PRIM(SHAPE, convex_hull_shape_create, _ARR _F64 PHYSMAT);

HL_PRIM StaticCompoundShapeSettings* HL_NAME(static_compound_shape_settings_alloc)() {
	return new StaticCompoundShapeSettings();
}
DEFINE_PRIM(STATICCOMPOUNDSHAPESETTINGS, static_compound_shape_settings_alloc, _NO_ARG);

HL_PRIM void HL_NAME(static_compound_shape_settings_add_shape)(StaticCompoundShapeSettings* settings, DVec3* inPosition, DVec3* inRotation, _ShapeRef* inShape, int inUserData) {
	settings->AddShape(
		Vec3Arg(inPosition->mF64[0], inPosition->mF64[1], inPosition->mF64[2]),
		QuatArg(inRotation->mF64[0],inRotation->mF64[1], inRotation->mF64[2], inRotation->mF64[3]),
		inShape->ref,
		inUserData
	);
}
DEFINE_PRIM(_VOID, static_compound_shape_settings_add_shape, STATICCOMPOUNDSHAPESETTINGS _STRUCT _STRUCT SHAPE _I32);

HL_PRIM _ShapeRef* HL_NAME(static_compound_shape_settings_create)(StaticCompoundShapeSettings* settings) {
	Shape* r = settings->Create().Get().GetPtr();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = r;
    return ref;
}
DEFINE_PRIM(SHAPE, static_compound_shape_settings_create, STATICCOMPOUNDSHAPESETTINGS);

HL_PRIM _ShapeRef* HL_NAME(scaled_shape_create)(_ShapeRef* inShape, DVec3* inScale) {
	ScaledShapeSettings* settings = new ScaledShapeSettings(
		inShape->ref,
		Vec3Arg(inScale->mF64[0], inScale->mF64[1], inScale->mF64[2])
	);

	Shape* r = settings->Create().Get().GetPtr();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = r;
    return ref;
}
DEFINE_PRIM(SHAPE, scaled_shape_create, SHAPE _STRUCT);

HL_PRIM _ShapeRef* HL_NAME(rotated_translated_shape_create)(_ShapeRef* inShape, DVec3* inPosition, DVec3* inRotation) {
	RotatedTranslatedShapeSettings* settings = new RotatedTranslatedShapeSettings(
		Vec3Arg(inPosition->mF64[0], inPosition->mF64[1], inPosition->mF64[2]),
		QuatArg(inRotation->mF64[0],inRotation->mF64[1], inRotation->mF64[2], inRotation->mF64[3]),
		inShape->ref
	);

	Shape* r = settings->Create().Get().GetPtr();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = r;
    return ref;
}
DEFINE_PRIM(SHAPE, rotated_translated_shape_create, SHAPE _STRUCT _STRUCT);

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
DEFINE_PRIM(BODYCREATIONSETTINGS, body_creation_settings_create, SHAPE _STRUCT _STRUCT _I32 _I32);

HL_PRIM void HL_NAME(body_creation_settings_set_allowed_dofs)(BodyCreationSettings* settings, int dofs) {
	settings->mAllowedDOFs = (EAllowedDOFs)dofs;
}
DEFINE_PRIM(_VOID, body_creation_settings_set_allowed_dofs, BODYCREATIONSETTINGS _I32);

HL_PRIM void HL_NAME(body_lock_interface_lock_write)(BodyLockInterface* body_lock_interface, uint32 bodyID, vclosure* callback) {
	hl_blocking(true);
	BodyLockWrite lock(*body_lock_interface, BodyID(bodyID));
	hl_blocking(false);
	
	if (lock.Succeeded()) {
		Body &body = lock.GetBody();
		
		if(callback->hasValue) {
			((void(*)(void*, Body*))callback->fun)(callback->value, &body);
		} else {
			((void(*)(Body*))callback->fun)(&body);
		}
	}
}
DEFINE_PRIM(_VOID, body_lock_interface_lock_write, BODYLOCKIF _I32 _FUN(_VOID, BODY));

HL_PRIM void HL_NAME(body_lock_interface_lock_read)(BodyLockInterface* body_lock_interface, uint32 bodyID, vclosure* callback) {
	hl_blocking(true);
	BodyLockRead lock(*body_lock_interface, BodyID(bodyID));
	hl_blocking(false);
	
	if (lock.Succeeded()) {
		const Body &body = lock.GetBody();
		
		if(callback->hasValue) {
			((void(*)(void*, const Body*))callback->fun)(callback->value, &body);
		} else {
			((void(*)(const Body*))callback->fun)(&body);
		}
	}
}
DEFINE_PRIM(_VOID, body_lock_interface_lock_read, BODYLOCKIF _I32 _FUN(_VOID, BODY));

HL_PRIM int HL_NAME(body_interface_create_body)(BodyInterface* body_interface, BodyCreationSettings* settings) {
	hl_blocking(true);
	int id = body_interface->CreateBody(*settings)->GetID().GetIndexAndSequenceNumber();
	hl_blocking(false);
	delete settings;
	return id;
}
DEFINE_PRIM(_I32, body_interface_create_body, BODYIF BODYCREATIONSETTINGS);

HL_PRIM void HL_NAME(body_interface_add_body)(BodyInterface* body_interface, uint32 bodyID, bool activate) {
	hl_blocking(true);
	body_interface->AddBody(BodyID(bodyID), (EActivation)!activate);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_add_body, BODYIF _I32 _BOOL);

HL_PRIM void HL_NAME(body_interface_remove_body)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	body_interface->RemoveBody(BodyID(bodyID));
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_remove_body, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_destroy_body)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	body_interface->DestroyBody(BodyID(bodyID));
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_destroy_body, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_position)(BodyInterface* body_interface, uint32 bodyID, DVec3* inPosition, bool activate) {
	hl_blocking(true);
	body_interface->SetPosition(
		BodyID(bodyID),
		RVec3(inPosition->mF64[0], inPosition->mF64[1], inPosition->mF64[2]),
		(EActivation)!activate
	);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_set_position, BODYIF _I32 _STRUCT _BOOL);

HL_PRIM void HL_NAME(body_interface_activate_body)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	body_interface->ActivateBody(BodyID(bodyID));
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_activate_body, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_deactivate_body)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	body_interface->DeactivateBody(BodyID(bodyID));
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_deactivate_body, BODYIF _I32);

HL_PRIM bool HL_NAME(body_interface_is_active)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	bool res = body_interface->IsActive(BodyID(bodyID));
	hl_blocking(false);
	return res;
}
DEFINE_PRIM(_BOOL, body_interface_is_active, BODYIF _I32);

HL_PRIM _ShapeRef* HL_NAME(body_interface_get_shape)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	const Shape* r = body_interface->GetShape(BodyID(bodyID));
	hl_blocking(false);

	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = (Shape*)r;
    return ref;
}
DEFINE_PRIM(SHAPE, body_interface_get_shape, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_shape)(BodyInterface* body_interface, uint32 bodyID, _ShapeRef* inShape, bool inUpdateMassProperties, bool activate) {
	hl_blocking(true);
	body_interface->SetShape(BodyID(bodyID), inShape->ref, inUpdateMassProperties, (EActivation)!activate);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_set_shape, BODYIF _I32 SHAPE _BOOL _BOOL);

HL_PRIM DVec3* HL_NAME(body_interface_get_position)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	RVec3 r = body_interface->GetPosition(BodyID(bodyID));
	hl_blocking(false);

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_position, BODYIF _I32);

HL_PRIM DVec3* HL_NAME(body_interface_get_center_of_mass_position)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	RVec3 r = body_interface->GetCenterOfMassPosition(BodyID(bodyID));
	hl_blocking(false);

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_center_of_mass_position, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_rotation)(BodyInterface* body_interface, uint32 bodyID, DVec3* inRotation, bool activate) {
	hl_blocking(true);
	body_interface->SetRotation(
		BodyID(bodyID),
		Quat((float)inRotation->mF64[0],(float)inRotation->mF64[1], (float)inRotation->mF64[2], (float)inRotation->mF64[3]),
		(EActivation)!activate
	);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_set_rotation, BODYIF _I32 _STRUCT _BOOL);

HL_PRIM DVec3* HL_NAME(body_interface_get_rotation)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	Quat r = body_interface->GetRotation(BodyID(bodyID));
	hl_blocking(false);

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
	d->mF64[3] = r.GetW();
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_rotation, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_linear_velocity)(BodyInterface* body_interface, uint32 bodyID, DVec3* inLinearVelocity) {
	hl_blocking(true);
	body_interface->SetLinearVelocity(BodyID(bodyID), inLinearVelocity->ToVec3RoundDown());
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_set_linear_velocity, BODYIF _I32 _STRUCT);

HL_PRIM DVec3* HL_NAME(body_interface_get_linear_velocity)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	Vec3 r = body_interface->GetLinearVelocity(BodyID(bodyID));
	hl_blocking(false);

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_linear_velocity, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_angular_velocity)(BodyInterface* body_interface, uint32 bodyID, DVec3* inAngularVelocity) {
	hl_blocking(true);
	body_interface->SetAngularVelocity(BodyID(bodyID), inAngularVelocity->ToVec3RoundDown());
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_set_angular_velocity, BODYIF _I32 _STRUCT);

HL_PRIM DVec3* HL_NAME(body_interface_get_angular_velocity)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	Vec3 r = body_interface->GetAngularVelocity(BodyID(bodyID));
	hl_blocking(false);

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_angular_velocity, BODYIF _I32);

HL_PRIM DVec3* HL_NAME(body_interface_get_point_velocity)(BodyInterface* body_interface, uint32 bodyID, DVec3* inPoint) {
	hl_blocking(true);
	Vec3 r = body_interface->GetPointVelocity(
		BodyID(bodyID),
		RVec3(inPoint->mF64[0], inPoint->mF64[1], inPoint->mF64[2])
	);
	hl_blocking(false);

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(r.GetX(), r.GetY(), r.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_interface_get_point_velocity, BODYIF _I32 _STRUCT);

HL_PRIM void HL_NAME(body_interface_add_force)(BodyInterface* body_interface, uint32 bodyID, DVec3* inForce, bool activate) {
	hl_blocking(true);
	body_interface->AddForce(
		BodyID(bodyID),
		Vec3Arg(inForce->mF64[0], inForce->mF64[1], inForce->mF64[2]),
		(EActivation)!activate
	);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_add_force, BODYIF _I32 _STRUCT _BOOL);

HL_PRIM void HL_NAME(body_interface_add_force_at_position)(BodyInterface* body_interface, uint32 bodyID, DVec3* inForce, DVec3* inPoint, bool activate) {
	hl_blocking(true);
	body_interface->AddForce(
		BodyID(bodyID),
		Vec3Arg(inForce->mF64[0], inForce->mF64[1], inForce->mF64[2]),
		RVec3(inPoint->mF64[0], inPoint->mF64[1], inPoint->mF64[2]),
		(EActivation)!activate
	);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_add_force_at_position, BODYIF _I32 _STRUCT _STRUCT _BOOL);

HL_PRIM void HL_NAME(body_interface_add_torque)(BodyInterface* body_interface, uint32 bodyID, DVec3* inTorque, bool activate) {
	hl_blocking(true);
	body_interface->AddTorque(
		BodyID(bodyID),
		Vec3Arg(inTorque->mF64[0], inTorque->mF64[1], inTorque->mF64[2]),
		(EActivation)!activate
	);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_add_torque, BODYIF _I32 _STRUCT _BOOL);

HL_PRIM void HL_NAME(body_interface_add_impulse)(BodyInterface* body_interface, uint32 bodyID, DVec3* inImpulse) {
	hl_blocking(true);
	body_interface->AddImpulse(
		BodyID(bodyID),
		Vec3Arg(inImpulse->mF64[0], inImpulse->mF64[1], inImpulse->mF64[2])
	);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_add_impulse, BODYIF _I32 _STRUCT);

HL_PRIM void HL_NAME(body_interface_add_impulse_at_position)(BodyInterface* body_interface, uint32 bodyID, DVec3* inImpulse, DVec3* inPoint) {
	hl_blocking(true);
	body_interface->AddImpulse(
		BodyID(bodyID),
		Vec3Arg(inImpulse->mF64[0], inImpulse->mF64[1], inImpulse->mF64[2]),
		RVec3(inPoint->mF64[0], inPoint->mF64[1], inPoint->mF64[2])
	);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_add_impulse_at_position, BODYIF _I32 _STRUCT _STRUCT);

HL_PRIM void HL_NAME(body_interface_add_angular_impulse)(BodyInterface* body_interface, uint32 bodyID, DVec3* inAngularImpulse) {
	hl_blocking(true);
	body_interface->AddAngularImpulse(
		BodyID(bodyID),
		Vec3Arg(inAngularImpulse->mF64[0], inAngularImpulse->mF64[1], inAngularImpulse->mF64[2])
	);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_add_angular_impulse, BODYIF _I32 _STRUCT);

HL_PRIM void HL_NAME(body_interface_set_restitution)(BodyInterface* body_interface, uint32 bodyID, double inRestitution) {
	hl_blocking(true);
	body_interface->SetRestitution(BodyID(bodyID), (float)inRestitution);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_set_restitution, BODYIF _I32 _F64);

HL_PRIM double HL_NAME(body_interface_get_restitution)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	double res = body_interface->GetRestitution(BodyID(bodyID));
	hl_blocking(false);
	return res;
}
DEFINE_PRIM(_F64, body_interface_get_restitution, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_friction)(BodyInterface* body_interface, uint32 bodyID, double inFriction) {
	hl_blocking(true);
	body_interface->SetFriction(BodyID(bodyID), (float)inFriction);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_set_friction, BODYIF _I32 _F64);

HL_PRIM double HL_NAME(body_interface_get_friction)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	double res = body_interface->GetFriction(BodyID(bodyID));
	hl_blocking(false);
	return res;
}
DEFINE_PRIM(_F64, body_interface_get_friction, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_gravity_factor)(BodyInterface* body_interface, uint32 bodyID, double inGravityFactor) {
	hl_blocking(true);
	body_interface->SetGravityFactor(BodyID(bodyID), (float)inGravityFactor);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_set_gravity_factor, BODYIF _I32 _F64);

HL_PRIM double HL_NAME(body_interface_get_gravity_factor)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	double res = body_interface->GetGravityFactor(BodyID(bodyID));
	hl_blocking(false);
	return res;
}
DEFINE_PRIM(_F64, body_interface_get_gravity_factor, BODYIF _I32);

HL_PRIM void HL_NAME(body_interface_set_user_data)(BodyInterface* body_interface, uint32 bodyID, void* inUserData) {
	hl_blocking(true);
	body_interface->SetUserData(BodyID(bodyID), (unsigned long long)inUserData);
	hl_blocking(false);
}
DEFINE_PRIM(_VOID, body_interface_set_user_data, BODYIF _I32 _DYN);

HL_PRIM void* HL_NAME(body_interface_get_user_data)(BodyInterface* body_interface, uint32 bodyID) {
	hl_blocking(true);
	void* res = (void*)body_interface->GetUserData(BodyID(bodyID));
	hl_blocking(false);
	return res;
}
DEFINE_PRIM(_DYN, body_interface_get_user_data, BODYIF _I32);

// TODO: also expose variant that supports multiple hits
HL_PRIM RayCastResult* HL_NAME(narrow_phase_query_cast_ray)(NarrowPhaseQuery* narrow_phase_query, DVec3* origin, DVec3* direction, vclosure* broadPhaseLayerFilterCallback, vclosure* objectLayerFilterCallback, vclosure* bodyFilterCallback) {
	RRayCast ray;
	ray.mOrigin = RVec3(origin->mF64[0], origin->mF64[1], origin->mF64[2]);
	ray.mDirection = Vec3(direction->mF64[0], direction->mF64[1], direction->mF64[2]);

	RayCastResult* hit = (RayCastResult*)hl_gc_alloc_noptr(sizeof(RayCastResult));
	hit->Reset();

	BroadPhaseLayerFilterHL broadPhaseLayerFilter = {};
	if (broadPhaseLayerFilterCallback) {
		broadPhaseLayerFilter.shouldCollide = broadPhaseLayerFilterCallback;
	}

	ObjectLayerFilterHL objectLayerFilter = {};
	if (objectLayerFilterCallback) {
		objectLayerFilter.shouldCollide = objectLayerFilterCallback;
	}

	BodyFilterHL bodyFilter = {};
	if (bodyFilterCallback) {
		bodyFilter.shouldCollide = bodyFilterCallback;
	}
	
	hl_blocking(true);
	narrow_phase_query->CastRay(ray, *hit, broadPhaseLayerFilter, objectLayerFilter, bodyFilter);
	hl_blocking(false);

	return hit;
}
DEFINE_PRIM(_STRUCT, narrow_phase_query_cast_ray, NARROWQUERY _STRUCT _STRUCT _FUN(_BOOL, _I32) _FUN(_BOOL, _I32) _FUN(_BOOL, _I32));

HL_PRIM void HL_NAME(narrow_phase_query_cast_shape)(NarrowPhaseQuery* narrow_phase_query, _ShapeRef* shape, DVec3* origin, DVec3* rotation, DVec3* direction, vclosure* broadPhaseLayerFilterCallback, vclosure* objectLayerFilterCallback, vclosure* bodyFilterCallback, vclosure* callback) {
	RVec3 pos = RVec3(origin->mF64[0], origin->mF64[1], origin->mF64[2]);
	RShapeCast shapeCast = RShapeCast::sFromWorldTransform(
		shape->ref,
		Vec3::sOne(),
		RMat44::sRotationTranslation(
			QuatArg(rotation->mF64[0], rotation->mF64[1], rotation->mF64[2], rotation->mF64[3]),
			pos
		),
		Vec3Arg(direction->mF64[0], direction->mF64[1], direction->mF64[2])
	);

	// TODO: expose more options
	ShapeCastSettings settings = {};

	AllHitCollisionCollector<CastShapeCollector> collector = {};

	BroadPhaseLayerFilterHL broadPhaseLayerFilter = {};
	if (broadPhaseLayerFilterCallback) {
		broadPhaseLayerFilter.shouldCollide = broadPhaseLayerFilterCallback;
	}

	ObjectLayerFilterHL objectLayerFilter = {};
	if (objectLayerFilterCallback) {
		objectLayerFilter.shouldCollide = objectLayerFilterCallback;
	}

	BodyFilterHL bodyFilter = {};
	if (bodyFilterCallback) {
		bodyFilter.shouldCollide = bodyFilterCallback;
	}

	hl_blocking(true);
	narrow_phase_query->CastShape(
		shapeCast,
		settings,
		pos,
		collector,
		broadPhaseLayerFilter,
		objectLayerFilter,
		bodyFilter
		// shapeFilter
	);
	hl_blocking(false);

	varray* hits;
	if (collector.HadHit()) {
		hits = hl_alloc_array(&hlt_dynobj, collector.mHits.size());
		ShapeCastResult** arr = hl_aptr(hits, ShapeCastResult*);
		for (unsigned int i = 0; i < collector.mHits.size(); ++i){
			arr[i] = &collector.mHits[i];
		}
	} else {
		hits = nullptr;
	}

	if (callback->hasValue) {
		((void(*)(void*, varray*))callback->fun)(callback->value, hits);
	} else {
		((void(*)(varray*))callback->fun)(hits);
	}
}
DEFINE_PRIM(_VOID, narrow_phase_query_cast_shape, NARROWQUERY SHAPE _STRUCT _STRUCT _STRUCT _FUN(_BOOL, _I32) _FUN(_BOOL, _I32) _FUN(_BOOL, _I32) _FUN(_VOID, _ARR));

HL_PRIM DVec3* HL_NAME(shape_cast_result_get_contact_point_on1)(ShapeCastResult* shape_cast_result) {
	Vec3 point = shape_cast_result->mContactPointOn1;

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(point.GetX(), point.GetY(), point.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, shape_cast_result_get_contact_point_on1, SHAPECASTRES);

HL_PRIM DVec3* HL_NAME(shape_cast_result_get_contact_point_on2)(ShapeCastResult* shape_cast_result) {
	Vec3 point = shape_cast_result->mContactPointOn2;

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(point.GetX(), point.GetY(), point.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, shape_cast_result_get_contact_point_on2, SHAPECASTRES);

HL_PRIM DVec3* HL_NAME(shape_cast_result_get_penetration_axis)(ShapeCastResult* shape_cast_result) {
	Vec3 axis = shape_cast_result->mPenetrationAxis;

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(axis.GetX(), axis.GetY(), axis.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, shape_cast_result_get_penetration_axis, SHAPECASTRES);

HL_PRIM double HL_NAME(shape_cast_result_get_penetration_depth)(ShapeCastResult* shape_cast_result) {
	return shape_cast_result->mPenetrationDepth;
}
DEFINE_PRIM(_F64, shape_cast_result_get_penetration_depth, SHAPECASTRES);

HL_PRIM int HL_NAME(shape_cast_result_get_sub_shape_id1)(ShapeCastResult* shape_cast_result) {
	return shape_cast_result->mSubShapeID1.GetValue();
}
DEFINE_PRIM(_I32, shape_cast_result_get_sub_shape_id1, SHAPECASTRES);

HL_PRIM int HL_NAME(shape_cast_result_get_sub_shape_id2)(ShapeCastResult* shape_cast_result) {
	return shape_cast_result->mSubShapeID2.GetValue();
}
DEFINE_PRIM(_I32, shape_cast_result_get_sub_shape_id2, SHAPECASTRES);

HL_PRIM int HL_NAME(shape_cast_result_get_body_id2)(ShapeCastResult* shape_cast_result) {
	return shape_cast_result->mBodyID2.GetIndexAndSequenceNumber();
}
DEFINE_PRIM(_I32, shape_cast_result_get_body_id2, SHAPECASTRES);

HL_PRIM double HL_NAME(shape_cast_result_get_fraction)(ShapeCastResult* shape_cast_result) {
	return shape_cast_result->mFraction;
}
DEFINE_PRIM(_F64, shape_cast_result_get_fraction, SHAPECASTRES);

HL_PRIM bool HL_NAME(shape_cast_result_is_backface)(ShapeCastResult* shape_cast_result) {
	return shape_cast_result->mIsBackFaceHit;
}
DEFINE_PRIM(_BOOL, shape_cast_result_is_backface, SHAPECASTRES);

HL_PRIM void HL_NAME(body_reset_force)(Body* body) {
	body->ResetForce();
}
DEFINE_PRIM(_VOID, body_reset_force, BODY);

HL_PRIM void HL_NAME(body_reset_torque)(Body* body) {
	body->ResetTorque();
}
DEFINE_PRIM(_VOID, body_reset_torque, BODY);

HL_PRIM _ShapeRef* HL_NAME(body_get_shape)(Body* body) {
	const Shape* r = body->GetShape();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = (Shape*)r;
    return ref;
}
DEFINE_PRIM(SHAPE, body_get_shape, BODY);

HL_PRIM MotionProperties* HL_NAME(body_get_motion_properties)(Body* body) {
	return body->GetMotionProperties();
}
DEFINE_PRIM(MOTIONPROPS, body_get_motion_properties, BODY);

HL_PRIM DVec3* HL_NAME(body_get_world_space_surface_normal)(Body* body, int inSubShapeID, DVec3* inPosition) {
	RVec3 pos = RVec3(inPosition->mF64[0], inPosition->mF64[1], inPosition->mF64[2]);
	SubShapeID subShapeID;
	subShapeID.SetValue(inSubShapeID);
	Vec3 normal = body->GetWorldSpaceSurfaceNormal(subShapeID, pos);

	DVec3* d = (DVec3*)hl_gc_alloc_noptr(sizeof(DVec3));
	d->Set(normal.GetX(), normal.GetY(), normal.GetZ());
    return d;
}
DEFINE_PRIM(_STRUCT, body_get_world_space_surface_normal, BODY _I32 _STRUCT);

HL_PRIM double HL_NAME(motion_properties_get_inverse_mass)(MotionProperties* motion_properties) {
	return motion_properties->GetInverseMass();
}
DEFINE_PRIM(_F64, motion_properties_get_inverse_mass, MOTIONPROPS);

HL_PRIM void HL_NAME(motion_properties_scale_to_mass)(MotionProperties* motion_properties, double inMass) {
	motion_properties->ScaleToMass(inMass);
}
DEFINE_PRIM(_VOID, motion_properties_scale_to_mass, MOTIONPROPS _F64);

HL_PRIM double HL_NAME(motion_properties_get_linear_damping)(MotionProperties* motion_properties) {
	return motion_properties->GetLinearDamping();
}
DEFINE_PRIM(_F64, motion_properties_get_linear_damping, MOTIONPROPS);

HL_PRIM void HL_NAME(motion_properties_set_linear_damping)(MotionProperties* motion_properties, double damping) {
	motion_properties->SetLinearDamping(damping);
}
DEFINE_PRIM(_VOID, motion_properties_set_linear_damping, MOTIONPROPS _F64);

HL_PRIM double HL_NAME(motion_properties_get_angular_damping)(MotionProperties* motion_properties) {
	return motion_properties->GetAngularDamping();
}
DEFINE_PRIM(_F64, motion_properties_get_angular_damping, MOTIONPROPS);

HL_PRIM void HL_NAME(motion_properties_set_angular_damping)(MotionProperties* motion_properties, double damping) {
	motion_properties->SetAngularDamping(damping);
}
DEFINE_PRIM(_VOID, motion_properties_set_angular_damping, MOTIONPROPS _F64);

HL_PRIM int HL_NAME(shape_get_sub_type)(_ShapeRef* shape) {
	return (int)shape->ref->GetSubType();
}
DEFINE_PRIM(_I32, shape_get_sub_type, SHAPE);

HL_PRIM _ShapeRef* HL_NAME(shape_scale)(_ShapeRef* shape, DVec3* inScale) {
	Shape::ShapeResult scaled = shape->ref->ScaleShape(Vec3Arg(inScale->mF64[0], inScale->mF64[1], inScale->mF64[2]));
	// JPH_ASSERT(scaled.IsValid());

	Shape* r = scaled.Get();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = (Shape*)r;
    return ref;
}
DEFINE_PRIM(SHAPE, shape_scale, SHAPE _STRUCT);

HL_PRIM _ShapeRef* HL_NAME(shape_scaled_get_inner_shape)(_ShapeRef* shape) {
	JPH_ASSERT(shape->ref->GetSubType() == EShapeSubType::Scaled);

	const Shape* r = ((ScaledShape*)shape->ref)->GetInnerShape();
	r->AddRef();

	_ShapeRef* ref = (_ShapeRef*)hl_gc_alloc_finalizer(sizeof(_ShapeRef));
    ref->finalise = finalize_shape_ref;
    ref->ref = (Shape*)r;
    return ref;
}
DEFINE_PRIM(SHAPE, shape_scaled_get_inner_shape, SHAPE);
