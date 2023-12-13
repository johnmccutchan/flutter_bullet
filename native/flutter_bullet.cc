#include "flutter_bullet.h"

#include <cstddef>
#include <memory>

// Linear math
#include "bullet3/src/BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "bullet3/src/LinearMath/btDefaultMotionState.h"
#include "bullet3/src/LinearMath/btMatrix3x3.h"
#include "bullet3/src/LinearMath/btMotionState.h"
#include "bullet3/src/LinearMath/btQuaternion.h"
#include "bullet3/src/LinearMath/btTransform.h"
#include "bullet3/src/LinearMath/btVector3.h"

// Interfaces
#include "bullet3/src/BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "bullet3/src/BulletCollision/CollisionDispatch/btCollisionConfiguration.h"
#include "bullet3/src/BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btConstraintSolver.h"

// Implementation
#include "bullet3/src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "bullet3/src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "bullet3/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

// Collidable
#include "bullet3/src/BulletCollision/CollisionDispatch/btCollisionObject.h"

// Rigid body
#include "bullet3/src/BulletDynamics/Dynamics/btRigidBody.h"

// Shapes
#include "bullet3/src/BulletCollision/CollisionShapes/btBoxShape.h"
#include "bullet3/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet3/src/BulletCollision/CollisionShapes/btStaticPlaneShape.h"

#include "dart_api.h"
#include "dart_api_dl.h"

static void NoopFinalizer(void *isolate_callback_data, void *peer) {
  // We must pass in a non-null callback to Dart_NewWeakPersistentHandle_DL.
}

class WrappedPhysicsWorld {
public:
  WrappedPhysicsWorld() {
    configuration_ = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher_ = std::make_unique<btCollisionDispatcher>(configuration_.get());
    broadphase_ = std::make_unique<btDbvtBroadphase>();
    constraint_solver_ =
        std::make_unique<btSequentialImpulseConstraintSolver>();
    world_ = std::make_unique<btDiscreteDynamicsWorld>(
        dispatcher_.get(), broadphase_.get(), constraint_solver_.get(),
        configuration_.get());
  }

  ~WrappedPhysicsWorld() {}

  btDynamicsWorld *world() { return world_.get(); }

private:
  std::unique_ptr<btCollisionConfiguration> configuration_;
  std::unique_ptr<btCollisionDispatcher> dispatcher_;
  std::unique_ptr<btBroadphaseInterface> broadphase_;
  std::unique_ptr<btConstraintSolver> constraint_solver_;
  std::unique_ptr<btDynamicsWorld> world_;
};

// An instance of this is allocated and stored inside each btCollisionObject's userPointer field.
class CollidableState : public btMotionState {
 public:
  CollidableState(btCollisionObject* object,
                  Dart_Handle owner,
                  void (*transform_update)(Dart_Handle)) :
                  object_(object),
                  owner_(Dart_NewWeakPersistentHandle_DL(owner, nullptr, 0, NoopFinalizer)),
                  transform_update_(transform_update) {
    // Identity.
    matrix_[0] = 1.0;
    matrix_[3] = 1.0;
    matrix_[7] = 1.0;
    matrix_[11] = 1.0;
    matrix_[15] = 1.0;
  }

  virtual ~CollidableState() {
    Dart_DeleteWeakPersistentHandle_DL(owner_);
  }

  virtual void getWorldTransform(btTransform& worldTrans) const {
    worldTrans.setFromOpenGLMatrix(&matrix_[0]);
  }

	virtual void setWorldTransform(const btTransform& worldTrans) {
    worldTrans.getOpenGLMatrix(&matrix_[0]);
    if (transform_update_ != nullptr) {
      transform_update_(Dart_HandleFromWeakPersistent_DL(owner_));
    }
  }

  btCollisionObject* object() {
    return object_;
  }

  float* matrix() {
    return &matrix_[0];
  }

  Dart_WeakPersistentHandle owner() {
    return owner_;
  }

 private:
  // The raw Transformation matrix.
  btScalar matrix_[16];
  btCollisionObject* object_;
  Dart_WeakPersistentHandle owner_;
  void (*transform_update_)(Dart_Handle);
};


FFI_PLUGIN_EXPORT wpWorld *create_world() {
  return reinterpret_cast<wpWorld *>(new WrappedPhysicsWorld());
}

FFI_PLUGIN_EXPORT void step_world(wpWorld *world, float dt) {
  WrappedPhysicsWorld *_world = reinterpret_cast<WrappedPhysicsWorld *>(world);
  _world->world()->stepSimulation(dt);
}

FFI_PLUGIN_EXPORT void world_add_rigid_body(wpWorld *world, wpBody *body) {
  WrappedPhysicsWorld *_world = reinterpret_cast<WrappedPhysicsWorld *>(world);
  btRigidBody *_body = reinterpret_cast<btRigidBody *>(body);
  _world->world()->addRigidBody(_body);
}

FFI_PLUGIN_EXPORT void world_remove_rigid_body(wpWorld *world, wpBody *body) {
  WrappedPhysicsWorld *_world = reinterpret_cast<WrappedPhysicsWorld *>(world);
  btRigidBody *_body = reinterpret_cast<btRigidBody *>(body);
  _world->world()->removeRigidBody(_body);
}

FFI_PLUGIN_EXPORT void world_add_collidable(wpWorld *world,
                                            wpCollidable *collidable) {
  WrappedPhysicsWorld *_world = reinterpret_cast<WrappedPhysicsWorld *>(world);
  btCollisionObject *_object =
      reinterpret_cast<btCollisionObject *>(collidable);
  _world->world()->addCollisionObject(_object);
}

FFI_PLUGIN_EXPORT void world_remove_collidable(wpWorld *world,
                                               wpCollidable *collidable) {
  WrappedPhysicsWorld *_world = reinterpret_cast<WrappedPhysicsWorld *>(world);
  btCollisionObject *_object =
      reinterpret_cast<btCollisionObject *>(collidable);
  _world->world()->removeCollisionObject(_object);
}

struct ImplRayResultCallback : public btCollisionWorld::RayResultCallback {
public:
  ImplRayResultCallback(float (*cb)(Dart_Handle collidable, float fraction,
                                    const float *m))
      : m_cb(cb) {}

  virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult &rayResult,
                                   bool normalInWorldSpace) {
    btVector3 hitNormalWorld;
    if (normalInWorldSpace) {
      hitNormalWorld = rayResult.m_hitNormalLocal;
    } else {
      /// need to transform normal into worldspace
      hitNormalWorld =
          rayResult.m_collisionObject->getWorldTransform().getBasis() *
          rayResult.m_hitNormalLocal;
    }
    CollidableState* state = reinterpret_cast<CollidableState*>(rayResult.m_collisionObject->getUserPointer());
    return m_cb(Dart_HandleFromWeakPersistent_DL(state->owner()),
                rayResult.m_hitFraction,
                static_cast<btScalar *>(hitNormalWorld));
  };

  float (*m_cb)(Dart_Handle collidable, float fraction, const float *m);
};

void world_raycast(wpWorld *world, float sx, float sy, float sz, float ex,
                   float ey, float ez,
                   float (*cb)(Dart_Handle collidable, float fraction,
                               const float *n)) {
  WrappedPhysicsWorld *_world = reinterpret_cast<WrappedPhysicsWorld *>(world);
  btVector3 start(sx, sy, sz);
  btVector3 end(ex, ey, ez);
  ImplRayResultCallback callback(cb);
  _world->world()->rayTest(start, end, callback);
}

FFI_PLUGIN_EXPORT void destroy_world(wpWorld *world) {
  WrappedPhysicsWorld *_world = reinterpret_cast<WrappedPhysicsWorld *>(world);
  delete _world;
}

FFI_PLUGIN_EXPORT wpCollidable *create_collidable() {
  btCollisionObject *object = new btCollisionObject();
  return reinterpret_cast<wpCollidable *>(object);
}

FFI_PLUGIN_EXPORT void collidable_set_dart_owner(wpCollidable *collidable,
                                                 Dart_Handle owner) {
  btCollisionObject *_object =
      reinterpret_cast<btCollisionObject *>(collidable);
  Dart_WeakPersistentHandle weak_ref =
      Dart_NewWeakPersistentHandle_DL(owner, nullptr, 0, NoopFinalizer);
  _object->setUserPointer(reinterpret_cast<void *>(weak_ref));
}

FFI_PLUGIN_EXPORT void collidable_set_shape(wpCollidable *collidable,
                                            wpShape *shape) {
  btCollisionObject *_object =
      reinterpret_cast<btCollisionObject *>(collidable);
  btCollisionShape *_shape = reinterpret_cast<btCollisionShape *>(shape);
  _object->setCollisionShape(_shape);
}

FFI_PLUGIN_EXPORT void destroy_collidable(wpCollidable *collidable) {
  btCollisionObject *_object =
      reinterpret_cast<btCollisionObject *>(collidable);
  if (_object->getUserPointer() != nullptr) {
    CollidableState* state = reinterpret_cast<CollidableState*>(_object->getUserPointer());
    delete state;
  }
  delete _object;
}

FFI_PLUGIN_EXPORT wpCollidableState* collidable_create_state(wpCollidable* collidable,
                                                             Dart_Handle owner,
                                                             void (*transform_update)(Dart_Handle)) {
  btCollisionObject* object = reinterpret_cast<btCollisionObject*>(collidable);
  CollidableState* state = new CollidableState(object, owner, transform_update);
  btCollisionObject *_object =
      reinterpret_cast<btCollisionObject *>(collidable);
  _object->setUserPointer(state);
  btRigidBody *_body = btRigidBody::upcast(_object);
  if (_body != nullptr) {
    _body->setMotionState(state);
  }
  return reinterpret_cast<wpCollidableState*>(state);
}

FFI_PLUGIN_EXPORT Dart_Handle collidable_state_get_dart_owner(wpCollidableState* collidable_state) {
  CollidableState* state = reinterpret_cast<CollidableState*>(collidable_state);
  return Dart_HandleFromWeakPersistent_DL(
        reinterpret_cast<Dart_WeakPersistentHandle>(state->owner()));
}

FFI_PLUGIN_EXPORT float* collidable_state_get_matrix(wpCollidableState* collidable_state) {
  CollidableState* state = reinterpret_cast<CollidableState*>(collidable_state);
  return state->matrix();
}

FFI_PLUGIN_EXPORT void collidable_state_matrix_updated(wpCollidableState* collidable_state) {
  CollidableState* state = reinterpret_cast<CollidableState*>(collidable_state);
  btCollisionObject *_object = state->object();
  btTransform tform;
  tform.setFromOpenGLMatrix(state->matrix());
  btRigidBody *_body = btRigidBody::upcast(_object);
  if (_body != nullptr) {
    _body->setCenterOfMassTransform(tform);
  } else {
    _object->setWorldTransform(tform);
  }
}

FFI_PLUGIN_EXPORT wpBody *create_rigid_body(float mass, wpShape *shape) {
  btCollisionShape *_shape = reinterpret_cast<btCollisionShape *>(shape);
  btTransform t;
  t.setIdentity();

  bool isDynamic = (mass != 0.0f);

  btVector3 localInertia(0, 0, 0);
  if (isDynamic)
    _shape->calculateLocalInertia(mass, localInertia);

  // TODO(johnmccutchan): We probably need to allow for Dart code to implement a
  // motion state in the future as this is the primary way to synchronize
  // graphics transforms with physics simulation. For now we can poll.
  btDefaultMotionState *motionState = new btDefaultMotionState();

  // When using a motion state you have to specify the initial transform in the
  // motion state.
  motionState->setWorldTransform(t);

  return reinterpret_cast<wpBody *>(
      new btRigidBody(mass, motionState, _shape, localInertia));
}

FFI_PLUGIN_EXPORT wpShape *create_box_shape(float x, float y, float z) {
  btVector3 halfExtents(x, y, z);
  return reinterpret_cast<wpShape *>(new btBoxShape(halfExtents));
}

FFI_PLUGIN_EXPORT wpShape *create_static_plane_shape(float nx, float ny,
                                                     float nz, float c) {
  btVector3 planeNormal(nx, ny, nz);
  btScalar planeConstant = c;
  return reinterpret_cast<wpShape *>(
      new btStaticPlaneShape(planeNormal, planeConstant));
}

FFI_PLUGIN_EXPORT void shape_set_dart_owner(wpShape *shape, Dart_Handle owner) {
  btCollisionShape *_shape = reinterpret_cast<btCollisionShape *>(shape);
  Dart_WeakPersistentHandle weak_ref =
      Dart_NewWeakPersistentHandle_DL(owner, nullptr, 0, NoopFinalizer);
  _shape->setUserPointer(reinterpret_cast<void *>(weak_ref));
}

FFI_PLUGIN_EXPORT Dart_Handle shape_get_dart_owner(wpShape *shape) {
  btCollisionShape *_shape = reinterpret_cast<btCollisionShape *>(shape);
  return Dart_HandleFromWeakPersistent_DL(
      reinterpret_cast<Dart_WeakPersistentHandle>(_shape->getUserPointer()));
}

FFI_PLUGIN_EXPORT void destroy_shape(wpShape *shape) {
  btCollisionShape *_shape = reinterpret_cast<btCollisionShape *>(shape);
  if (_shape->getUserPointer() != nullptr) {
    Dart_DeleteWeakPersistentHandle_DL(
        reinterpret_cast<Dart_WeakPersistentHandle>(_shape->getUserPointer()));
  }
  delete _shape;
}
