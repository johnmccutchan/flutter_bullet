#include "flutter_bullet.h"

#include <cstddef>
#include <memory>

// Linear math
#include "bullet3/src/BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "bullet3/src/LinearMath/btMatrix3x3.h"
#include "bullet3/src/LinearMath/btVector3.h"
#include "bullet3/src/LinearMath/btQuaternion.h"
#include "bullet3/src/LinearMath/btTransform.h"
#include "bullet3/src/LinearMath/btMotionState.h"
#include "bullet3/src/LinearMath/btDefaultMotionState.h"

// Interfaces
#include "bullet3/src/BulletCollision/CollisionDispatch/btCollisionConfiguration.h"
#include "bullet3/src/BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "bullet3/src/BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btConstraintSolver.h"

// Implementation
#include "bullet3/src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "bullet3/src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "bullet3/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

// Collidable
#include "bullet3/src/BulletCollision/CollisionDispatch/btCollisionObject.h"

// Rigid body
#include "bullet3/src/BulletDynamics/Dynamics/btRigidBody.h"

// Shapes
#include "bullet3/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet3/src/BulletCollision/CollisionShapes/btBoxShape.h"
#include "bullet3/src/BulletCollision/CollisionShapes/btStaticPlaneShape.h"

#include "dart_api.h"
#include "dart_api_dl.h"

static void NoopFinalizer(void* isolate_callback_data, void* peer) {
  // We must pass in a non-null callback to Dart_NewWeakPersistentHandle_DL.
}

class WrappedPhysicsWorld {
 public:
  WrappedPhysicsWorld() {
    configuration_ = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher_ = std::make_unique<btCollisionDispatcher>(configuration_.get());
    broadphase_ = std::make_unique<btDbvtBroadphase>();
    constraint_solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
    world_ = std::make_unique<btDiscreteDynamicsWorld>(dispatcher_.get(), broadphase_.get(), constraint_solver_.get(), configuration_.get());
  }

  ~WrappedPhysicsWorld() {
  }

  btDynamicsWorld* world() {
    return world_.get();
  }

 private:
  std::unique_ptr<btCollisionConfiguration> configuration_;
  std::unique_ptr<btCollisionDispatcher> dispatcher_;
  std::unique_ptr<btBroadphaseInterface> broadphase_;
  std::unique_ptr<btConstraintSolver> constraint_solver_;
  std::unique_ptr<btDynamicsWorld> world_;
};

FFI_PLUGIN_EXPORT wpWorld* create_world() {
  return reinterpret_cast<wpWorld*>(new WrappedPhysicsWorld());
}

FFI_PLUGIN_EXPORT void step_world(wpWorld* world, float dt) {
  WrappedPhysicsWorld* _world = reinterpret_cast<WrappedPhysicsWorld*>(world);
  _world->world()->stepSimulation(dt);
}

FFI_PLUGIN_EXPORT void world_add_rigid_body(wpWorld* world, wpBody* body) {
  WrappedPhysicsWorld* _world = reinterpret_cast<WrappedPhysicsWorld*>(world);
  btRigidBody* _body = reinterpret_cast<btRigidBody*>(body);
  _world->world()->addRigidBody(_body);
}

FFI_PLUGIN_EXPORT void world_remove_rigid_body(wpWorld* world, wpBody* body) {
  WrappedPhysicsWorld* _world = reinterpret_cast<WrappedPhysicsWorld*>(world);
  btRigidBody* _body = reinterpret_cast<btRigidBody*>(body);
  _world->world()->removeRigidBody(_body);
}

FFI_PLUGIN_EXPORT void world_add_collidable(wpWorld* world, wpCollidable* collidable) {
  WrappedPhysicsWorld* _world = reinterpret_cast<WrappedPhysicsWorld*>(world);
  btCollisionObject* _object = reinterpret_cast<btCollisionObject*>(collidable);
  _world->world()->addCollisionObject(_object);
}

FFI_PLUGIN_EXPORT void world_remove_collidable(wpWorld* world, wpCollidable* collidable) {
  WrappedPhysicsWorld* _world = reinterpret_cast<WrappedPhysicsWorld*>(world);
  btCollisionObject* _object = reinterpret_cast<btCollisionObject*>(collidable);
  _world->world()->removeCollisionObject(_object);
}

FFI_PLUGIN_EXPORT void destroy_world(wpWorld* world) {
  WrappedPhysicsWorld* _world = reinterpret_cast<WrappedPhysicsWorld*>(world);
  delete _world;
}

FFI_PLUGIN_EXPORT wpCollidable* create_collidable() {
  btCollisionObject* object = new btCollisionObject();
  return reinterpret_cast<wpCollidable*>(object);
}

FFI_PLUGIN_EXPORT void collidable_set_dart_owner(wpCollidable* collidable, Dart_Handle owner) {
  btCollisionObject* _object = reinterpret_cast<btCollisionObject*>(collidable);
  Dart_WeakPersistentHandle weak_ref = Dart_NewWeakPersistentHandle_DL(owner, nullptr, 0, NoopFinalizer);
  _object->setUserPointer(reinterpret_cast<void*>(weak_ref));
}

FFI_PLUGIN_EXPORT Dart_Handle collidable_get_dart_owner(wpCollidable* collidable) {
  btCollisionObject* _object = reinterpret_cast<btCollisionObject*>(collidable);
  return Dart_HandleFromWeakPersistent_DL(reinterpret_cast<Dart_WeakPersistentHandle>(_object->getUserPointer()));
}

FFI_PLUGIN_EXPORT void collidable_set_shape(wpCollidable* collidable, wpShape* shape) {
  btCollisionObject* _object = reinterpret_cast<btCollisionObject*>(collidable);
  btCollisionShape* _shape = reinterpret_cast<btCollisionShape*>(shape);
  _object->setCollisionShape(_shape);
}

FFI_PLUGIN_EXPORT const float* collidable_get_raw_transform(wpCollidable* collidable) {
  btCollisionObject* _object = reinterpret_cast<btCollisionObject*>(collidable);
  const btTransform& xform = _object->getWorldTransform();
  const btMatrix3x3& rotation = xform.getBasis();
  const btVector3& start = rotation.getRow(0);
  return static_cast<const btScalar*>(start);
}

FFI_PLUGIN_EXPORT void collidable_set_raw_transform(wpCollidable* collidable, const float* m) {
  btCollisionObject* _object = reinterpret_cast<btCollisionObject*>(collidable);
  btTransform tform;
  tform.setFromOpenGLMatrix(m);
  if (btRigidBody::upcast(_object) != nullptr) {
    btRigidBody* _body = btRigidBody::upcast(_object);
    _body->setCenterOfMassTransform(tform);
  } else {
    _object->setWorldTransform(tform);
  }
}

FFI_PLUGIN_EXPORT void destroy_collidable(wpCollidable* collidable) {
  btCollisionObject* _object = reinterpret_cast<btCollisionObject*>(collidable);
  if (_object->getUserPointer() != nullptr) {
    Dart_DeleteWeakPersistentHandle_DL(reinterpret_cast<Dart_WeakPersistentHandle>(_object->getUserPointer()));
  }
  if (btRigidBody::upcast(_object) != nullptr) {
    btRigidBody* _body = btRigidBody::upcast(_object);
    btMotionState* motionState = _body->getMotionState();
    delete motionState;
    delete _body;
  } else {
    delete _object;
  }
}

FFI_PLUGIN_EXPORT wpBody* create_rigid_body(float mass, wpShape* shape) {
  btCollisionShape* _shape = reinterpret_cast<btCollisionShape*>(shape);
  btTransform t;
  t.setIdentity();

  bool isDynamic = (mass != 0.0f);

  btVector3 localInertia(0, 0, 0);
  if (isDynamic)
			_shape->calculateLocalInertia(mass, localInertia);

  // TODO(johnmccutchan): We probably need to allow for Dart code to implement a motion state in the future
  // as this is the primary way to synchronize graphics transforms with physics simulation. For now
  // we can poll.
  btDefaultMotionState* motionState = new btDefaultMotionState();

  // When using a motion state you have to specify the initial transform in the motion state.
  motionState->setWorldTransform(t);

  return reinterpret_cast<wpBody*>(new btRigidBody(mass, motionState, _shape, localInertia));
}

FFI_PLUGIN_EXPORT wpShape* create_box_shape(float x, float y, float z) {
  btVector3 halfExtents(x, y, z);
  return reinterpret_cast<wpShape*>(new btBoxShape(halfExtents));
}

FFI_PLUGIN_EXPORT wpShape* create_static_plane_shape(float nx, float ny, float nz, float c) {
  btVector3 planeNormal(nx, ny, nz);
  btScalar planeConstant = c;
  return reinterpret_cast<wpShape*>(new btStaticPlaneShape(planeNormal, planeConstant));
}

FFI_PLUGIN_EXPORT void shape_set_dart_owner(wpShape* shape, Dart_Handle owner) {
  btCollisionShape* _shape = reinterpret_cast<btCollisionShape*>(shape);
  Dart_WeakPersistentHandle weak_ref = Dart_NewWeakPersistentHandle_DL(owner, nullptr, 0, NoopFinalizer);
  _shape->setUserPointer(reinterpret_cast<void*>(weak_ref));
}

FFI_PLUGIN_EXPORT Dart_Handle shape_get_dart_owner(wpShape* shape) {
  btCollisionShape* _shape = reinterpret_cast<btCollisionShape*>(shape);
  return Dart_HandleFromWeakPersistent_DL(reinterpret_cast<Dart_WeakPersistentHandle>(_shape->getUserPointer()));
}


FFI_PLUGIN_EXPORT void destroy_shape(wpShape* shape) {
  btCollisionShape* _shape = reinterpret_cast<btCollisionShape*>(shape);
  if (_shape->getUserPointer() != nullptr) {
    Dart_DeleteWeakPersistentHandle_DL(reinterpret_cast<Dart_WeakPersistentHandle>(_shape->getUserPointer()));
  }
  delete _shape;
}
