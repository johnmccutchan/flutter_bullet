#include "flutter_bullet.h"

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

// Rigid body
#include "bullet3/src/BulletDynamics/Dynamics/btRigidBody.h"

// Shapes
#include "bullet3/src/BulletCollision/CollisionShapes/btBoxShape.h"
#include "bullet3/src/BulletCollision/CollisionShapes/btStaticPlaneShape.h"

#include "dart_api.h"
#include "dart_api_dl.h"

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
  printf("LLL\n");
}

FFI_PLUGIN_EXPORT void world_remove_rigid_body(wpWorld* world, wpBody* body) {
  WrappedPhysicsWorld* _world = reinterpret_cast<WrappedPhysicsWorld*>(world);
  btRigidBody* _body = reinterpret_cast<btRigidBody*>(body);
  _world->world()->removeRigidBody(_body);
  printf("KKK\n");
}

FFI_PLUGIN_EXPORT void destroy_world(wpWorld* world) {
  WrappedPhysicsWorld* _world = reinterpret_cast<WrappedPhysicsWorld*>(world);
  delete _world;
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

FFI_PLUGIN_EXPORT void destroy_shape(wpShape* shape) {
  btCollisionShape* _shape = reinterpret_cast<btCollisionShape*>(shape);
  delete _shape;
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

static void NoopFinalizer(void* isolate_callback_data, void* peer) {
  // We must pass in a non-null callback to Dart_NewWeakPersistentHandle_DL.
}

FFI_PLUGIN_EXPORT void set_rigid_body_user_data(wpBody* body, Dart_Handle ref) {
  btRigidBody* _body = reinterpret_cast<btRigidBody*>(body);
  Dart_WeakPersistentHandle weak_ref = Dart_NewWeakPersistentHandle_DL(ref, nullptr, 0, NoopFinalizer);
  void* user_data = reinterpret_cast<void*>(weak_ref);
  _body->setUserPointer(user_data);
}

FFI_PLUGIN_EXPORT Dart_Handle get_rigid_body_user_data(wpBody* body) {
  btRigidBody* _body = reinterpret_cast<btRigidBody*>(body);
  void* user_data = _body->getUserPointer();
  return Dart_HandleFromWeakPersistent_DL(reinterpret_cast<Dart_WeakPersistentHandle>(_body->getUserPointer()));
}

FFI_PLUGIN_EXPORT const float* rigid_body_get_origin(wpBody* body) {
  btRigidBody* _body = reinterpret_cast<btRigidBody*>(body);
  return static_cast<const btScalar*>(_body->getCenterOfMassPosition());
}

FFI_PLUGIN_EXPORT const float* rigid_body_get_raw_transform(wpBody* body) {
  btRigidBody* _body = reinterpret_cast<btRigidBody*>(body);
  const btTransform& xform = _body->getCenterOfMassTransform();
  const btMatrix3x3& rotation = xform.getBasis();
  const btVector3& start = rotation.getRow(0);
  return static_cast<const btScalar*>(start);
}

FFI_PLUGIN_EXPORT void rigid_body_set_raw_transform(wpBody* body, const float* m) {
  btRigidBody* _body = reinterpret_cast<btRigidBody*>(body);
  btTransform tform;
  tform.setFromOpenGLMatrix(m);
  _body->setCenterOfMassTransform(tform);
}

FFI_PLUGIN_EXPORT void destroy_rigid_body(wpBody* body) {
  btRigidBody* _body = reinterpret_cast<btRigidBody*>(body);
  void* user_data = _body->getUserPointer();
  if (user_data != nullptr) {
    Dart_DeleteWeakPersistentHandle_DL(reinterpret_cast<Dart_WeakPersistentHandle>(user_data));
  }
  btMotionState* motionState = _body->getMotionState();
  delete motionState;
  delete _body;
}
