import 'dart:ffi' as ffi;

import 'src/flutter_bullet_bindings.dart';
import 'package:vector_math/vector_math.dart';

/// Physics world that can be populated with rigid bodies.
class World implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(bindings.addresses.destroy_world.cast());

  ffi.Pointer<wpWorld> _nativeWorld;

  Set<RigidBody> _bodies = new Set<RigidBody>();

  World._(this._nativeWorld);

  factory World() {
    final nativeWorld = bindings.create_world();
    final world = World._(nativeWorld);
    _finalizer.attach(world, nativeWorld.cast(), detach: world);
    return world;
  }

  // Step the simulation forward by dt.
  void step(double dt) {
    bindings.step_world(_nativeWorld, dt);
  }

  void addBody(RigidBody body) {
    if (!_bodies.add(body)) {
      // Already added.
      return;
    }
    bindings.world_add_rigid_body(_nativeWorld, body._nativeBody);
  }

  void removeBody(RigidBody body) {
    if (!_bodies.remove(body)) {
      // Never added.
      return;
    }
    bindings.world_remove_rigid_body(_nativeWorld, body._nativeBody);
  }
}

// Collision shape
class CollisionShape implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(bindings.addresses.destroy_shape.cast());

  ffi.Pointer<wpShape> _nativeShape;

  CollisionShape._(this._nativeShape) {
    _finalizer.attach(this, _nativeShape.cast(), detach: this);
  }
}

// Convex shape.
class ConvexShape extends CollisionShape {
  ConvexShape._(ffi.Pointer<wpShape> nativeShape) : super._(nativeShape);
}

class ConcaveShape extends CollisionShape {
  ConcaveShape._(ffi.Pointer<wpShape> nativeShape) : super._(nativeShape);
}

// Box shape.
class BoxShape extends ConvexShape {
  BoxShape._(ffi.Pointer<wpShape> nativeShape) : super._(nativeShape);

  factory BoxShape(Vector3 halfExtents) {
    final nativeShape =
        bindings.create_box_shape(halfExtents.x, halfExtents.y, halfExtents.z);
    return BoxShape._(nativeShape);
  }
}

class StaticPlaneShape extends ConcaveShape {
  StaticPlaneShape._(ffi.Pointer<wpShape> nativeShape) : super._(nativeShape);

  factory StaticPlaneShape(Vector3 planeNormal, double planeConstant) {
    final nativeShape = bindings.create_static_plane_shape(
        planeNormal.x, planeNormal.y, planeNormal.z, planeConstant);
    return StaticPlaneShape._(nativeShape);
  }
}

// Rigid body.
class RigidBody implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(bindings.addresses.destroy_rigid_body.cast());

  ffi.Pointer<wpBody> _nativeBody;
  CollisionShape _shape;

  RigidBody._(this._nativeBody, this._shape);

  factory RigidBody(double mass, CollisionShape shape, Vector3 origin) {
    final nativeBody = bindings.create_rigid_body(
        mass, shape._nativeShape, origin.x, origin.y, origin.z);
    final body = RigidBody._(nativeBody, shape);
    final weakBody = new WeakReference(body);
    bindings.set_rigid_body_user_data(nativeBody, weakBody);
    WeakReference<RigidBody> retrievedWeakBody = bindings
        .get_rigid_body_user_data(nativeBody) as WeakReference<RigidBody>;
    print(identical(weakBody.target, retrievedWeakBody.target));
    _finalizer.attach(body, nativeBody.cast(), detach: body);
    return body;
  }

  CollisionShape get shape => _shape;

  Vector3 get origin {
    final array = bindings.rigid_body_get_origin(_nativeBody).asTypedList(3);
    return new Vector3(array[0], array[1], array[2]);
  }
}
