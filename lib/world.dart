import 'dart:ffi' as ffi;

import 'src/flutter_bullet_bindings.dart';
import 'package:vector_math/vector_math.dart';

import 'rigid_body.dart';

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
    bindings.world_add_rigid_body(_nativeWorld, body.nativeBody);
  }

  void removeBody(RigidBody body) {
    if (!_bodies.remove(body)) {
      // Never added.
      return;
    }
    bindings.world_remove_rigid_body(_nativeWorld, body.nativeBody);
  }
}
