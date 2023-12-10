part of '../physics3d.dart';

/// Physics world that can be populated with rigid bodies.
class World implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(bindings.addresses.destroy_world.cast());

  ffi.Pointer<wpWorld> _nativeWorld;

  Set<RigidBody> _bodies = new Set<RigidBody>();
  Set<Collidable> _collidables = new Set<Collidable>();

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
      // Not present.
      return;
    }
    bindings.world_remove_rigid_body(_nativeWorld, body._nativeBody);
  }

  void addCollidable(Collidable collidable) {
    if (collidable is RigidBody) {
      addBody(collidable);
      return;
    }
    if (!_collidables.add(collidable)) {
      // Already added.
      return;
    }
    _collidables.add(collidable);
    bindings.world_add_collidable(_nativeWorld, collidable._nativeCollidable);
  }

  void removeCollidable(Collidable collidable) {
    if (collidable is RigidBody) {
      removeBody(collidable);
      return;
    }
    if (!_collidables.remove(collidable)) {
      // Not present.
      return;
    }
    _collidables.add(collidable);
    bindings.world_remove_collidable(
        _nativeWorld, collidable._nativeCollidable);
  }
}
