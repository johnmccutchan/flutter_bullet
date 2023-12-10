part of '../physics3d.dart';

class RigidBody extends Collidable {
  RigidBody._(ffi.Pointer<wpBody> body)
      : super._(body as ffi.Pointer<wpCollidable>);

  ffi.Pointer<wpBody> get _nativeBody {
    return _nativeCollidable as ffi.Pointer<wpBody>;
  }

  factory RigidBody(double mass, CollisionShape shape) {
    final nativeBody = bindings.create_rigid_body(mass, shape._nativeShape);
    final body = RigidBody._(nativeBody);
    body._shape = shape;
    return body;
  }
}
