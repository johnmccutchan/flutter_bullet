import 'dart:ffi' as ffi;

import 'src/flutter_bullet_bindings.dart';
import 'package:ffi/ffi.dart';
import 'package:vector_math/vector_math.dart';

import 'shapes.dart';
import 'transform.dart';

class RigidBody implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(bindings.addresses.destroy_rigid_body.cast());

  ffi.Pointer<wpBody> _nativeBody;

  CollisionShape _shape;

  RigidBody._(this._nativeBody, this._shape);

  factory RigidBody(double mass, CollisionShape shape) {
    final nativeBody = bindings.create_rigid_body(mass, shape.nativeShape);
    final body = RigidBody._(nativeBody, shape);
    bindings.set_rigid_body_user_data(nativeBody, body);
    _finalizer.attach(body, nativeBody.cast(), detach: body);
    return body;
  }

  CollisionShape get shape => _shape;

  ffi.Pointer<wpBody> get nativeBody {
    return _nativeBody;
  }

  Transform get xform {
    return Transform.fromList(
        bindings.rigid_body_get_raw_transform(_nativeBody).asTypedList(16));
  }

  set xform(Transform t) {
    final storage = t.storage;
    final a = malloc.allocate<ffi.Float>(4 * 16);
    final array = a.asTypedList(16);
    for (int i = 0; i < 16; i++) {
      array[i] = storage[i];
    }
    bindings.rigid_body_set_raw_transform(_nativeBody, a);
    malloc.free(a);
  }
}
