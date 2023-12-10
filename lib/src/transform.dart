part of '../physics3d.dart';

// Only supports translation and rotation.
class Transform {
  // TODO(johnmccutchan): A lot of this code should be optimized to allocate
  // fewer temporary objects, etc.
  Matrix4 _impl;
  static Matrix3 _matrix3 = Matrix3.identity();

  Transform() : _impl = Matrix4.identity();

  Transform.fromList(Float32List v) : _impl = Matrix4.fromFloat32List(v);

  Vector3 get origin {
    return _impl.getTranslation();
  }

  set origin(Vector3 o) {
    _impl.setTranslation(o);
  }

  Quaternion get rotation {
    _impl.copyRotation(_matrix3);
    return Quaternion.fromRotation(_matrix3);
  }

  set rotation(Quaternion q) {
    q.copyRotationInto(_matrix3);
    _impl.setRotation(_matrix3);
  }

  UnmodifiableFloat32ListView get storage {
    return UnmodifiableFloat32ListView(_impl.storage);
  }
}
