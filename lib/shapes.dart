import 'dart:ffi' as ffi;

import 'src/flutter_bullet_bindings.dart';
import 'package:vector_math/vector_math.dart';

class CollisionShape implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(bindings.addresses.destroy_shape.cast());

  ffi.Pointer<wpShape> _nativeShape;

  ffi.Pointer<wpShape> get nativeShape {
    return _nativeShape;
  }

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
