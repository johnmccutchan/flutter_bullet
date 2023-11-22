// You have generated a new plugin project without specifying the `--platforms`
// flag. An FFI plugin project that supports no platforms is generated.
// To add platforms, run `flutter create -t plugin_ffi --platforms <platforms> .`
// in this directory. You can also find a detailed instruction on how to
// add platforms in the `pubspec.yaml` at
// https://flutter.dev/docs/development/packages-and-plugins/developing-packages#plugin-platforms.

import 'dart:ffi' as ffi;
import 'dart:io';

import 'src/flutter_bullet_bindings_generated.dart';
import 'package:vector_math/vector_math.dart';

const String _libName = 'flutter_bullet';

/// The dynamic library in which the symbols for [FlutterBulletBindings] can be found.
final ffi.DynamicLibrary _dylib = () {
  if (Platform.isMacOS || Platform.isIOS) {
    return ffi.DynamicLibrary.open(
        '/Users/johnmccutchan/workspace/flutter_bullet/flutter_bullet_library/lib$_libName.dylib');
  }
  if (Platform.isAndroid || Platform.isLinux) {
    return ffi.DynamicLibrary.open('lib$_libName.so');
  }
  if (Platform.isWindows) {
    return ffi.DynamicLibrary.open('$_libName.dll');
  }
  throw UnsupportedError('Unknown platform: ${Platform.operatingSystem}');
}();

/// The bindings to the native functions in [_dylib].
final FlutterBulletBindings _bindings = FlutterBulletBindings(_dylib);

/// Physics world that can be populated with rigid bodies.
class World implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(_bindings.addresses.destroy_world.cast());

  ffi.Pointer<wpWorld> _nativeWorld;

  World._(this._nativeWorld);

  factory World() {
    final nativeWorld = _bindings.create_world();
    final world = World._(nativeWorld);
    _finalizer.attach(world, nativeWorld.cast(), detach: world);
    return world;
  }

  // Step the simulation forward by dt.
  void step(double dt) {
    _bindings.step_world(_nativeWorld, dt);
  }

  void addBody(RigidBody body) {
    _bindings.world_add_rigid_body(_nativeWorld, body._nativeBody);
  }

  void removeBody(RigidBody body) {
    _bindings.world_remove_rigid_body(_nativeWorld, body._nativeBody);
  }
}

// Collision shape
class CollisionShape implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(_bindings.addresses.destroy_shape.cast());

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
        _bindings.create_box_shape(halfExtents.x, halfExtents.y, halfExtents.z);
    return BoxShape._(nativeShape);
  }
}

class StaticPlaneShape extends ConcaveShape {
  StaticPlaneShape._(ffi.Pointer<wpShape> nativeShape) : super._(nativeShape);

  factory StaticPlaneShape(Vector3 planeNormal, double planeConstant) {
    final nativeShape = _bindings.create_static_plane_shape(
        planeNormal.x, planeNormal.y, planeNormal.z, planeConstant);
    return StaticPlaneShape._(nativeShape);
  }
}

// Rigid body.
class RigidBody implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(_bindings.addresses.destroy_rigid_body.cast());

  ffi.Pointer<wpBody> _nativeBody;
  CollisionShape _shape;

  RigidBody._(this._nativeBody, this._shape);

  factory RigidBody(double mass, CollisionShape shape, Vector3 origin) {
    final nativeBody = _bindings.create_rigid_body(
        mass, shape._nativeShape, origin.x, origin.y, origin.z);
    final body = RigidBody._(nativeBody, shape);
    _finalizer.attach(body, nativeBody.cast(), detach: body);
    return body;
  }

  CollisionShape get shape => _shape;

  Vector3 get origin {
    final array = _bindings.rigid_body_get_origin(_nativeBody).asTypedList(3);
    return new Vector3(array[0], array[1], array[2]);
  }
}
