// ignore_for_file: always_specify_types
// ignore_for_file: camel_case_types
// ignore_for_file: non_constant_identifier_names

// AUTO GENERATED FILE, DO NOT EDIT.
//
// Generated by `package:ffigen`.
// ignore_for_file: type=lint
import 'dart:ffi' as ffi;

/// Bindings for `flutter_bullet_library/flutter_bullet.h`.
///
/// Regenerate bindings with `flutter pub run ffigen --config ffigen.yaml`.
///
class FlutterBulletBindings {
  /// Holds the symbol lookup function.
  final ffi.Pointer<T> Function<T extends ffi.NativeType>(String symbolName)
      _lookup;

  /// The symbols are looked up in [dynamicLibrary].
  FlutterBulletBindings(ffi.DynamicLibrary dynamicLibrary)
      : _lookup = dynamicLibrary.lookup;

  /// The symbols are looked up with [lookup].
  FlutterBulletBindings.fromLookup(
      ffi.Pointer<T> Function<T extends ffi.NativeType>(String symbolName)
          lookup)
      : _lookup = lookup;

  ffi.Pointer<wpWorld> create_world() {
    return _create_world();
  }

  late final _create_worldPtr =
      _lookup<ffi.NativeFunction<ffi.Pointer<wpWorld> Function()>>(
          'create_world');
  late final _create_world =
      _create_worldPtr.asFunction<ffi.Pointer<wpWorld> Function()>();

  void step_world(
    ffi.Pointer<wpWorld> world,
    double dt,
  ) {
    return _step_world(
      world,
      dt,
    );
  }

  late final _step_worldPtr = _lookup<
      ffi.NativeFunction<
          ffi.Void Function(ffi.Pointer<wpWorld>, ffi.Float)>>('step_world');
  late final _step_world =
      _step_worldPtr.asFunction<void Function(ffi.Pointer<wpWorld>, double)>();

  void world_add_rigid_body(
    ffi.Pointer<wpWorld> world,
    ffi.Pointer<wpBody> body,
  ) {
    return _world_add_rigid_body(
      world,
      body,
    );
  }

  late final _world_add_rigid_bodyPtr = _lookup<
      ffi.NativeFunction<
          ffi.Void Function(ffi.Pointer<wpWorld>,
              ffi.Pointer<wpBody>)>>('world_add_rigid_body');
  late final _world_add_rigid_body = _world_add_rigid_bodyPtr
      .asFunction<void Function(ffi.Pointer<wpWorld>, ffi.Pointer<wpBody>)>();

  void world_remove_rigid_body(
    ffi.Pointer<wpWorld> world,
    ffi.Pointer<wpBody> body,
  ) {
    return _world_remove_rigid_body(
      world,
      body,
    );
  }

  late final _world_remove_rigid_bodyPtr = _lookup<
      ffi.NativeFunction<
          ffi.Void Function(ffi.Pointer<wpWorld>,
              ffi.Pointer<wpBody>)>>('world_remove_rigid_body');
  late final _world_remove_rigid_body = _world_remove_rigid_bodyPtr
      .asFunction<void Function(ffi.Pointer<wpWorld>, ffi.Pointer<wpBody>)>();

  void destroy_world(
    ffi.Pointer<wpWorld> world,
  ) {
    return _destroy_world(
      world,
    );
  }

  late final _destroy_worldPtr =
      _lookup<ffi.NativeFunction<ffi.Void Function(ffi.Pointer<wpWorld>)>>(
          'destroy_world');
  late final _destroy_world =
      _destroy_worldPtr.asFunction<void Function(ffi.Pointer<wpWorld>)>();

  ffi.Pointer<wpShape> create_box_shape(
    double x,
    double y,
    double z,
  ) {
    return _create_box_shape(
      x,
      y,
      z,
    );
  }

  late final _create_box_shapePtr = _lookup<
      ffi.NativeFunction<
          ffi.Pointer<wpShape> Function(
              ffi.Float, ffi.Float, ffi.Float)>>('create_box_shape');
  late final _create_box_shape = _create_box_shapePtr
      .asFunction<ffi.Pointer<wpShape> Function(double, double, double)>();

  ffi.Pointer<wpShape> create_static_plane_shape(
    double nx,
    double ny,
    double nz,
    double c,
  ) {
    return _create_static_plane_shape(
      nx,
      ny,
      nz,
      c,
    );
  }

  late final _create_static_plane_shapePtr = _lookup<
      ffi.NativeFunction<
          ffi.Pointer<wpShape> Function(ffi.Float, ffi.Float, ffi.Float,
              ffi.Float)>>('create_static_plane_shape');
  late final _create_static_plane_shape =
      _create_static_plane_shapePtr.asFunction<
          ffi.Pointer<wpShape> Function(double, double, double, double)>();

  void destroy_shape(
    ffi.Pointer<wpShape> shape,
  ) {
    return _destroy_shape(
      shape,
    );
  }

  late final _destroy_shapePtr =
      _lookup<ffi.NativeFunction<ffi.Void Function(ffi.Pointer<wpShape>)>>(
          'destroy_shape');
  late final _destroy_shape =
      _destroy_shapePtr.asFunction<void Function(ffi.Pointer<wpShape>)>();

  ffi.Pointer<wpBody> create_rigid_body(
    double mass,
    ffi.Pointer<wpShape> shape,
    double tx,
    double ty,
    double tz,
  ) {
    return _create_rigid_body(
      mass,
      shape,
      tx,
      ty,
      tz,
    );
  }

  late final _create_rigid_bodyPtr = _lookup<
      ffi.NativeFunction<
          ffi.Pointer<wpBody> Function(ffi.Float, ffi.Pointer<wpShape>,
              ffi.Float, ffi.Float, ffi.Float)>>('create_rigid_body');
  late final _create_rigid_body = _create_rigid_bodyPtr.asFunction<
      ffi.Pointer<wpBody> Function(
          double, ffi.Pointer<wpShape>, double, double, double)>();

  ffi.Pointer<ffi.Float> rigid_body_get_origin(
    ffi.Pointer<wpBody> body,
  ) {
    return _rigid_body_get_origin(
      body,
    );
  }

  late final _rigid_body_get_originPtr = _lookup<
      ffi.NativeFunction<
          ffi.Pointer<ffi.Float> Function(
              ffi.Pointer<wpBody>)>>('rigid_body_get_origin');
  late final _rigid_body_get_origin = _rigid_body_get_originPtr
      .asFunction<ffi.Pointer<ffi.Float> Function(ffi.Pointer<wpBody>)>();

  void destroy_rigid_body(
    ffi.Pointer<wpBody> body,
  ) {
    return _destroy_rigid_body(
      body,
    );
  }

  late final _destroy_rigid_bodyPtr =
      _lookup<ffi.NativeFunction<ffi.Void Function(ffi.Pointer<wpBody>)>>(
          'destroy_rigid_body');
  late final _destroy_rigid_body =
      _destroy_rigid_bodyPtr.asFunction<void Function(ffi.Pointer<wpBody>)>();

  late final addresses = _SymbolAddresses(this);
}

class _SymbolAddresses {
  final FlutterBulletBindings _library;
  _SymbolAddresses(this._library);
  ffi.Pointer<ffi.NativeFunction<ffi.Pointer<wpWorld> Function()>>
      get create_world => _library._create_worldPtr;
  ffi.Pointer<
          ffi
          .NativeFunction<ffi.Void Function(ffi.Pointer<wpWorld>, ffi.Float)>>
      get step_world => _library._step_worldPtr;
  ffi.Pointer<
          ffi.NativeFunction<
              ffi.Void Function(ffi.Pointer<wpWorld>, ffi.Pointer<wpBody>)>>
      get world_add_rigid_body => _library._world_add_rigid_bodyPtr;
  ffi.Pointer<
          ffi.NativeFunction<
              ffi.Void Function(ffi.Pointer<wpWorld>, ffi.Pointer<wpBody>)>>
      get world_remove_rigid_body => _library._world_remove_rigid_bodyPtr;
  ffi.Pointer<ffi.NativeFunction<ffi.Void Function(ffi.Pointer<wpWorld>)>>
      get destroy_world => _library._destroy_worldPtr;
  ffi.Pointer<
          ffi.NativeFunction<
              ffi.Pointer<wpShape> Function(ffi.Float, ffi.Float, ffi.Float)>>
      get create_box_shape => _library._create_box_shapePtr;
  ffi.Pointer<
          ffi.NativeFunction<
              ffi.Pointer<wpShape> Function(
                  ffi.Float, ffi.Float, ffi.Float, ffi.Float)>>
      get create_static_plane_shape => _library._create_static_plane_shapePtr;
  ffi.Pointer<ffi.NativeFunction<ffi.Void Function(ffi.Pointer<wpShape>)>>
      get destroy_shape => _library._destroy_shapePtr;
  ffi.Pointer<
      ffi.NativeFunction<
          ffi.Pointer<wpBody> Function(
              ffi.Float,
              ffi.Pointer<wpShape>,
              ffi.Float,
              ffi.Float,
              ffi.Float)>> get create_rigid_body =>
      _library._create_rigid_bodyPtr;
  ffi.Pointer<
          ffi
          .NativeFunction<ffi.Pointer<ffi.Float> Function(ffi.Pointer<wpBody>)>>
      get rigid_body_get_origin => _library._rigid_body_get_originPtr;
  ffi.Pointer<ffi.NativeFunction<ffi.Void Function(ffi.Pointer<wpBody>)>>
      get destroy_rigid_body => _library._destroy_rigid_bodyPtr;
}

final class wpWorld extends ffi.Opaque {}

final class wpShape extends ffi.Opaque {}

final class wpBody extends ffi.Opaque {}
