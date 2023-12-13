part of '../physics3d.dart';

void _updateCollidableTransform(Object? collidable) {
  if (collidable == null) {
    return;
  }
  assert(collidable is Collidable);
  (collidable as Collidable)._xform._runListeners();
}

// TODO(johnmccutchan): We should close this somehow.
final _updateCollidableTransformCallback =
    ffi.NativeCallable<ffi.Void Function(ffi.Handle)>.isolateLocal(
        _updateCollidableTransform);

class Collidable implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(bindings.addresses.destroy_collidable.cast());

  final ffi.Pointer<wpCollidable> _nativeCollidable;
  late final ffi.Pointer<wpCollidableState> _nativeCollidableState;

  CollisionShape? _shape;
  late final Transform _xform;

  Collidable._(this._nativeCollidable) {
    _finalizer.attach(this, _nativeCollidable.cast(), detach: this);
    _nativeCollidableState = bindings.collidable_create_state(_nativeCollidable,
        this, _updateCollidableTransformCallback.nativeFunction);
    _xform = Transform._(
        this,
        bindings
            .collidable_state_get_matrix(_nativeCollidableState)
            .asTypedList(16));
  }

  factory Collidable() {
    final nativeCollidable = bindings.create_collidable();
    return Collidable._(nativeCollidable);
  }

  CollisionShape? get shape => _shape;

  set shape(CollisionShape? shape) {
    _shape = shape;
    bindings.collidable_set_shape(
        _nativeCollidable,
        (shape != null)
            ? shape._nativeShape
            : ffi.Pointer<wpShape>.fromAddress(0));
  }

  _notifyNativeCode() {
    bindings.collidable_state_matrix_updated(_nativeCollidableState);
  }

  Transform get xform => _xform;
}
