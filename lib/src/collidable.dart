part of '../physics3d.dart';

class Collidable implements ffi.Finalizable {
  static final _finalizer =
      ffi.NativeFinalizer(bindings.addresses.destroy_collidable.cast());

  ffi.Pointer<wpCollidable> _nativeCollidable;

  CollisionShape? _shape;

  Collidable._(this._nativeCollidable) {
    _finalizer.attach(this, _nativeCollidable.cast(), detach: this);
    bindings.collidable_set_dart_owner(_nativeCollidable, this);
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

  Transform get xform {
    return Transform.fromList(bindings
        .collidable_get_raw_transform(_nativeCollidable)
        .asTypedList(16));
  }

  set xform(Transform t) {
    final storage = t.storage;
    final a = malloc.allocate<ffi.Float>(4 * 16);
    final array = a.asTypedList(16);
    for (int i = 0; i < 16; i++) {
      array[i] = storage[i];
    }
    bindings.collidable_set_raw_transform(_nativeCollidable, a);
    malloc.free(a);
  }
}
