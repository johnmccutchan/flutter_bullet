part of '../physics3d.dart';

typedef VoidCallback = void Function();

// Only supports translation and rotation.
class Transform {
  static Matrix3 _tempStorage = Matrix3.identity();
  final Matrix4 _impl;
  final Collidable _collidable;
  LinkedHashSet<VoidCallback> _listeners = new LinkedHashSet<VoidCallback>();

  Transform._(this._collidable, Float32List list)
      : _impl = Matrix4.fromFloat32List(list);

  void _runListeners() {
    for (final cb in _listeners) {
      cb();
    }
  }

  void _notifyNativeCode() {
    _collidable._notifyNativeCode();
  }

  Vector3 get origin {
    return _impl.getTranslation();
  }

  set origin(Vector3 o) {
    _impl.setTranslation(o);
    _notifyNativeCode();
    _runListeners();
  }

  Quaternion get rotation {
    _impl.copyRotation(_tempStorage);
    return Quaternion.fromRotation(_tempStorage);
  }

  set rotation(Quaternion q) {
    q.copyRotationInto(_tempStorage);
    _impl.setRotation(_tempStorage);
    _notifyNativeCode();
    _runListeners();
  }

  UnmodifiableFloat32ListView get matrix {
    return UnmodifiableFloat32ListView(_impl.storage);
  }

  void setFromTransform(Transform t) {
    _impl.copyFromArray(t.matrix);
    _notifyNativeCode();
    _runListeners();
  }

  void setFromMatrix(Matrix4 t) {
    _impl.copyFromArray(t.storage);
    _notifyNativeCode();
    _runListeners();
  }

  // Be notified whenever this Transform changes (either by Dart code
  // explicitly changing this or by the simulation update).
  // Use to synchronize the graphics object.
  void addListener(VoidCallback cb) {
    _listeners.add(cb);
  }

  void removeListener(VoidCallback cb) {
    _listeners.remove(cb);
  }
}
