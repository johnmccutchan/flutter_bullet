import 'dart:io';
import 'dart:ffi' as ffi;
import 'flutter_bullet_bindings_generated.dart';
export 'flutter_bullet_bindings_generated.dart'
    show wpBody, wpShape, wpWorld, wpCollidable;

/// The dynamic library in which the symbols for [FlutterBulletBindings] can be found.
final ffi.DynamicLibrary _dylib = () {
  const String _libName = 'flutter_bullet';
  if (Platform.isMacOS || Platform.isIOS) {
    return ffi.DynamicLibrary.open('native/lib$_libName.dylib');
  }
  if (Platform.isAndroid || Platform.isLinux) {
    return ffi.DynamicLibrary.open('lib$_libName.so');
  }
  if (Platform.isWindows) {
    return ffi.DynamicLibrary.open('$_libName.dll');
  }
  throw UnsupportedError('Unknown platform: ${Platform.operatingSystem}');
}();

FlutterBulletBindings _initBindings() {
  FlutterBulletBindings bindings = FlutterBulletBindings(_dylib);
  int r = bindings.Dart_InitializeApiDL(ffi.NativeApi.initializeApiDLData);
  if (r != 0) {
    throw new UnsupportedError('Dart_InitializeApiDL returned $r');
  }
  return bindings;
}

/// The bindings to the native functions.
final FlutterBulletBindings bindings = _initBindings();
