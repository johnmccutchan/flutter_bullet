#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#if _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <unistd.h>
#endif

#include "dart_api_dl.h"

#ifdef __cplusplus
extern "C" {
#endif

#if _WIN32
#define FFI_PLUGIN_EXPORT __declspec(dllexport)
#else
#define FFI_PLUGIN_EXPORT
#endif

// Opaque type of a world.
typedef struct wpWorld wpWorld;
// Opaque type of a shape.
typedef struct wpShape wpShape;
// Opaque type of an object.
typedef struct wpCollidable wpCollidable;
// Opaque type of a rigid body in the world.
typedef struct wpBody wpBody;  // Extends wpCollidable.

FFI_PLUGIN_EXPORT wpWorld* create_world();

FFI_PLUGIN_EXPORT void step_world(wpWorld* world, float dt);

FFI_PLUGIN_EXPORT void world_add_rigid_body(wpWorld* world, wpBody* body);

FFI_PLUGIN_EXPORT void world_remove_rigid_body(wpWorld* world, wpBody* body);

FFI_PLUGIN_EXPORT void world_add_collidable(wpWorld* world, wpCollidable* collidable);

FFI_PLUGIN_EXPORT void world_remove_collidable(wpWorld* world, wpCollidable* collidable);

FFI_PLUGIN_EXPORT void destroy_world(wpWorld* world);

// Collidables
FFI_PLUGIN_EXPORT wpCollidable* create_collidable();

FFI_PLUGIN_EXPORT void collidable_set_dart_owner(wpCollidable* collidable, Dart_Handle owner);

FFI_PLUGIN_EXPORT Dart_Handle collidable_get_dart_owner(wpCollidable* collidable);

FFI_PLUGIN_EXPORT void collidable_set_shape(wpCollidable* collidable, wpShape* shape);

FFI_PLUGIN_EXPORT const float* collidable_get_raw_transform(wpCollidable* collidable);

FFI_PLUGIN_EXPORT void collidable_set_raw_transform(wpCollidable* collidable, const float* m);

FFI_PLUGIN_EXPORT void destroy_collidable(wpCollidable* collidable);

// Rigid bodies (which are also Collidables)
FFI_PLUGIN_EXPORT wpBody* create_rigid_body(float mass, wpShape* shape);

// Shapes.
FFI_PLUGIN_EXPORT wpShape* create_box_shape(float x, float y, float z);

FFI_PLUGIN_EXPORT wpShape* create_static_plane_shape(float nx, float ny, float nz, float c);

FFI_PLUGIN_EXPORT void shape_set_dart_owner(wpShape* shape, Dart_Handle owner);

FFI_PLUGIN_EXPORT Dart_Handle shape_get_dart_owner(wpShape* shape);

FFI_PLUGIN_EXPORT void destroy_shape(wpShape* shape);

#ifdef __cplusplus
}
#endif
