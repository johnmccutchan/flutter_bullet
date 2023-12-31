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
// Opaque type of collidable state.
typedef struct wpCollidableState wpCollidableState;

FFI_PLUGIN_EXPORT wpWorld* create_world();

FFI_PLUGIN_EXPORT void step_world(wpWorld* world, float dt);

FFI_PLUGIN_EXPORT void world_add_rigid_body(wpWorld* world, wpBody* body);

FFI_PLUGIN_EXPORT void world_remove_rigid_body(wpWorld* world, wpBody* body);

FFI_PLUGIN_EXPORT void world_add_collidable(wpWorld* world, wpCollidable* collidable);

FFI_PLUGIN_EXPORT void world_remove_collidable(wpWorld* world, wpCollidable* collidable);

FFI_PLUGIN_EXPORT void world_raycast(wpWorld* world,
                                     float sx, float sy, float sz, 
                                     float ex, float ey, float ez,
                                     float (*cb)(Dart_Handle collidable, float fraction, const float* n));

FFI_PLUGIN_EXPORT void destroy_world(wpWorld* world);

// Collidables
FFI_PLUGIN_EXPORT wpCollidable* create_collidable();

FFI_PLUGIN_EXPORT void collidable_set_shape(wpCollidable* collidable, wpShape* shape);

FFI_PLUGIN_EXPORT void destroy_collidable(wpCollidable* collidable);

// Rigid bodies (which are also Collidables)
FFI_PLUGIN_EXPORT wpBody* create_rigid_body(float mass, wpShape* shape);

// NOTE: destroy_rigid_body is handled by destroy_collidable.

// Get collidable state for the collidable.
FFI_PLUGIN_EXPORT wpCollidableState* collidable_create_state(wpCollidable* collidable,
                                                             Dart_Handle owner,
                                                             void (*transform_update)(Dart_Handle));

FFI_PLUGIN_EXPORT void collidable_state_set_dart_owner(wpCollidableState* collidable_state, Dart_Handle owner);

FFI_PLUGIN_EXPORT Dart_Handle collidable_state_get_dart_owner(wpCollidableState* collidable_state);

FFI_PLUGIN_EXPORT float* collidable_state_get_matrix(wpCollidableState* motion_state);

FFI_PLUGIN_EXPORT void collidable_state_matrix_updated(wpCollidableState* collidable_state);

// Shapes.
FFI_PLUGIN_EXPORT wpShape* create_box_shape(float x, float y, float z);

FFI_PLUGIN_EXPORT wpShape* create_static_plane_shape(float nx, float ny, float nz, float c);

FFI_PLUGIN_EXPORT void shape_set_dart_owner(wpShape* shape, Dart_Handle owner);

FFI_PLUGIN_EXPORT Dart_Handle shape_get_dart_owner(wpShape* shape);

FFI_PLUGIN_EXPORT void destroy_shape(wpShape* shape);

#ifdef __cplusplus
}
#endif
