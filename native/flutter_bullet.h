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
// Opaque type of a rigid body in the world.
typedef struct wpBody wpBody;

FFI_PLUGIN_EXPORT wpWorld* create_world();

FFI_PLUGIN_EXPORT void step_world(wpWorld* world, float dt);

FFI_PLUGIN_EXPORT void world_add_rigid_body(wpWorld* world, wpBody* body);

FFI_PLUGIN_EXPORT void world_remove_rigid_body(wpWorld* world, wpBody* body);

FFI_PLUGIN_EXPORT void destroy_world(wpWorld* world);

FFI_PLUGIN_EXPORT wpShape* create_box_shape(float x, float y, float z);

FFI_PLUGIN_EXPORT wpShape* create_static_plane_shape(float nx, float ny, float nz, float c);

FFI_PLUGIN_EXPORT void destroy_shape(wpShape* shape);

FFI_PLUGIN_EXPORT wpBody* create_rigid_body(float mass, wpShape* shape);

FFI_PLUGIN_EXPORT void set_rigid_body_user_data(wpBody* body, Dart_Handle ref);

FFI_PLUGIN_EXPORT Dart_Handle get_rigid_body_user_data(wpBody* body);

FFI_PLUGIN_EXPORT const float* rigid_body_get_raw_transform(wpBody* body);

FFI_PLUGIN_EXPORT void rigid_body_set_raw_transform(wpBody* body, const float* xform);

FFI_PLUGIN_EXPORT void destroy_rigid_body(wpBody* body);

#ifdef __cplusplus
}
#endif
