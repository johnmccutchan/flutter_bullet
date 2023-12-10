import 'package:flutter_bullet/physics3d.dart';
import 'package:vector_math/vector_math.dart';

main() {
  // Create a physics world. Gravity is -Y.
  var world = World();

  // Create a unit box
  var box = BoxShape(Vector3(.5, .5, .5));

  // Create a static plane in the X-Z axis.
  var plane = StaticPlaneShape(Vector3(0, 1, 0), 0);

  // Make a dynamic body with mass 1.0 with the box shape.
  // Place it 10 units in the air.
  var dynamicBody = RigidBody(1.0, box);
  dynamicBody.xform = Transform()..origin = Vector3(0, 10, 0);

  // Make a static collidable.
  var floor = Collidable();
  floor.shape = plane;

  world.addBody(dynamicBody);
  world.addCollidable(floor);

  // 1/60.
  final dt = 0.0625;

  // 200 steps.
  for (int i = 0; i < 200; i++) {
    world.step(dt);
    var origin = dynamicBody.xform.origin;
    var rotation = dynamicBody.xform.rotation;
    print(origin);
  }

  world.removeBody(dynamicBody);
  world.removeCollidable(floor);
}
