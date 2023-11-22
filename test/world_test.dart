import 'package:flutter_bullet/flutter_bullet.dart';
import 'package:vector_math/vector_math.dart';

main() {
  var world = World();

  // Create a unit box
  var box = BoxShape(Vector3(.5, .5, .5));

  // Create a static plane in the X-Z axis.
  var plane = StaticPlaneShape(Vector3(0, 1, 0), 0);

  // Make a dynamic body with mass 1.0 with the box shape.
  // Place it 10 units in the air.
  var dynamicBody = RigidBody(1.0, box, Vector3(0, 10, 0));

  // Make a static body (mass == 0.0) with the static plane shape
  // place it at the origin.
  var floorBody = RigidBody(0.0, plane, Vector3(0, 0, 0));

  world.addBody(dynamicBody);
  world.addBody(floorBody);

  final dt = 0.0625;

  for (int i = 0; i < 200; i++) {
    world.step(dt);
    var origin = dynamicBody.origin;
    print(origin);
  }
}
