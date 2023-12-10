import 'package:flutter_bullet/physics3d.dart';
import 'package:vector_math/vector_math.dart';

main() {
  // Create a physics world. Gravity is -Y.
  var world = World();

  // Create a static plane in the X-Z axis.
  var plane = StaticPlaneShape(Vector3(0, 1, 0), 0);

  // Make a static collidable that represents the floor.
  var floor = Collidable();
  floor.shape = plane;

  world.addCollidable(floor);

  // Create a unit box
  var box = BoxShape(Vector3(.5, .5, .5));

  var bodies = <RigidBody>[];

  // Create a vertical tower of 4 bodies.
  for (int i = 0; i < 4; i++) {
    // Make a dynamic body with mass 1.0 with the box shape.
    var body = new RigidBody(1.0, box);
    body.xform = Transform()..origin = Vector3(0, 6 - i * 1.1, 0);
    world.addBody(body);
    bodies.add(body);
  }

  int bodyHitCount = 0;
  rayCast(world, Vector3(0, 0, 0), Vector3(0, 6, 0), (hit) {
    print(hit);
    if (hit.object is RigidBody) {
      bodyHitCount++;
    }
    return 1.0;
  });
  if (bodyHitCount != bodies.length) {
    throw new Exception("wrong number of bodies hit.");
  }

  // 1/60.
  final dt = 0.0625;

  // 200 steps.
  for (int i = 0; i < 200; i++) {
    world.step(dt);
    for (int b = 0; b < bodies.length; b++) {
      var body = bodies[b];
      print('[$b] - ${body.xform.origin}');
    }
  }

  // Clear world.
  for (final b in bodies) {
    world.removeBody(b);
  }
  world.removeCollidable(floor);

  world.step(dt);
}
