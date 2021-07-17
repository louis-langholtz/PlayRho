# Testbed GUI Application Tests Directory

This directory is for classes which sub-class the Testbed's `Test` base class.
You don't really need to know anything about it unless you want to add your own
code to the Testbed.

Subclassing `Test` is *part one* of the way to integrate code into the Testbed
GUI application for demoing and visually testing code. After getting the
subclass initially setup, *part two* of integrating code into the Testbed,
is to add a reference to the subclass to the
[TestEntry.cpp](../Framework/TestEntry.cpp) file that's over in the `Framework`
directory.

See also the [Testbed README](../README.md).

## Images

Here's some images of what these look like when run.

### [`AddPair.cpp`](AddPair.cpp)

![Image of AddPair.cpp running](../../Documentation/images/Testbed/AddPairStressTest.png)

### [`ApplyForce.cpp`](ApplyForce.cpp)

![Image of ApplyForce.cpp running](../../Documentation/images/Testbed/ApplyForce.png)

### [`BagOfDisks.cpp`](BagOfDisks.cpp)

![Image of BagOfDisks.cpp running](../../Documentation/images/Testbed/BagOfDisks.png)

### [`BasicSliderCrank.cpp`](BasicSliderCrank.cpp)

![Image of BasicSliderCrank.cpp running](../../Documentation/images/Testbed/BasicSliderCrank.png)

### [`BodyTypes.cpp`](BodyTypes.cpp)

![Image of BodyTypes.cpp running](../../Documentation/images/Testbed/BodyTypes.png)

### [`Breakable.cpp`](Breakable.cpp)

![Image of Breakable.cpp running](../../Documentation/images/Testbed/Breakable.png)

### [`BreakableTwo.cpp`](BreakableTwo.cpp)

![Image of BreakableTwo.cpp running](../../Documentation/images/Testbed/BreakableTwo.png)

### [`Bridge.cpp`](Bridge.cpp)

![Image of Bridge.cpp running](../../Documentation/images/Testbed/Bridge.png)

### [`BulletTest.cpp`](BulletTest.cpp)

![Image of BulletTest.cpp running](../../Documentation/images/Testbed/BulletTest.png)

### [`Cantilever.cpp`](Cantilever.cpp)

![Image of Cantilever.cpp running](../../Documentation/images/Testbed/Cantilever.png)

### [`Car.cpp`](Car.cpp)

![Image of Car.cpp running](../../Documentation/images/Testbed/Car.png)

### [`Chain.cpp`](Chain.cpp)

![Image of Chain.cpp running](../../Documentation/images/Testbed/Chain.png)

### [`CharacterCollision.cpp`](CharacterCollision.cpp)

![Image of CharacterCollision.cpp running](../../Documentation/images/Testbed/CharacterCollision.png)

### [`CollisionFiltering.cpp`](CollisionFiltering.cpp)

![Image of CollisionFiltering.cpp running](../../Documentation/images/Testbed/CollisionFiltering.png)

### [`CollisionProcessing.cpp`](CollisionProcessing.cpp)

![Image of CollisionProcessing.cpp running](../../Documentation/images/Testbed/CollisionProcessing.png)

### [`CompoundShapes.cpp`](CompoundShapes.cpp)

![Image of CompoundShapes.cpp running](../../Documentation/images/Testbed/CompoundShapes.png)

### [`Confined.cpp`](Confined.cpp)

![Image of Confined.cpp running](../../Documentation/images/Testbed/Confined.png)

### [`ContinuousTest.cpp`](ContinuousTest.cpp)

![Image of ContinuousTest.cpp running](../../Documentation/images/Testbed/ContinuousTest.png)

### [`ConvexHull.cpp`](ConvexHull.cpp)

![Image of ConvexHull.cpp running](../../Documentation/images/Testbed/ConvexHull.png)

### [`ConveyorBelt.cpp`](ConveyorBelt.cpp)

![Image of ConveyorBelt.cpp running](../../Documentation/images/Testbed/ConveyorBelt.png)

### [`DistanceTest.cpp`](DistanceTest.cpp)

![Image of DistanceTest.cpp running](../../Documentation/images/Testbed/DistanceTest.png)

### [`Dominos.cpp`](Dominos.cpp)

![Image of Dominos.cpp running](../../Documentation/images/Testbed/Dominos.png)

### [`DumpShell.cpp`](DumpShell.cpp)

![Image of DumpShell.cpp running](../../Documentation/images/Testbed/DumpShell.png)

### [`EdgeShapes.cpp`](EdgeShapes.cpp)

![Image of EdgeShapes.cpp running](../../Documentation/images/Testbed/EdgeShapes.png)

### [`EdgeTest.cpp`](EdgeTest.cpp)

![Image of EdgeTest.cpp running](../../Documentation/images/Testbed/EdgeTest.png)

### [`FifteenPuzzle.cpp`](FifteenPuzzle.cpp)

![Image of FifteenPuzzle.cpp running](../../Documentation/images/Testbed/FifteenPuzzle.png)

### [`Gears.cpp`](Gears.cpp)

![Image of Gears.cpp running](../../Documentation/images/Testbed/Gears.png)

### [`HalfPipe.cpp`](HalfPipe.cpp)

![Image of HalfPipe.cpp running](../../Documentation/images/Testbed/HalfPipe.png)

### [`HeavyOnLight.cpp`](HeavyOnLight.cpp)

![Image of HeavyOnLight.cpp running](../../Documentation/images/Testbed/HeavyOnLight.png)

### [`HeavyOnLightTwo.cpp`](HeavyOnLightTwo.cpp)

![Image of HeavyOnLightTwo.cpp running](../../Documentation/images/Testbed/HeavyOnLightTwo.png)

### [`JointsTest.cpp`](JointsTest.cpp)

![Image of JointsTest.cpp running](../../Documentation/images/Testbed/JointsTest.png)

### [`Mobile.cpp`](Mobile.cpp)

![Image of Mobile.cpp running](../../Documentation/images/Testbed/Mobile.png)

### [`MobileBalanced.cpp`](MobileBalanced.cpp)

![Image of MobileBalanced.cpp running](../../Documentation/images/Testbed/MobileBalanced.png)

### [`MotorJoint.cpp`](MotorJoint.cpp)

![Image of MotorJoint.cpp running](../../Documentation/images/Testbed/MotorJoint.png)

### [`NewtonsCradle.cpp`](NewtonsCradle.cpp)

![Image of NewtonsCradle.cpp running](../../Documentation/images/Testbed/NewtonsCradle.png)

### [`OneSidedPlatform.cpp`](OneSidedPlatform.cpp)

![Image of OneSidedPlatform.cpp running](../../Documentation/images/Testbed/OneSidedPlatform.png)

### [`Orbiter.cpp`](Orbiter.cpp)

![Image of Orbiter.cpp running](../../Documentation/images/Testbed/Orbiter.png)

### [`Pinball.cpp`](Pinball.cpp)

![Image of Pinball.cpp running](../../Documentation/images/Testbed/Pinball.png)

### [`PolyCollision.cpp`](PolyCollision.cpp)

![Image of PolyCollision.cpp running](../../Documentation/images/Testbed/PolyCollision.png)

### [`PolyShapes.cpp`](PolyShapes.cpp)

![Image of PolyShapes.cpp running](../../Documentation/images/Testbed/PolyShapes.png)

### [`Prismatic.cpp`](Prismatic.cpp)

![Image of Prismatic.cpp running](../../Documentation/images/Testbed/Prismatic.png)

### [`Pulleys.cpp`](Pulleys.cpp)

![Image of Pulleys.cpp running](../../Documentation/images/Testbed/Pulleys.png)

### [`Pyramid.cpp`](Pyramid.cpp)

![Image of Pyramid.cpp running](../../Documentation/images/Testbed/Pyramid.png)

### [`RayCast.cpp`](RayCast.cpp)

![Image of RayCast.cpp running](../../Documentation/images/Testbed/RayCast.png)

### [`Revolute.cpp`](Revolute.cpp)

![Image of Revolute.cpp running](../../Documentation/images/Testbed/Revolute.png)

### [`RopeJoint.cpp`](RopeJoint.cpp)

![Image of RopeJoint.cpp running](../../Documentation/images/Testbed/RopeJoint.png)

### [`SensorTest.cpp`](SensorTest.cpp)

![Image of SensorTest.cpp running](../../Documentation/images/Testbed/SensorTest.png)

### [`ShapeEditing.cpp`](ShapeEditing.cpp)

![Image of ShapeEditing.cpp running](../../Documentation/images/Testbed/ShapeEditing.png)

### [`SliderCrank.cpp`](SliderCrank.cpp)

![Image of SliderCrank.cpp running](../../Documentation/images/Testbed/SliderCrank.png)

### [`SphereStack.cpp`](SphereStack.cpp)

![Image of SphereStack.cpp running](../../Documentation/images/Testbed/SphereStack.png)

### [`SpinningCircle.cpp`](SpinningCircle.cpp)

![Image of SpinningCircle.cpp running](../../Documentation/images/Testbed/SpinningCircle.png)

### [`TheoJansen.cpp`](TheoJansen.cpp)

![Image of TheoJansen.cpp running](../../Documentation/images/Testbed/TheoJansen.png)

### [`Tiles.cpp`](Tiles.cpp)

![Image of Tiles.cpp running](../../Documentation/images/Testbed/Tiles.png)

### [`TimeOfImpact.cpp`](TimeOfImpact.cpp)

![Image of TimeOfImpact.cpp running](../../Documentation/images/Testbed/TimeOfImpact.png)

### [`Tumbler.cpp`](Tumbler.cpp)

![Image of Tumbler.cpp running](../../Documentation/images/Testbed/Tumbler.png)

### [`VaryingFriction.cpp`](VaryingFriction.cpp)

![Image of VaryingFriction.cpp running](../../Documentation/images/Testbed/VaryingFriction.png)

### [`VaryingRestitution.cpp`](VaryingRestitution.cpp)

![Image of VaryingRestitution.cpp running](../../Documentation/images/Testbed/VaryingRestitution.png)

### [`VerticalStack.cpp`](VerticalStack.cpp)

![Image of VerticalStack.cpp running](../../Documentation/images/Testbed/VerticalStack.png)

### [`Web.cpp`](Web.cpp)

![Image of Web.cpp running](../../Documentation/images/Testbed/Web.png)

### [`iforce2d_TopdownCar.cpp`](iforce2d_TopdownCar.cpp)

![Image of iforce2d_TopdownCar.cpp running](../../Documentation/images/Testbed/iforce2d_TopdownCar.png)

### [`iforce2d_Trajectories.cpp`](iforce2d_Trajectories.cpp)

![Image of iforce2d_Trajectories.cpp running](../../Documentation/images/Testbed/iforce2d_Trajectories.png)
