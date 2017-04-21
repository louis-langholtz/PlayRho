# Collision Handling

This is an initial write-up of how collision handling has changed.

## The Original Handling

Initially a close-up (zoomed-in) inspection of the Character Collision
Testbed test, showed this:

![Image of Original Character Collision](images/OriginalRectRectCollision.png)

This had the noted effect of the top rectangle getting "stuck" at this point
when it had been dragged across the lower rectangles from the left side of the
image towards the right.

## Rounded Corner Handling

As I looked into the minutia of how collisions were handled at the "skin"
scale, I realized that there was a round corner collision radius but that in
the calculations for polygons these were being treated as squared corners. See
the previously shown image for an example of this. Not satisfied with squared
off calculations, I introduced rounded corner collisions that instead look like
this:

![Image of Round Corner Character Collision](images/RoundCornerRectRectCollision.png)

While rounded corner collisions aren't as simple computationally, they've been
appealing to me ever since. For one, they had the effect of reducing how much
the top rectangle would get stuck when dragged across flat aligned edges or
rectangles. For two, the rounded collision effect suggested to me that the
collision manifold calculation for two polygons could be generalized for
calculating the collision manifold for any two N-gons from an N of 1 on up.

## Modified Corner Handling

![Image of Modified Corner Handling](images/ThresholdRectRectCollision.png)
