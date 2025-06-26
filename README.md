### Requirements

Requries 'math', 'mathutils' and 'bmesh' modules.

### Draw road

To draw a road, 3 vertices need to be selected in a specific order. The first vertex is where the turn starts, second vertex is where turn ends, third vertex is the point where the two roads would intersect if there was no turn. This order is important, as the last vertex selected is used to find the angle in the road.

### Configuration options

- Update Modes: Sets update mode.
	- Manual Update only updates when Update Road button is used.
	- Auto Update will update when object used to draw road or any of its vertices are moved.
- Segments: Changes how many segments make up the road object.
- Runoff Segments: Changes how many segments make up the runoff sections of the road object.
- Design Speed: Sets the design speed for this road. This unit is in MPH.
- Road Width: Sets the design width of the road. This unit is in Meters.
- Superelevation: Sets the superelevation in percent for this road.