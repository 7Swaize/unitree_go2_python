import numpy as np

from go2sim import TerrainGenerator, GeometryType


ROOM_SIZE: np.ndarray = [20.0, 20.0]
WALL_HEIGHT = 15.0
WALL_THICKNESS = 0.1
PILLAR_SIZE = 0.15
DOOR_WIDTH = 1
ROOF_THICKNESS = 0.1


# Builds a rectangular room centered on the origin using four walls, four corner pillars, a roof slab
# and a gap in the +Y wall to act as a doorway.
def add_room(generator: TerrainGenerator) -> None:
    length_x, length_y = ROOM_SIZE
    half_x, half_y = length_x / 2.0, length_y / 2.0
    z_center = WALL_HEIGHT / 2.0

    generator.add_geometry(
        position=[0.0, -half_y, z_center],
        euler=[0.0, 0.0, 0.0],
        size=[length_x, WALL_THICKNESS, WALL_HEIGHT],
        geo_type=GeometryType.BOX
    )

    segment_len = (length_x - DOOR_WIDTH) / 2.0
    if segment_len > 0:
        generator.add_geometry(
            position=[-(DOOR_WIDTH / 2.0 + segment_len / 2.0), half_y, z_center],
            euler=[0.0, 0.0, 0.0],
            size=[segment_len, WALL_THICKNESS, WALL_HEIGHT],
            geo_type=GeometryType.BOX
        )
        generator.add_geometry(
            position=[(DOOR_WIDTH / 2.0 + segment_len / 2.0), half_y, z_center],
            euler=[0.0, 0.0, 0.0],
            size=[segment_len, WALL_THICKNESS, WALL_HEIGHT],
            geo_type=GeometryType.BOX
        )

    generator.add_geometry(
        position=[half_x, 0.0, z_center],
        euler=[0.0, 0.0, 90.0],
        size=[length_y, WALL_THICKNESS, WALL_HEIGHT],
        geo_type=GeometryType.BOX
    )

    generator.add_geometry(
        position=[-half_x, 0.0, z_center],
        euler=[0.0, 0.0, 90.0],
        size=[length_y, WALL_THICKNESS, WALL_HEIGHT],
        geo_type=GeometryType.BOX
    )

    corners = [
        (half_x, half_y),
        (half_x, -half_y),
        (-half_x, half_y),
        (-half_x, -half_y),
    ]

    for cx, cy in corners:
        generator.add_geometry(
            position=[cx, cy, z_center],
            euler=[0.0, 0.0, 0.0],
            size=[PILLAR_SIZE, PILLAR_SIZE, WALL_HEIGHT],
            geo_type=GeometryType.BOX
        )

    generator.add_geometry(
        position=[0.0, 0.0, WALL_HEIGHT + ROOF_THICKNESS / 2.0],
        euler=[0.0, 0.0, 0.0],
        size=[length_x, length_y],
        geo_type=GeometryType.BOX
    )


# Create a TerrainGenerator instance.
generator = TerrainGenerator()

# Remove any current items in the scene.
generator.reset_to_base()

# Add the room.
add_room(generator)

# Save changes to the scene.
generator.save()
