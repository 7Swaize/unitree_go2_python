from pathlib import Path
from go2sim import TerrainGenerator, GeometryType


def add_textured_geometry(generator: TerrainGenerator) -> None:
    # Get the absolute path of the texture. The generator cannot accept a relative path.
    absolute_path = Path(Path(__file__).parent.resolve() / "resources/image.png").resolve()

    # Register a texture
    texture_path = generator.resources.import_texture(absolute_path)

    # Add textured box to the scene using the returned path.
    # This is a thin box to mimic aruco markers.
    generator.add_textured_geometry(texture_path, position=[1, 1, 0.5], euler=[0, 90, 0], size=[0.5, 0.5, 0.02], geo_type=GeometryType.BOX)

    # Add a full, 3D textured box to the scene using the returned path.
    generator.add_textured_geometry(texture_path, position=[1, -1, 0.5], euler=[0, 90, 0], size=[0.5, 0.5, 0.5], geo_type=GeometryType.BOX)

    # Add some other textured objects to the scene using the returned path.
    generator.add_textured_geometry(texture_path, position=[2, 2, 1.5], euler=[0, 90, 0], size=[0.5, 0.5, 0.02], geo_type=GeometryType.SPHERE)
    generator.add_textured_geometry(texture_path, position=[3, 3, 2.5], euler=[0, 90, 0], size=[0.5, 0.5, 0.02], geo_type=GeometryType.CAPSULE)
    generator.add_textured_geometry(texture_path, position=[4, 4, 3.5], euler=[0, 90, 0], size=[0.5, 0.5, 0.02], geo_type=GeometryType.ELLIPSOID)
    generator.add_textured_geometry(texture_path, position=[5, 5, 4.5], euler=[0, 90, 0], size=[0.5, 0.5, 0.02], geo_type=GeometryType.CYLINDER)


generator = TerrainGenerator()

# Reset the scene to an empty scene.
generator.reset_to_base(reset_resources=True)

add_textured_geometry(generator)

# Write all added geometry to the internal xml file used my Mujoco.
# Essentially, this saves all your changes.
# Without calling this method, any geometry added above will not be there when launching the simulator.
generator.save()