Terrain Generation
------------------

.. py:module:: go2sim
   :synopsis: Terrain generation module for MuJoCo environments.

.. py:class:: GeometryType(str, Enum)

   Enumeration of supported geometric shapes for the terrain generator.

   .. py:attribute:: PLANE
      :value: "plane"

   .. py:attribute:: SPHERE
      :value: "sphere"

   .. py:attribute:: CAPSULE
      :value: "capsule"

   .. py:attribute:: ELLIPSOID
      :value: "ellipsoid"

   .. py:attribute:: CYLINDER
      :value: "cylinder"

   .. py:attribute:: BOX
      :value: "box"

.. py:class:: TerrainGenerator

   A utility class to dynamically generate and modify MuJoCo XML scene files
   for the robot simulation environment. In addition to raw geometry, the
   generator supports textured geometries and ArUco markers backed by an
   internal :py:class:`ResourceManager` that manages imported texture assets.

   .. py:attribute:: ROBOT
      :type: str
      :value: "go2"

      The identifier of the robot model.

   .. py:attribute:: BASE_DIR
      :type: pathlib.Path

      The root directory of the module.

   .. py:attribute:: PUBLIC_RESOURCES_DIR
      :type: pathlib.Path

      The directory containing user-facing resources (e.g. textures) that
      may be imported into a scene. Backs the public :py:attr:`resources`
      manager.

   .. py:attribute:: INTERNAL_RESOURCES_DIR
      :type: pathlib.Path

      The directory containing internal resources, including the base and
      generated scene XML files.

   .. py:attribute:: GENERATED_SCENE_PATH
      :type: pathlib.Path

      The file path where the generated scene is saved.

   .. py:attribute:: BASE_SCENE_PATH
      :type: pathlib.Path

      The file path to the baseline XML scene.

   .. py:attribute:: resources
      :type: ResourceManager

      A :py:class:`ResourceManager` rooted at :py:attr:`PUBLIC_RESOURCES_DIR`,
      used to import and validate texture files referenced by
      :py:meth:`add_textured_geometry`.

   .. py:method:: __init__() -> None

      Initializes the TerrainGenerator, setting up the public
      :py:attr:`resources` manager, parsing the generated and base XML
      scenes, and setting up the root XML element pointers.

   .. py:method:: add_geometry(position: List[float] = [1.0, 0.0, 0.0], euler: List[float] = [0.0, 0.0, 0.0], size: List[float] = [0.1, 0.1, 0.1], geo_type: GeometryType = GeometryType.BOX) -> None

      Adds a generic, untextured geometry object to the MuJoCo worldbody.

      :param position: The [x, y, z] coordinates of the geometry. Defaults to [1.0, 0.0, 0.0].
      :type position: List[float]
      :param euler: The Euler angles [roll, pitch, yaw] for rotation. Defaults to [0.0, 0.0, 0.0].
      :type euler: List[float]
      :param size: The full size [x, y, z] of the geometry (halved internally for MuJoCo). Defaults to [0.1, 0.1, 0.1].
      :type size: List[float]
      :param geo_type: The type of geometry to add. Defaults to GeometryType.BOX.
      :type geo_type: GeometryType

   .. py:method:: add_textured_geometry(texture_path: str | pathlib.Path, position: List[float] = [0.0, 0.0, 0.0], euler: List[float] = [0.0, 0.0, 0.0], size: List[float] = [0.1, 0.1, 0.1], geo_type: GeometryType = GeometryType.BOX) -> None

      Adds a geometry object textured with a previously-imported texture.
      The texture and its backing material are registered as scene assets
      on first use and reused on subsequent calls with the same texture
      path.

      :param texture_path: The path to the texture, relative to :py:attr:`PUBLIC_RESOURCES_DIR`. Must already exist via :py:attr:`resources` (e.g. imported with ``resources.import_texture``).
      :type texture_path: str | pathlib.Path
      :param position: The [x, y, z] coordinates of the geometry. Defaults to [0.0, 0.0, 0.0].
      :type position: List[float]
      :param euler: The Euler angles [roll, pitch, yaw] for rotation. Defaults to [0.0, 0.0, 0.0].
      :type euler: List[float]
      :param size: The full size [x, y, z] of the geometry (halved internally for MuJoCo). Defaults to [0.1, 0.1, 0.1].
      :type size: List[float]
      :param geo_type: The type of geometry to add. Defaults to GeometryType.BOX.
      :type geo_type: GeometryType
      :raises FileNotFoundError: If `texture_path` has not been imported into :py:attr:`resources`.

   .. py:method:: add_stairs(init_pos: List[float] = [1.0, 0.0, 0.0], yaw: float = 0.0, width: float = 0.2, height: float = 0.15, length: float = 1.5, stair_nums: int = 10) -> None

      Generates a sequence of ascending stair geometries.

      :param init_pos: The starting [x, y, z] position of the stair sequence. Defaults to [1.0, 0.0, 0.0].
      :type init_pos: List[float]
      :param yaw: The yaw angle rotation of the stair sequence in radians. Defaults to 0.0.
      :type yaw: float
      :param width: The tread depth (width along the walking path) of each stair. Defaults to 0.2.
      :type width: float
      :param height: The vertical riser height of each stair. Defaults to 0.15.
      :type height: float
      :param length: The span length of each stair perpendicular to the walking path. Defaults to 1.5.
      :type length: float
      :param stair_nums: The total number of stairs to generate. Defaults to 10.
      :type stair_nums: int

   .. py:method:: add_suspend_stairs(init_pos: List[float] = [1.0, 0.0, 0.0], yaw: float = 1.0, width: float = 0.2, height: float = 0.15, length: float = 1.5, gap: float = 0.1, stair_nums: int = 10) -> None

      Generates a sequence of suspended (floating) stair geometries with vertical gaps.

      :param init_pos: The starting [x, y, z] position of the stair sequence. Defaults to [1.0, 0.0, 0.0].
      :type init_pos: List[float]
      :param yaw: The yaw angle rotation of the stair sequence in radians. Defaults to 1.0.
      :type yaw: float
      :param width: The tread depth of each suspended stair. Defaults to 0.2.
      :type width: float
      :param height: The overall vertical displacement between stairs. Defaults to 0.15.
      :type height: float
      :param length: The span length of each stair. Defaults to 1.5.
      :type length: float
      :param gap: The vertical gap distance removing volume from the riser area. Defaults to 0.1.
      :type gap: float
      :param stair_nums: The total number of suspended stairs to generate. Defaults to 10.
      :type stair_nums: int

   .. py:method:: add_rough_ground(init_pos: List[float] = [1.0, 0.0, 0.0], euler: List[float] = [0.0, -0.0, 0.0], nums: List[int] = [10, 10], box_size: List[float] = [0.5, 0.5, 0.5], box_euler: List[float] = [0.0, 0.0, 0.0], separation: List[float] = [0.2, 0.2], box_size_rand: List[float] = [0.05, 0.05, 0.05], box_euler_rand: List[float] = [0.2, 0.2, 0.2], separation_rand: List[float] = [0.05, 0.05]) -> None

      Generates a randomized rough terrain grid composed of multiple box geometries.

      :param init_pos: The starting [x, y, z] baseline position. Defaults to [1.0, 0.0, 0.0].
      :type init_pos: List[float]
      :param euler: The base Euler angles [roll, pitch, yaw] for the entire grid. Defaults to [0.0, -0.0, 0.0].
      :type euler: List[float]
      :param nums: A list representing the [x_count, y_count] of boxes in the grid. Defaults to [10, 10].
      :type nums: List[int]
      :param box_size: The baseline [x, y, z] size for each terrain box. Defaults to [0.5, 0.5, 0.5].
      :type box_size: List[float]
      :param box_euler: The baseline Euler orientation for each terrain box. Defaults to [0.0, 0.0, 0.0].
      :type box_euler: List[float]
      :param separation: The baseline [x, y] spacing between box centers. Defaults to [0.2, 0.2].
      :type separation: List[float]
      :param box_size_rand: The randomization limits for the size of each box. Defaults to [0.05, 0.05, 0.05].
      :type box_size_rand: List[float]
      :param box_euler_rand: The randomization limits for the orientation of each box. Defaults to [0.2, 0.2, 0.2].
      :type box_euler_rand: List[float]
      :param separation_rand: The randomization limits for the spacing between boxes. Defaults to [0.05, 0.05].
      :type separation_rand: List[float]

   .. py:method:: add_aruco_marker(position: List[float] = [1.0, 0.0, 0.0], euler: List[float] = [0.0, 0.0, 0.0], size: List[float] = [0.1, 0.1, 0.1], marker_num: int = 0) -> None

      Adds a textured ArUco marker box to the scene. The marker texture is
      resolved internally (``markers/marker_7x7_{marker_num}.png``) and its
      material asset is registered automatically.

      :param position: The [x, y, z] coordinates of the marker. Defaults to [1.0, 0.0, 0.0].
      :type position: List[float]
      :param euler: The Euler angles [roll, pitch, yaw] for rotation. Defaults to [0.0, 0.0, 0.0].
      :type euler: List[float]
      :param size: The full size [x, y, z] of the marker geometry. Defaults to [0.1, 0.1, 0.1].
      :type size: List[float]
      :param marker_num: The ArUco marker ID (must be between 0 and 20 inclusive). Defaults to 0.
      :type marker_num: int
      :raises ValueError: If `marker_num` falls outside the 0-20 range.

   .. py:method:: save() -> None

      Formats and saves the current state of the generated XML scene to the active path.

   .. py:method:: reset_to_base(reset_resources: bool = False) -> None

      Overwrites the generated scene with the clean baseline scene and resets internal XML pointers.

      :param reset_resources: If True, also deletes all imported resources under :py:attr:`PUBLIC_RESOURCES_DIR` via ``resources.remove_all()``. Defaults to False.
      :type reset_resources: bool

   .. py:method:: load_scene_from_path(absolute_path: str | pathlib.Path) -> None

      Loads an external MuJoCo XML scene, sets it as the active generation target, and updates internal pointers.

      :param absolute_path: The absolute path to the target XML file.
      :type absolute_path: str | pathlib.Path
      :raises FileNotFoundError: If the provided path does not exist.

   .. py:method:: export_scene_to_directory(absolute_path: str | pathlib.Path) -> None

      Exports the current generated scene tree as 'scene.xml' to a specified directory.

      :param absolute_path: The path to the destination directory.
      :type absolute_path: str | pathlib.Path
      :raises FileNotFoundError: If the destination directory does not exist.


Resource Management
====================

.. py:exception:: ResourceError

   Raised for resource-management errors that are not better represented by
   a built-in exception, such as unsupported file formats or attempts to
   resolve a path outside the managed resource root.

.. py:class:: ResourceManager

   Manages a root directory of importable resource files (currently texture
   images) used by :py:class:`TerrainGenerator`. Accessible on a generator
   instance via its :py:attr:`TerrainGenerator.resources` property. All
   relative paths passed to its methods are resolved against, and confined
   to, the manager's root directory.

   .. py:attribute:: SUPPORTED_IMAGE_TYPES
      :type: set[str]
      :value: {".png"}

      The set of file extensions accepted by :py:meth:`import_texture`.

   .. py:method:: __init__(resources_root: pathlib.Path) -> None

      Initializes the manager, creating `resources_root` on disk if it does not already exist.

      :param resources_root: The directory this manager is rooted at.
      :type resources_root: pathlib.Path

   .. py:property:: root
      :type: pathlib.Path

      The resolved root directory managed by this instance.

   .. py:method:: import_texture(src: str | pathlib.Path, dest_folder: str = "") -> pathlib.Path

      Copies a texture file from an arbitrary source path into the managed
      resource root, under an optional destination subfolder.

      :param src: The path to the source image file to import.
      :type src: str | pathlib.Path
      :param dest_folder: The subfolder, relative to the resource root, to copy the file into. Defaults to the root itself.
      :type dest_folder: str
      :returns: The path of the imported file, relative to the resource root.
      :rtype: pathlib.Path
      :raises FileNotFoundError: If `src` does not exist.
      :raises ResourceError: If `src` has an unsupported file extension (see :py:attr:`SUPPORTED_IMAGE_TYPES`).

   .. py:method:: add_folder(relative_path: str | pathlib.Path) -> None

      Creates a folder (and any missing parent folders) under the resource root.

      :param relative_path: The folder path, relative to the resource root, to create.
      :type relative_path: str | pathlib.Path

   .. py:method:: remove_folder(relative_path: str | pathlib.Path, recursive: bool = True) -> None

      Removes a folder under the resource root.

      :param relative_path: The folder path, relative to the resource root, to remove.
      :type relative_path: str | pathlib.Path
      :param recursive: If True, removes the folder and all of its contents. If False, only removes the folder if it is empty. Defaults to True.
      :type recursive: bool
      :raises FileNotFoundError: If the folder does not exist.
      :raises ResourceError: If the path is not a directory, or if `recursive` is False and the folder is not empty.

   .. py:method:: remove_file(relative_path: str | pathlib.Path) -> None

      Removes a single file under the resource root.

      :param relative_path: The file path, relative to the resource root, to remove.
      :type relative_path: str | pathlib.Path
      :raises FileNotFoundError: If the file does not exist.
      :raises ResourceError: If the path is not a file.

   .. py:method:: remove_all() -> None

      Deletes the entire resource root directory and recreates it empty.

   .. py:method:: list_dirs(relative_path: str | pathlib.Path = "") -> None

      Prints a tree view of the contents of a directory under the resource
      root (defaults to the root itself) to stdout. Directories are listed
      before files at each level, sorted alphabetically.

      :param relative_path: The directory to list, relative to the resource root. Defaults to the resource root.
      :type relative_path: str | pathlib.Path
      :raises FileNotFoundError: If the directory does not exist.
      :raises NotADirectoryError: If the path is not a directory.

   .. py:method:: exists(relative_path: str | pathlib.Path = "") -> bool

      Checks whether a given path exists under the resource root.

      :param relative_path: The path to check, relative to the resource root.
      :type relative_path: str | pathlib.Path
      :returns: True if the path exists and lies within the resource root, False otherwise (including if the path would resolve outside the root).
      :rtype: bool