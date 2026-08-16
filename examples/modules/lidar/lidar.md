# LIDAR Module Examples

This folder contains examples for working with the controller's LIDAR capabilities in the **SIMULATOR**.
The controller uses the same interface when running on the actual robot, but examples for the physical robot are not provided here.
These examples assume that you are already familiar with using the `TerrainGenerator` class to modify the scene layout. If not, please see the [Terrain Generator Examples](../../terrain_generator/terrain_generator.md).
These examples also assume familiarity with the controller.

These examples rely on `matplotlib` for rendering decoded `PointCloud2` data.
Of course, there are **much** better tools available for this. However, `matplotlib` is the most consistent
on a per-machine basis and doesn't have any troublesome dependencies. If you seek better performance, I recommend using
[Open3D](https://www.open3d.org/).

Run the following command to install the package.

```bash
pip install matplotlib
```

> [!NOTE]
> The example files for this module are mostly AI Generated. That's not *really* an issue given the code has been tested for validity.
> In fact, it is encouraged that you use AI to generate scenes when using `TerrainGenerator` class.
> Manually creating scenes by hand is **extremely tedious**. (Of course, make sure to always proof-read).


## What's Here

The following scripts should be ran in order.

- [create_sim_scene.py](create_sim_scene.py) — Script to setup the simulator scene for the LIDAR example. 
- [render_points_sim.py](render_points_sim.py) — Example showing how to render decoded `PointCloud2` data from the controller's LIDAR module.