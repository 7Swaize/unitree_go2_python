LiDAR Control Module
--------------------

Access LiDAR sensor data from the Unitree Go2 robot or the Mujoco Simulator.
Users are *recommended* to interact with :class:`~modules.lidar.lidar_module.LIDARModule` when
``PointCloud2`` data is needed.

If the provided modules do not sufficiently meet your needs, you can bypass them and use an alternative approach.
You can also submit a feature request on the main `GitHub <https://github.com/7Swaize/go2-control/tree/v1.1.0>`_ page,
and it will be considered for future updates.


.. autoclass:: go2.modules.lidar.lidar_module.LIDARModule
   :members:
   :undoc-members:
   :show-inheritance:


.. autoclass:: go2.modules.lidar.native_lidar_module.NativeLIDARModule
   :members:
   :undoc-members:
   :show-inheritance:


.. autoclass:: go2.modules.lidar.virtual_lidar_module.VirtualLIDARModule
   :members:
   :undoc-members:
   :show-inheritance:
