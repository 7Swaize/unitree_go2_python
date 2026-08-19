import numpy as np
import pytest
import fast_pointcloud as fp
from dataclasses import dataclass
 
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


POINTFIELD_TO_INTERNAL_CTYPE = {
    PointField.INT8: fp.PointFieldType["INT8"],
    PointField.UINT8: fp.PointFieldType["UINT8"],
    PointField.INT16: fp.PointFieldType["INT16"],
    PointField.UINT16: fp.PointFieldType["UINT16"],
    PointField.INT32: fp.PointFieldType["INT32"],
    PointField.UINT32: fp.PointFieldType["UINT32"],
    PointField.FLOAT32: fp.PointFieldType["FLOAT32"],
    PointField.FLOAT64: fp.PointFieldType["FLOAT64"],
}

@dataclass(frozen=True)
class PointCloudLayout:
    has_intensity: bool
    x_offset: int
    y_offset: int
    z_offset: int
    intensity_offset: int
    xyz_internal_type: int
    intensity_internal_type: int


def make_pointcloud2(xyz: np.ndarray, intensity: np.ndarray = None) -> PointCloud2:
    header = Header(frame_id='lidar')
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    if intensity is not None:
        fields.append(PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1))
        points = np.hstack([
            xyz.astype(np.float32),
            intensity.reshape(-1, 1).astype(np.float32)
        ])
    else:
        points = xyz.astype(np.float32)

    return point_cloud2.create_cloud(header, fields, points)


def make_random_cloud(n_points: int, has_intensity: bool = True, nan_fraction: float = 0.0, seed: int = 42) -> PointCloud2:
    rng = np.random.default_rng(seed)
 
    xyz = rng.uniform(-50.0, 50.0, size=(n_points, 3)).astype(np.float32)
 
    if nan_fraction > 0.0:
        n_nan = int(n_points * nan_fraction)
        nan_idx = rng.choice(n_points, size=n_nan, replace=False)
        xyz[nan_idx, 0] = np.nan
 
    intensity = (
        rng.uniform(0.0, 255.0, size=n_points).astype(np.float32)
        if has_intensity
        else None
    )
 
    msg = make_pointcloud2(xyz, intensity)
    msg.is_dense = (nan_fraction == 0.0)
    return msg



def init_pointcloud_layout(msg: PointCloud2) -> PointCloudLayout:
    fields: dict[str, PointField] = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        raise ValueError("PointCloud2 missing XYZ fields")

    dtype_xyz = fields["x"].datatype
    if not all(fields[k].datatype == dtype_xyz for k in ("x", "y", "z")):
        raise TypeError("Mixed XYZ datatypes not supported")
    
    xyz_internal = POINTFIELD_TO_INTERNAL_CTYPE.get(dtype_xyz)
    if xyz_internal is None:
        raise TypeError(f"Unsupported XYZ datatype: {dtype_xyz}")

    has_intensity = "intensity" in fields
    if has_intensity:
        intensity_dtype = fields["intensity"].datatype
        intensity_internal = POINTFIELD_TO_INTERNAL_CTYPE.get(intensity_dtype)
        if intensity_internal is None:
            raise TypeError(f"Unsupported intensity datatype: {intensity_dtype}")
        intensity_offset = fields["intensity"].offset
    else:
        intensity_internal = fp.PointFieldType["INT8"]
        intensity_offset = -1

    return PointCloudLayout(
        has_intensity=has_intensity,
        x_offset=fields["x"].offset,
        y_offset=fields["y"].offset,
        z_offset=fields["z"].offset,
        intensity_offset=intensity_offset,
        xyz_internal_type=xyz_internal,
        intensity_internal_type=intensity_internal
    )


def decode_unoptimized(msg: PointCloud2, layout: PointCloudLayout, skip_nans: bool):
    if layout.has_intensity:
        return point_cloud2.read_points_numpy(
            msg,
            field_names=["x", "y", "z", "intensity"],
            skip_nans=skip_nans,
        ).astype(np.float32)
    else:
        return point_cloud2.read_points_numpy(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=skip_nans,
        ).astype(np.float32)
 
 
def decode_optimized(msg: PointCloud2, layout: PointCloudLayout, skip_nans: bool):
    return fp.decode_xyz_intensity(
        msg.data,
        msg.point_step,
        layout.x_offset,
        layout.y_offset,
        layout.z_offset,
        layout.intensity_offset,
        msg.is_bigendian,
        layout.xyz_internal_type,
        layout.intensity_internal_type,
        skip_nans
    )
 
 
@pytest.fixture(params=[1_000, 10_000, 50_000, 300_000], ids=lambda n: f"n={n}")
def n_points(request):
    return request.param
 
@pytest.fixture(params=[True, False], ids=["intensity", "no_intensity"])
def has_intensity(request):
    return request.param
 
@pytest.fixture(params=[0.0, 0.05], ids=["dense", "5pct_nan"])
def nan_fraction(request):
    return request.param

@pytest.fixture(params=[True, False], ids=["skip_nans", "keep_nans"])
def skip_nans(request):
    return request.param
 
@pytest.fixture
def cloud_msg(n_points, has_intensity, nan_fraction):
    return make_random_cloud(n_points, has_intensity=has_intensity, nan_fraction=nan_fraction)
 
@pytest.fixture
def layout(cloud_msg):
    return init_pointcloud_layout(cloud_msg)


@pytest.mark.benchmark(group="decode")
def test_decode_unoptimized(benchmark, cloud_msg, layout, skip_nans):
    out = benchmark(decode_unoptimized, cloud_msg, layout, skip_nans)
    assert out.dtype == np.float32
 
 
@pytest.mark.benchmark(group="decode")
def test_decode_optimized(benchmark, cloud_msg, layout, skip_nans):
    out = benchmark(decode_optimized, cloud_msg, layout, skip_nans)
    assert out.dtype == np.float32


'''
pytest decode_optimized_vs_unoptimized.py -v --benchmark-min-rounds=50 --benchmark-warmup=on --benchmark-disable-gc
'''