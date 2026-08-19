import numpy as np
import numpy.typing as npt


PointFieldType: dict[str, int]
"""
Enum for point field data types.

Contains the following constants:
- INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64
"""


def decode_xyz_intensity(
    data: bytes,
    point_step: int,
    ox: int,
    oy: int,
    oz: int,
    oi: int,
    is_bigendian: int,
    is_dense: int,
    dtype_xyz: int,
    dtype_intensity: int,
    skip_nans: int
) -> npt.NDArray[np.float32]:
    """
    Decode XYZ coordinates and optionally intensity from a PointCloud2 buffer.

    The returned array is a compact, owning NumPy array with one row per
    decoded point. Each row contains ``[x, y, z]`` when intensity is absent,
    or ``[x, y, z, intensity]`` when intensity is present.

    Parameters
    ----------
    data : bytes
        Raw PointCloud2 point data.
    point_step : int
        Size of each point record in bytes.
    ox, oy, oz : int
        Byte offsets of the x, y, and z fields within each point record.
    oi : int
        Byte offset of the intensity field within each point record, or
        ``-1`` if no intensity field is present.
    is_bigendian : int
        PointCloud2 endianness flag. ``1`` indicates big-endian data and
        ``0`` indicates little-endian data.
    is_dense : int
        Dense flag. ``1`` indicates dense data and ``0`` indicates not dense data.
    dtype_xyz : int
        Point field datatype of the x, y, and z fields, using ``PF_*``
        constants.
    dtype_intensity : int
        Point field datatype of the intensity field, using ``PF_*``
        constants. Ignored when ``oi == -1``.
    skip_nans : int
        If nonzero, omit points containing a NaN in any decoded field.
        Otherwise, all points are returned.

    Returns
    -------
    numpy.ndarray
        A two-dimensional ``float32`` array of shape ``(n, 3)`` when
        intensity is absent or ``(n, 4)`` when intensity is present.
        When ``skip_nans`` is nonzero, ``n`` may be smaller than the number
        of points in the input buffer.
    """
    ...