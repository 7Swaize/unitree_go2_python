#define PY_SSIZE_T_CLEAN
#define NO_IMPORT_ARRAY
#define PY_ARRAY_UNIQUE_SYMBOL FPC_ARRAY_API
#include <Python.h>
#include <numpy/arrayobject.h>

#include <array>
#include <bit>
#include <cstring>
#include <tuple>

#include "methods.h"
#include "utils/compiler.hpp"
#include "utils/raii.hpp"
#include "utils/typing.hpp"

namespace pcdecode {

enum PointFieldType : int {
    PF_INT8    = 1,
    PF_UINT8   = 2,
    PF_INT16   = 3,
    PF_UINT16  = 4,
    PF_INT32   = 5,
    PF_UINT32  = 6,
    PF_FLOAT32 = 7,
    PF_FLOAT64 = 8,
};

static bool host_little_endian() {
    uint16_t x = 1;
    return static_cast<bool>(*reinterpret_cast<uint8_t*>(&x));
}

template <Numeric T>
FORCE_INLINE T byteswap_val(T v) {
    if constexpr (sizeof(T) == 1) {
        return v;
    } else if constexpr (sizeof(T) == 2) {
        return std::bit_cast<T>(__builtin_bswap16(std::bit_cast<uint16_t>(v)));
    } else if constexpr (sizeof(T) == 4) {
        return std::bit_cast<T>(__builtin_bswap32(std::bit_cast<uint32_t>(v)));
    } else {
        return std::bit_cast<T>(__builtin_bswap64(std::bit_cast<uint64_t>(v)));
    }
}

template <Numeric T, bool Swap>
FORCE_INLINE T read_val(const void* p) {
    T v; 
    std::memcpy(&v, p, sizeof(T));
    if constexpr (Swap) {
        v = byteswap_val(v);
    }

    return v;
}

struct NoIntensity {};

using decode_fn_t = Py_ssize_t (*)(
    const char* base, Py_ssize_t n_points, int point_step,
    int ox, int oy, int oz, int oi, bool skip_nans,
    float* RESTRICT xyz_data, float* RESTRICT i_data);


template <Numeric TXyz, NumericOrSpecial<NoIntensity> TInten, bool Swap>
static Py_ssize_t decode_loop(const char* base, Py_ssize_t n_points, int point_step,
                               int ox, int oy, int oz, int oi, bool skip_nans,
                               float* RESTRICT xyz_data, float* RESTRICT i_data)
{
    constexpr bool has_intensity = !std::is_same_v<TInten, NoIntensity>;
    
    Py_ssize_t count = 0;
    const char* p = base;
    for (Py_ssize_t idx = 0; idx < n_points; idx++, p += point_step) { 
        float x = static_cast<float>(read_val<TXyz, Swap>(p + ox));
        float y = static_cast<float>(read_val<TXyz, Swap>(p + oy));
        float z = static_cast<float>(read_val<TXyz, Swap>(p + oz));
    

        float inten = 0.0f;
        if constexpr (has_intensity) {
            inten = static_cast<float>(read_val<TInten, Swap>(p + oi));
        }

        bool drop = skip_nans && (std::isnan(x) || std::isnan(y) || std::isnan(z) || (has_intensity && std::isnan(inten)));
        if (drop) {
            continue;
        }

        xyz_data[count * 3 + 0] = x;
        xyz_data[count * 3 + 1] = y;
        xyz_data[count * 3 + 2] = z;
        if constexpr (has_intensity) {
            i_data[count] = inten;
        }

        ++count;
    }

    return count;
}

using XyzTypes = std::tuple<int8_t, uint8_t, int16_t, uint16_t,
                            int32_t, uint32_t, float, double>;
using IntenTypes = std::tuple<NoIntensity, int8_t, uint8_t, int16_t, uint16_t,
                              int32_t, uint32_t, float, double>;

constexpr int kXyzTypesSize = std::tuple_size_v<XyzTypes>;
constexpr int kIntenTypesSize = std::tuple_size_v<IntenTypes>;

template <bool Swap, std::size_t Xi, std::size_t Ii>
consteval decode_fn_t make_fn() {
    return &decode_loop<
        std::tuple_element_t<Xi, XyzTypes>,
        std::tuple_element_t<Ii, IntenTypes>,
        Swap
    >;
}

template <bool Swap, std::size_t Xi, std::size_t... Iis>
consteval auto make_row(std::index_sequence<Iis...>) {
    return std::array{
        make_fn<Swap, Xi, Iis>()...
    };
}

template <bool Swap, std::size_t... Xis>
consteval auto make_table(std::index_sequence<Xis...>) {
    return std::array{
        make_row<Swap, Xis>(std::make_index_sequence<kIntenTypesSize>{})...
    };
}

constexpr auto kTableNoSwap = make_table<false>(std::make_index_sequence<kXyzTypesSize>{});
constexpr auto kTableSwap = make_table<true>(std::make_index_sequence<kXyzTypesSize>{});


static decode_fn_t select_decode_fn(PointFieldType dtype_xyz, bool has_intensity, PointFieldType dtype_intensity, bool swap) {
    int xi = static_cast<int>(dtype_xyz) - PF_INT8;
    if (xi < 0 || static_cast<size_t>(xi) >= kXyzTypesSize) {
        return nullptr;
    }

    int ii = has_intensity ? (static_cast<int>(dtype_intensity) - PF_INT8 + 1) : 0;
    if (ii < 0 || static_cast<size_t>(ii) >= kIntenTypesSize) {
        return nullptr;
    }

    return (swap ? kTableSwap : kTableNoSwap)[xi][ii];
}


template <std::integral... Shape>
static PyRef build_xyz(int typenum, Shape... shape) {
    npy_intp shape_arr[] = { static_cast<npy_intp>(shape)... };
    return PyRef(PyArray_SimpleNew(sizeof...(Shape), shape_arr, typenum)); 
}

template <std::integral... Shape>
static PyRef build_intensity_optional(bool has_intensity, int typenum, Shape... shape) {
    npy_intp shape_arr[] = { static_cast<npy_intp>(shape)... };
    return has_intensity
        ? PyRef(PyArray_SimpleNew(sizeof...(Shape), shape_arr, typenum))
        : PyRef::NewRef(Py_None);
}

static PyObject* build_empty_ret(bool has_intensity) { 
    PyRef xyz = build_xyz(NPY_FLOAT32, 0, 3);
    if (!xyz) return nullptr;

    PyRef intensity = build_intensity_optional(has_intensity, NPY_FLOAT32, 0);
    if (!intensity) return nullptr;

    return PyTuple_Pack(2, xyz.get(), intensity.get());
}


} // namespace pcdecode


extern "C" __attribute__((unused)) PyObject* decode_xyz_intensity(PyObject* self, PyObject* args) {
    using namespace pcdecode;

    PyObject* data_obj;
    int point_step, ox, oy, oz, oi;
    int is_bigendian, dtype_xyz_val, dtype_intensity_val, skip_nans;

    if (!PyArg_ParseTuple(args, "Oiiiiiiiii", 
            &data_obj, &point_step, &ox, &oy, &oz, &oi,
            &is_bigendian, &dtype_xyz_val, &dtype_intensity_val, &skip_nans))
    {
        return nullptr;
    }

    PointFieldType dtype_xyz = static_cast<PointFieldType>(dtype_xyz_val);
    PointFieldType dtype_intensity = static_cast<PointFieldType>(dtype_intensity_val);
    bool has_intensity = (oi >= 0);

    PyBufferGuard buf(data_obj, PyBUF_SIMPLE);
    if (!buf.ok()) {
        PyErr_SetString(PyExc_BufferError, "Failed to get simple buffer from argument.");
        return nullptr;
    }

    if (buf.get().len == 0 || point_step <= 0) { 
        return build_empty_ret(has_intensity);
    }

    Py_ssize_t n_points = buf.get().len / point_step;
    const char* base = static_cast<const char*>(buf.get().buf);
    bool swap = (host_little_endian() == static_cast<bool>(is_bigendian));

    decode_fn_t fn = select_decode_fn(dtype_xyz, has_intensity, dtype_intensity, swap);
    if (!fn) {
        PyErr_SetString(PyExc_ValueError, "Unsupported dtype combination.");
        return nullptr;
    }

    PyRef xyz = build_xyz(NPY_FLOAT32, n_points, 3);
    if (!xyz) return nullptr;

    PyRef intensity = build_intensity_optional(has_intensity, NPY_FLOAT32, n_points);
    if (!intensity) return nullptr;

    float* RESTRICT xyz_data = static_cast<float*>(PyArray_DATA(reinterpret_cast<PyArrayObject*>(xyz.get())));
    float* RESTRICT i_data = has_intensity
        ? static_cast<float*>(PyArray_DATA(reinterpret_cast<PyArrayObject*>(intensity.get())))
        : nullptr;

    Py_ssize_t count;
    Py_BEGIN_ALLOW_THREADS
    count = fn(base, n_points, point_step, ox, oy, oz, oi, static_cast<bool>(skip_nans), xyz_data, i_data);
    Py_END_ALLOW_THREADS

    if (count != n_points) {
        npy_intp resize_dims_xyz[2] = {count, 3};
        PyArray_Dims newshape_xyz = {resize_dims_xyz, 2};
        PyRef resize_ret_xyz(PyArray_Resize(reinterpret_cast<PyArrayObject*>(xyz.get()), &newshape_xyz, 1, NPY_CORDER));
        if (!resize_ret_xyz) return nullptr;

        if (has_intensity) {
            npy_intp resize_dims_i[1] = {count};
            PyArray_Dims newshape_i = {resize_dims_i, 1};
            PyRef resize_ret_i(PyArray_Resize(reinterpret_cast<PyArrayObject*>(intensity.get()), &newshape_i, 1, NPY_CORDER));
            if (!resize_ret_i) return nullptr;
        }
    }

    return PyTuple_Pack(2, xyz.get(), intensity.get());
}