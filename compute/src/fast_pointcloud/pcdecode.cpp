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
#include "compiler.hpp"
#include "raii.hpp"
#include "typing.hpp"

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
FORCE_INLINE T read_val(const void* RESTRICT p) {
    T v; 
    std::memcpy(&v, p, sizeof(T));
    if constexpr (Swap) {
        v = byteswap_val(v);
    }

    return v;
}


struct NoIntensity {};

using decode_kernel_t = Py_ssize_t (*)(
    const char* base, Py_ssize_t n_points, int point_step,
    int ox, int oy, int oz, int oi,
    float* RESTRICT out);


template <bool Intensity>
static Py_ssize_t wide_decode_kernel(const char* base, Py_ssize_t n_points, int, int, int, int, int, float* RESTRICT out) {
    constexpr std::size_t kByteStride = (Intensity ? 4 : 3) * sizeof(float);

    std::memcpy(out, base, static_cast<std::size_t>(n_points) * kByteStride);
    return n_points;
}

template <bool Intensity, bool PaddedIntensity>
static Py_ssize_t laned_decode_kernel(const char * base, Py_ssize_t n_points, int point_step, int ox, int, int, int oi, float* RESTRICT out) {
    constexpr int kOutStride = Intensity ? 4 : 3;
    constexpr std::size_t kContiguousBytes = ((Intensity && !PaddedIntensity) ? 4 : 3) * sizeof(float);
    constexpr int kLaneSize = 4;
    
    const Py_ssize_t bulks = n_points & ~(kLaneSize - 1); 
    Py_ssize_t idx = 0;

    for (; idx < bulks; idx += 4) {
        const char* RESTRICT p0 = base + (idx + 0) * point_step;
        const char* RESTRICT p1 = base + (idx + 1) * point_step;
        const char* RESTRICT p2 = base + (idx + 2) * point_step;
        const char* RESTRICT p3 = base + (idx + 3) * point_step;

        std::memcpy(out + (idx + 0) * kOutStride, p0 + ox, kContiguousBytes);
        std::memcpy(out + (idx + 1) * kOutStride, p1 + ox, kContiguousBytes);
        std::memcpy(out + (idx + 2) * kOutStride, p2 + ox, kContiguousBytes);
        std::memcpy(out + (idx + 3) * kOutStride, p3 + ox, kContiguousBytes);

        if constexpr (Intensity && PaddedIntensity) {
            std::memcpy(out + (idx + 0) * kOutStride + 3, p0 + oi, sizeof(float));
            std::memcpy(out + (idx + 1) * kOutStride + 3, p1 + oi, sizeof(float));
            std::memcpy(out + (idx + 2) * kOutStride + 3, p2 + oi, sizeof(float));
            std::memcpy(out + (idx + 3) * kOutStride + 3, p3 + oi, sizeof(float));
        }
    }

    for (; idx < n_points; idx++) {
        const char* RESTRICT p = base + idx * point_step;
        std::memcpy(out + idx * kOutStride, p + ox, kContiguousBytes);

        if constexpr (Intensity && PaddedIntensity) {
            std::memcpy(out + idx * kOutStride + 3, p + oi, sizeof(float));
        }
    }

    return n_points;
}

template <Numeric TXyz, NumericOrSpecial<NoIntensity> TInten, bool Swap, bool SkipNans>
static Py_ssize_t generic_decode_kernel(const char* RESTRICT base, Py_ssize_t n_points, int point_step,
                                        int ox, int oy, int oz, int oi,
                                        float* RESTRICT out)
{
    constexpr bool kHasIntensity = !std::is_same_v<TInten, NoIntensity>;
    constexpr int kStride = kHasIntensity ? 4 : 3;

    Py_ssize_t count = 0;
    for (Py_ssize_t idx = 0; idx < n_points; idx++) { 
        const char* RESTRICT p = base + idx * point_step;
        float x = static_cast<float>(read_val<TXyz, Swap>(p + ox));
        float y = static_cast<float>(read_val<TXyz, Swap>(p + oy));
        float z = static_cast<float>(read_val<TXyz, Swap>(p + oz));

        float inten = 0.0f;
        if constexpr (kHasIntensity) {
            inten = static_cast<float>(read_val<TInten, Swap>(p + oi));
        }

        if constexpr (SkipNans) {
            if (std::isnan(x) || std::isnan(y) || std::isnan(z) || (kHasIntensity && std::isnan(inten))) {
                continue;
            }
        }

        out[count * kStride + 0] = x;
        out[count * kStride + 1] = y;
        out[count * kStride + 2] = z;

        if constexpr (kHasIntensity) {
            out[count * kStride + 3] = inten;
        }

        ++count;
    }

    return count;
}


using XyzTypes = std::tuple<int8_t, uint8_t, int16_t, uint16_t,
                            int32_t, uint32_t, float, double>;
using IntenTypes = std::tuple<NoIntensity, int8_t, uint8_t, int16_t, uint16_t,
                              int32_t, uint32_t, float, double>;

constexpr std::size_t kXyzTypesSize = std::tuple_size_v<XyzTypes>;
constexpr std::size_t kIntenTypesSize = std::tuple_size_v<IntenTypes>;

template <std::size_t Xi, std::size_t Ii, bool Swap, bool SkipNans>
consteval decode_kernel_t make_generic_decode_kernel_spec() {
    return &generic_decode_kernel<
        std::tuple_element_t<Xi, XyzTypes>,
        std::tuple_element_t<Ii, IntenTypes>,
        Swap,
        SkipNans
    >;
}

template <bool Swap, bool SkipNans, std::size_t Xi, std::size_t... Iis>
consteval auto build_generic_kernel_permute_inten_dim(std::index_sequence<Iis...>) {
    return std::array{make_generic_decode_kernel_spec<Xi, Iis, Swap, SkipNans>()...};
}

template <bool Swap, bool SkipNans, std::size_t... Xis>
consteval auto build_generic_kernel_permute_xyz_dim(std::index_sequence<Xis...>) {
    return std::array{build_generic_kernel_permute_inten_dim<Swap, SkipNans, Xis>(std::make_index_sequence<kIntenTypesSize>{})...};
}

template <bool Swap>
consteval auto build_generic_kernel_permute_nan_dim() {
    return std::array{
        build_generic_kernel_permute_xyz_dim<Swap, false>(std::make_index_sequence<kXyzTypesSize>{}),
        build_generic_kernel_permute_xyz_dim<Swap, true>(std::make_index_sequence<kXyzTypesSize>{})
    };
}

constexpr auto kGenericKernelTableNoSwap = build_generic_kernel_permute_nan_dim<false>();
constexpr auto kGenericKernelTableSwap = build_generic_kernel_permute_nan_dim<true>();


decode_kernel_t select_generic_decode_kernel_spec(PointFieldType dtype_xyz, bool has_intensity,
                                                    PointFieldType dtype_intensity, bool swap, bool skip_nans)
{
    int xi = static_cast<int>(dtype_xyz) - PF_INT8;
    if (xi < 0 || static_cast<size_t>(xi) >= kXyzTypesSize) {
        return nullptr;
    }

    int ii = has_intensity ? (static_cast<int>(dtype_intensity) - PF_INT8 + 1) : 0;
    if (ii < 0 || static_cast<size_t>(ii) >= kIntenTypesSize) {
        return nullptr;
    }

    int skip_idx = skip_nans ? 1 : 0;

    return (swap ? kGenericKernelTableSwap : kGenericKernelTableNoSwap)[skip_idx][xi][ii];
}

decode_kernel_t select_laned_decode_kernel_spec(bool has_intensity, bool padded_intensity) {
    if (!has_intensity) {
        return &laned_decode_kernel<false, false>;
    }

    if (padded_intensity) {
        return &laned_decode_kernel<true, true>;
    }

    return &laned_decode_kernel<true, false>;
}

decode_kernel_t select_decode_kernel(PointFieldType dtype_xyz, bool has_intensity, PointFieldType dtype_intensity,
                                      bool swap, bool skip_nans,
                                      int point_step, int ox, int oy, int oz, int oi)
{
    if (swap || skip_nans) {
        return select_generic_decode_kernel_spec(dtype_xyz, has_intensity, dtype_intensity, swap, skip_nans);
    }

    if (dtype_xyz != PF_FLOAT32 || (has_intensity && dtype_intensity != PF_FLOAT32)) {
        return select_generic_decode_kernel_spec(dtype_xyz, has_intensity, dtype_intensity, swap, skip_nans);
    }

    if (oy != ox + 4 || oz != ox + 8) {
        return select_generic_decode_kernel_spec(dtype_xyz, has_intensity, dtype_intensity, swap, skip_nans);
    }

    const bool intensity_contiguous = has_intensity && (oi == ox + 12);
    const int record_bytes = has_intensity ? 16 : 12;

    if (ox == 0 && point_step == record_bytes && (!has_intensity || intensity_contiguous)) {
        return has_intensity ? &wide_decode_kernel<true> : &wide_decode_kernel<false>;
    }

    return select_laned_decode_kernel_spec(has_intensity, !intensity_contiguous);
}


template <std::integral... Shape>
static PyRef build_arr_new(int typenum, Shape... shape) {
    const npy_intp shape_arr[] = { static_cast<npy_intp>(shape)... };
    return PyRef(PyArray_SimpleNew(sizeof...(Shape), shape_arr, typenum));
}

template <std::integral... Shape>
static PyRef build_owning_view_from_data(int typenum, PyArrayObject* root, Shape... shape) {
    const npy_intp shape_arr[] = { static_cast<npy_intp>(shape)... };

    PyRef view(PyArray_SimpleNewFromData(sizeof...(Shape), shape_arr, typenum, PyArray_DATA(root)));
    Py_INCREF(reinterpret_cast<PyObject*>(root));
    PyArray_SetBaseObject(reinterpret_cast<PyArrayObject*>(view.get()), reinterpret_cast<PyObject*>(root));

    return view;
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
    const npy_intp channels = has_intensity ? 4 : 3;

    PyBufferGuard buf(data_obj, PyBUF_SIMPLE);
    if (!buf.ok()) {
        PyErr_SetString(PyExc_BufferError, "Failed to get buffer from argument.");
        return nullptr;
    }

    if (buf.get().len == 0 || point_step <= 0) {
        return build_arr_new(NPY_FLOAT32, 0, channels).release();
    }

    Py_ssize_t n_points = buf.get().len / point_step;
    const char* RESTRICT base = static_cast<const char*>(buf.get().buf);
    bool swap = (host_little_endian() == static_cast<bool>(is_bigendian));

    decode_kernel_t kern = select_decode_kernel(dtype_xyz, has_intensity, dtype_intensity, swap, skip_nans, point_step, ox, oy, oz, oi);
    if (!kern) {
        PyErr_SetString(PyExc_ValueError, "Unsupported dtype combination.");
        return nullptr;
    }

    PyRef out = build_arr_new(NPY_FLOAT32, n_points, channels);
    if (!out) return nullptr;

    float* RESTRICT out_data = static_cast<float*>(PyArray_DATA(reinterpret_cast<PyArrayObject*>(out.get())));

    Py_ssize_t count;
    Py_BEGIN_ALLOW_THREADS
    count = kern(base, n_points, point_step, ox, oy, oz, oi, out_data);
    Py_END_ALLOW_THREADS

    if (count != n_points) {
        return build_owning_view_from_data(
            NPY_FLOAT32, 
            reinterpret_cast<PyArrayObject*>(out.get()), 
            count, 
            channels
        ).release();
    }

    return out.release();
}