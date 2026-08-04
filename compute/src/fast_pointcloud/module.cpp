#define PY_SSIZE_T_CLEAN
#define PY_ARRAY_UNIQUE_SYMBOL FPC_ARRAY_API
#include <Python.h>
#include <numpy/arrayobject.h>

#include "methods.h"
#include "raii.hpp"


extern "C" PyObject* decode_xyz_intensity(PyObject*, PyObject*);

static PyRef create_point_field_type_enum(void) {
    PyRef enum_dict(PyDict_New());
    if (!enum_dict) return PyRef(nullptr);

    // Person on stack overflow said we need to decref on exit
    // See ref: https://stackoverflow.com/questions/40700251/reference-counting-using-pydict-setitemstring
    PyRef int_int8(PyLong_FromLong(1));
    PyRef int_uint8(PyLong_FromLong(2));
    PyRef int_int16(PyLong_FromLong(3));
    PyRef int_uint16(PyLong_FromLong(4));
    PyRef int_int32(PyLong_FromLong(5));
    PyRef int_uint32(PyLong_FromLong(6));
    PyRef int_float32(PyLong_FromLong(7));
    PyRef int_float64(PyLong_FromLong(8));

    if (!(int_int8 && int_uint8 && int_int16 && int_uint16 && int_int32 && int_uint32 && int_float32 && int_float64)) {
        return PyRef(nullptr);
    }

    PyObject* enum_dict_pyobj = enum_dict.get();

    PyDict_SetItemString(enum_dict_pyobj, "INT8", int_int8.get());
    PyDict_SetItemString(enum_dict_pyobj, "UINT8", int_uint8.get());
    PyDict_SetItemString(enum_dict_pyobj, "INT16", int_int16.get());
    PyDict_SetItemString(enum_dict_pyobj, "UINT16", int_uint16.get());
    PyDict_SetItemString(enum_dict_pyobj, "INT32", int_int32.get());
    PyDict_SetItemString(enum_dict_pyobj, "UINT32", int_uint32.get());
    PyDict_SetItemString(enum_dict_pyobj, "FLOAT32", int_float32.get());
    PyDict_SetItemString(enum_dict_pyobj, "FLOAT64", int_float64.get());

    return enum_dict;
}

static PyMethodDef module_methods[] = {
    FAST_PC_DECODE_METHODS,
    FAST_PC_METHODS_END
};

static struct PyModuleDef moduledef = {
    PyModuleDef_HEAD_INIT,
    "fast_pointcloud",
    "Fast PointCloud2 XYZ(+intensity) decoder",
    -1,
    module_methods
};

extern "C" PyMODINIT_FUNC PyInit_fast_pointcloud(void) { 
    import_array()
    
    PyRef module(PyModule_Create(&moduledef));
    if (!module) return nullptr;

    PyRef point_field_enum = create_point_field_type_enum();
    if (!point_field_enum) return nullptr;

    if (PyModule_AddObjectRef(module.get(), "PointFieldType", point_field_enum.get()) < 0) { 
        return nullptr;
    }

    return module.release();
}
