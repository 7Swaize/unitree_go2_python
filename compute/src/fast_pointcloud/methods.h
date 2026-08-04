#ifndef METHODS_H
#define METHODS_H

#include <Python.h>

extern "C" __attribute__((unused)) PyObject* decode_xyz_intensity(PyObject* self, PyObject* args);

// https://docs.python.org/3/extending/extending.html#the-module-s-method-table-and-initialization-function
#define FAST_PC_DECODE_METHODS \
    { \
        "decode_xyz_intensity", \
        decode_xyz_intensity, \
        METH_VARARGS, \
        "Fast PointCloud2 XYZ(+intensity) decoder" \
    }

#define FAST_PC_METHODS_END \
    { NULL, NULL, 0, NULL }

#endif