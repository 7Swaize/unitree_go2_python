#ifndef UTILS_RAII_HPP_
#define UTILS_RAII_HPP_

#include <Python.h>

class PyRef {
public:
    explicit PyRef(PyObject* obj = nullptr) noexcept : obj_(obj) {}

    ~PyRef() {
        if (obj_ != nullptr) {
            Py_XDECREF(obj_);
        }
    }

    PyRef(const PyRef&) = delete;
    PyRef& operator=(const PyRef&) = delete;

    PyRef(PyRef&& other) noexcept : obj_(other.release()) {}

    PyRef& operator=(PyRef&& other) noexcept {
        if (this != &other) {
            reset(other.release());
        }
        return *this;
    }

    [[nodiscard]]
    static PyRef NewRef(PyObject* obj) noexcept {
        Py_XINCREF(obj);
        return PyRef(obj);
    }

    [[nodiscard]]
    PyObject* get() const noexcept {
        return obj_;
    }

    [[nodiscard]]
    PyObject* release() noexcept {
        PyObject* tmp = obj_;
        obj_ = nullptr;
        return tmp;
    }

    void reset(PyObject* obj = nullptr) noexcept {
        Py_XDECREF(obj_);
        obj_ = obj;
    }

    [[nodiscard]]
    explicit operator bool() const noexcept {
        return obj_ != nullptr;
    }

private:
    PyObject* obj_;
};

class PyBufferGuard {
public:
    PyBufferGuard(PyObject* obj, int flags) {
        ok_ = (PyObject_GetBuffer(obj, &buf_, flags) == 0);
    }

    ~PyBufferGuard() {
        if (ok_) {
            PyBuffer_Release(&buf_);
        }
    }

    PyBufferGuard(const PyBufferGuard&) = delete;
    PyBufferGuard& operator=(const PyBufferGuard&) = delete;

    [[nodiscard]]
    bool ok() const noexcept {
        return ok_;
    }

    [[nodiscard]]
    Py_buffer& get() noexcept {
        return buf_;
    }

private:
    Py_buffer buf_{};
    bool ok_ = false;
};

#endif