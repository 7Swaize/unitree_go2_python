import os
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext as _build_ext

# https://stackoverflow.com/questions/27817190/what-does-cmdclass-do-in-pythons-setuptools
class build_ext_with_numpy(_build_ext):
    def finalize_options(self):
        _build_ext.finalize_options(self)

        import numpy
        self.include_dirs.append(numpy.get_include())


ext_modules = [
    Extension(
        "fast_pointcloud",
        sources=[
            os.path.join("src", "fast_pointcloud", "module.cpp"),
            os.path.join("src", "fast_pointcloud", "pcdecode.cpp")
        ],
        include_dirs=[
            os.path.join("src", "fast_pointcloud"),
            os.path.join("src", "utils"),
        ],
        language="c++",
        define_macros=[('NPY_NO_DEPRECATED_API', 'NPY_2_0_API_VERSION')],
        extra_compile_args=[
            "-std=c++20",
            "-O3",
            "-DNDEBUG",
            "-flto",
            "-fvisibility=hidden",
            "-march=native",
            "-Wall"
        ],
        extra_link_args=[
            "-flto"
        ]
    )
]

setup(
    cmdclass={'build_ext': build_ext_with_numpy},
    ext_modules=ext_modules,
)
