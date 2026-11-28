"""Optional native accelerator for the TI C2000 (C28x + CLA) devices.

Everything is configured in pyproject.toml; this file only declares the C
extension.  It is optional: without a C compiler the install succeeds and the
C2000 devices run on the (much slower) pure-Python executor.
"""
from setuptools import Extension, setup

setup(ext_modules=[Extension("marsemu.c28x._c28x", ["marsemu/c28x/_c28x.c"], optional=True,
                             depends=["marsemu/c28x/_c28x_exec.h", "marsemu/c28x/_c28x_keys.h",
                                      "marsemu/c28x/_cla.h", "marsemu/c28x/_f2803x.h",
                                      "marsemu/c28x/_f2803x_api.h"],
                             extra_compile_args=["-O3"])])
