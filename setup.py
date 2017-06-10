from distutils.core import setup
from Cython.Build import cythonize

setup(
      name = "trajlap",
      ext_modules = cythonize('colors.pyx'),  # accepts a glob pattern
    )
