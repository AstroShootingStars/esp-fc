import os
from SCons.Script import Import

Import("env")

# Work around intermittent hangs in riscv32-unknown-elf-ranlib on Windows.
platform = env.PioPlatform()
core_pkg = platform.get_package_dir("toolchain-rp2040-earlephilhower")
if core_pkg:
    bin_dir = os.path.join(core_pkg, "bin")
    gcc_ar = os.path.join(bin_dir, "riscv32-unknown-elf-gcc-ar.exe")
    gcc_ranlib = os.path.join(bin_dir, "riscv32-unknown-elf-gcc-ranlib.exe")

    if os.path.exists(gcc_ar):
        env.Replace(AR=gcc_ar)
    if os.path.exists(gcc_ranlib):
        env.Replace(RANLIB=gcc_ranlib)
