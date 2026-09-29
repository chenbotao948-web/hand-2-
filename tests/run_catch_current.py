"""Run the current CATCH timing/UART regression against real ARM firmware sources."""
import argparse
import pathlib
import struct
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument("--keil", default="D:/Keil_v5/ARM/ARMCC/bin")
parser.add_argument("--deps")
args = parser.parse_args()
if args.deps:
    sys.path.insert(0, args.deps)
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

root = pathlib.Path(__file__).resolve().parents[1]
includes = ["Core/Inc", "MDK-ARM/Hardware", "Drivers/STM32F1xx_HAL_Driver/Inc",
            "Drivers/CMSIS/Include", "Drivers/CMSIS/Device/ST/STM32F1xx/Include"]
sources = [root / "tests/test_catch_current.c",
           root / "MDK-ARM/Hardware/stage.c", root / "MDK-ARM/Hardware/usart.c"]

with tempfile.TemporaryDirectory(prefix="hand-catch-test-") as temporary:
    work = pathlib.Path(temporary)
    objects = []
    for source in sources:
        obj = work / (source.stem + ".o")
        cmd = [str(pathlib.Path(args.keil) / "armcc.exe"), "--cpu=Cortex-M3", "--c99",
               "--library_type=microlib", "-DSTM32F103xE", "-DUSE_HAL_DRIVER", "-O0", "-g"]
        for directory in includes:
            cmd += ["-I", str(root / directory)]
        subprocess.run(cmd + ["-c", str(source), "-o", str(obj)], check=True, cwd=work)
        objects.append(str(obj))
    elf_path = work / "catch.axf"
    subprocess.run([str(pathlib.Path(args.keil) / "armlink.exe"), "--cpu=Cortex-M3",
                    "--library_type=microlib", "--entry=main", "--ro-base=0x08000000",
                    "--rw-base=0x20000000", "--datacompressor=off", *objects,
                    "-o", str(elf_path)], check=True, cwd=work)
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.mem_map(0x08000000, 0x100000)
    uc.mem_map(0x20000000, 0x10000)
    with elf_path.open("rb") as handle:
        elf = ELFFile(handle)
        for section in elf.iter_sections():
            if section["sh_flags"] & 2 and section["sh_type"] != "SHT_NOBITS":
                uc.mem_write(section["sh_addr"], section.data())
        symbols = {symbol.name: symbol["st_value"] for symbol in
                   elf.get_section_by_name(".symtab").iter_symbols()}
    uc.reg_write(UC_ARM_REG_SP, 0x2000F000)
    uc.reg_write(UC_ARM_REG_LR, 0x080FF001)
    uc.emu_start(symbols["main"] | 1, 0x080FF000, timeout=30000000, count=20000000)
    checks = struct.unpack("<I", uc.mem_read(symbols["test_checks"], 4))[0]
    failed = struct.unpack("<I", uc.mem_read(symbols["test_failed_line"], 4))[0]
    if failed:
        raise AssertionError(f"test_catch_current.c:{failed}, {checks} checks reached")
    if uc.reg_read(UC_ARM_REG_PC) != 0x080FF000:
        raise AssertionError("CATCH test did not complete")
    print(f"CATCH: PASS ({checks} checks, current ARM stage.c/usart.c)")
