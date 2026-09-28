"""Compile actual firmware C with ARMCC and execute in a Cortex-M3 emulator.

Dependencies: unicorn, pyelftools (can be installed in a temporary directory).
Usage: python tests/run_tests.py --deps <pip --target directory>
"""
import argparse
import pathlib
import struct
import subprocess
import sys
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--keil', default='D:/Keil_v5/ARM/ARMCC/bin')
parser.add_argument('--deps')
args = parser.parse_args()
if args.deps:
    sys.path.insert(0, args.deps)
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

root = pathlib.Path(__file__).resolve().parents[1]
includes = ['Core/Inc', 'MDK-ARM/Hardware', 'Drivers/STM32F1xx_HAL_Driver/Inc',
            'Drivers/CMSIS/Include', 'Drivers/CMSIS/Device/ST/STM32F1xx/Include']
suites = {'control': ['pid.c', 'usart.c'], 'stage': ['pid.c', 'usart.c'], 'imu': ['imu.c'],
          'driver': ['icm42688.c', 'imu.c', 'pid.c']}

with tempfile.TemporaryDirectory(prefix='car-control-tests-') as temporary:
    work = pathlib.Path(temporary)
    for suite, sources in suites.items():
        objects = []
        files = [root / 'tests' / ('test_' + suite + '.c')]
        files += [root / 'MDK-ARM/Hardware' / name for name in sources]
        for source in files:
            obj = work / (source.stem + '.o')
            cmd = [str(pathlib.Path(args.keil) / 'armcc.exe'), '--cpu=Cortex-M3', '--c99',
                   '--library_type=microlib', '-DSTM32F103xE', '-DUSE_HAL_DRIVER', '-O0', '-g']
            for directory in includes:
                cmd += ['-I', str(root / directory)]
            cmd += ['-c', str(source), '-o', str(obj)]
            subprocess.run(cmd, check=True, cwd=work)
            objects.append(str(obj))
        elf_path = work / (suite + '.axf')
        subprocess.run([str(pathlib.Path(args.keil) / 'armlink.exe'), '--cpu=Cortex-M3',
                        '--library_type=microlib', '--entry=main', '--ro-base=0x08000000',
                        '--rw-base=0x20000000', '--datacompressor=off', *objects, '-o', str(elf_path)], check=True, cwd=work)
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(0x08000000, 0x100000)
        uc.mem_map(0x20000000, 0x10000)
        with elf_path.open('rb') as handle:
            elf = ELFFile(handle)
            # 直接进入 main，所以在执行地址装入 RW 初值，替代启动代码 scatter-load。
            for section in elf.iter_sections():
                if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS':
                    uc.mem_write(section['sh_addr'], section.data())
            symbols = {symbol.name: symbol['st_value'] for symbol in elf.get_section_by_name('.symtab').iter_symbols()}
        uc.reg_write(UC_ARM_REG_SP, 0x2000F000)
        uc.reg_write(UC_ARM_REG_LR, 0x080FF001)
        uc.emu_start(symbols['main'] | 1, 0x080FF000, timeout=30000000, count=20000000)
        checks = struct.unpack('<I', uc.mem_read(symbols['test_checks'], 4))[0]
        failed = struct.unpack('<I', uc.mem_read(symbols['test_failed_line'], 4))[0]
        if failed:
            raise AssertionError(f'{suite}: test_{suite}.c:{failed}, {checks} checks reached')
        from unicorn.arm_const import UC_ARM_REG_PC
        if uc.reg_read(UC_ARM_REG_PC) != 0x080FF000:
            raise AssertionError(f'{suite}: did not complete')
        print(f'{suite}: PASS ({checks} checks, actual ARM firmware C)')
