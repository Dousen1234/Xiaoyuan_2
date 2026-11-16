"""Compile/run actual claw_control.c with passive HAL stubs; never opens CAN."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
compiler = next((shutil.which(c) for c in ('clang', 'gcc') if shutil.which(c)), None)
if compiler:
    command = [compiler]
elif importlib.util.find_spec('ziglang'):
    command = [sys.executable, '-m', 'ziglang', 'cc']
else:
    raise SystemExit('Install clang/gcc or: python -m pip install ziglang')
with tempfile.TemporaryDirectory(prefix='claw_native_') as temporary:
    executable = str(Path(temporary) / 'control_test.exe')
    for mask, limit in ((0,3000), (1,3000), (2,3000), (4,3000), (7,3000), (7,10000), (7,0)):
        subprocess.run(command + ['-std=c99', '-Wall', '-Wextra', '-Werror',
                                 '-DCLAW_PROFILE_GROUP=2',
                                 f'-DCLAW_COMMISSIONED_MASK={mask}',
                                 f'-DCLAW_TRIAL_MAX_STEP_MDEG={limit}',
                                 '-I', str(root / 'tests/native_stubs'),
                                 '-I', str(root / 'App/Inc'),
                                 str(root / 'tests/control_native.c'), str(root / 'App/Src/claw_protocol.c'), '-o', executable], check=True)
        subprocess.run([executable], check=True)
