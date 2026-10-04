#!/usr/bin/env python3
"""Compile/run the actual native set helper against APK-derived B/G formulae."""
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]

def main():
    candidates = [Path(os.environ['JAVA_HOME'])] if os.environ.get('JAVA_HOME') else []
    if platform.system() == 'Darwin':
        candidates += [Path('/Applications/Android Studio.app/Contents/jbr/Contents/Home')]
        default_jdk = subprocess.run(['/usr/libexec/java_home'], capture_output=True, text=True)
        if default_jdk.returncode == 0:
            candidates += [Path(default_jdk.stdout.strip())]
    else:
        import shutil
        javac = shutil.which('javac')
        if javac:
            candidates += [Path(javac).resolve().parents[1]]
    jdk = next((p for p in candidates if (p / 'include/jni.h').is_file()), None)
    if jdk is None:
        print('Set JAVA_HOME to a JDK with JNI headers.', file=sys.stderr)
        return 1
    binary = ROOT / 'build/set_pose_model_probe'
    binary.parent.mkdir(exist_ok=True)
    subprocess.run([
        os.environ.get('CXX', 'c++'), '-std=c++17', '-O2',
        '-I' + str(jdk / 'include'),
        '-I' + str(jdk / 'include' / platform.system().lower()),
        '-Iapp/src/main/cpp', 'tools/oracle/set_pose_model_probe.cpp',
        'app/src/main/cpp/apk_model.cpp', 'app/src/main/cpp/pulse_conversion.cpp',
        '-o', str(binary),
    ], cwd=ROOT, check=True)
    return subprocess.run([str(binary)], cwd=ROOT).returncode

if __name__ == '__main__':
    raise SystemExit(main())
