#!/usr/bin/env python3
"""Run the actual Java engine against the actual JNI library on the host JVM."""
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]

def main():
    candidates = [Path(os.environ['JAVA_HOME'])] if os.environ.get('JAVA_HOME') else []
    if platform.system() == 'Darwin':
        result = subprocess.run(['/usr/libexec/java_home'], capture_output=True, text=True)
        if result.returncode == 0:
            candidates.append(Path(result.stdout.strip()))
    else:
        import shutil
        javac = shutil.which('javac')
        if javac:
            candidates.append(Path(javac).resolve().parents[1])
    jdk = next((p for p in candidates if (p/'include/jni.h').is_file()), None)
    if jdk is None:
        print('Set JAVA_HOME to a JDK with JNI headers.', file=sys.stderr)
        return 1
    folder = ROOT/'build/set-worker-jni-probe'
    folder.mkdir(parents=True, exist_ok=True)
    suffix = 'dylib' if platform.system() == 'Darwin' else 'so'
    subprocess.run([
        os.environ.get('CXX', 'c++'), '-std=c++17', '-O2', '-shared', '-fPIC',
        '-I'+str(jdk/'include'), '-I'+str(jdk/'include'/platform.system().lower()),
        '-Iapp/src/main/cpp', 'app/src/main/cpp/chica_gait_jni.cpp',
        'app/src/main/cpp/apk_model.cpp', 'app/src/main/cpp/pulse_conversion.cpp',
        '-o', str(folder/f'libchica_gait.{suffix}'),
    ], cwd=ROOT, check=True)
    subprocess.run([
        str(jdk/'bin/javac'), '-d', str(folder),
        'app/src/main/java/com/makeyourpet/chicaserver/gait/ChicaGaitEngine.java',
        'tools/oracle/SetWorkerJniProbe.java',
    ], cwd=ROOT, check=True)
    return subprocess.run([
        str(jdk/'bin/java'), '--enable-native-access=ALL-UNNAMED', '-Xcheck:jni', '-Djava.library.path='+str(folder),
        '-cp', str(folder), 'SetWorkerJniProbe',
    ], cwd=ROOT).returncode

if __name__ == '__main__':
    raise SystemExit(main())
