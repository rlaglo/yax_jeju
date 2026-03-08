import os
import types
import marshal

def get_path(file_name=None):
    # p = os.path.dirname(os.path.abspath(__file__)).split("/")
    # LIB_PATH = os.path.join("/", *p[1:4], "src", *p[5:6],*p[5:6], "lib", file_name)
    # return LIB_PATH
    current_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(current_dir, file_name)
    return file_path


def get_pyc(module_file):
    file_path = get_path(module_file)
    print(f'\n[INFO] 라이브러리 로드 시도: {file_path}\n')
    
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"파일을 찾을 수 없습니다: {file_path}")

    with open(file_path, 'rb') as f:
        pyc = f.read()
        
    # Python 3.7+ 버전의 .pyc 헤더는 16바이트입니다.
    code = marshal.loads(pyc[16:])
    module = types.ModuleType('camera_perception_func_lib')
    exec(code, module.__dict__)
    return module

#/home/qwer123/yax/hlfma2025/src/camera_perception_pkg/camera_perception_pkg/lib/camera_perception_func_lib.cpython-310.pyc
#/home/qwer123/yax/hlfma2025/build/camera_perception_pkg/camera_perception_pkg/lib/lib/camera_perception_func_lib.cpython-310.pyc

lidar_perception_func_lib = get_pyc("lidar_perception_func_lib.cpython-310.pyc")
