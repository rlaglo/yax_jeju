import serial
import marshal
import types
import os
import time
import pygame
from pygame.locals import *

real_path = os.path.dirname(os.path.realpath(__file__))
pyc = open((real_path)+'/data_collection_func_lib.cpython-310.pyc', 'rb').read()
code = marshal.loads(pyc[16:])
module = types.ModuleType('module_name')
exec(code, module.__dict__)

def main():
    DATA_PATH= os.path.dirname(real_path) + '/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets' 
    CAMERA_NUM = 0
    SERIAL_PORT = "/dev/ttyUSB0"
    MAX_STEERING = 7  # 사용자 정의 최대 조향 단계
    STEP_STEERING = 1 # 한 번 누를 때마다 변하는 조향 단계
    MAX_SPEED = 250   # 사용자 정의 최대 속도 단계
    STEP_SPEED = 10   # 한 번 누를 때마다 변하는 속도 단계

    print(DATA_PATH)

    # 데이터 수집 객체 초기화
    data_collector = module.Data_Collect(path=DATA_PATH, cam_num=CAMERA_NUM, max_steering=MAX_STEERING)
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    time.sleep(1)
    
    # pygame 및 조이스틱 초기화
    pygame.init()
    pygame.joystick.init()

    # 연결된 조이스틱 확인
    if pygame.joystick.get_count() == 0:
        print("조이스틱이 연결되지 않았습니다. 컨트롤러를 연결해 주세요.")
        return

    # 첫 번째 조이스틱 사용 (인덱스 0)
    controller = pygame.joystick.Joystick(0)
    controller.init()
    print(f"컨트롤러가 인식되었습니다: {controller.get_name()}")

    # 조향 및 속도 값을 저장할 변수
    current_steering = 0
    current_speed = 0

    try:
        # 숨겨진 코드 프로세스 시작
        while True:
            # pygame 이벤트 큐를 처리하여 입력 업데이트
            for event in pygame.event.get():
                if event.type == QUIT:
                    raise KeyboardInterrupt
                
                # D-패드(방향키) 입력 감지
                if event.type == pygame.JOYHATMOTION:
                    x_axis, y_axis = controller.get_hat(0)

                    # 조향 값 설정 (좌우)
                    if x_axis == 1:
                        current_steering = min(current_steering + STEP_STEERING, MAX_STEERING)
                    elif x_axis == -1:
                        current_steering = max(current_steering - STEP_STEERING, -MAX_STEERING)
                    
                    # 속도 값 설정 (위/아래)
                    if y_axis == 1:
                        current_speed = min(current_speed + STEP_SPEED, MAX_SPEED)
                    elif y_axis == -1:
                        current_speed = max(current_speed - STEP_SPEED, -MAX_SPEED)

            # 시리얼 송신
            message = f"s{current_steering}l{current_speed}r{current_speed}\n"
            ser.write(message.encode())

            # 디버깅용 출력
            print(f"Sent: {message.strip()}")

            time.sleep(0.05) # CPU 과부하 방지


    except KeyboardInterrupt:
        steering = 0
        left_speed = 0
        right_speed = 0
        message = f"s{steering}l{left_speed}r{right_speed}\n"
        ser.write(message.encode())
        print("Program interrupted.")


    finally:
        ser.close()
        data_collector.cleanup()
        print("Serial connection closed.")
        pygame.quit() # pygame 종료

if __name__ == "__main__":
    main()