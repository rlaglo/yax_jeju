# calibration_tuner.py
import cv2
import numpy as np

#경로 끝 png 파일 이름 변경
# --- 설정 ---
IMAGE_PATH = '0_steer_0_left_speed_40_right_speed_40.png' # 캘리브레이션에 사용할 이미지 파일 경로
# \\wsl.localhost\Ubuntu-22.04\home\jeanho\ros2_ws\src\camera_perception_pkg\camera_perception_pkg\lib\Collected_Datasets\data1\0_steer_0_left_speed_40_right_speed_40.png
IMAGE_WIDTH = 640  # 이미지 너비
IMAGE_HEIGHT = 480 # 이미지 높이

# 클릭한 좌표를 저장할 리스트
src_points = []

# 마우스 클릭 콜백 함수
def mouse_callback(event, x, y, flags, param):
    global src_points
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(src_points) < 4:
            src_points.append([x, y])
            print(f"클릭한 좌표: [{x}, {y}], {len(src_points)}/4 완료")
        if len(src_points) == 4:
            print("\n4개 좌표 선택 완료. src_points:", src_points)
            update_perspective()

# 버드아이뷰 업데이트 및 표시 함수
def update_perspective():
    global src_points
    if len(src_points) < 4:
        return

    # 정수형으로 변환
    src_np = np.array(src_points, dtype=np.float32)

    # 목적 좌표 (결과물이 될 직사각형)
    # dst_points_ratio 파라미터와 동일한 로직으로 계산
    dst_ratio = [0.3, 0.0, 0.7, 0.0, 0.7, 1.0, 0.3, 1.0]
    dst_np = np.array([
        [IMAGE_WIDTH * dst_ratio[0], IMAGE_HEIGHT * dst_ratio[1]],
        [IMAGE_WIDTH * dst_ratio[2], IMAGE_HEIGHT * dst_ratio[3]],
        [IMAGE_WIDTH * dst_ratio[4], IMAGE_HEIGHT * dst_ratio[5]],
        [IMAGE_WIDTH * dst_ratio[6], IMAGE_HEIGHT * dst_ratio[7]],
    ], dtype=np.float32)

    # 투시 변환 행렬 계산
    M = cv2.getPerspectiveTransform(src_np, dst_np)
    # 투시 변환 적용
    warped_img = cv2.warpPerspective(img, M, (IMAGE_WIDTH, IMAGE_HEIGHT))

    cv2.imshow('Bird-eye View Result', warped_img)
    print("\nLaunch 파일에 넣을 파라미터 값:")
    # launch 파일에 넣기 쉬운 형태로 출력
    flat_list = [int(p) for point in src_points for p in point]
    print(f"perspective.src_points:={flat_list}")


# 메인 로직
img = cv2.imread(IMAGE_PATH)
if img is None:
    print("이미지를 불러올 수 없습니다. IMAGE_PATH를 확인하세요.")
else:
    img = cv2.resize(img, (IMAGE_WIDTH, IMAGE_HEIGHT))
    cv2.imshow('Original Image - Click 4 points', img)
    cv2.setMouseCallback('Original Image - Click 4 points', mouse_callback)

    print("원본 이미지에서 차선이 만드는 사다리꼴의 꼭짓점 4개를 **왼쪽 위 -> 오른쪽 위 -> 오른쪽 아래 -> 왼쪽 아래** 순서로 클릭하세요.")

    while True:
        key = cv2.waitKey(1) & 0xFF
        # 'r' 키를 누르면 좌표 초기화
        if key == ord('r'):
            src_points = []
            print("\n좌표를 초기화합니다. 다시 클릭하세요.")
            cv2.destroyWindow('Bird-eye View Result')
        # 'q' 키를 누르면 종료
        elif key == ord('q'):
            break

    cv2.destroyAllWindows()