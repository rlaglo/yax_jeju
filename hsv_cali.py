import cv2
import numpy as np
import sys

# ======================================================================
# 1. 튜닝하고 싶은 이미지 파일의 전체 경로를 여기에 입력하세요.
# 예: IMAGE_PATH = '/home/hyeonseo/hlfma2025/src/test_images/blue_cone_daylight.jpg'
IMAGE_PATH = '/home/hyeonseo/hlfma2025/1232.png'
# ======================================================================

def nothing(x):
    """트랙바를 위한 빈 콜백 함수"""
    pass

def create_hsv_tuner():
    """HSV 튜닝을 위한 GUI 윈도우와 트랙바를 생성합니다."""
    cv2.namedWindow('HSV Tuner')
    cv2.createTrackbar('Hue Min', 'HSV Tuner', 0, 179, nothing)
    cv2.createTrackbar('Hue Max', 'HSV Tuner', 179, 179, nothing)
    cv2.createTrackbar('Sat Min', 'HSV Tuner', 0, 255, nothing)
    cv2.createTrackbar('Sat Max', 'HSV Tuner', 255, 255, nothing)
    cv2.createTrackbar('Val Min', 'HSV Tuner', 0, 255, nothing)
    cv2.createTrackbar('Val Max', 'HSV Tuner', 255, 255, nothing)

def main():
    # 1. 이미지 파일 로드
    image = cv2.imread(IMAGE_PATH)
    if image is None:
        print(f"오류: 이미지 파일을 찾을 수 없거나 열 수 없습니다.")
        print(f"경로를 확인해주세요: {IMAGE_PATH}")
        sys.exit()

    # 이미지 크기를 실제 차량 카메라와 유사하게 조절 (선택 사항)
    image = cv2.resize(image, (640, 480))
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 2. 튜닝 창 생성
    create_hsv_tuner()

    print("\nHSV 튜닝을 시작합니다...")
    print("트랙바를 조절하여 원하는 색상만 'Mask' 창에 흰색으로 표시되도록 하세요.")
    print("완료되면 키보드에서 'q'를 눌러 종료합니다.")

    while True:
        # 3. 트랙바에서 현재 값 가져오기
        h_min = cv2.getTrackbarPos('Hue Min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('Hue Max', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('Sat Min', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('Sat Max', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('Val Min', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('Val Max', 'HSV Tuner')

        # 4. HSV 값으로 마스크 생성
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        # 5. 원본 이미지에 마스크 적용 (결과 확인용)
        result = cv2.bitwise_and(image, image, mask=mask)

        # 6. 창 보여주기
        cv2.imshow('Original Image', image)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)
        
        # 'q' 키를 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 7. 최종 값 출력
    print("\n=======================================================")
    print("튜닝이 완료되었습니다! 아래 값을 복사하여 사용하세요.")
    print(f"lower_bound = np.array([{h_min}, {s_min}, {v_min}])")
    print(f"upper_bound = np.array([{h_max}, {s_max}, {v_max}])")
    print("=======================================================")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()