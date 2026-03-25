import cv2
import numpy as np

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    masked = cv2.bitwise_and(img, mask)
    return masked

def perspective_transform(frame):
    height, width = frame.shape[:2]

    # Define source points (trapezoid around lanes)
    src = np.float32([
        [width*0.45, height*0.65],
        [width*0.55, height*0.65],
        [width*0.9, height*0.95],
        [width*0.1, height*0.95]
    ])
    
    # Define destination points (rectangle)
    dst = np.float32([
        [width*0.2, 0],
        [width*0.8, 0],
        [width*0.8, height],
        [width*0.2, height]
    ])
    
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    
    warped = cv2.warpPerspective(frame, M, (width, height), flags=cv2.INTER_LINEAR)
    return warped, M, Minv

def detect_edges(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blur, 50, 150)
    return edges

def sliding_window_polyfit(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255

    midpoint = np.int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 9
    window_height = np.int(binary_warped.shape[0]//nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 100
    minpix = 50

    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Fit 2nd degree polynomial
    left_fit = np.polyfit(lefty, leftx, 2) if len(leftx) > 0 else None
    right_fit = np.polyfit(righty, rightx, 2) if len(rightx) > 0 else None

    return left_fit, right_fit, out_img

def draw_lane(original_img, binary_warped, left_fit, right_fit, Minv):
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    color_warp = np.zeros_like(original_img).astype(np.uint8)

    if left_fit is not None and right_fit is not None:
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        cv2.fillPoly(color_warp, np.int_([pts]), (0,255,0))

    newwarp = cv2.warpPerspective(color_warp, Minv, (original_img.shape[1], original_img.shape[0])) 
    result = cv2.addWeighted(original_img, 1, newwarp, 0.3, 0)
    return result

def lane_center_offset(left_fit, right_fit, image_width, y_eval=720):
    if left_fit is None or right_fit is None:
        return 0
    left_x = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
    right_x = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
    lane_center = (left_x + right_x) / 2.0
    car_center = image_width / 2
    offset_pixels = car_center - lane_center
    xm_per_pix = 3.7 / 700  # meters per pixel in x
    offset_meters = offset_pixels * xm_per_pix
    return offset_meters

# -------------------- Main loop --------------------
cap = cv2.VideoCapture("test_videos/lane.mp4")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    edges = detect_edges(frame)
    
    # Apply ROI mask
    height, width = frame.shape[:2]
    vertices = np.array([[(0, height), (width*0.45, height*0.65), 
                          (width*0.55, height*0.65), (width, height)]], dtype=np.int32)
    masked_edges = region_of_interest(edges, vertices)

    # Perspective transform
    warped, M, Minv = perspective_transform(frame)
    warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    _, binary_warped = cv2.threshold(warped_gray, 160, 255, cv2.THRESH_BINARY)

    # Detect lane lines
    left_fit, right_fit, _ = sliding_window_polyfit(binary_warped)

    # Draw lane on original frame
    lane_img = draw_lane(frame, binary_warped, left_fit, right_fit, Minv)

    # Calculate lane center offset
    offset = lane_center_offset(left_fit, right_fit, frame.shape[1])
    cv2.putText(lane_img, f"Offset from center: {offset:.2f} m", (50,50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

    cv2.imshow('Robust Lane Detection', lane_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
