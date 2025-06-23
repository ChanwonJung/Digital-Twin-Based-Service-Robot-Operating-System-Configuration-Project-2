import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# ----------------- ROS 2 퍼블리셔 클래스 -----------------
class LanePublisher(Node):
    def __init__(self):
        super().__init__('lane_visualizer')
        self.publisher_ = self.create_publisher(Float64, '/detect/lane', 10)

    def publish_center(self, x_value):
        msg = Float64()
        msg.data = float(x_value)
        self.publisher_.publish(msg)
        self.get_logger().info(f"/detect/lane 발행됨: {msg.data:.2f}")

# ----------------- 트랙바 설정 함수 -----------------
def nothing(x): pass

def create_trackbars():
    cv2.namedWindow('Control Panel', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Control Panel', 400, 400)
    cv2.createTrackbar('White H min', 'Control Panel', 0, 180, nothing)
    cv2.createTrackbar('White S min', 'Control Panel', 0, 255, nothing)
    cv2.createTrackbar('White V min', 'Control Panel', 239, 255, nothing)
    cv2.createTrackbar('White H max', 'Control Panel', 180, 180, nothing)
    cv2.createTrackbar('White S max', 'Control Panel', 50, 255, nothing)
    cv2.createTrackbar('White V max', 'Control Panel', 255, 255, nothing)
    cv2.createTrackbar('Yellow H min', 'Control Panel', 11, 180, nothing)
    cv2.createTrackbar('Yellow S min', 'Control Panel', 21, 255, nothing)
    cv2.createTrackbar('Yellow V min', 'Control Panel', 100, 255, nothing)
    cv2.createTrackbar('Yellow H max', 'Control Panel', 52, 180, nothing)
    cv2.createTrackbar('Yellow S max', 'Control Panel', 255, 255, nothing)
    cv2.createTrackbar('Yellow V max', 'Control Panel', 255, 255, nothing)
    cv2.createTrackbar('top_x', 'Control Panel', 167, 320, nothing)
    cv2.createTrackbar('top_y', 'Control Panel', 20, 240, nothing)
    cv2.createTrackbar('bottom_x', 'Control Panel', 233, 320, nothing)
    cv2.createTrackbar('bottom_y', 'Control Panel', 159, 240, nothing)

# ----------------- 조감도 -----------------
def bird_eye_view(image, top_x, top_y, bottom_x, bottom_y):
    center_x = image.shape[1] // 2
    top_y_coord = 160 - top_y
    bottom_y_coord = 320 + bottom_y

    pts_src = np.array([
        [center_x - top_x, top_y_coord],
        [center_x + top_x, top_y_coord],
        [center_x + bottom_x, bottom_y_coord],
        [center_x - bottom_x, bottom_y_coord]
    ])

    image_with_roi = image.copy()
    cv2.polylines(image_with_roi, [pts_src.astype(np.int32)], isClosed=True, color=(0, 0, 255), thickness=2)

    pts_dst = np.array([
        [200, 0], [800, 0], [800, 600], [200, 600]
    ])

    h, _ = cv2.findHomography(pts_src, pts_dst)
    warped = cv2.warpPerspective(image, h, (1000, 600))

    return warped, image_with_roi

# ----------------- 곡선 피팅 및 시각화 -----------------
def fit_and_draw_lanes(image, white_mask, yellow_mask, lane_node):
    out = image.copy()
    height = image.shape[0]
    ploty = np.linspace(0, height - 1, height)

    def get_fit(mask):
        nonzero = mask.nonzero()
        if len(nonzero[0]) < 2000:
            return None, None
        x = nonzero[1]
        y = nonzero[0]
        fit = np.polyfit(y, x, 2)
        fitx = fit[0] * ploty**2 + fit[1] * ploty + fit[2]
        # 수정된 곡선 (a,b 수정)
        a_mod = fit[0]
        if a_mod > 0:
            a_mod = max(0, a_mod - 0.0001)
        else:
            a_mod = min(0, a_mod + 0.0001)
        #fitx = a_mod * ploty**2 + fit[1] * ploty + fit[2]

        b_mod = fit[1] 
        if b_mod > 0:
            b_mod =  b_mod + 0.0001
        else:
            b_mod =  b_mod - 0.0001
        fitx = a_mod * ploty**2 + b_mod * ploty + fit[2]
        return fitx, fit

    left_fitx, _ = get_fit(yellow_mask)
    right_fitx, _ = get_fit(white_mask)

    centerx = None
    if left_fitx is not None:
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))], np.int32)
        cv2.polylines(out, pts_left, isClosed=False, color=(0, 0, 255), thickness=5)
 
    if right_fitx is not None:
        pts_right = np.array([np.transpose(np.vstack([right_fitx, ploty]))], np.int32)
        cv2.polylines(out, pts_right, isClosed=False, color=(255, 255, 0), thickness=5) 

    if left_fitx is not None and right_fitx is not None:
        centerx = (left_fitx + right_fitx) / 2
        pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))], np.int32)
        cv2.polylines(out, pts_center, isClosed=False, color=(0, 255, 255), thickness=3)

        if rclpy.ok():
            lane_node.publish_center(centerx[500])

    return out

# -------------------- 가장 큰 덩어리 ------------------
def keep_largest_component(mask):
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return mask  # 아무것도 없으면 그대로
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])  # 배경 제외하고 최대
    largest_mask = np.uint8(labels == largest_label) * 255
    return largest_mask


# ----------------- 마스킹 + 시각화 -----------------
def process_and_visualize(image, hsv, wh_range, yh_range, lane_node):
    white_mask_raw = cv2.inRange(hsv, wh_range[0], wh_range[1])
    yellow_mask_raw = cv2.inRange(hsv, yh_range[0], yh_range[1])

    masked = np.zeros_like(image)
    masked[(white_mask_raw > 0)] = [255, 255, 255]
    masked[(yellow_mask_raw > 0)] = [0, 255, 255]
    masked[(white_mask_raw > 0) & (yellow_mask_raw > 0)] = [0, 0, 255]
    cv2.rectangle(masked, (0, 0), (masked.shape[1] - 1, masked.shape[0] - 1), (255, 255, 0), 2)

    white_mask = cv2.bitwise_and(white_mask_raw, cv2.bitwise_not(yellow_mask_raw))
    yellow_mask = yellow_mask_raw.copy()

    # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    # white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    # yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # --- 경로용: 겹치는 영역 제거한 후 가장 큰 덩어리만 남김 ---
    # 여기서 가장 큰 덩어리만 남기도록 변경
    white_mask = keep_largest_component(white_mask)
    yellow_mask = keep_largest_component(yellow_mask)

    path_masked = np.zeros_like(image)
    path_masked[(white_mask > 0)] = [255, 255, 255]
    path_masked[(yellow_mask > 0)] = [0, 255, 255]

    result = fit_and_draw_lanes(path_masked, white_mask, yellow_mask, lane_node)

    return cv2.hconcat([image, masked, result])

# ----------------- 메인 루프 -----------------
def main():
    rclpy.init()
    lane_node = LanePublisher()
    create_trackbars()

    cv2.namedWindow('Masked Grid Output', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Masked Grid Output', 1200, 600)

    cv2.namedWindow('Original with ROI', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Original with ROI', 640, 480)
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 웹캠을 열 수 없습니다.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 프레임 캡처 실패")
            break

        frame = cv2.resize(frame, (640, 480))

        top_x = cv2.getTrackbarPos('top_x', 'Control Panel')
        top_y = cv2.getTrackbarPos('top_y', 'Control Panel')
        bottom_x = cv2.getTrackbarPos('bottom_x', 'Control Panel')
        bottom_y = cv2.getTrackbarPos('bottom_y', 'Control Panel')

        bird_view, original_with_roi = bird_eye_view(frame, top_x, top_y, bottom_x, bottom_y)
        #blur_bird_view = cv2.GaussianBlur(bird_view, (3,3), 0)
        hsv = cv2.cvtColor(bird_view, cv2.COLOR_BGR2HSV)

        # === 조명 강한 환경 대비 강화 ===
        h, s, v = cv2.split(hsv)
        v = cv2.equalizeHist(v)  # 밝기 균일화
        v = np.clip(v*0.95, 0, 255).astype(np.uint8)
        s = cv2.equalizeHist(s)  # saturation 균일화
        hsv = cv2.merge([h, s, v])

        wh_min = (cv2.getTrackbarPos('White H min', 'Control Panel'),
                  cv2.getTrackbarPos('White S min', 'Control Panel'),
                  cv2.getTrackbarPos('White V min', 'Control Panel'))
        wh_max = (cv2.getTrackbarPos('White H max', 'Control Panel'),
                  cv2.getTrackbarPos('White S max', 'Control Panel'),
                  cv2.getTrackbarPos('White V max', 'Control Panel'))
        yh_min = (cv2.getTrackbarPos('Yellow H min', 'Control Panel'),
                  cv2.getTrackbarPos('Yellow S min', 'Control Panel'),
                  cv2.getTrackbarPos('Yellow V min', 'Control Panel'))
        yh_max = (cv2.getTrackbarPos('Yellow H max', 'Control Panel'),
                  cv2.getTrackbarPos('Yellow S max', 'Control Panel'),
                  cv2.getTrackbarPos('Yellow V max', 'Control Panel'))

        out = process_and_visualize(bird_view, hsv, (wh_min, wh_max), (yh_min, yh_max), lane_node)

        cv2.imshow('Masked Grid Output', out)
        equalized_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        cv2.imshow('Original with ROI', original_with_roi)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    lane_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
