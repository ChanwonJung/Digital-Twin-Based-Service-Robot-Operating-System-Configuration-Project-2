import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage  # 추가

class LanePublisher(Node):
    def __init__(self):
        super().__init__('lane_visualizer')
        self.publisher_ = self.create_publisher(Float64, '/detect/lane', 10)
        self.image_pub_ = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)

    def publish_center(self, x_value):
        msg = Float64()
        msg.data = float(x_value)
        self.publisher_.publish(msg)
        self.get_logger().info(f"/detect/lane 발행됨: {msg.data:.2f}")

    def publish_image(self, frame):  # 추가
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
        success, compressed_image = cv2.imencode('.jpg', frame, encode_param)
        if success:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.format = "jpeg"
            msg.data = compressed_image.tobytes()
            self.image_pub_.publish(msg)
            self.get_logger().info("Compressed image published")

# 트랙바 콜백
def nothing(x): pass

def create_trackbars():
    cv2.namedWindow('Control Panel', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Control Panel', 400, 400)
    # White thresholds
    cv2.createTrackbar('White H min', 'Control Panel', 0, 180, nothing)
    cv2.createTrackbar('White S min', 'Control Panel', 0, 255, nothing)
    cv2.createTrackbar('White V min', 'Control Panel', 239, 255, nothing)
    cv2.createTrackbar('White H max', 'Control Panel', 180, 180, nothing)
    cv2.createTrackbar('White S max', 'Control Panel', 50, 255, nothing)
    cv2.createTrackbar('White V max', 'Control Panel', 255, 255, nothing)
    # Yellow thresholds
    cv2.createTrackbar('Yellow H min', 'Control Panel', 11, 180, nothing)
    cv2.createTrackbar('Yellow S min', 'Control Panel', 21, 255, nothing)
    cv2.createTrackbar('Yellow V min', 'Control Panel', 100, 255, nothing)
    cv2.createTrackbar('Yellow H max', 'Control Panel', 60, 180, nothing)
    cv2.createTrackbar('Yellow S max', 'Control Panel', 255, 255, nothing)
    cv2.createTrackbar('Yellow V max', 'Control Panel', 255, 255, nothing)
    # ROI trackbars
    cv2.createTrackbar('top_x', 'Control Panel', 140, 320, nothing) #167
    cv2.createTrackbar('top_y', 'Control Panel', 20, 240, nothing)
    cv2.createTrackbar('bottom_x', 'Control Panel', 203, 320, nothing) #233
    cv2.createTrackbar('bottom_y', 'Control Panel', 159, 240, nothing)

# Bird's Eye View 변환
def bird_eye_view(image, top_x, top_y, bottom_x, bottom_y):
    center_x = image.shape[1] // 2
    top_y_coord = 160 - top_y
    bottom_y_coord = 320 + bottom_y

    src = np.array([
        [center_x - top_x, top_y_coord],
        [center_x + top_x, top_y_coord],
        [center_x + bottom_x, bottom_y_coord],
        [center_x - bottom_x, bottom_y_coord]
    ], dtype=np.float32)
    dst = np.array([[200,0],[800,0],[800,600],[200,600]], dtype=np.float32)

    H, _ = cv2.findHomography(src, dst)
    warped = cv2.warpPerspective(image, H, (1000,600))
    return warped

# 가장 큰 컴포넌트 유지
def keep_largest_component(mask):
    
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return mask
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    return (labels == largest).astype(np.uint8) * 255

# 차선 피팅 및 퍼블리시
# def fit_and_draw_lanes(image, white_mask, yellow_mask, lane_node):
#     out = image.copy()
#     h = image.shape[0]
#     ploty = np.linspace(0, h-1, h)

#     def fit_curve(mask):
#         ys, xs = mask.nonzero()
#         if len(xs) < 2000:
#             return None
#         a, b, c = np.polyfit(ys, xs, 2)
#         a -= np.sign(a)*0.0001
#         b += np.sign(b)*0.0001
#         return a*ploty**2 + b*ploty + c

#     lx = fit_curve(yellow_mask)
#     rx = fit_curve(white_mask)
#     if lx is not None:
#         pts = np.vstack([lx, ploty]).T.astype(np.int32)[None,:,:]
#         cv2.polylines(out, pts, False, (0,0,255), 5)
#     if rx is not None:
#         pts = np.vstack([rx, ploty]).T.astype(np.int32)[None,:,:]
#         cv2.polylines(out, pts, False, (255,255,0), 5)
#     if lx is not None and rx is not None:
#         cx = (lx + rx)/2
#         pts = np.vstack([cx, ploty]).T.astype(np.int32)[None,:,:]
#         cv2.polylines(out, pts, False, (0,255,255), 3)
#         lane_node.publish_center(cx[int(h/2)])
#     return out


def fit_and_draw_lanes(image, white_mask, yellow_mask, lane_node):
    out = image.copy()
    h = image.shape[0]
    ploty = np.linspace(0, h-1, h)
    def fit_curve(mask):
        ys, xs = mask.nonzero()
        if len(xs) < 20000:
            return None
        a, b, c = np.polyfit(ys, xs, 2)
        a -= np.sign(a)*0.0001
        b += np.sign(b)*0.0001
        return a*ploty**2 + b*ploty + c
    lx = fit_curve(yellow_mask)  # 왼쪽 (노란색) 차선
    rx = fit_curve(white_mask)   # 오른쪽 (하얀색) 차선
    center_line = None
    if lx is not None:
        pts = np.vstack([lx, ploty]).T.astype(np.int32)[None, :, :]
        cv2.polylines(out, pts, False, (0, 0, 255), 5)
    if rx is not None:
        pts = np.vstack([rx, ploty]).T.astype(np.int32)[None, :, :]
        cv2.polylines(out, pts, False, (255, 255, 0), 5)
    # 두 차선 모두 검출된 경우
    if lx is not None and rx is not None:
        cx = (lx + rx) / 2
        center_line = cx
        cv2.polylines(out, np.vstack([cx, ploty]).T.astype(np.int32)[None, :, :], False, (0, 255, 255), 3)
    # 하나만 검출된 경우
    elif lx is not None:
        cx = lx + 350  # 왼쪽에서 오른쪽으로 200px 이동
        center_line = cx
        cv2.polylines(out, np.vstack([cx, ploty]).T.astype(np.int32)[None, :, :], False, (0, 255, 255), 3)
    elif rx is not None:
        cx = rx - 350  # 오른쪽에서 왼쪽으로 200px 이동
        center_line = cx
        cv2.polylines(out, np.vstack([cx, ploty]).T.astype(np.int32)[None, :, :], False, (0, 255, 255), 3)
    # 중심 퍼블리시
    if center_line is not None:
        #lane_node.publish_center(center_line[int(h/2)])
        lane_node.publish_center(center_line[int(h*0.7)])
    return out


# 마스킹 및 픽셀 수 표시
def process_and_visualize(image, hsv, wh_range, yh_range, lane_node):
    # raw masks: 모든 마스킹 보존
    wm_raw = cv2.inRange(hsv, wh_range[0], wh_range[1])
    ym_raw = cv2.inRange(hsv, yh_range[0], yh_range[1])

    # largest-only masks for fitting
    wm = keep_largest_component(cv2.bitwise_and(wm_raw, cv2.bitwise_not(ym_raw)))
    ym = keep_largest_component(ym_raw)

    # 픽셀 수
    white_count = int(cv2.countNonZero(wm))
    yellow_count = int(cv2.countNonZero(ym))

    # 자동 V min 조정 (White & Yellow)
    # White V min
    w_vmin_name = 'White V min'
    # Yellow V min
    y_vmin_name = 'Yellow V min'
    V_MIN_LOWER, V_MIN_UPPER = 0, 255
    # 현재 트랙바 값
    current_w_vmin = cv2.getTrackbarPos(w_vmin_name, 'Control Panel')
    current_y_vmin = cv2.getTrackbarPos(y_vmin_name, 'Control Panel')
    # White Count 기준 조정
    if white_count >= 64000:
        new_w_vmin = min(current_w_vmin + 2, 235)
    elif white_count <= 45000:
        new_w_vmin = max(current_w_vmin - 2, V_MIN_LOWER)
    else:
        new_w_vmin = current_w_vmin
    cv2.setTrackbarPos(w_vmin_name, 'Control Panel', new_w_vmin)
    # Yellow Count 기준 조정
    if yellow_count >= 64000:
        new_y_vmin = min(current_y_vmin + 5, V_MIN_UPPER)
    elif yellow_count <= 45000:
        new_y_vmin = max(current_y_vmin - 5, V_MIN_LOWER)
    else:
        new_y_vmin = current_y_vmin
    cv2.setTrackbarPos(y_vmin_name, 'Control Panel', new_y_vmin)

    # fitting & result
    disp = np.zeros_like(image)
    disp[wm>0] = [255,255,255]
    disp[ym>0] = [0,255,255]
    result = fit_and_draw_lanes(disp, wm, ym, lane_node)

    # overlay counts
    rw = result.shape[1]
    cv2.putText(result, f"White: {white_count}", (rw-200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
    cv2.putText(result, f"Yellow: {yellow_count}", (rw-200, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    # masked 창: raw mask 시각화
    masked = np.zeros_like(image)
    masked[wm_raw>0] = [255,255,255]
    masked[ym_raw>0] = [0,255,255]

    return cv2.hconcat([image, masked, result])

# 메인함수
def main():
    rclpy.init()
    node = LanePublisher()
    create_trackbars()
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 웹캠 열기 실패")
        return

    cv2.namedWindow('Equalized Birdview', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Output', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Output', 1200, 600)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.resize(frame, (640,480))

        # 퍼블리시 전 이미지 전송
        node.publish_image(frame)  # 추가
        
        tx = cv2.getTrackbarPos('top_x','Control Panel')
        ty = cv2.getTrackbarPos('top_y','Control Panel')
        bx = cv2.getTrackbarPos('bottom_x','Control Panel')
        by = cv2.getTrackbarPos('bottom_y','Control Panel')

        bird = bird_eye_view(frame, tx, ty, bx, by)
        hsv = cv2.cvtColor(bird, cv2.COLOR_BGR2HSV)

        # S, V 히스토그램 평탄화
        h, s, v = cv2.split(hsv)
        s_eq = cv2.equalizeHist(s)
        v_eq = cv2.equalizeHist(v)
        hsv_eq = cv2.merge([h, s_eq, v_eq])
        bird_eq = cv2.cvtColor(hsv_eq, cv2.COLOR_HSV2BGR)

        cv2.imshow('Equalized Birdview', bird_eq)

        # thresholds
        wh_min = (cv2.getTrackbarPos('White H min','Control Panel'), cv2.getTrackbarPos('White S min','Control Panel'), cv2.getTrackbarPos('White V min','Control Panel'))
        wh_max = (cv2.getTrackbarPos('White H max','Control Panel'), cv2.getTrackbarPos('White S max','Control Panel'), cv2.getTrackbarPos('White V max','Control Panel'))
        yh_min = (cv2.getTrackbarPos('Yellow H min','Control Panel'), cv2.getTrackbarPos('Yellow S min','Control Panel'), cv2.getTrackbarPos('Yellow V min','Control Panel'))
        yh_max = (cv2.getTrackbarPos('Yellow H max','Control Panel'), cv2.getTrackbarPos('Yellow S max','Control Panel'), cv2.getTrackbarPos('Yellow V max','Control Panel'))

        out = process_and_visualize(bird_eq, hsv_eq, (wh_min, wh_max), (yh_min, yh_max), node)
        cv2.imshow('Output', out)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
