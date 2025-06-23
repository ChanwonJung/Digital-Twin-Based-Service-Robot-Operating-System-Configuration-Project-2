import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

from visualization_msgs.msg import Marker, MarkerArray
from turtlebot_cosmo_interface.srv import MoveitControl
from srv_call_test import TurtlebotArmClient


'''ArUco 마커 위치 정보를 수신하여 로봇 상태를 제어하는 노드'''
class ArucoInfoListener(Node):
    def __init__(self):
        super().__init__('aruco_info_listener')

        # Marker 메시지를 구독 (마커 위치 수신)
        self.subscription = self.create_subscription(Marker,'detected_markers',self.aruco_callback,10)

        # 로봇 상태를 퍼블리시
        self.state_publisher = self.create_publisher(String, '/robot_state', 10)

        # 마커의 x, y 좌표값 저장 변수
        self.position_x = 0
        self.position_y = 0

        # robot_state 플래그
        self.pick_mode = False
        self.slow_mode = False


    '''ArUco 마커 정보를 수신했을 때 실행되는 콜백 함수'''
    def aruco_callback(self, msg):
        try:
            # 수신한 마커의 위치 정보를 저장
            self.position_x = msg.pose.position.x
            self.position_y = msg.pose.position.y

            # 마커가 특정 위치(x ≤ 0.60) 안에 들어오면 slow 모드 진입
            if self.position_x <= 0.60:
                msg_str = f'좌표확인: x={self.position_x:.4f}, y={self.position_y:.4f}'

                # 아직 slow 모드가 아니라면 'slow' 상태를 퍼블리시하고 slow_mode 활성화
                if not self.slow_mode:
                    self.get_logger().info(msg_str)
                    self.slow_mode = True
                    msg = String()
                    msg.data = 'slow'
                    self.state_publisher.publish(msg)

                # 마커가 더 가까이 들어오면(x ≤ 0.33) 'stop' 상태로 전환하고 집기 실행
                if self.position_x <= 0.31:
                    msg_str = f'좌표확인: x={self.position_x:.4f}, y={self.position_y:.4f}'

                    # 아직 pick 동작을 수행하지 않았다면 'stop' 상태 퍼블리시 후 pick 수행
                    if not self.pick_mode:
                        self.get_logger().info(msg_str)
                        self.pick_mode = True
                        msg = String()
                        msg.data = 'stop'
                        self.state_publisher.publish(msg)
                        self.pick_marker()

        except Exception as e:
            # 예외 발생 시 경고 출력
            self.get_logger().warn(f'파싱 오류: {e}')
            print(f'파싱 오류: {e}')


    '''아루코 마커를 인식한 후 집기 및 배치 동작을 수행하는 함수'''
    def pick_marker(self):
        # 로봇팔 제어를 위한 클라이언트 생성
        arm_client = TurtlebotArmClient()

        print ("task start!")

        # 그리퍼 열기
        response = arm_client.send_request(2, "open")
        arm_client.get_logger().info(f'Response: {response.response}')
        
        # 픽업 위치로 이동
        response = arm_client.send_request(1, "block_pick")
        arm_client.get_logger().info(f'Response: {response.response}')

        # 그리퍼 닫기 (블록 집기)
        response = arm_client.send_request(2, "close")
        arm_client.get_logger().info(f'Response: {response.response}')

        # 블록 첫 번째 위치에 배치
        response = arm_client.send_request(1, "lane_tracking_01")
        arm_client.get_logger().info(f'Response: {response.response}')

        response = arm_client.send_request(1, "block_place_1")
        arm_client.get_logger().info(f'Response: {response.response}')

        # 그리퍼 열기 (블록 놓기)
        response = arm_client.send_request(2, "open")
        arm_client.get_logger().info(f'Response: {response.response}')

        # 다시 라인 트래킹 위치로 이동
        response = arm_client.send_request(1, "block_place_2")
        arm_client.get_logger().info(f'Response: {response.response}')

        response = arm_client.send_request(1, "lane_tracking_01")
        arm_client.get_logger().info(f'Response: {response.response}')\

        time.sleep(1)

        # 주행 재개 명령 퍼블리시
        msg = String()
        msg.data = 'drive'
        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoInfoListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()