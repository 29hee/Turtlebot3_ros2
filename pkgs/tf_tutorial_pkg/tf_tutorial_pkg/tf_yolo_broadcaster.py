import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import ObjectDetectionArray
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TfYoloBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_yolo_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(
            ObjectDetectionArray,
            '/image_yolo',
            self.callback,
            10
        )

    def callback(self, msg):
        now = self.get_clock().now().to_msg()
        for i, detection in enumerate(msg.detections):
            if detection.class_name != 'person':
                continue

            # bbox 중심을 3D 공간에 배치
            # 640x480 이미지에서 중심 좌표 계산 320x 240이면 이미지 중앙이므로, 중심에서의 상대 좌표를 -2 ~ +2 범위로 매핑    
            cx, cy, w, h = detection.bbox
            x = (cx - 320) / 320.0 * 2.0  # -2 ~ +2
            y = (cy - 240) / 240.0 * 2.0  # -2 ~ +2
            z = 1.0  # 카메라 앞 2m (고정 깊이 추정)

            # TF 메시지 생성
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'camera_link'
            t.child_frame_id = f'object_{detection.class_name}_{0}'
            
            # TF 좌표계는 오른손 법칙을 따르므로, 카메라 앞이 z축, 오른쪽이 x축, 아래가 y축
            # transform.translation.x 는 카메라 앞이므로 z값을 할당
            t.transform.translation.x = z
            t.transform.translation.y = -x
            t.transform.translation.z = -y      #
            t.transform.rotation.w = 0.10  # 회전 없음
            self.broadcaster.sendTransform(t)
            self.get_logger().info(
                f'TF for {t.child_frame_id} at ({z:.2f}, {-x:.2f}, {-y:.2f})'
            )
            break  # 첫 번째 사람 객체만 처리

def main(args=None):
    rclpy.init(args=args)
    node = TfYoloBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()