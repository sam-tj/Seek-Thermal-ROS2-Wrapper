import rclpy
from rclpy.node import Node

from seekcamera import (
    SeekCameraIOType,
    SeekCameraColorPalette,
    SeekCameraManager,
    SeekCameraManagerEvent,
    SeekCameraFrameFormat,
    SeekCamera,
    SeekFrame,
)

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ThermalImagePublisher(Node):

    def __init__(self):
        super().__init__('thermalImagePublisher')

        self.description = "colorPalette has the following options: \n\
                            WHITE_HOT \n\
                            BLACK_HOT \n\
                            SPECTRA \n\
                            PRISM \n\
                            TYRIAN -- Default \n\
                            IRON \n\
                            AMBER \n\
                            HI \n\
                            GREEN \n"

        self.get_logger().info(self.description)

        from rcl_interfaces.msg import ParameterDescriptor
        param_list = ParameterDescriptor(description=self.description)

        self.declare_parameter('colorPalette', 'TYRIAN', param_list)
        self.colorPalette_input = str(self.get_parameter('colorPalette').get_parameter_value().string_value).upper()

        self.publisher_ = self.create_publisher(Image, 'thermalImage', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.busy = False
        self.frame = SeekFrame()
        self.camera = SeekCamera()
        self.first_frame = True

        self.manager = SeekCameraManager(SeekCameraIOType.USB)
        self.br = CvBridge()

    def timer_callback(self):
        img = self.frame
        if img.is_empty == False:
            self.publisher_.publish(self.br.cv2_to_imgmsg(img.data))

    def on_frame(self, _camera, camera_frame, _):
        self.frame = camera_frame.color_argb8888

    def on_event(self, camera, event_type, event_status, _ ):

        self.get_logger().info(("{}: {}".format(str(event_type), camera.chipid)))

        if event_type == SeekCameraManagerEvent.CONNECT:
            if self.busy:
                return

            self.busy = True
            self.camera = camera

            self.first_frame = True

            camera.color_palette = SeekCameraColorPalette[self.colorPalette_input]

            camera.register_frame_available_callback(self.on_frame, self)
            camera.capture_session_start(SeekCameraFrameFormat.COLOR_ARGB8888)

        elif event_type == SeekCameraManagerEvent.DISCONNECT:
            if self.camera == camera:
                camera.capture_session_stop()
                self.camera = None
                self.frame = None
                self.busy = False

        elif event_type == SeekCameraManagerEvent.ERROR:
             self.get_logger().info("{}: {}".format(str(event_status), camera.chipid))

        elif event_type == SeekCameraManagerEvent.READY_TO_PAIR:
            return


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ThermalImagePublisher()
    minimal_publisher.manager.register_event_callback(minimal_publisher.on_event)

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()