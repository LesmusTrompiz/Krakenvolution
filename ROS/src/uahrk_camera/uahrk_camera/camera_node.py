import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from random import randint

class KrakenCamera():
    def __init__(self, id = 1):
        self.id = id
       
    def get_aruco(self):
        ar17 = randint(0,1)
        ar35 = randint(0,1)
        ar42 = randint(0,1)
        return (ar17,ar35, ar42)


class CameraNode(Node):
    def __init__(self, id = 1, photo_time = 0.1):
        super().__init__('camera_'+ str(id) +'_node')

        # Create a Kraken Camera Device to manage the camera Device
        self.camera = KrakenCamera(id)

        # Publishers that will show if the camera see an aruco track
        self.pub_ar17 = self.create_publisher(UInt8, 'camera' + str(id) + "_17", 10)
        self.pub_ar35 = self.create_publisher(UInt8, 'camera' + str(id) + "_35", 10)
        self.pub_ar42 = self.create_publisher(UInt8, 'camera' + str(id) + "_42", 10)

        # Create a timer to avoid system saturation
        self.timer = self.create_timer(photo_time, self.get_aruco)

    def get_aruco(self):
        # Takes a photo and retrieves Aruco 
        # information from the photo
        ar17, ar35, ar42 = self.camera.get_aruco()
        id_17      = UInt8()
        id_35      = UInt8()
        id_42      = UInt8()

        id_17.data = ar17
        id_35.data = ar35
        id_42.data = ar42

        self.pub_ar17.publish(id_17)
        self.pub_ar35.publish(id_35)
        self.pub_ar42.publish(id_42)

        return

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode(1)
    rclpy.spin(camera_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()