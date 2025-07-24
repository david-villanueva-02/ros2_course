import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")

        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "David")

        self.add_on_set_parameters_callback(self.param_change_callback)

    def param_change_callback(self, params):
        result = SetParametersResult()

        for param in params: 
            if param.name == "simple_int_param":
                if isinstance(param.value, int):
                    self.get_logger().info(f"Param simple_int_param changed, new value is {param.value}")
                    result.successful = True
                else: self.get_logger().info("Not int")
            
            if param.name == "simple_string_param":
                if isinstance(param.value, str):
                    self.get_logger().info(f"Param simple_string_param changed, new value is {param.value}")
                    result.successful = True
                else: self.get_logger().info("Not string")

        return result

def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown() 

if __name__ == "__main__":
    main()