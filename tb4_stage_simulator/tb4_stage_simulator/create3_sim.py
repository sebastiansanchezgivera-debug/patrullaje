import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import BatteryState, LaserScan
from irobot_create_msgs.msg import DockStatus, IrOpcode


def yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class StageSimulator(Node):
    def __init__(self):
        super().__init__('tb4_stage_simulator')
        defaults = {
            'dock_x': 0.4259, 'dock_y': 0.3630, 'dock_yaw': math.pi,
            'dock_width': 0.30, 'dock_depth': 0.15,
            'ir_detection_range': 1.50, 'battery_initial_pct': 100.0,
            'battery_discharge_pct_per_min': 2.0, 'battery_override_pct': -1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('Solo Stage: use_sim_time debe ser true')
        self.battery = max(0.0, min(100.0, self.p('battery_initial_pct')))
        self.last_time = None
        self.scan = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.dock_pub = self.create_publisher(DockStatus, '/dock_status', 10)
        self.ir_pub = self.create_publisher(IrOpcode, '/ir_opcode', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info('Stage: bateria, DockStatus e IR; sin control de motores')

    def p(self, name):
        return self.get_parameter(name).value

    def on_scan(self, msg):
        self.scan = msg

    def detection(self, now):
        # Modelo geometrico aproximado, pose estimada por AMCL.
        if self.scan is None:
            return False, False
        stamp = Time.from_msg(self.scan.header.stamp, clock_type=now.clock_type)
        age = (now-stamp).nanoseconds*1e-9
        if age < -0.2 or age > 0.7:
            return False, False
        try:
            base = self.tf.lookup_transform('map', 'base_link', stamp).transform
            laser = self.tf.lookup_transform('map', self.scan.header.frame_id, stamp).transform
        except TransformException:
            return False, False
        heading = self.p('dock_yaw')
        fx = self.p('dock_x') + 0.5*self.p('dock_depth')*math.cos(heading)
        fy = self.p('dock_y') + 0.5*self.p('dock_depth')*math.sin(heading)
        vx, vy = fx-base.translation.x, fy-base.translation.y
        distance = math.hypot(vx, vy)
        if not 0.01 < distance <= self.p('ir_detection_range'):
            return False, False
        if abs(wrap(math.atan2(-vy, -vx)-heading)) > math.radians(65):
            return False, False
        # Oclusion aproximada con el rayo LiDAR mas cercano a la cara del dock.
        lx, ly = fx-laser.translation.x, fy-laser.translation.y
        target_range = math.hypot(lx, ly)
        angle = wrap(math.atan2(ly, lx)-yaw(laser.rotation))
        inc = self.scan.angle_increment
        if not math.isfinite(inc) or inc <= 0 or target_range > self.scan.range_max:
            return False, False
        idx = round((angle-self.scan.angle_min)/inc)
        if not 0 <= idx < len(self.scan.ranges):
            return False, False
        hit = self.scan.ranges[idx]
        if math.isnan(hit) or hit < self.scan.range_min:
            return False, False
        if math.isfinite(hit) and hit < target_range-0.12:
            return False, False
        front = abs(wrap(math.atan2(vy, vx)-yaw(base.rotation))) <= math.radians(30)
        return True, front

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        dt = 0.0 if self.last_time is None else max(
            0.0, (now-self.last_time).nanoseconds*1e-9)
        self.last_time = now
        override = self.p('battery_override_pct')
        if math.isfinite(override) and 0.0 <= override <= 100.0:
            self.battery = override
        else:
            self.battery = max(0.0, self.battery -
                max(0.0, self.p('battery_discharge_pct_per_min'))*dt/60.0)
        battery = BatteryState()
        battery.header.stamp = now.to_msg()
        battery.percentage = self.battery/100.0
        battery.voltage = float('nan')
        battery.current = float('nan')
        battery.temperature = float('nan')
        battery.charge = float('nan')
        battery.capacity = float('nan')
        battery.design_capacity = float('nan')
        battery.present = True
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(battery)
        visible, front = self.detection(now)
        status = DockStatus()
        status.header.stamp = now.to_msg()
        status.dock_visible = visible
        status.is_docked = False
        self.dock_pub.publish(status)
        if visible:
            self.publish_ir(now, IrOpcode.SENSOR_OMNI)
        if front:
            self.publish_ir(now, IrOpcode.SENSOR_DIRECTIONAL_FRONT)

    def publish_ir(self, now, sensor):
        msg = IrOpcode()
        msg.header.stamp = now.to_msg()
        msg.sensor = sensor
        msg.opcode = IrOpcode.CODE_IR_BUOY_BOTH
        self.ir_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StageSimulator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
