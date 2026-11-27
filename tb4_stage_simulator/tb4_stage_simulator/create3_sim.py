import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.task import Future
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
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
            'battery_charge_pct_per_min': 10.0,
            'dock_motion_enabled': False,
            'dock_standoff_m': 0.33, 'docking_speed': 0.03,
            'undocking_speed': 0.05, 'undocking_distance': 0.30,
            'dock_action_timeout_sec': 45.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('Solo Stage: use_sim_time debe ser true')
        self.battery = max(0.0, min(100.0, self.p('battery_initial_pct')))
        self.last_time = None
        self.scan = None
        self.scan_wall_time = 0.0
        self.mode = None
        self.goal_handle = None
        self.completion = None
        self.docked = False
        self.dock_pose = None
        self.deadline = 0.0
        self.settle_count = 0
        self.last_control_clock = None
        self.last_clock_progress = time.monotonic()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.dock_pub = self.create_publisher(DockStatus, '/dock_status', 10)
        self.ir_pub = self.create_publisher(IrOpcode, '/ir_opcode', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.tick)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.dock_server = ActionServer(self, Dock, '/dock', self.execute_dock,
            goal_callback=lambda _: self.reserve('dock'), cancel_callback=self.cancel_goal)
        self.undock_server = ActionServer(self, Undock, '/undock', self.execute_undock,
            goal_callback=lambda _: self.reserve('undock'), cancel_callback=self.cancel_goal)
        # Reloj monotónico independiente: vigila incluso si /clock se detiene.
        self.motion_timer = self.create_timer(0.10, self.motion_tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Stage: telemetria y acciones Dock/Undock; movimiento opt-in')

    def p(self, name):
        return self.get_parameter(name).value

    def on_scan(self, msg):
        self.scan = msg
        self.scan_wall_time = time.monotonic()

    def detection(self, now):
        # Modelo geometrico aproximado, pose estimada por AMCL.
        if self.scan is None or time.monotonic()-self.scan_wall_time > 0.8:
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
        if not math.isfinite(hit) or abs(hit-target_range) > 0.12:
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
        elif self.docked and self.mode != 'undock':
            self.battery = min(100.0, self.battery +
                max(0.0, self.p('battery_charge_pct_per_min'))*dt/60.0)
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
        charging = self.docked and self.mode != 'undock'
        battery.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_FULL if charging and self.battery >= 100.0
            else BatteryState.POWER_SUPPLY_STATUS_CHARGING if charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.battery_pub.publish(battery)
        visible, front = self.detection(now)
        status = DockStatus()
        status.header.stamp = now.to_msg()
        status.dock_visible = visible
        status.is_docked = self.docked
        self.dock_pub.publish(status)
        if visible:
            self.publish_ir(now, IrOpcode.SENSOR_OMNI)
        if front:
            self.publish_ir(now, IrOpcode.SENSOR_DIRECTIONAL_FRONT)


    def pose(self, now):
        if self.scan is None or time.monotonic()-self.scan_wall_time > 0.8:
            return None
        stamp = Time.from_msg(self.scan.header.stamp, clock_type=now.clock_type)
        age = (now-stamp).nanoseconds*1e-9
        if not -0.2 <= age <= 0.7:
            return None
        try:
            t = self.tf.lookup_transform('map', 'base_link', stamp).transform
        except TransformException:
            return None
        pose = (t.translation.x, t.translation.y, yaw(t.rotation))
        return pose if all(math.isfinite(v) for v in pose) else None

    def geometry(self):
        h = self.p('dock_yaw')
        nx, ny = math.cos(h), math.sin(h)
        fx = self.p('dock_x') + self.p('dock_depth')*0.5*nx
        fy = self.p('dock_y') + self.p('dock_depth')*0.5*ny
        return fx, fy, nx, ny, wrap(h+math.pi)

    def monitor_present(self):
        pubs = self.get_publishers_info_by_topic('/cmd_vel')
        return len(pubs) == 1 and pubs[0].node_name == 'collision_monitor'

    def reserve(self, mode):
        if not self.p('dock_motion_enabled') or self.mode is not None:
            return GoalResponse.REJECT
        if (mode == 'dock' and self.docked) or (mode == 'undock' and not self.docked):
            return GoalResponse.REJECT
        if not self.monitor_present():
            self.get_logger().warn('Movimiento rechazado: /cmd_vel sin Collision Monitor exclusivo')
            return GoalResponse.REJECT
        now = self.get_clock().now()
        pos = self.pose(now)
        if pos is None or not self.detection(now)[0]:
            self.get_logger().warn('Movimiento rechazado: TF, scan o dock no valido')
            return GoalResponse.REJECT
        fx, fy, nx, ny, heading = self.geometry()
        longitudinal = (pos[0]-fx)*nx+(pos[1]-fy)*ny
        lateral = -(pos[0]-fx)*ny+(pos[1]-fy)*nx
        standoff = self.p('dock_standoff_m')
        retreat = self.p('undocking_distance')
        timeout = self.p('dock_action_timeout_sec')
        if not all(math.isfinite(v) for v in (longitudinal, lateral, standoff, retreat, timeout)):
            return GoalResponse.REJECT
        if not (0.32 <= standoff <= 0.45 and 0.20 <= retreat <= 0.50 and 5 <= timeout <= 90):
            return GoalResponse.REJECT
        if mode == 'dock' and not (standoff-0.025 <= longitudinal <= 0.95 and abs(lateral) <= 0.25):
            self.get_logger().warn('Dock rechazado: aproximacion fuera del corredor')
            return GoalResponse.REJECT
        if mode == 'undock' and not (abs(longitudinal-standoff) <= 0.08 and abs(lateral) <= 0.08 and abs(wrap(pos[2]-heading)) <= 0.20):
            self.get_logger().warn('Undock rechazado: pose acoplada no valida')
            return GoalResponse.REJECT
        self.mode = mode
        self.deadline = time.monotonic()+timeout
        self.settle_count = 0
        self.last_control_clock = now.nanoseconds
        self.last_clock_progress = time.monotonic()
        return GoalResponse.ACCEPT

    def cancel_goal(self, _):
        return CancelResponse.ACCEPT

    async def execute_dock(self, handle):
        return await self.execute_motion(handle)

    async def execute_undock(self, handle):
        return await self.execute_motion(handle)

    async def execute_motion(self, handle):
        self.goal_handle = handle
        self.completion = Future()
        return await self.completion

    def finish(self, ok, reason, canceled=False):
        # Cero en la entrada protegida, nunca en /cmd_vel.
        self.vel_pub.publish(Twist())
        mode = self.mode
        handle, completion = self.goal_handle, self.completion
        if ok and mode == 'dock':
            self.docked = True
            self.dock_pose = self.pose(self.get_clock().now())
        elif ok and mode == 'undock':
            self.docked = False
            self.dock_pose = None
        result = Dock.Result() if mode == 'dock' else Undock.Result()
        result.is_docked = self.docked
        if canceled:
            handle.canceled()
        elif ok:
            handle.succeed()
        else:
            handle.abort()
        self.get_logger().info(f'{mode}: {reason}; is_docked={self.docked}')
        self.mode = self.goal_handle = self.completion = None
        completion.set_result(result)

    def motion_tick(self):
        now = self.get_clock().now()
        pose = self.pose(now)
        # No seguir simulando carga si el robot se ha alejado del acoplamiento.
        if self.docked and pose is not None and self.dock_pose is not None:
            if math.hypot(pose[0]-self.dock_pose[0], pose[1]-self.dock_pose[1]) > 0.09:
                self.docked = False
                self.dock_pose = None
        if self.goal_handle is None:
            return
        if self.goal_handle.is_cancel_requested:
            self.finish(False, 'cancelado', True)
            return
        wall = time.monotonic()
        if now.nanoseconds != self.last_control_clock:
            if now.nanoseconds < self.last_control_clock:
                self.finish(False, 'reloj reiniciado')
                return
            self.last_control_clock = now.nanoseconds
            self.last_clock_progress = wall
        if wall > self.deadline or wall-self.last_clock_progress > 0.8:
            self.finish(False, 'timeout o reloj detenido')
            return
        if not self.p('dock_motion_enabled') or not self.monitor_present() or pose is None:
            self.finish(False, 'movimiento no autorizado, TF o scan obsoleto')
            return
        visible, front = self.detection(now)
        if self.mode == 'dock':
            feedback = Dock.Feedback()
            feedback.sees_dock = visible
            self.goal_handle.publish_feedback(feedback)
        if not visible:
            self.finish(False, 'dock no visible')
            return
        fx, fy, nx, ny, heading = self.geometry()
        x, y, angle = pose
        s = (x-fx)*nx+(y-fy)*ny
        lateral = -(x-fx)*ny+(y-fy)*nx
        standoff = self.p('dock_standoff_m')
        retreat = self.p('undocking_distance')
        if abs(lateral) > 0.26 or s < standoff-0.04 or s > 1.05:
            self.finish(False, 'fuera del corredor de aproximacion')
            return
        target_s = standoff if self.mode == 'dock' else standoff+retreat
        # Alineacion precisa antes del acoplamiento; retroceso a HOME.
        tx, ty = fx+target_s*nx, fy+target_s*ny
        dist = math.hypot(tx-x, ty-y)
        heading_error = wrap(heading-angle)
        cmd = Twist()
        if dist <= 0.035:
            if abs(heading_error) > 0.08:
                cmd.angular.z = max(-0.20, min(0.20, 1.5*heading_error))
                self.settle_count = 0
            else:
                self.settle_count += 1
                if self.settle_count >= 4:
                    self.finish(True, 'pose virtual confirmada')
                    return
        else:
            self.settle_count = 0
            desired = math.atan2(ty-y, tx-x)
            if self.mode == 'undock':
                desired = wrap(desired+math.pi)
            err = wrap(desired-angle)
            # No avanzar mas cerca de la cara que el standoff.
            if self.mode == 'dock' and s <= standoff+0.005:
                self.finish(False, 'desalineacion cerca del dock')
                return
            if self.mode == 'undock' and s >= target_s+0.005:
                self.finish(False, 'desalineacion al salir')
                return
            cmd.angular.z = max(-0.25, min(0.25, 1.5*err))
            if abs(err) < 0.18:
                speed = self.p('docking_speed') if self.mode == 'dock' else self.p('undocking_speed')
                if not math.isfinite(speed) or not 0.0 < speed <= 0.06:
                    self.finish(False, 'velocidad invalida')
                    return
                cmd.linear.x = min(speed, 0.6*dist)*(1 if self.mode == 'dock' else -1)
        self.vel_pub.publish(cmd)

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
