import subprocess
import time
import signal
import sys


class LidarAvoidanceNode:
    def __init__(self):
        self.running = True
        self.last_command_time = 0
        self.command_interval = 0.1
        self.current_linear = 0.0
        self.current_angular = 0.0

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        print("\n[INFO] Stopping node...")
        self.running = False
        self.stop_robot()
        sys.exit(0)

    def publish_cmd_vel(self, linear_x, angular_z):
        cmd = [
            "ign",
            "topic",
            "-t",
            "/cmd_vel",
            "-m",
            "ignition.msgs.Twist",
            "-p",
            f"linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}",
        ]
        try:
            result = subprocess.run(cmd, check=False, capture_output=True, timeout=1.0)
            print("[DEBUG] Published cmd_vel:", result.args)
            print("[DEBUG] Publish result:", result.stdout, result.stderr)
        except subprocess.TimeoutExpired:
            print("[ERROR] Command timed out while publishing cmd_vel")
        except Exception as e:
            print("[DEBUG] Exception while publishing cmd_vel:", e)
            pass

    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)
        print("[INFO] Robot stopped")

    def read_lidar_once(self):
        cmd = ["ign", "topic", "-e", "-t", "/lidar", "-n", "1"]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
            if result.returncode != 0:
                print(
                    "[DEBUG] Failed to read /lidar topic. Return code:",
                    result.returncode,
                )
                return None

            ranges = []
            data_count = 0
            for line in result.stdout.split("\n"):
                line = line.strip()
                if "ranges:" in line:
                    try:
                        value_str = line.split("ranges:")[1].strip()
                        if value_str:  # 确保有值
                            value = float(value_str)
                            if value != float("inf") and value > 0:
                                ranges.append(value)
                                data_count += 1
                    except (ValueError, IndexError) as e:
                        continue

            if ranges:
                min_dist = min(ranges)
                max_dist = max(ranges)
                avg_dist = sum(ranges) / len(ranges)
                print(f"\n[DEBUG] 激光雷达数据统计:")
                print(f"总点数: {data_count}, 有效点数: {len(ranges)}")
                print(
                    f"距离范围: {min_dist:.3f}m - {max_dist:.3f}m (平均: {avg_dist:.3f}m)"
                )
                print(f"样本数据: {ranges[:5]}...")  # 只显示前5个点
            else:
                print("\n[WARN] 未检测到有效的距离数据")

            return ranges

        except Exception as e:
            print(f"[ERROR] 读取激光雷达数据时出错: {e}")
            return None

    def process_lidar_data(self, ranges):
        if not ranges:
            return

        # 计算有效的距离值
        valid_ranges = [r for r in ranges if r > 0 and r != float("inf")]
        if not valid_ranges:
            print("[WARN] No valid range measurements")
            return

        # 计算关键统计数据
        min_distance = min(valid_ranges)
        num_points = len(valid_ranges)

        # 将数据分为前、左、右三个扇区
        sector_size = len(valid_ranges) // 3
        front_sector = valid_ranges[sector_size : 2 * sector_size]
        left_sector = valid_ranges[:sector_size]
        right_sector = valid_ranges[2 * sector_size :]

        # 计算每个扇区的最小距离
        front_min = min(front_sector) if front_sector else float("inf")
        left_min = min(left_sector) if left_sector else float("inf")
        right_min = min(right_sector) if right_sector else float("inf")

        # 障碍物检测逻辑
        front_obstacle = front_min < 0.8  # 前方0.8米内有障碍物
        any_obstacle = min_distance < 0.4  # 任何方向0.4米内有障碍物

        if any_obstacle:
            # 紧急情况：非常接近障碍物
            self.current_linear = -0.2  # 后退
            self.current_angular = 0.8  # 快速转向
            status = f"Emergency | obs:{min_distance:.2f}m"
        elif front_obstacle:
            # 前方有障碍物，选择转向空旷的一侧
            self.current_linear = 0.0
            if left_min > right_min:
                self.current_angular = 0.5  # 左转
                status = f"Left | front:{front_min:.2f}m"
            else:
                self.current_angular = -0.5  # 右转
                status = f"Right | front:{front_min:.2f}m"
        else:
            # 路径畅通
            self.current_linear = 0.3  # 适中速度前进
            self.current_angular = 0.0  # 不转向
            status = f"Forward | front:{front_min:.2f}m"

        # 打印状态信息
        print(
            f"\r[状态] {status} | L:{left_min:.2f}m F:{front_min:.2f}m R:{right_min:.2f}m | 点数:{num_points}",
            end="",
            flush=True,
        )

    def run(self):
        print("Lidar Avoidance Node Started")
        print("Subscribe: /lidar")
        print("Publish: /cmd_vel")
        print("Press Ctrl+C to stop")
        print("-" * 50)

        consecutive_failures = 0

        while self.running:
            current_time = time.time()
            if current_time - self.last_command_time >= self.command_interval:
                self.publish_cmd_vel(self.current_linear, self.current_angular)
                self.last_command_time = current_time

            ranges = self.read_lidar_once()

            if ranges:
                consecutive_failures = 0
                self.process_lidar_data(ranges)
            else:
                consecutive_failures += 1
                if consecutive_failures >= 10:
                    print("\n[ERROR] Cannot read lidar data")
                    print(
                        "Check: 1) Gazebo running 2) /lidar topic exists 3) Lidar sensor configured"
                    )
                    consecutive_failures = 0

            time.sleep(0.01)


def main():
    try:
        node = LidarAvoidanceNode()
        node.run()
    except KeyboardInterrupt:
        print("\nNode stopped")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        return 1
    return 0


if __name__ == "__main__":
    exit(main())
