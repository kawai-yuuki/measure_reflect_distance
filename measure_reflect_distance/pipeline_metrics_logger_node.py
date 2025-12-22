#!/usr/bin/env python3
import csv
import math
import os
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


def _coerce_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _coerce_float(value, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class PipelineMetricsLogger(Node):
    def __init__(self) -> None:
        super().__init__("pipeline_metrics_logger")

        self.declare_parameter("input_topic", "/camera/unet/image_raw")
        self.declare_parameter("output_topic", "/mask_image")
        self.declare_parameter("inference_time_topic", "/unet/inference_time_ms")
        self.declare_parameter("report_interval_sec", 1.0)
        self.declare_parameter("duration_sec", 30.0)
        self.declare_parameter("warmup_sec", 0.0)
        self.declare_parameter("output_csv", "~/.ros/unet_metrics.csv")
        self.declare_parameter("stop_when_done", True)
        self.declare_parameter("csv_mode", "minimal")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        inference_time_topic = self.get_parameter("inference_time_topic").value
        report_interval_sec = _coerce_float(
            self.get_parameter("report_interval_sec").value, 1.0
        )
        duration_sec = _coerce_float(self.get_parameter("duration_sec").value, 30.0)
        warmup_sec = _coerce_float(self.get_parameter("warmup_sec").value, 0.0)
        output_csv = self.get_parameter("output_csv").value
        self._stop_when_done = _coerce_bool(
            self.get_parameter("stop_when_done").value
        )
        csv_mode = str(self.get_parameter("csv_mode").value).strip().lower()
        if csv_mode not in {"minimal", "full"}:
            self.get_logger().warn(
                f"csv_mode must be 'minimal' or 'full'. Falling back to 'minimal' (got '{csv_mode}')."
            )
            csv_mode = "minimal"
        self._csv_mode = csv_mode

        if report_interval_sec <= 0.0:
            self.get_logger().warn(
                "report_interval_sec must be > 0. Falling back to 1.0."
            )
            report_interval_sec = 1.0

        if duration_sec < 0.0:
            self.get_logger().warn(
                "duration_sec must be >= 0. Falling back to 0 (no auto-stop)."
            )
            duration_sec = 0.0

        if warmup_sec < 0.0:
            self.get_logger().warn(
                "warmup_sec must be >= 0. Falling back to 0."
            )
            warmup_sec = 0.0

        self._report_interval_sec = report_interval_sec
        self._duration_sec = duration_sec
        self._warmup_sec = warmup_sec
        self._recording = warmup_sec <= 0.0
        self._finished = False

        self._start_time = self.get_clock().now()
        self._recording_start_time = self._start_time if self._recording else None
        self._last_report_time = self._start_time

        self._interval_input_count = 0
        self._interval_output_count = 0
        self._latency_samples_ms: List[float] = []
        self._inference_samples_ms: List[float] = []

        self._csv_path = os.path.expanduser(output_csv)
        csv_dir = os.path.dirname(self._csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        if self._csv_mode == "full":
            header = [
                "timestamp_sec",
                "interval_sec",
                "input_count",
                "output_count",
                "input_fps",
                "output_fps",
                "output_input_ratio",
                "latency_count",
                "latency_mean_ms",
                "latency_median_ms",
                "latency_p95_ms",
                "latency_max_ms",
                "inference_count",
                "inference_mean_ms",
                "inference_median_ms",
                "inference_p95_ms",
                "inference_max_ms",
            ]
        else:
            header = [
                "timestamp_sec",
                "interval_sec",
                "input_count",
                "output_count",
                "input_fps",
                "output_fps",
                "output_input_ratio",
                "latency_mean_ms",
                "latency_p95_ms",
                "inference_mean_ms",
                "inference_p95_ms",
            ]
        self._csv_writer.writerow(header)
        self._csv_file.flush()

        self.create_subscription(
            Image,
            input_topic,
            self._on_input,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            output_topic,
            self._on_output,
            qos_profile_sensor_data,
        )
        self._inference_sub = None
        if inference_time_topic:
            self._inference_sub = self.create_subscription(
                Float32,
                inference_time_topic,
                self._on_inference_time,
                10,
            )

        self._report_timer = self.create_timer(
            self._report_interval_sec, self._on_report
        )

        self.get_logger().info(
            "PipelineMetricsLogger started. "
            f"input_topic={input_topic}, output_topic={output_topic}, "
            f"inference_time_topic={inference_time_topic}, "
            f"report_interval_sec={self._report_interval_sec:.2f}, "
            f"duration_sec={self._duration_sec:.2f}, warmup_sec={self._warmup_sec:.2f}, "
            f"output_csv={self._csv_path}, csv_mode={self._csv_mode}"
        )

    def _maybe_start_recording(self) -> None:
        if self._recording:
            return
        now = self.get_clock().now()
        elapsed_ns = (now - self._start_time).nanoseconds
        if elapsed_ns >= int(self._warmup_sec * 1e9):
            self._recording = True
            self._recording_start_time = now
            self._last_report_time = now
            self._reset_interval()
            self.get_logger().info("Metrics recording started after warmup.")

    def _reset_interval(self) -> None:
        self._interval_input_count = 0
        self._interval_output_count = 0
        self._latency_samples_ms.clear()
        self._inference_samples_ms.clear()

    def _on_input(self, msg: Image) -> None:
        self._maybe_start_recording()
        if not self._recording:
            return
        self._interval_input_count += 1

    def _on_output(self, msg: Image) -> None:
        self._maybe_start_recording()
        if not self._recording:
            return
        self._interval_output_count += 1

        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            return
        now = self.get_clock().now()
        delay_ns = (now - stamp).nanoseconds
        if delay_ns < 0:
            return
        self._latency_samples_ms.append(delay_ns / 1e6)

    def _on_inference_time(self, msg: Float32) -> None:
        self._maybe_start_recording()
        if not self._recording:
            return
        self._inference_samples_ms.append(float(msg.data))

    def _on_report(self) -> None:
        self._maybe_start_recording()
        if not self._recording or self._finished:
            return

        now = self.get_clock().now()
        self._write_report(now)

        if self._duration_sec > 0.0 and self._recording_start_time is not None:
            elapsed_sec = (now - self._recording_start_time).nanoseconds / 1e9
            if elapsed_sec >= self._duration_sec:
                self._finish()

    def _calc_stats(self, values: List[float]) -> Tuple[float, float, float, float, int]:
        if not values:
            return ("", "", "", "", 0)
        sorted_vals = sorted(values)
        count = len(sorted_vals)
        mean_val = sum(sorted_vals) / count
        median_val = (
            sorted_vals[count // 2]
            if count % 2 == 1
            else (sorted_vals[count // 2 - 1] + sorted_vals[count // 2]) / 2.0
        )
        p95_val = self._percentile(sorted_vals, 0.95)
        max_val = sorted_vals[-1]
        return (mean_val, median_val, p95_val, max_val, count)

    def _percentile(self, sorted_vals: List[float], percentile: float) -> float:
        if not sorted_vals:
            return ""
        if len(sorted_vals) == 1:
            return sorted_vals[0]
        k = (len(sorted_vals) - 1) * percentile
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return sorted_vals[int(k)]
        return sorted_vals[f] + (sorted_vals[c] - sorted_vals[f]) * (k - f)

    def _has_interval_data(self) -> bool:
        return (
            self._interval_input_count > 0
            or self._interval_output_count > 0
            or self._latency_samples_ms
            or self._inference_samples_ms
        )

    def _write_report(self, now: Time) -> None:
        interval_sec = (now - self._last_report_time).nanoseconds / 1e9
        if interval_sec <= 0.0:
            return

        input_fps = self._interval_input_count / interval_sec
        output_fps = self._interval_output_count / interval_sec
        output_input_ratio = (
            self._interval_output_count / self._interval_input_count
            if self._interval_input_count > 0
            else 0.0
        )

        latency_stats = self._calc_stats(self._latency_samples_ms)
        inference_stats = self._calc_stats(self._inference_samples_ms)

        if self._csv_mode == "full":
            row = [
                now.nanoseconds / 1e9,
                interval_sec,
                self._interval_input_count,
                self._interval_output_count,
                input_fps,
                output_fps,
                output_input_ratio,
                latency_stats[4],
                latency_stats[0],
                latency_stats[1],
                latency_stats[2],
                latency_stats[3],
                inference_stats[4],
                inference_stats[0],
                inference_stats[1],
                inference_stats[2],
                inference_stats[3],
            ]
        else:
            row = [
                now.nanoseconds / 1e9,
                interval_sec,
                self._interval_input_count,
                self._interval_output_count,
                input_fps,
                output_fps,
                output_input_ratio,
                latency_stats[0],
                latency_stats[2],
                inference_stats[0],
                inference_stats[2],
            ]
        self._csv_writer.writerow(row)
        self._csv_file.flush()

        self._last_report_time = now
        self._reset_interval()

    def _finish(self) -> None:
        if self._finished:
            return
        try:
            if self._recording and self._has_interval_data():
                self._write_report(self.get_clock().now())
        finally:
            self._finished = True
            if not self._csv_file.closed:
                self._csv_file.close()
            self.get_logger().info(f"Saved metrics CSV to {self._csv_path}")
            if self._stop_when_done and rclpy.ok():
                rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PipelineMetricsLogger()
    try:
        rclpy.spin(node)
    finally:
        node._finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
