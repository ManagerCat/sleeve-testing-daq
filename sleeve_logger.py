#!/usr/bin/env python3
"""
Cooling Sleeve Testing Jig - Elegoo Mega 2560 R3 Monitor & Data Logger

Measures:
  - Coolant flow rate (Koolance pulse flowmeter on Digital Pin 2 / INT4)
  - Coolant inlet & outlet temperatures (Koolance inline 10k NTC thermistors on A0, A1)
  - 8x Mock motor edge temperatures (Ametherm PANE 103395-410 10k NTC thermistors on A2-A9)
  - Coolant heat absorption rate (Watts: Q = m_dot * Cp * delta_T)
  - Motor edge temperature distribution (Avg, Min, Max, Spread, Hotspot)
  - Thermal resistance (Rth = (T_motor_avg - T_water_in) / Q)

Outputs:
  - Live Rich terminal dashboard
  - Timestamped CSV log saved automatically to ~/Downloads
"""

import csv
import datetime
import json
import math
import sys
import time
import traceback
from collections import deque
from pathlib import Path

# ==============================================================================
# CONFIGURATION LOADER (Loads from config.toml or config.json)
# ==============================================================================
CONFIG_FILE_PATH = None

def load_config() -> tuple[dict, str]:
    """
    Loads configuration settings from config.toml (or config.json).
    Returns (config_dict, config_filename).
    """
    global CONFIG_FILE_PATH
    script_dir = Path(__file__).resolve().parent

    # Check CLI arguments for --config <file>
    if "--config" in sys.argv:
        try:
            idx = sys.argv.index("--config")
            target_path = Path(sys.argv[idx + 1])
        except IndexError:
            print("[ERROR] Missing filepath argument for --config")
            sys.exit(1)
    elif (script_dir / "config.toml").exists():
        target_path = script_dir / "config.toml"
    elif (script_dir / "config.json").exists():
        target_path = script_dir / "config.json"
    else:
        target_path = script_dir / "config.toml"

    CONFIG_FILE_PATH = target_path

    if not target_path.exists():
        return {}, "defaults (no config found)"

    try:
        if target_path.suffix.lower() == ".toml":
            try:
                import tomllib
                with open(target_path, "rb") as f:
                    return tomllib.load(f), target_path.name
            except ImportError:
                import tomli as tomllib
                with open(target_path, "rb") as f:
                    return tomllib.load(f), target_path.name
        else:
            with open(target_path, "r", encoding="utf-8") as f:
                return json.load(f), target_path.name
    except Exception as e:
        print(f"[ERROR] Failed to parse config file {target_path}: {e}")
        sys.exit(1)


# Load active configuration
CONFIG, CONFIG_NAME = load_config()

# Hardware & Serial
HW_CFG = CONFIG.get("hardware", {})
SERIAL_PORT = HW_CFG.get("serial_port", "AUTO")
BAUD_RATE = int(HW_CFG.get("baud_rate", 115200))
SERIAL_TIMEOUT = float(HW_CFG.get("timeout_s", 2.0))

# Circuit & Pull-up Resistor (Ohms)
CIRCUIT_CFG = CONFIG.get("circuit", {})
DEFAULT_R_FIXED = float(CIRCUIT_CFG.get("default_r_fixed", 9820.0))
RESISTOR_OHMS_MAP = CIRCUIT_CFG.get("resistor_ohms", {})

def get_r_fixed(channel_name: str) -> float:
    """Returns the pull-up resistor ohm value for a specific sensor or the circuit default."""
    return float(RESISTOR_OHMS_MAP.get(channel_name, DEFAULT_R_FIXED))

# Analog Channel Mapping
CHANNELS_CFG = CONFIG.get("channels", {})
WATER_CHANNELS = CHANNELS_CFG.get("water", {
    "water_in": 0,
    "water_out": 1,
})
MOTOR_CHANNELS = CHANNELS_CFG.get("motor", {
    "T1": 2, "T2": 3, "T3": 4, "T4": 5, "T5": 6, "T6": 7, "T7": 8, "T8": 9
})

# Thermistor Calibration Parameters (Steinhart-Hart / Beta equation)
SENSORS_CFG = CONFIG.get("sensors", {})
COOLANT_CFG = SENSORS_CFG.get("coolant", {})
COOLANT_R0 = float(COOLANT_CFG.get("r0", 10000.0))
COOLANT_T0_K = float(COOLANT_CFG.get("t0_c", 25.0)) + 273.15
COOLANT_BETA = float(COOLANT_CFG.get("beta", 3435.0))

MOTOR_CFG = SENSORS_CFG.get("motor", {})
MOTOR_R0 = float(MOTOR_CFG.get("r0", 10000.0))
MOTOR_T0_K = float(MOTOR_CFG.get("t0_c", 25.0)) + 273.15
MOTOR_BETA = float(MOTOR_CFG.get("beta", 3950.0))

# Motor Geometry & Conduction Offset Parameters
MOTOR_GEOM_CFG = CONFIG.get("motor_geometry", {})
OFFSET_ENABLED = bool(MOTOR_GEOM_CFG.get("enabled", True))
OFFSET_DIST_IN = float(MOTOR_GEOM_CFG.get("distance_from_edge_in", 0.095))
MOCK_OD_IN = float(MOTOR_GEOM_CFG.get("outer_diameter_in", 3.778))
MOCK_LENGTH_IN = float(MOTOR_GEOM_CFG.get("length_in", 3.6))
K_ALUMINUM = float(MOTOR_GEOM_CFG.get("k_aluminum", 167.0))

def compute_aluminum_conduction_rth(od_in: float, dist_in: float, length_in: float, k_val: float) -> float:
    """
    Computes cylindrical shell conduction thermal resistance (°C/W) across the aluminum layer
    between embedded thermistor centerline and the mock motor outer surface:
    R_cond = ln(r_outer / (r_outer - dist)) / (2 * pi * k * length)
    """
    if not (od_in > 0 and dist_in > 0 and length_in > 0 and k_val > 0):
        return 0.0
    r_outer_m = (od_in / 2.0) * 0.0254
    dist_m = dist_in * 0.0254
    r_sensor_m = r_outer_m - dist_m
    length_m = length_in * 0.0254
    if r_sensor_m <= 0 or r_outer_m <= r_sensor_m:
        return 0.0
    return math.log(r_outer_m / r_sensor_m) / (2.0 * math.pi * k_val * length_m)

R_COND_ALUMINUM = compute_aluminum_conduction_rth(MOCK_OD_IN, OFFSET_DIST_IN, MOCK_LENGTH_IN, K_ALUMINUM) if OFFSET_ENABLED else 0.0

# Flow Rate Sensor Parameters
FLOW_CFG = CONFIG.get("flow", {})
LPM_PER_HZ = float(FLOW_CFG.get("lpm_per_hz", 0.308))
FLOW_WINDOW = float(FLOW_CFG.get("window_s", 1.0))
FLOW_HISTORY = int(FLOW_CFG.get("history_samples", 10))

# Physical Fluid Constants (Water)
FLUIDS_CFG = CONFIG.get("fluids", {})
CP_WATER = float(FLUIDS_CFG.get("cp_water", 4180.0))
RHO_WATER = float(FLUIDS_CFG.get("rho_water", 1.0))

# Sampling and Smoothing Parameters
LOGGING_CFG = CONFIG.get("logging", {})
OUTPUT_DIR = Path(LOGGING_CFG.get("output_dir", "~/Downloads")).expanduser()
CSV_PREFIX = LOGGING_CFG.get("csv_prefix", "sleeve_test")
TARGET_HZ = float(LOGGING_CFG.get("target_hz", 20.0))
SMOOTH_SAMPLES = int(LOGGING_CFG.get("smooth_samples", 200))
WATTS_SMOOTH_SAMPLES = int(LOGGING_CFG.get("watts_smooth_samples", 200))
RATE_WINDOW_S = float(LOGGING_CFG.get("rate_window_s", 10.0))
DASHBOARD_REFRESH_HZ = float(LOGGING_CFG.get("dashboard_refresh_hz", 4.0))

# Real-Time Terminal Temperature Graph Parameters
GRAPH_CFG = CONFIG.get("graph", {})
GRAPH_ENABLED = bool(GRAPH_CFG.get("enabled", True))
GRAPH_HISTORY_S = float(GRAPH_CFG.get("history_seconds", 60.0))

ADC_MAX = 1023.0               # 10-bit ADC resolution (0 to 1023)

# ==============================================================================
# LIBRARY IMPORTS & SYSTEM CHECKS
# ==============================================================================
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("\n[ERROR] The 'pyserial' library is not installed.")
    print("Please install it by running: pip install pyserial\n")
    sys.exit(1)

try:
    from rich.console import Console, ConsoleOptions, RenderResult
    from rich.layout import Layout
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
except ImportError:
    print("\n[ERROR] The 'rich' library is not installed.")
    print("Please install it by running: pip install rich\n")
    sys.exit(1)


# ==============================================================================
# SERIAL PORT AUTO-DISCOVERY
# ==============================================================================
def find_mega_port() -> str | None:
    """
    Automatically detects the connected Elegoo / Arduino Mega 2560 board.
    Inspects port metadata (description, manufacturer, hwid) and macOS /dev nodes.
    Filters out system virtual ports (bluetooth, debug console).
    """
    if SERIAL_PORT != "AUTO":
        return SERIAL_PORT

    available_ports = list(serial.tools.list_ports.comports())
    if not available_ports:
        return None

    # Filter out virtual macOS system ports
    real_ports = [
        p for p in available_ports
        if not any(v in p.device.lower() for v in ["bluetooth", "debug-console", "wlan"])
    ]
    if not real_ports:
        return None

    # Priority 1: Match vendor/product keywords
    for p in real_ports:
        desc = (p.description or "").lower()
        mfg = (p.manufacturer or "").lower()
        hwid = (p.hwid or "").lower()
        dev = p.device.lower()
        if any(keyword in text for text in (desc, mfg, hwid, dev) for keyword in ["mega", "arduino", "ch340", "ch341", "elegoo", "wch"]):
            return p.device

    # Priority 2: Standard macOS USB modem / USB serial paths
    for p in real_ports:
        if "usbmodem" in p.device.lower() or "usbserial" in p.device.lower():
            return p.device

    # Priority 3: Fall back to first non-virtual candidate
    return real_ports[0].device


# ==============================================================================
# CALCULATION & CONVERSION UTILITIES
# ==============================================================================
def adc_to_celsius(adc_val: float, r_fixed: float, r0: float, t0_k: float, beta: float) -> float:
    """
    Converts 10-bit ADC reading (0-1023) to temperature (°C) using ratiometric
    voltage divider and Beta-equation NTC thermistor modeling.
    Circuit: Mega 5V -> r_fixed -> Analog Pin -> Thermistor (R_th) -> GND
    Since ADC reference and pull-up both reference the 5V rail:
      ADC / 1023 = R_th / (r_fixed + R_th)
      => R_th = r_fixed * ADC / (1023 - ADC)
    """
    if not (1 <= adc_val <= (ADC_MAX - 1)):
        return math.nan
    try:
        r_thermistor = r_fixed * adc_val / (ADC_MAX - adc_val)
        if r_thermistor <= 0:
            return math.nan
        # 1/T = 1/T0 + (1/Beta) * ln(R / R0)
        temp_k = 1.0 / ((1.0 / t0_k) + (math.log(r_thermistor / r0) / beta))
        return temp_k - 273.15
    except (ValueError, ZeroDivisionError):
        return math.nan


def mean_valid(values) -> float:
    """Computes the arithmetic mean of all non-NaN/finite elements."""
    valid = [x for x in values if math.isfinite(x)]
    return sum(valid) / len(valid) if valid else math.nan


def calculate_rate_of_change(history: deque, window_s: float) -> tuple[float, float]:
    """
    Computes smooth dT/dt (°C/s and °C/min) from a rolling deque of (timestamp, temp) pairs
    using least-squares linear regression slope over the specified time window.
    Returns (rate_per_sec, rate_per_min).
    """
    if len(history) < 3:
        return math.nan, math.nan

    t_now = history[-1][0]
    valid_points = [(t, val) for (t, val) in history if (t_now - t) <= window_s and math.isfinite(val)]
    if len(valid_points) < 3:
        return math.nan, math.nan

    times, temps = zip(*valid_points)
    dt = times[-1] - times[0]
    if dt < 0.2:  # Need at least 200 ms of elapsed data
        return math.nan, math.nan

    t_mean = sum(times) / len(times)
    temp_mean = sum(temps) / len(temps)

    numerator = sum((t - t_mean) * (temp - temp_mean) for t, temp in valid_points)
    denominator = sum((t - t_mean) ** 2 for t in times)

    if denominator <= 0:
        return math.nan, math.nan

    rate_s = numerator / denominator  # °C per second
    rate_min = rate_s * 60.0          # °C per minute
    return rate_s, rate_min


# ==============================================================================
# REAL-TIME METRIC GRAPH (Braille Unicode Canvas)
# ==============================================================================
BRAILLE_MAP = {
    (0, 0): 0x01,
    (1, 0): 0x02,
    (2, 0): 0x04,
    (0, 1): 0x08,
    (1, 1): 0x10,
    (2, 1): 0x20,
    (3, 0): 0x40,
    (3, 1): 0x80,
}

class LiveMetricGraph:
    """
    Auto-sizing Unicode Braille multi-series metric graph for Rich terminal dashboards.
    Renders high-resolution continuous telemetry curves (2x4 dots per character cell).
    Supports customizable title, units (°C, °C/W), precision formatting, and border styling.
    """
    def __init__(
        self,
        title: str = "Real-Time Temperature History",
        unit: str = "°C",
        precision: int = 1,
        time_window_s: float = 60.0,
        border_style: str = "cyan",
    ):
        self.title = title
        self.unit = unit
        self.precision = precision
        self.time_window_s = time_window_s
        self.border_style = border_style
        self.series: list[dict] = []

    def set_data(self, series: list[dict]):
        self.series = series

    def __rich_console__(self, console: Console, options: ConsoleOptions) -> RenderResult:
        avail_w = options.max_width or 80
        avail_h = options.max_height or 12

        content_w = max(25, avail_w - 4)
        # Panel borders take 2 lines; legend (1), axis (1), and time bar (1) take 3 lines.
        # Keep plot_h constrained so the panel never overflows its container.
        plot_h = max(2, (avail_h - 5) if (avail_h is not None and avail_h > 5) else 6)
        
        # Extract values in active time window
        now = time.monotonic()
        t_min_threshold = now - self.time_window_s

        all_vals = []
        for s in self.series:
            for t_val, v in s.get("data", []):
                if t_val >= t_min_threshold and math.isfinite(v):
                    all_vals.append(v)

        if not all_vals:
            msg = Text(f"Waiting for {self.title.lower()}...", justify="center", style="dim italic")
            yield Panel(msg, title=f"[bold]{self.title}[/bold]", border_style=self.border_style)
            return

        min_v = min(all_vals)
        max_v = max(all_vals)
        span = max_v - min_v

        min_span = 1.0 if self.unit == "°C" else 0.001
        if span < min_span:
            mid = (max_v + min_v) / 2.0
            half = min_span / 2.0
            y_min = mid - half
            y_max = mid + half
        else:
            y_min = min_v - 0.06 * span
            y_max = max_v + 0.06 * span

        # Compute exact Y-axis label width so every row has identical prefix width
        max_str = f"{y_max:.{self.precision}f}{self.unit}"
        min_str = f"{y_min:.{self.precision}f}{self.unit}"
        mid_str = f"{((y_max + y_min) / 2.0):.{self.precision}f}{self.unit}"
        val_w = max(len(max_str), len(min_str), len(mid_str))
        y_label_w = max(8, val_w + 3)  # space + val_w + " ┤"
        plot_w = max(10, content_w - y_label_w)

        sub_w = plot_w * 2
        sub_h = plot_h * 4

        grid_dots = [[0] * plot_w for _ in range(plot_h)]
        grid_color = [[None] * plot_w for _ in range(plot_h)]
        grid_prio = [[-1] * plot_w for _ in range(plot_h)]

        def plot_pt(px: int, py: int, col_style: str, prio: int):
            if 0 <= px < sub_w and 0 <= py < sub_h:
                c_idx = px // 2
                r_idx = py // 4
                dot_c = px % 2
                dot_r = py % 4
                grid_dots[r_idx][c_idx] |= BRAILLE_MAP[(dot_r, dot_c)]
                if prio >= grid_prio[r_idx][c_idx]:
                    grid_color[r_idx][c_idx] = col_style
                    grid_prio[r_idx][c_idx] = prio

        def draw_line(x0: int, y0: int, x1: int, y1: int, col_style: str, prio: int):
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy
            while True:
                plot_pt(x0, y0, col_style, prio)
                if x0 == x1 and y0 == y1:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy

        for s_idx, s in enumerate(self.series):
            c_style = s["color"]
            pts = []
            for t_val, v in s.get("data", []):
                if not math.isfinite(v) or t_val < t_min_threshold:
                    continue
                t_rel = (t_val - t_min_threshold) / max(0.001, self.time_window_s)
                px = int(max(0, min(sub_w - 1, t_rel * (sub_w - 1))))
                v_rel = (v - y_min) / max(1e-9, y_max - y_min)
                py = int(max(0, min(sub_h - 1, (1.0 - v_rel) * (sub_h - 1))))
                pts.append((px, py))

            for i in range(len(pts) - 1):
                draw_line(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], c_style, s_idx)
            if len(pts) == 1:
                plot_pt(pts[0][0], pts[0][1], c_style, s_idx)

        t_out = Text(no_wrap=True)

        # Legend header
        leg = Text(no_wrap=True)
        for idx, s in enumerate(self.series):
            if idx > 0:
                leg.append("  ")
            leg.append("■ ", style=s["color"])
            curr = s.get("current", math.nan)
            if math.isfinite(curr):
                curr_str = f"{curr:.{self.precision}f}{self.unit}"
            else:
                curr_str = "--"
            leg.append(f"{s['name']}:", style="bold white")
            leg.append(f"{curr_str}", style=s["color"])
        t_out.append(leg)
        t_out.append("\n")

        # Rows with Y-axis labels - all guaranteed to be exactly y_label_w in length
        target_val_w = y_label_w - 2  # subtract " ┤"
        for r in range(plot_h):
            row_frac = r / max(1, plot_h - 1)
            y_val = y_max - row_frac * (y_max - y_min)
            if r == 0 or r == (plot_h - 1) or r == (plot_h // 2):
                v_str = f"{y_val:.{self.precision}f}{self.unit}"
                y_label = f"{v_str:>{target_val_w}} ┤"
            else:
                y_label = " " * (y_label_w - 1) + "│"

            t_out.append(y_label, style=f"dim {self.border_style}")
            for c in range(plot_w):
                d = grid_dots[r][c]
                ch = chr(0x2800 + d) if d else " "
                col = grid_color[r][c] or "white"
                t_out.append(ch, style=col)
            t_out.append("\n")

        # Bottom axis and time labels
        t_out.append(" " * (y_label_w - 1) + "└" + "─" * plot_w, style=f"dim {self.border_style}")
        t_out.append("\n")

        t_start = f"-{int(self.time_window_s)}s"
        t_mid = f"-{int(self.time_window_s / 2)}s"
        t_end = "Now"
        gap1 = max(1, (plot_w // 2) - len(t_start) - (len(t_mid) // 2))
        gap2 = max(1, plot_w - len(t_start) - gap1 - len(t_mid) - len(t_end))
        time_bar = " " * y_label_w + f"{t_start}{' ' * gap1}{t_mid}{' ' * gap2}{t_end}"
        t_out.append(time_bar[:content_w], style="dim")

        yield Panel(t_out, title=f"[bold]{self.title}[/bold]", border_style=self.border_style, padding=(0, 1))


# ==============================================================================
# DASHBOARD RENDERING
# ==============================================================================
def create_dashboard_layout(show_graph: bool = True) -> Layout:
    """Defines the Rich terminal layout structure."""
    layout = Layout()
    if show_graph:
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="main", size=15),
            Layout(name="graph_row", ratio=1, minimum_size=9),
            Layout(name="footer", size=3),
        )
        layout["graph_row"].split_row(
            Layout(name="temp_graph_panel", ratio=1),
            Layout(name="rth_graph_panel", ratio=1),
        )
    else:
        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="main", size=18),
            Layout(name="footer", size=3),
        )
    layout["main"].split_row(
        Layout(name="coolant_panel", ratio=1),
        Layout(name="motor_panel", ratio=1),
    )
    return layout


def build_dashboard(
    elapsed_s: float,
    device_info: str,
    output_filename: str,
    row_count: int,
    loop_hz: float,
    water_in_c: float,
    water_out_c: float,
    delta_water_c: float,
    flow_hz: float,
    flow_lpm: float,
    watts_raw: float,
    watts_smooth: float,
    motor_temps: dict,
    motor_avg: float,
    motor_min: float,
    motor_max: float,
    motor_spread: float,
    motor_rate_s: float,
    motor_rate_min: float,
    hotspot_name: str,
    rth: float,
    edge_offset_deg: float = 0.0,
    raw_adc_map: dict | None = None,
    temp_graph_renderable: LiveMetricGraph | None = None,
    rth_graph_renderable: LiveMetricGraph | None = None,
) -> Layout:
    """Constructs the Rich renderable tree for display."""
    has_graph = (temp_graph_renderable is not None) or (rth_graph_renderable is not None)
    layout = create_dashboard_layout(show_graph=has_graph)

    def format_temp_str(temp_val: float, raw_adc: int | None) -> str:
        if math.isfinite(temp_val):
            return f"{temp_val:6.2f} °C"
        if raw_adc is not None:
            if raw_adc <= 0:
                return "[dim red]-- (0V/GND)[/dim red]"
            elif raw_adc >= 1023:
                return "[dim yellow]-- (5V/Open)[/dim yellow]"
        return "--"

    # --- Header ---
    header_table = Table.grid(expand=True)
    header_table.add_column(justify="left", ratio=2)
    header_table.add_column(justify="center", ratio=2)
    header_table.add_column(justify="right", ratio=2)
    header_table.add_row(
        Text("⚡ COOLING SLEEVE TEST JIG", style="bold cyan"),
        Text(f"Device: {device_info}", style="bold green"),
        Text(f"Elapsed: {elapsed_s:06.1f} s", style="bold yellow"),
    )
    layout["header"].update(Panel(header_table, style="blue"))

    # --- Coolant & Hydraulic Circuit Panel ---
    c_table = Table(box=None, expand=True, show_header=False)
    c_table.add_column("Metric", style="dim cyan", ratio=3)
    c_table.add_column("Value", style="bold white", justify="right", ratio=3)

    win_adc = raw_adc_map.get("water_in") if raw_adc_map else None
    wout_adc = raw_adc_map.get("water_out") if raw_adc_map else None

    c_table.add_row("Coolant Inlet Temp", format_temp_str(water_in_c, win_adc))
    c_table.add_row("Coolant Outlet Temp", format_temp_str(water_out_c, wout_adc))
    c_table.add_row("Coolant Delta-T (ΔT)", f"{delta_water_c:+6.2f} °C" if math.isfinite(delta_water_c) else "--")
    c_table.add_section()
    c_table.add_row("Flow Meter Frequency", f"{flow_hz:6.1f} Hz" if math.isfinite(flow_hz) else "--")
    c_table.add_row("Coolant Flow Rate", f"{flow_lpm:6.2f} LPM" if math.isfinite(flow_lpm) else "--")
    c_table.add_section()
    c_table.add_row("Heat Absorbed (Raw)", f"{watts_raw:7.1f} W" if math.isfinite(watts_raw) else "--")
    c_table.add_row("Heat Absorbed (Smoothed)", f"{watts_smooth:7.1f} W" if math.isfinite(watts_smooth) else "--")
    c_table.add_section()
    c_table.add_row("Thermal Resistance (Rth)", f"{rth:8.5f} °C/W" if math.isfinite(rth) else "--")

    layout["coolant_panel"].update(
        Panel(c_table, title="[bold]Hydraulic & Coolant Circuit[/bold]", border_style="cyan")
    )

    # --- Mock Motor Thermal Profile Panel ---
    m_layout = Layout()
    table_size = 7 if has_graph else 8
    summary_size = 5 if has_graph else 6
    m_layout.split_column(
        Layout(name="motor_table", size=table_size),
        Layout(name="motor_summary", size=summary_size),
    )

    # Table of 8 edge thermistors (2 columns of 4 for space efficiency)
    edge_table = Table(box=None, expand=True, pad_edge=False)
    edge_table.add_column("Sensor", style="cyan", ratio=1)
    edge_table.add_column("Temp", justify="right", ratio=2)
    edge_table.add_column("Sensor", style="cyan", ratio=1)
    edge_table.add_column("Temp", justify="right", ratio=2)

    keys = list(motor_temps.keys())
    for i in range(4):
        k1 = keys[i] if i < len(keys) else ""
        v1 = motor_temps.get(k1, math.nan)
        adc1 = raw_adc_map.get(k1) if raw_adc_map else None
        s1 = format_temp_str(v1, adc1)

        k2 = keys[i + 4] if (i + 4) < len(keys) else ""
        v2 = motor_temps.get(k2, math.nan)
        adc2 = raw_adc_map.get(k2) if raw_adc_map else None
        s2 = format_temp_str(v2, adc2)

        edge_table.add_row(k1, s1, k2, s2)

    m_layout["motor_table"].update(edge_table)

    # Rate of temperature change formatting
    if math.isfinite(motor_rate_s):
        sign_s = "+" if motor_rate_s >= 0 else ""
        sign_m = "+" if motor_rate_min >= 0 else ""
        if abs(motor_rate_min) < 0.005:
            rate_s_str = f"[dim]{sign_s}{motor_rate_s:7.4f} °C/s[/dim]"
            rate_min_str = f"[dim]{sign_m}{motor_rate_min:6.3f} °C/min[/dim]"
        elif motor_rate_min > 0:
            rate_s_str = f"[bold yellow]{sign_s}{motor_rate_s:7.4f} °C/s[/bold yellow]"
            rate_min_str = f"[bold yellow]{sign_m}{motor_rate_min:6.3f} °C/min[/bold yellow]"
        else:
            rate_s_str = f"[bold cyan]{sign_s}{motor_rate_s:7.4f} °C/s[/bold cyan]"
            rate_min_str = f"[bold cyan]{sign_m}{motor_rate_min:6.3f} °C/min[/bold cyan]"
    else:
        rate_s_str = "--"
        rate_min_str = "--"

    # Summary statistics
    stats_table = Table.grid(expand=True)
    stats_table.add_column("Label", style="dim magenta", ratio=2)
    stats_table.add_column("Val", style="bold white", justify="right", ratio=2)
    stats_table.add_column("Label2", style="dim magenta", ratio=2)
    stats_table.add_column("Val2", style="bold white", justify="right", ratio=2)

    offset_str = f"-{edge_offset_deg:4.2f} °C" if (edge_offset_deg > 0.0001) else "0.00 °C"
    stats_table.add_row(
        "Average Edge:", f"{motor_avg:6.2f} °C" if math.isfinite(motor_avg) else "--",
        "Hotspot (Max):", f"{motor_max:6.2f} °C [{hotspot_name}]" if math.isfinite(motor_max) else "--",
    )
    stats_table.add_row(
        "Minimum Edge:", f"{motor_min:6.2f} °C" if math.isfinite(motor_min) else "--",
        "Edge Spread (Δ):", f"{motor_spread:6.2f} °C" if math.isfinite(motor_spread) else "--",
    )
    stats_table.add_row(
        "Edge Rate (min):", rate_min_str,
        "Al Offset (0.095\"): ", f"[cyan]{offset_str}[/cyan]",
    )

    m_layout["motor_summary"].update(stats_table)

    motor_panel_title = (
        f"[bold]Mock Motor Edge Temperatures (0.095\" Al Offset: -{edge_offset_deg:4.2f} °C)[/bold]"
        if (edge_offset_deg > 0.0001)
        else "[bold]Mock Motor Edge Temperatures (8x NTC)[/bold]"
    )
    layout["motor_panel"].update(
        Panel(m_layout, title=motor_panel_title, border_style="magenta")
    )

    # --- Live Metric Graph Panels (Temp & Rth) ---
    if has_graph:
        if temp_graph_renderable is not None:
            layout["temp_graph_panel"].update(temp_graph_renderable)
        if rth_graph_renderable is not None:
            layout["rth_graph_panel"].update(rth_graph_renderable)

    # --- Footer ---
    footer_text = (
        f" [green]Config:[/green] {CONFIG_NAME}  |  "
        f"[green]Log:[/green] {output_filename}  |  "
        f"[yellow]Samples:[/yellow] {row_count}  |  "
        f"[cyan]Rate:[/cyan] {loop_hz:4.1f} Hz  |  "
        f"[bold red]Ctrl+C to Stop[/bold red]"
    )
    layout["footer"].update(Panel(Text.from_markup(footer_text), style="grey50"))

    return layout


# ==============================================================================
# MAIN APPLICATION LOOP
# ==============================================================================
def main():
    console = Console()
    console.print("\n[bold cyan]=====================================================[/bold cyan]")
    console.print("[bold cyan]    Elegoo Mega 2560 R3 Cooling Sleeve Monitor       [/bold cyan]")
    console.print("[bold cyan]=====================================================[/bold cyan]\n")
    console.print(f"[dim]Loaded configuration from: [bold]{CONFIG_NAME}[/bold] (Default R_fixed = {DEFAULT_R_FIXED:.1f} Ω)[/dim]")
    if OFFSET_ENABLED and R_COND_ALUMINUM > 0:
        console.print(f"[dim]Dynamic Offset: 0.095\" Al 6061-T6 (OD={MOCK_OD_IN}\", L={MOCK_LENGTH_IN}\", R_cond={R_COND_ALUMINUM:.6f} °C/W)[/dim]")

    # Check for --mock simulation mode
    mock_mode = "--mock" in sys.argv

    target_port = None
    if not mock_mode:
        target_port = find_mega_port()
        if not target_port:
            console.print("[bold red][ERROR] No compatible Arduino / Mega 2560 serial port detected.[/bold red]\n")
            console.print("Available serial ports:")
            all_ports = list(serial.tools.list_ports.comports())
            if all_ports:
                for p in all_ports:
                    console.print(f"  - [cyan]{p.device}[/cyan] ({p.description or 'No description'})")
            else:
                console.print("  [dim](No serial devices found. Check USB cable and ensure Elegoo Mega is plugged in)[/dim]")
            console.print(f"\nTip: Specify port explicitly in [bold cyan]{CONFIG_NAME}[/bold cyan], or run with [bold cyan]--mock[/bold cyan] for test mode.\n")
            sys.exit(1)
    else:
        console.print("[bold yellow]*** RUNNING IN MOCK SIMULATION MODE (--mock) ***[/bold yellow]\n")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_filepath = OUTPUT_DIR / f"{CSV_PREFIX}_{timestamp_str}.csv"

    ser = None
    row_count = 0
    csv_file = None

    try:
        if not mock_mode:
            console.print(f"Connecting to Mega 2560 on [cyan]{target_port}[/cyan] at {BAUD_RATE} baud...")
            ser = serial.Serial(port=target_port, baudrate=BAUD_RATE, timeout=SERIAL_TIMEOUT)
            
            # Flush initial bootloader/reset noise
            time.sleep(1.8)
            ser.reset_input_buffer()
            console.print("[bold green]Serial connection established![/bold green]")

            # Pre-flight check: read initial JSON packet from Arduino
            console.print("[dim]Waiting for initial sensor telemetry packet...[/dim]")
            preflight_packet = None
            for _ in range(30):
                raw_line = ser.readline().decode("utf-8", errors="ignore").strip()
                if raw_line.startswith("{") and "a" in raw_line and "flow_count" in raw_line:
                    try:
                        preflight_packet = json.loads(raw_line)
                        break
                    except json.JSONDecodeError:
                        continue

            if not preflight_packet:
                console.print("[bold yellow][WARNING] Did not receive standard JSON packet during startup.[/bold yellow]")
                console.print("Verify that [bold cyan]sleeve_daq/sleeve_daq.ino[/bold cyan] is flashed to the Mega 2560.")
                console.print("Continuing anyway...\n")
            else:
                adc_data = preflight_packet.get("a", [])
                console.print(f"[bold green]Telemetry verified.[/bold green] {len(adc_data)} analog channels reporting.")
                # Print initial temperature check with channel-specific resistor ohm values
                for label, idx in {**WATER_CHANNELS, **MOTOR_CHANNELS}.items():
                    if idx < len(adc_data):
                        raw_adc = adc_data[idx]
                        r_fixed = get_r_fixed(label)
                        beta = COOLANT_BETA if label in WATER_CHANNELS else MOTOR_BETA
                        r0 = COOLANT_R0 if label in WATER_CHANNELS else MOTOR_R0
                        t0 = COOLANT_T0_K if label in WATER_CHANNELS else MOTOR_T0_K
                        temp = adc_to_celsius(raw_adc, r_fixed, r0, t0, beta)
                        if math.isnan(temp):
                            if raw_adc <= 0:
                                status_str = "[bold red]-- (ADC=0: 0V / Missing 5V pull-up or short to GND)[/bold red]"
                            elif raw_adc >= 1023:
                                status_str = "[bold yellow]-- (ADC=1023: 5V / Open circuit or disconnected)[/bold yellow]"
                            else:
                                status_str = f"[bold red]-- (ADC={raw_adc}: Out of range)[/bold red]"
                        else:
                            status_str = f"{temp:5.2f} °C"
                        console.print(f"  - {label:>9} (A{idx}): R_fixed={r_fixed:6.1f} Ω, ADC={raw_adc:4d}  -->  {status_str}")

                initial_count = preflight_packet.get("flow_count", 0)
                console.print(f"  - Initial Flow Pulse Count (Pin 2): {initial_count}")
        else:
            console.print("[bold green]Mock simulated telemetry initialized (10 channels @ 20 Hz).[/bold green]")
            for label, idx in {**WATER_CHANNELS, **MOTOR_CHANNELS}.items():
                r_fixed = get_r_fixed(label)
                console.print(f"  - [dim]{label:>9} (A{idx}): Configured R_fixed = {r_fixed:.1f} Ω[/dim]")

        console.print(f"\n[bold green]Ready. Logging to:[/bold green] {output_filepath}\n")
        time.sleep(1.0)

        # Smoothing buffers
        water_buffers = {k: deque(maxlen=SMOOTH_SAMPLES) for k in WATER_CHANNELS}
        motor_buffers = {k: deque(maxlen=SMOOTH_SAMPLES) for k in MOTOR_CHANNELS}
        watts_buffer = deque(maxlen=WATTS_SMOOTH_SAMPLES)
        flow_buffer = deque(maxlen=FLOW_HISTORY)
        motor_rate_buffer = deque(maxlen=int(TARGET_HZ * 10))

        # Real-time metric graph history buffers (Temperature & Thermal Resistance)
        graph_buffer_len = max(300, int(GRAPH_HISTORY_S * TARGET_HZ * 1.5))
        graph_history_win = deque(maxlen=graph_buffer_len)
        graph_history_wout = deque(maxlen=graph_buffer_len)
        graph_history_mavg = deque(maxlen=graph_buffer_len)
        graph_history_rth = deque(maxlen=graph_buffer_len)
        temp_graph = (
            LiveMetricGraph(
                title="Temperature History",
                unit="°C",
                precision=1,
                time_window_s=GRAPH_HISTORY_S,
                border_style="cyan",
            )
            if GRAPH_ENABLED
            else None
        )
        rth_graph = (
            LiveMetricGraph(
                title="Thermal Resistance (Rth)",
                unit="°C/W",
                precision=4,
                time_window_s=GRAPH_HISTORY_S,
                border_style="magenta",
            )
            if GRAPH_ENABLED
            else None
        )

        flow_lpm = math.nan
        flow_hz = math.nan
        watts_smooth = math.nan
        watts_raw = math.nan
        edge_offset_deg = 0.0
        rth = math.nan

        # Open CSV file and write header
        csv_file = open(output_filepath, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow([
            "timestamp",
            "elapsed_s",
            "water_in_C",
            "water_out_C",
            "delta_water_C",
            "flow_Hz",
            "flow_LPM",
            "watts_raw",
            "watts_smoothed",
            "edge_offset_C",
            "motor_1_C",
            "motor_2_C",
            "motor_3_C",
            "motor_4_C",
            "motor_5_C",
            "motor_6_C",
            "motor_7_C",
            "motor_8_C",
            "motor_avg_C",
            "motor_min_C",
            "motor_max_C",
            "motor_spread_C",
            "motor_rate_C_per_s",
            "motor_rate_C_per_min",
            "hotspot_sensor",
            "thermal_resistance_C_per_W",
        ])
        csv_file.flush()

        start_time = time.monotonic()
        last_flow_time = start_time
        last_count = preflight_packet.get("flow_count", 0) if (not mock_mode and preflight_packet) else 0
        last_sample_time = start_time
        last_dashboard_time = 0.0
        actual_hz = TARGET_HZ

        if not mock_mode and ser is not None:
            ser.reset_input_buffer()

        with Live(console=console, auto_refresh=False, screen=True) as live:
            while True:
                loop_start = time.monotonic()

                if mock_mode:
                    time.sleep(1.0 / TARGET_HZ)
                    mock_t = time.monotonic() - start_time
                    # Simulate realistic thermistor ADC counts:
                    # A0: water_in (~22°C), A1: water_out (~26°C), A2-A9: motor edges (~35-45°C with T3 hotspot)
                    adc_values = [
                        int(520 + 3 * math.sin(mock_t * 0.05)),
                        int(495 + 4 * math.sin(mock_t * 0.05)),
                        int(450 + 6 * math.sin(mock_t * 0.1 + 0.5)),
                        int(440 + 6 * math.sin(mock_t * 0.1 + 1.0)),
                        int(390 + 8 * math.sin(mock_t * 0.1 + 1.5)),  # T3 Hotspot
                        int(430 + 6 * math.sin(mock_t * 0.1 + 2.0)),
                        int(445 + 6 * math.sin(mock_t * 0.1 + 2.5)),
                        int(450 + 6 * math.sin(mock_t * 0.1 + 3.0)),
                        int(438 + 6 * math.sin(mock_t * 0.1 + 3.5)),
                        int(442 + 6 * math.sin(mock_t * 0.1 + 4.0)),
                    ]
                    current_count = int(mock_t * 26.5)  # ~26.5 Hz flow pulses (~8.1 LPM)
                else:
                    # Read line from Arduino Mega serial stream
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if not line or not line.startswith("{"):
                        continue

                    try:
                        packet = json.loads(line)
                    except json.JSONDecodeError:
                        continue

                    adc_values = packet.get("a", [])
                    if not adc_values:
                        continue

                    current_count = packet.get("flow_count", last_count)

                # Capture raw ADC readings for diagnostics
                raw_adc_map = {
                    label: (adc_values[idx] if idx < len(adc_values) else None)
                    for label, idx in {**WATER_CHANNELS, **MOTOR_CHANNELS}.items()
                }

                # Process coolant temperatures with per-channel resistor ohms
                water_temps = {}
                for label, idx in WATER_CHANNELS.items():
                    raw_val = adc_values[idx] if idx < len(adc_values) else 0
                    r_fixed = get_r_fixed(label)
                    c_val = adc_to_celsius(raw_val, r_fixed, COOLANT_R0, COOLANT_T0_K, COOLANT_BETA)
                    water_buffers[label].append(c_val)
                    water_temps[label] = mean_valid(water_buffers[label])

                win = water_temps["water_in"]
                wout = water_temps["water_out"]
                delta_water = wout - win if (math.isfinite(win) and math.isfinite(wout)) else math.nan

                now = time.monotonic()

                # Flow rate calculation window
                flow_dt = now - last_flow_time
                if flow_dt >= FLOW_WINDOW:
                    pulse_delta = max(0, current_count - last_count)
                    flow_hz = pulse_delta / flow_dt
                    flow_buffer.append(flow_hz * LPM_PER_HZ)
                    flow_lpm = mean_valid(flow_buffer)
                    last_count = current_count
                    last_flow_time = now

                # Thermal power absorption calculation:
                # Q = m_dot * Cp * delta_T = (flow_LPM / 60) * rho_water * Cp_water * (T_out - T_in)
                if math.isfinite(flow_lpm) and math.isfinite(delta_water):
                    m_dot = (flow_lpm / 60.0) * RHO_WATER
                    watts_raw = m_dot * CP_WATER * delta_water
                    watts_buffer.append(watts_raw)
                    watts_smooth = mean_valid(watts_buffer)
                else:
                    watts_raw = math.nan
                    watts_smooth = math.nan

                # Dynamic Aluminum Conduction Offset (0.095" from edge):
                # When mock motor transfers heat to cooling sleeve (watts_smooth > 0),
                # thermistors inside the aluminum read hotter than the true outer contact surface.
                if OFFSET_ENABLED and math.isfinite(watts_smooth) and watts_smooth > 0 and R_COND_ALUMINUM > 0:
                    edge_offset_deg = watts_smooth * R_COND_ALUMINUM
                else:
                    edge_offset_deg = 0.0

                # Process motor edge temperatures with per-channel resistor ohms & dynamic conduction offset
                m_temps = {}
                for label, idx in MOTOR_CHANNELS.items():
                    raw_val = adc_values[idx] if idx < len(adc_values) else 0
                    r_fixed = get_r_fixed(label)
                    c_val = adc_to_celsius(raw_val, r_fixed, MOTOR_R0, MOTOR_T0_K, MOTOR_BETA)
                    motor_buffers[label].append(c_val)
                    c_meas = mean_valid(motor_buffers[label])
                    m_temps[label] = (c_meas - edge_offset_deg) if math.isfinite(c_meas) else math.nan

                valid_m = [v for v in m_temps.values() if math.isfinite(v)]
                if valid_m:
                    m_avg = sum(valid_m) / len(valid_m)
                    m_min = min(valid_m)
                    m_max = max(valid_m)
                    m_spread = m_max - m_min
                    hotspot_name = max(m_temps.items(), key=lambda item: item[1] if math.isfinite(item[1]) else -999)[0]
                else:
                    m_avg = m_min = m_max = m_spread = math.nan
                    hotspot_name = "N/A"

                # Calculate motor edge rate of temperature change (dT/dt)
                if math.isfinite(m_avg):
                    motor_rate_buffer.append((now, m_avg))
                motor_rate_s, motor_rate_min = calculate_rate_of_change(motor_rate_buffer, RATE_WINDOW_S)

                # Overall Thermal Resistance: Rth = (T_motor_avg - T_water_in) / Q
                if math.isfinite(m_avg) and math.isfinite(win) and math.isfinite(watts_smooth) and watts_smooth > 1.0:
                    rth = (m_avg - win) / watts_smooth
                else:
                    rth = math.nan

                # Record telemetry history points for real-time graphs
                if GRAPH_ENABLED:
                    if math.isfinite(win):
                        graph_history_win.append((now, win))
                    if math.isfinite(wout):
                        graph_history_wout.append((now, wout))
                    if math.isfinite(m_avg):
                        graph_history_mavg.append((now, m_avg))
                    if math.isfinite(rth):
                        graph_history_rth.append((now, rth))

                elapsed = now - start_time
                row_count += 1

                # Write to CSV
                writer.writerow([
                    datetime.datetime.now().isoformat(),
                    f"{elapsed:.3f}",
                    f"{win:.3f}",
                    f"{wout:.3f}",
                    f"{delta_water:.3f}",
                    f"{flow_hz:.2f}",
                    f"{flow_lpm:.3f}",
                    f"{watts_raw:.2f}",
                    f"{watts_smooth:.2f}",
                    f"{edge_offset_deg:.4f}",
                    f"{m_temps.get('T1', math.nan):.3f}",
                    f"{m_temps.get('T2', math.nan):.3f}",
                    f"{m_temps.get('T3', math.nan):.3f}",
                    f"{m_temps.get('T4', math.nan):.3f}",
                    f"{m_temps.get('T5', math.nan):.3f}",
                    f"{m_temps.get('T6', math.nan):.3f}",
                    f"{m_temps.get('T7', math.nan):.3f}",
                    f"{m_temps.get('T8', math.nan):.3f}",
                    f"{m_avg:.3f}",
                    f"{m_min:.3f}",
                    f"{m_max:.3f}",
                    f"{m_spread:.3f}",
                    f"{motor_rate_s:.4f}" if math.isfinite(motor_rate_s) else "nan",
                    f"{motor_rate_min:.3f}" if math.isfinite(motor_rate_min) else "nan",
                    hotspot_name,
                    f"{rth:.6f}",
                ])
                csv_file.flush()

                # Refresh dashboard at DASHBOARD_REFRESH_HZ rate
                if (now - last_dashboard_time) >= (1.0 / DASHBOARD_REFRESH_HZ):
                    if GRAPH_ENABLED:
                        if temp_graph is not None:
                            temp_graph.set_data([
                                {"name": "Water In", "color": "cyan", "data": list(graph_history_win), "current": win},
                                {"name": "Water Out", "color": "bright_blue", "data": list(graph_history_wout), "current": wout},
                                {"name": "Motor Avg", "color": "bright_yellow", "data": list(graph_history_mavg), "current": m_avg},
                            ])
                        if rth_graph is not None:
                            rth_graph.set_data([
                                {"name": "Rth", "color": "bright_magenta", "data": list(graph_history_rth), "current": rth},
                            ])

                    dev_label = "Simulated Mock" if mock_mode else f"Mega 2560 ({Path(target_port).name})"
                    dashboard_renderable = build_dashboard(
                        elapsed_s=elapsed,
                        device_info=dev_label,
                        output_filename=output_filepath.name,
                        row_count=row_count,
                        loop_hz=actual_hz,
                        water_in_c=win,
                        water_out_c=wout,
                        delta_water_c=delta_water,
                        flow_hz=flow_hz,
                        flow_lpm=flow_lpm,
                        watts_raw=watts_raw,
                        watts_smooth=watts_smooth,
                        motor_temps=m_temps,
                        motor_avg=m_avg,
                        motor_min=m_min,
                        motor_max=m_max,
                        motor_spread=m_spread,
                        motor_rate_s=motor_rate_s,
                        motor_rate_min=motor_rate_min,
                        hotspot_name=hotspot_name,
                        rth=rth,
                        edge_offset_deg=edge_offset_deg,
                        raw_adc_map=raw_adc_map,
                        temp_graph_renderable=temp_graph,
                        rth_graph_renderable=rth_graph,
                    )
                    live.update(dashboard_renderable)
                    live.refresh()
                    last_dashboard_time = now

                # Telemetry sample rate tracking
                dt_sample = now - last_sample_time
                if dt_sample > 0.001:
                    actual_hz = 0.9 * actual_hz + 0.1 * (1.0 / dt_sample)
                    last_sample_time = now

    except KeyboardInterrupt:
        console.print("\n[bold yellow]Stopping test (Ctrl+C received)...[/bold yellow]")
    except Exception:
        console.print("\n[bold red]ERROR ENCOUNTERED:[/bold red]")
        traceback.print_exc()
    finally:
        # Clean up serial connection
        if ser is not None and ser.is_open:
            try:
                ser.close()
                console.print("[dim]Serial connection closed cleanly.[/dim]")
            except Exception:
                pass

        if csv_file is not None and not csv_file.closed:
            csv_file.close()

        if row_count > 0:
            console.print("\n[bold green]=====================================================[/bold green]")
            console.print(" [bold green]Test Run Complete![/bold green]")
            console.print(f" Total Rows Recorded: [cyan]{row_count}[/cyan]")
            console.print(f" Saved Data File:     [bold underline]{output_filepath}[/bold underline]")
            console.print("[bold green]=====================================================[/bold green]\n")


if __name__ == "__main__":
    main()
