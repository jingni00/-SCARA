import math
import csv
import re
import sys
from datetime import datetime

from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


BAUD_RATES = (115200, 57600, 38400, 19200, 9600)
POS_RE = re.compile(
    r"POS\s+X(?P<x>[-+]?\d+(?:\.\d+)?)\s+Y(?P<y>[-+]?\d+(?:\.\d+)?)"
    r"\s+M1(?P<m1>[-+]?\d+)\s+M2(?P<m2>[-+]?\d+)\s+(?P<state>\w+)"
    r"(?:\s+P(?P<pen>[01]))?"
)
POS_NO_FLOAT_RE = re.compile(
    r"POS\s+X\s+Y\s+M1(?P<m1>[-+]?\d+)\s+M2(?P<m2>[-+]?\d+)\s+(?P<state>\w+)"
    r"(?:\s+P(?P<pen>[01]))?"
)
SW_RE = re.compile(r"(?:EV\s+)?SW\s+A(?P<a>[01])\s+B(?P<b>[01])")


class WorkspaceView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.current = QPointF(0.0, 314.94)
        self.target = None
        self.path = []
        self.preview_path = []
        self.setAutoFillBackground(False)

    def set_current(self, x, y):
        self.current = QPointF(float(x), float(y))
        self.update()

    def set_target(self, x, y):
        self.target = QPointF(float(x), float(y))
        self.update()

    def add_path_point(self, x, y):
        point = QPointF(float(x), float(y))
        if not self.path or (self.path[-1] - point).manhattanLength() > 0.2:
            self.path.append(point)
            if len(self.path) > 500:
                self.path = self.path[-500:]
        self.update()

    def clear_path(self):
        self.path.clear()
        self.preview_path.clear()
        self.target = None
        self.update()

    def set_preview_path(self, points):
        self.preview_path = [
            None if point is None else QPointF(float(point[0]), float(point[1]))
            for point in points
        ]
        self.update()

    def _to_screen(self, point):
        margin = 28.0
        usable_w = max(1.0, self.width() - 2.0 * margin)
        usable_h = max(1.0, self.height() - 2.0 * margin)
        scale = min(usable_w / 420.0, usable_h / 360.0)
        cx = self.width() * 0.5
        base_y = self.height() - margin - 20.0
        return QPointF(cx + point.x() * scale, base_y - point.y() * scale)

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#f7f8fa"))

        margin_pen = QPen(QColor("#d8dde6"), 1)
        axis_pen = QPen(QColor("#9098a8"), 1)
        path_pen = QPen(QColor("#2d7dd2"), 2)
        preview_pen = QPen(QColor("#ef8354"), 1, Qt.DashLine)
        target_pen = QPen(QColor("#ef8354"), 2)
        robot_pen = QPen(QColor("#1f2937"), 2)

        painter.setPen(margin_pen)
        for x in range(-200, 201, 50):
            p1 = self._to_screen(QPointF(x, 80))
            p2 = self._to_screen(QPointF(x, 330))
            painter.drawLine(p1, p2)
        for y in range(100, 331, 50):
            p1 = self._to_screen(QPointF(-210, y))
            p2 = self._to_screen(QPointF(210, y))
            painter.drawLine(p1, p2)

        painter.setPen(axis_pen)
        painter.drawLine(self._to_screen(QPointF(-230, 0)), self._to_screen(QPointF(230, 0)))
        painter.drawLine(self._to_screen(QPointF(0, 60)), self._to_screen(QPointF(0, 340)))

        left_motor = self._to_screen(QPointF(-80, 0))
        right_motor = self._to_screen(QPointF(80, 0))
        current = self._to_screen(self.current)
        painter.setPen(robot_pen)
        painter.drawEllipse(left_motor, 4, 4)
        painter.drawEllipse(right_motor, 4, 4)
        painter.drawLine(left_motor, current)
        painter.drawLine(right_motor, current)

        if len(self.path) > 1:
            painter.setPen(path_pen)
            for a, b in zip(self.path, self.path[1:]):
                painter.drawLine(self._to_screen(a), self._to_screen(b))

        if len(self.preview_path) > 1:
            painter.setPen(preview_pen)
            for a, b in zip(self.preview_path, self.preview_path[1:]):
                if a is None or b is None:
                    continue
                painter.drawLine(self._to_screen(a), self._to_screen(b))

        if self.target is not None:
            target = self._to_screen(self.target)
            painter.setPen(target_pen)
            painter.drawLine(QPointF(target.x() - 7.0, target.y()), QPointF(target.x() + 7.0, target.y()))
            painter.drawLine(QPointF(target.x(), target.y() - 7.0), QPointF(target.x(), target.y() + 7.0))

        painter.setBrush(QColor("#111827"))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(current, 5, 5)
        painter.end()


class ScaraHost(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("并联 SCARA 机器人上位机")
        self.resize(1120, 720)

        self.serial = QSerialPort(self)
        self.serial.readyRead.connect(self.on_ready_read)
        self.rx_buffer = bytearray()
        self.last_x = 0.0
        self.last_y = 314.94
        self.command_queue = []
        self.queue_running = False
        self.teach_points = []

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(500)
        self.status_timer.timeout.connect(self.poll_status)

        self.build_ui()
        self.refresh_ports()
        self.apply_style()

    def build_ui(self):
        root = QWidget()
        layout = QHBoxLayout(root)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.build_left_panel())
        splitter.addWidget(self.build_right_panel())
        splitter.addWidget(self.build_motion_panel())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 0)
        layout.addWidget(splitter)
        self.setCentralWidget(root)

    def build_left_panel(self):
        panel = QWidget()
        panel.setMinimumWidth(250)
        panel.setMaximumWidth(310)
        layout = QVBoxLayout(panel)
        layout.setSpacing(10)

        conn = QGroupBox("串口连接")
        conn_layout = QGridLayout(conn)
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(str(rate) for rate in BAUD_RATES)
        self.refresh_btn = QPushButton("刷新")
        self.connect_btn = QPushButton("连接")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(QLabel("串口"), 0, 0)
        conn_layout.addWidget(self.port_combo, 0, 1)
        conn_layout.addWidget(QLabel("波特率"), 1, 0)
        conn_layout.addWidget(self.baud_combo, 1, 1)
        conn_layout.addWidget(self.refresh_btn, 2, 0)
        conn_layout.addWidget(self.connect_btn, 2, 1)
        layout.addWidget(conn)

        status = QGroupBox("状态")
        status_layout = QFormLayout(status)
        self.state_label = QLabel("未连接")
        self.xy_label = QLabel("X0.000  Y314.940")
        self.motor_label = QLabel("M1 0  M2 0")
        self.pen_label = QLabel("抬笔")
        self.auto_poll = QCheckBox("自动查询")
        self.auto_poll.setChecked(True)
        self.auto_poll.toggled.connect(self.on_auto_poll)
        status_layout.addRow("状态", self.state_label)
        status_layout.addRow("坐标", self.xy_label)
        status_layout.addRow("电机", self.motor_label)
        status_layout.addRow("舵机", self.pen_label)
        status_layout.addRow(self.auto_poll)
        layout.addWidget(status)

        safety = QGroupBox("机器控制")
        safety_layout = QGridLayout(safety)
        self.enable_btn = QPushButton("使能")
        self.disable_btn = QPushButton("失能")
        self.stop_btn = QPushButton("急停")
        self.home_btn = QPushButton("设为原点")
        self.pen_up_btn = QPushButton("抬笔")
        self.pen_down_btn = QPushButton("下笔")
        self.enable_btn.clicked.connect(lambda: self.send_command("M17"))
        self.disable_btn.clicked.connect(lambda: self.send_immediate_command("M18"))
        self.stop_btn.clicked.connect(lambda: self.send_immediate_command("!"))
        self.home_btn.clicked.connect(lambda: self.send_command("HOME"))
        self.pen_up_btn.clicked.connect(lambda: self.send_command("P0"))
        self.pen_down_btn.clicked.connect(lambda: self.send_command("P1"))
        safety_layout.addWidget(self.enable_btn, 0, 0)
        safety_layout.addWidget(self.disable_btn, 0, 1)
        safety_layout.addWidget(self.home_btn, 1, 0)
        safety_layout.addWidget(self.stop_btn, 1, 1)
        safety_layout.addWidget(self.pen_up_btn, 2, 0)
        safety_layout.addWidget(self.pen_down_btn, 2, 1)
        layout.addWidget(safety)

        setup = QGroupBox("参数设置")
        setup_layout = QFormLayout(setup)
        self.ppr_spin = QSpinBox()
        self.ppr_spin.setRange(1, 200000)
        self.ppr_spin.setValue(6124)
        self.zero1_spin = QSpinBox()
        self.zero2_spin = QSpinBox()
        for spin in (self.zero1_spin, self.zero2_spin):
            spin.setRange(-2000000000, 2000000000)
        self.set_ppr_btn = QPushButton("设置每圈脉冲")
        self.set_zero_btn = QPushButton("设置零位")
        self.set_ppr_btn.clicked.connect(self.set_ppr)
        self.set_zero_btn.clicked.connect(
            lambda: self.send_command(f"SZ M1{self.zero1_spin.value()} M2{self.zero2_spin.value()}")
        )
        setup_layout.addRow("PPR", self.ppr_spin)
        setup_layout.addRow(self.set_ppr_btn)
        setup_layout.addRow("零位 M1", self.zero1_spin)
        setup_layout.addRow("零位 M2", self.zero2_spin)
        setup_layout.addRow(self.set_zero_btn)
        layout.addWidget(setup)

        layout.addStretch(1)
        return panel

    def build_motion_panel(self):
        panel = QWidget()
        panel.setMinimumWidth(340)
        panel.setMaximumWidth(430)
        layout = QVBoxLayout(panel)
        layout.setSpacing(10)

        tabs = QTabWidget()
        motion_tab = QWidget()
        draw_tab = QWidget()
        text_tab = QWidget()
        teach_tab = QWidget()
        setup_tab = QWidget()
        motion_tab_layout = QVBoxLayout(motion_tab)
        draw_tab_layout = QVBoxLayout(draw_tab)
        text_tab_layout = QVBoxLayout(text_tab)
        teach_tab_layout = QVBoxLayout(teach_tab)
        setup_tab_layout = QVBoxLayout(setup_tab)
        for tab_layout in (motion_tab_layout, draw_tab_layout, text_tab_layout, teach_tab_layout, setup_tab_layout):
            tab_layout.setSpacing(10)

        common = QGroupBox("运动参数")
        common_layout = QFormLayout(common)
        self.feed_spin = self.make_double_spin(0.1, 1000.0, 12.0, " mm/s")
        self.accel_spin = self.make_double_spin(1.0, 10000.0, 35.0, " mm/s²")
        self.jog_spin = self.make_double_spin(0.01, 100.0, 5.0, " mm")
        self.profile_combo = QComboBox()
        self.profile_combo.addItem("梯形速度", 0)
        self.profile_combo.addItem("S型速度", 1)
        self.profile_combo.setCurrentIndex(1)
        common_layout.addRow("速度", self.feed_spin)
        common_layout.addRow("加速度", self.accel_spin)
        common_layout.addRow("点动步长", self.jog_spin)
        common_layout.addRow("速度曲线", self.profile_combo)
        motion_tab_layout.addWidget(common)

        jog = QGroupBox("点动")
        jog_layout = QGridLayout(jog)
        jog_buttons = [
            ("Y+", 0, 1, lambda: self.jog("Y", +1.0)),
            ("X-", 1, 0, lambda: self.jog("X", -1.0)),
            ("X+", 1, 2, lambda: self.jog("X", +1.0)),
            ("Y-", 2, 1, lambda: self.jog("Y", -1.0)),
        ]
        for text, row, col, slot in jog_buttons:
            button = QPushButton(text)
            button.setMinimumHeight(42)
            button.clicked.connect(slot)
            jog_layout.addWidget(button, row, col)
        motion_tab_layout.addWidget(jog)

        line = QGroupBox("直线 G1")
        line_layout = QGridLayout(line)
        self.line_x_spin = self.make_double_spin(-300.0, 300.0, 0.0, " mm")
        self.line_y_spin = self.make_double_spin(0.0, 330.0, 314.94, " mm")
        self.line_current_btn = QPushButton("填入当前位置")
        self.goto_btn = QPushButton("执行直线")
        self.line_current_btn.clicked.connect(self.fill_line_from_current)
        self.goto_btn.clicked.connect(self.move_line)
        line_layout.addWidget(QLabel("X"), 0, 0)
        line_layout.addWidget(self.line_x_spin, 0, 1)
        line_layout.addWidget(QLabel("Y"), 1, 0)
        line_layout.addWidget(self.line_y_spin, 1, 1)
        line_layout.addWidget(self.line_current_btn, 2, 0, 1, 2)
        line_layout.addWidget(self.goto_btn, 3, 0, 1, 2)
        motion_tab_layout.addWidget(line)

        arc = QGroupBox("圆弧 G2/G3")
        arc_layout = QGridLayout(arc)
        self.arc_x_spin = self.make_double_spin(-300.0, 300.0, 0.0, " mm")
        self.arc_y_spin = self.make_double_spin(0.0, 330.0, 314.94, " mm")
        self.arc_i_spin = self.make_double_spin(-300.0, 300.0, 30.0, " mm")
        self.arc_j_spin = self.make_double_spin(-300.0, 300.0, 0.0, " mm")
        self.g2_btn = QPushButton("顺时针 G2")
        self.g3_btn = QPushButton("逆时针 G3")
        self.g2_btn.clicked.connect(lambda: self.move_arc(cw=True))
        self.g3_btn.clicked.connect(lambda: self.move_arc(cw=False))
        arc_layout.addWidget(QLabel("终点 X"), 0, 0)
        arc_layout.addWidget(self.arc_x_spin, 0, 1)
        arc_layout.addWidget(QLabel("终点 Y"), 1, 0)
        arc_layout.addWidget(self.arc_y_spin, 1, 1)
        arc_layout.addWidget(QLabel("I"), 2, 0)
        arc_layout.addWidget(self.arc_i_spin, 2, 1)
        arc_layout.addWidget(QLabel("J"), 3, 0)
        arc_layout.addWidget(self.arc_j_spin, 3, 1)
        arc_layout.addWidget(self.g2_btn, 4, 0)
        arc_layout.addWidget(self.g3_btn, 4, 1)
        motion_tab_layout.addWidget(arc)

        draw = QGroupBox("绘图轨迹")
        draw_layout = QGridLayout(draw)
        self.draw_pattern_combo = QComboBox()
        self.draw_pattern_combo.addItem("考核图案", 1)
        self.draw_pattern_combo.addItem("五角星", 2)
        self.draw_pattern_combo.addItem("爱心", 3)
        self.draw_pattern_combo.addItem("花形", 4)
        self.draw_pattern_combo.currentIndexChanged.connect(self.update_draw_preview)
        self.draw_start_btn = QPushButton("移动到绘图起点")
        self.draw_path_btn = QPushButton("执行绘图轨迹")
        self.draw_path_btn.setText("S型绘图")
        self.draw_s_path_btn = QPushButton("梯形绘图")
        self.draw_start_btn.clicked.connect(self.move_to_draw_start)
        self.draw_path_btn.clicked.connect(lambda: self.draw_selected_path(profile=1))
        self.draw_s_path_btn.clicked.connect(lambda: self.draw_selected_path(profile=0))
        draw_layout.addWidget(QLabel("图案"), 0, 0)
        draw_layout.addWidget(self.draw_pattern_combo, 0, 1)
        draw_layout.addWidget(self.draw_start_btn, 1, 0, 1, 2)
        draw_layout.addWidget(self.draw_path_btn, 2, 0)
        draw_layout.addWidget(self.draw_s_path_btn, 2, 1)
        draw_tab_layout.addWidget(draw)

        text_draw = QGroupBox("写字")
        text_layout = QGridLayout(text_draw)
        self.text_line = QLineEdit()
        self.text_line.setPlaceholderText("HELLO 123")
        self.text_x_spin = self.make_double_spin(-120.0, 120.0, -45.0, " mm")
        self.text_y_spin = self.make_double_spin(120.0, 300.0, 245.0, " mm")
        self.text_size_spin = self.make_double_spin(8.0, 80.0, 32.0, " mm")
        self.text_auto_fit_check = QCheckBox("安全居中")
        self.text_auto_fit_check.setChecked(True)
        self.text_preview_btn = QPushButton("预览文字")
        self.text_write_btn = QPushButton("开始写字")
        self.text_preview_btn.clicked.connect(self.preview_text)
        self.text_write_btn.clicked.connect(self.write_text)
        text_layout.addWidget(QLabel("内容"), 0, 0)
        text_layout.addWidget(self.text_line, 0, 1)
        text_layout.addWidget(QLabel("左 X"), 1, 0)
        text_layout.addWidget(self.text_x_spin, 1, 1)
        text_layout.addWidget(QLabel("上 Y"), 2, 0)
        text_layout.addWidget(self.text_y_spin, 2, 1)
        text_layout.addWidget(QLabel("字号"), 3, 0)
        text_layout.addWidget(self.text_size_spin, 3, 1)
        text_layout.addWidget(self.text_auto_fit_check, 4, 0, 1, 2)
        text_layout.addWidget(self.text_preview_btn, 5, 0)
        text_layout.addWidget(self.text_write_btn, 5, 1)
        text_tab_layout.addWidget(text_draw)

        teach = QGroupBox("示教轨迹")
        teach_layout = QGridLayout(teach)
        self.teach_count_label = QLabel("点数 0")
        self.teach_record_btn = QPushButton("记录当前点")
        self.teach_undo_btn = QPushButton("删除最后点")
        self.teach_clear_btn = QPushButton("清空")
        self.teach_preview_btn = QPushButton("预览")
        self.teach_replay_btn = QPushButton("复现轨迹")
        self.teach_save_btn = QPushButton("保存")
        self.teach_load_btn = QPushButton("载入")
        self.teach_record_btn.clicked.connect(self.teach_record_point)
        self.teach_undo_btn.clicked.connect(self.teach_undo_point)
        self.teach_clear_btn.clicked.connect(self.teach_clear_points)
        self.teach_preview_btn.clicked.connect(self.teach_preview_path)
        self.teach_replay_btn.clicked.connect(self.teach_replay_path)
        self.teach_save_btn.clicked.connect(self.teach_save_points)
        self.teach_load_btn.clicked.connect(self.teach_load_points)
        teach_layout.addWidget(self.teach_count_label, 0, 0, 1, 2)
        teach_layout.addWidget(self.teach_record_btn, 1, 0)
        teach_layout.addWidget(self.teach_undo_btn, 1, 1)
        teach_layout.addWidget(self.teach_preview_btn, 2, 0)
        teach_layout.addWidget(self.teach_replay_btn, 2, 1)
        teach_layout.addWidget(self.teach_save_btn, 3, 0)
        teach_layout.addWidget(self.teach_load_btn, 3, 1)
        teach_layout.addWidget(self.teach_clear_btn, 4, 0, 1, 2)
        teach_tab_layout.addWidget(teach)

        set_pos = QGroupBox("设置当前坐标")
        set_layout = QGridLayout(set_pos)
        self.set_x_spin = self.make_double_spin(-300.0, 300.0, 0.0, " mm")
        self.set_y_spin = self.make_double_spin(0.0, 330.0, 314.94, " mm")
        self.set_xy_btn = QPushButton("设置坐标")
        self.set_xy_btn.clicked.connect(self.set_xy)
        set_layout.addWidget(QLabel("X"), 0, 0)
        set_layout.addWidget(self.set_x_spin, 0, 1)
        set_layout.addWidget(QLabel("Y"), 1, 0)
        set_layout.addWidget(self.set_y_spin, 1, 1)
        set_layout.addWidget(self.set_xy_btn, 2, 0, 1, 2)
        setup_tab_layout.addWidget(set_pos)

        manual = QGroupBox("手动命令")
        manual_layout = QHBoxLayout(manual)
        self.manual_line = QLineEdit()
        self.manual_line.setPlaceholderText("G1 X0 Y314.94 F12 A35 C1")
        self.manual_line.returnPressed.connect(self.send_manual)
        self.manual_btn = QPushButton("发送")
        self.manual_btn.clicked.connect(self.send_manual)
        manual_layout.addWidget(self.manual_line)
        manual_layout.addWidget(self.manual_btn)
        setup_tab_layout.addWidget(manual)

        motion_tab_layout.addStretch(1)
        draw_tab_layout.addStretch(1)
        text_tab_layout.addStretch(1)
        teach_tab_layout.addStretch(1)
        setup_tab_layout.addStretch(1)
        tabs.addTab(motion_tab, "运动")
        tabs.addTab(draw_tab, "绘图")
        tabs.addTab(text_tab, "写字")
        tabs.addTab(teach_tab, "示教")
        tabs.addTab(setup_tab, "设置")
        layout.addWidget(tabs)
        return panel

    def build_right_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setSpacing(10)

        self.workspace = WorkspaceView()
        layout.addWidget(self.workspace, stretch=5)

        log_box = QGroupBox("串口日志")
        log_layout = QVBoxLayout(log_box)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        clear_btn = QPushButton("清空日志")
        clear_btn.clicked.connect(self.log.clear)
        log_layout.addWidget(self.log)
        log_layout.addWidget(clear_btn)
        layout.addWidget(log_box, stretch=2)
        return panel

    def make_double_spin(self, minimum, maximum, value, suffix):
        spin = QDoubleSpinBox()
        spin.setDecimals(3)
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setSingleStep(1.0)
        return spin

    def apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow { background: #eceff4; }
            QGroupBox {
                border: 1px solid #c9ced8;
                border-radius: 6px;
                margin-top: 8px;
                padding: 8px;
                background: #ffffff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                color: #293241;
                font-weight: 600;
            }
            QPushButton {
                min-height: 30px;
                border-radius: 4px;
                border: 1px solid #aeb6c4;
                background: #f6f7f9;
                color: #1f2937;
            }
            QPushButton:hover { background: #e9eef6; }
            QPushButton:pressed { background: #d8e1ee; }
            QPushButton:disabled { color: #8c94a3; background: #f0f1f4; }
            QTabWidget::pane {
                border: 1px solid #c9ced8;
                border-radius: 6px;
                background: #ffffff;
                top: -1px;
            }
            QTabBar::tab {
                min-width: 54px;
                min-height: 28px;
                padding: 4px 8px;
                border: 1px solid #c9ced8;
                border-bottom: none;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
                background: #eef2f7;
                color: #374151;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                color: #1f2937;
                font-weight: 600;
            }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
                min-height: 28px;
                border: 1px solid #bcc4d0;
                border-radius: 4px;
                padding: 2px 6px;
                background: #ffffff;
            }
            QPlainTextEdit {
                border: 1px solid #c9ced8;
                border-radius: 4px;
                background: #fbfcfe;
                font-family: Consolas, monospace;
                font-size: 10pt;
            }
            """
        )
        self.stop_btn.setStyleSheet(
            "QPushButton { background:#e63946; color:white; border-color:#b92532; font-weight:700; }"
            "QPushButton:hover { background:#d62839; }"
        )

    def refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = QSerialPortInfo.availablePorts()
        for port in ports:
            label = port.portName()
            if port.description():
                label = f"{port.portName()} - {port.description()}"
            self.port_combo.addItem(label, port.portName())

        if not ports:
            self.port_combo.addItem("未发现串口", "")
        elif current:
            index = self.port_combo.findText(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def toggle_connection(self):
        if self.serial.isOpen():
            self.status_timer.stop()
            self.serial.close()
            self.connect_btn.setText("连接")
            self.state_label.setText("未连接")
            self.log_line("已断开连接")
            return

        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "串口", "没有选择可用串口。")
            return

        self.serial.setPortName(port)
        self.serial.setBaudRate(int(self.baud_combo.currentText()))
        self.serial.setDataBits(QSerialPort.Data8)
        self.serial.setParity(QSerialPort.NoParity)
        self.serial.setStopBits(QSerialPort.OneStop)
        self.serial.setFlowControl(QSerialPort.NoFlowControl)

        if not self.serial.open(QSerialPort.ReadWrite):
            QMessageBox.warning(self, "串口", self.serial.errorString())
            return

        self.connect_btn.setText("断开")
        self.state_label.setText("已连接")
        self.log_line(f"已连接 {port} @ {self.baud_combo.currentText()}")
        self.send_command("?")
        if self.auto_poll.isChecked():
            self.status_timer.start()

    def on_auto_poll(self, checked):
        if checked and self.serial.isOpen():
            self.status_timer.start()
        else:
            self.status_timer.stop()

    def poll_status(self):
        if self.serial.isOpen() and not self.queue_running:
            self.send_command("?", log_tx=False)

    def send_manual(self):
        text = self.manual_line.text().strip()
        if text:
            self.send_command(text)
            self.manual_line.clear()

    def send_immediate_command(self, command):
        self.command_queue.clear()
        self.queue_running = False
        if command.strip() == "!":
            self.send_estop()
            return
        self.send_command(command)

    def set_ppr(self):
        self.send_command(f"PPR N{self.ppr_spin.value()}")
        self.send_command("?", log_tx=False)

    def send_estop(self):
        if not self.serial.isOpen():
            self.log_line("急停失败，串口未连接")
            return
        self.serial.write(b"!\r\n")
        self.serial.flush()
        self.state_label.setText("急停")
        self.log_line("> !")

    def feed_text(self):
        profile = int(self.profile_combo.currentData())
        return f"F{self.feed_spin.value():.3f} A{self.accel_spin.value():.3f} C{profile}"

    def jog(self, axis, direction):
        sign = "+" if direction > 0.0 else "-"
        self.send_command(f"J {axis}{sign} {self.jog_spin.value():.3f} {self.feed_text()}")

    def move_line(self):
        x = self.line_x_spin.value()
        y = self.line_y_spin.value()
        self.workspace.set_target(x, y)
        self.send_command(f"G1 X{x:.3f} Y{y:.3f} {self.feed_text()}")

    def fill_line_from_current(self):
        self.line_x_spin.setValue(self.last_x)
        self.line_y_spin.setValue(self.last_y)

    def move_arc(self, cw):
        x = self.arc_x_spin.value()
        y = self.arc_y_spin.value()
        i = self.arc_i_spin.value()
        j = self.arc_j_spin.value()
        self.workspace.set_target(x, y)
        code = "G2" if cw else "G3"
        self.send_command(f"{code} X{x:.3f} Y{y:.3f} I{i:.3f} J{j:.3f} {self.feed_text()}")

    def move_to_draw_start(self):
        pattern_id = self.selected_draw_pattern()
        x, y = self.draw_pattern_start(pattern_id)
        self.workspace.set_target(x, y)
        self.workspace.set_preview_path(self.draw_path_preview_points(pattern_id))
        self.send_command(f"G1 X{x:.3f} Y{y:.3f} {self.feed_text()}")

    def draw_path_1(self, profile=0):
        self.draw_selected_path(profile)

    def draw_selected_path(self, profile=0):
        pattern_id = self.selected_draw_pattern()
        start_x, start_y = self.draw_pattern_start(pattern_id)
        end_x, end_y = self.draw_pattern_end(pattern_id)
        self.workspace.clear_path()
        self.workspace.set_preview_path(self.draw_path_preview_points(pattern_id))
        self.workspace.set_target(end_x, end_y)
        commands = [
            "P0",
            f"G1 X{start_x:.3f} Y{start_y:.3f} {self.feed_text()}",
            "P1",
            "@WAIT 500",
            (
                f"DRAW{pattern_id} F{self.feed_spin.value():.3f} "
                f"A{self.accel_spin.value():.3f} C{int(profile)}"
            ),
            "P0",
        ]
        self.start_command_queue(commands)

    def selected_draw_pattern(self):
        if not hasattr(self, "draw_pattern_combo"):
            return 1
        return int(self.draw_pattern_combo.currentData())

    def update_draw_preview(self):
        self.workspace.set_preview_path(self.draw_path_preview_points(self.selected_draw_pattern()))

    def draw_pattern_start(self, pattern_id):
        points = self.draw_path_preview_points(pattern_id)
        if points:
            return points[0]
        return (0.0, 250.1)

    def draw_pattern_end(self, pattern_id):
        points = self.draw_path_preview_points(pattern_id)
        if points:
            return points[-1]
        return (0.0, 230.1)

    def draw_path_preview_points(self, pattern_id=1):
        if pattern_id == 2:
            return self.star_preview_points()
        if pattern_id == 3:
            return self.heart_preview_points()
        if pattern_id == 4:
            return self.flower_preview_points()

        points = [
            (0.0, 250.1),
            (0.0, 230.1),
            (-15.0, 230.1),
            (-15.0, 210.1),
            (-45.0, 180.1),
            (-20.0, 180.1),
        ]
        tail = [
            (20.0, 180.1),
            (45.0, 180.1),
            (15.0, 210.1),
            (15.0, 230.1),
            (0.0, 230.1),
        ]
        rounded = self.round_polyline(points, 0.8, 6)
        rounded.extend(self.sample_arc((-20.0, 180.1), (20.0, 180.1), (0.0, 180.0), cw=False, steps=24)[1:])
        rounded.extend(self.round_polyline(tail, 0.8, 6)[1:])
        rounded.append(points[0])
        return rounded

    def star_preview_points(self):
        center = (0.0, 210.0)
        outer_r = 38.0
        inner_r = 17.0
        points = []
        for index in range(11):
            radius = outer_r if index % 2 == 0 else inner_r
            angle = math.tau * 0.25 + index * math.tau / 10.0
            points.append((
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle),
            ))
        return self.round_polyline(points, 1.2, 6)

    def heart_preview_points(self):
        center = (0.0, 208.0)
        scale = 2.0
        points = []
        for index in range(73):
            t = index * math.tau / 72.0
            st = math.sin(t)
            points.append((
                center[0] + scale * 16.0 * st * st * st,
                center[1] + scale * (
                    13.0 * math.cos(t)
                    - 5.0 * math.cos(2.0 * t)
                    - 2.0 * math.cos(3.0 * t)
                    - math.cos(4.0 * t)
                ),
            ))
        return points

    def flower_preview_points(self):
        center = (0.0, 210.0)
        points = []
        for index in range(73):
            t = index * math.tau / 72.0
            angle = math.tau * 0.25 + t
            radius = 22.0 + 10.0 * math.cos(6.0 * t)
            points.append((
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle),
            ))
        return points

    def make_text_strokes(self):
        text = self.text_line.text().strip()
        if not text:
            QMessageBox.information(self, "写字", "请输入要写的内容。")
            return []

        font = QFont("Microsoft YaHei")
        font.setPixelSize(100)
        path = QPainterPath()
        path.addText(0.0, 0.0, font, text)
        polygons = path.toSubpathPolygons()
        if not polygons:
            QMessageBox.warning(self, "写字", "没有生成文字轮廓。")
            return []

        bounds = path.boundingRect()
        if bounds.width() <= 0.0 or bounds.height() <= 0.0:
            return []

        scale, origin_x, top_y = self.text_layout_transform(bounds)
        strokes = []

        for poly in polygons:
            stroke = []
            last = None
            for point in poly:
                x = origin_x + (point.x() - bounds.left()) * scale
                y = top_y - (point.y() - bounds.top()) * scale
                if last is None or math.hypot(x - last[0], y - last[1]) >= 0.6:
                    stroke.append((x, y))
                    last = (x, y)

            if len(stroke) > 1:
                if self.distance(stroke[0], stroke[-1]) > 0.6:
                    stroke.append(stroke[0])
                strokes.append(stroke)

        return strokes

    def text_layout_transform(self, bounds):
        requested_scale = self.text_size_spin.value() / 100.0
        if not self.text_auto_fit_check.isChecked():
            return requested_scale, self.text_x_spin.value(), self.text_y_spin.value()

        safe_left = -90.0
        safe_right = 90.0
        safe_top = 275.0
        safe_bottom = 145.0
        safe_w = safe_right - safe_left
        safe_h = safe_top - safe_bottom
        scale = min(requested_scale, safe_w / bounds.width(), safe_h / bounds.height())
        draw_w = bounds.width() * scale
        draw_h = bounds.height() * scale
        origin_x = safe_left + (safe_w - draw_w) * 0.5
        top_y = safe_top - (safe_h - draw_h) * 0.5
        return scale, origin_x, top_y

    def flatten_strokes_for_preview(self, strokes):
        points = []
        for stroke in strokes:
            if points:
                points.append(None)
            points.extend(stroke)
        return points

    def text_strokes_bounds(self, strokes):
        xs = [point[0] for stroke in strokes for point in stroke]
        ys = [point[1] for stroke in strokes for point in stroke]
        if not xs or not ys:
            return None
        return min(xs), min(ys), max(xs), max(ys)

    def log_text_bounds(self, strokes):
        bounds = self.text_strokes_bounds(strokes)
        if bounds is None:
            return
        x0, y0, x1, y1 = bounds
        self.log_line(f"文字范围 X{x0:.1f}..{x1:.1f} Y{y0:.1f}..{y1:.1f}")

    def preview_text(self):
        strokes = self.make_text_strokes()
        if not strokes:
            return
        first = strokes[0][0]
        self.workspace.set_preview_path(self.flatten_strokes_for_preview(strokes))
        self.workspace.set_target(first[0], first[1])
        self.log_text_bounds(strokes)
        self.log_line(f"文字笔画段 {len(strokes)}")

    def write_text(self):
        strokes = self.make_text_strokes()
        if not strokes:
            return

        commands = []
        commands.append("P0")
        for stroke in strokes:
            start = stroke[0]
            commands.append(f"G1 X{start[0]:.3f} Y{start[1]:.3f} {self.feed_text()}")
            commands.append("P1")
            for x, y in stroke[1:]:
                commands.append(f"G1 X{x:.3f} Y{y:.3f} {self.feed_text()}")
            commands.append("P0")

        first = strokes[0][0]
        self.workspace.clear_path()
        self.workspace.set_preview_path(self.flatten_strokes_for_preview(strokes))
        self.workspace.set_target(first[0], first[1])
        self.log_text_bounds(strokes)
        self.log_line(f"文字笔画段 {len(strokes)}，命令 {len(commands)}")
        self.start_command_queue(commands)

    def update_teach_count(self):
        self.teach_count_label.setText(f"点数 {len(self.teach_points)}")

    def teach_record_point(self):
        point = (self.last_x, self.last_y)
        if self.teach_points and self.distance(self.teach_points[-1], point) < 0.2:
            self.log_line("示教点未变化，已忽略")
            return
        self.teach_points.append(point)
        self.update_teach_count()
        self.teach_preview_path()
        self.log_line(f"记录示教点 {len(self.teach_points)}: X{point[0]:.3f} Y{point[1]:.3f}")

    def teach_undo_point(self):
        if self.teach_points:
            point = self.teach_points.pop()
            self.log_line(f"删除示教点 X{point[0]:.3f} Y{point[1]:.3f}")
        self.update_teach_count()
        self.teach_preview_path()

    def teach_clear_points(self):
        self.teach_points.clear()
        self.update_teach_count()
        self.workspace.set_preview_path([])
        self.log_line("示教点已清空")

    def teach_preview_path(self):
        if not self.teach_points:
            self.workspace.set_preview_path([])
            return
        self.workspace.set_preview_path(self.teach_points)
        self.workspace.set_target(self.teach_points[-1][0], self.teach_points[-1][1])

    def teach_replay_path(self):
        if len(self.teach_points) < 2:
            QMessageBox.information(self, "示教", "至少需要记录 2 个点。")
            return

        commands = ["P0"]
        start = self.teach_points[0]
        commands.append(f"G1 X{start[0]:.3f} Y{start[1]:.3f} {self.feed_text()}")
        commands.append("P1")
        for x, y in self.teach_points[1:]:
            commands.append(f"G1 X{x:.3f} Y{y:.3f} {self.feed_text()}")
        commands.append("P0")

        self.workspace.clear_path()
        self.teach_preview_path()
        self.log_line(f"复现示教轨迹：{len(self.teach_points)} 点，{len(commands)} 条命令")
        self.start_command_queue(commands)

    def teach_save_points(self):
        if not self.teach_points:
            QMessageBox.information(self, "示教", "没有可保存的示教点。")
            return
        path, _ = QFileDialog.getSaveFileName(self, "保存示教轨迹", "teach_path.csv", "CSV Files (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y"])
            for x, y in self.teach_points:
                writer.writerow([f"{x:.3f}", f"{y:.3f}"])
        self.log_line(f"示教轨迹已保存：{path}")

    def teach_load_points(self):
        path, _ = QFileDialog.getOpenFileName(self, "载入示教轨迹", "", "CSV Files (*.csv)")
        if not path:
            return
        points = []
        with open(path, "r", newline="", encoding="utf-8") as file:
            reader = csv.reader(file)
            for row in reader:
                if len(row) < 2 or row[0].strip().lower() == "x":
                    continue
                try:
                    points.append((float(row[0]), float(row[1])))
                except ValueError:
                    continue
        if len(points) < 1:
            QMessageBox.warning(self, "示教", "文件中没有有效坐标。")
            return
        self.teach_points = points
        self.update_teach_count()
        self.teach_preview_path()
        self.log_line(f"示教轨迹已载入：{path}")

    def start_command_queue(self, commands):
        if self.queue_running:
            QMessageBox.warning(self, "写字", "当前还有队列在执行，请等它完成。")
            return
        if not self.serial.isOpen():
            QMessageBox.warning(self, "串口", "串口尚未连接。")
            return
        self.command_queue = list(commands)
        self.queue_running = True
        self.log_line(f"队列开始，共 {len(self.command_queue)} 条命令")
        self.send_next_queued_command()

    def send_next_queued_command(self):
        if not self.queue_running:
            return
        if not self.command_queue:
            self.queue_running = False
            self.log_line("队列完成")
            return
        command = self.command_queue.pop(0)
        if command.startswith("@WAIT"):
            parts = command.split()
            delay_ms = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 500
            self.log_line(f"等待 {delay_ms} ms")
            QTimer.singleShot(delay_ms, self.send_next_queued_command)
            return
        self.send_command(command)

    def round_polyline(self, points, radius, arc_steps):
        if len(points) < 3:
            return list(points)

        result = [points[0]]
        cursor = points[0]
        for index in range(1, len(points) - 1):
            prev = points[index - 1]
            corner = points[index]
            nxt = points[index + 1]
            segment = self.round_corner_points(prev, corner, nxt, radius, arc_steps)
            if segment is None:
                if self.distance(cursor, corner) > 0.001:
                    result.append(corner)
                cursor = corner
                continue

            t1, arc_points = segment
            if self.distance(cursor, t1) > 0.001:
                result.append(t1)
            result.extend(arc_points[1:])
            cursor = arc_points[-1]

        if self.distance(cursor, points[-1]) > 0.001:
            result.append(points[-1])
        return result

    def round_corner_points(self, prev, corner, nxt, radius, arc_steps):
        in_dir = self.normalize((corner[0] - prev[0], corner[1] - prev[1]))
        out_dir = self.normalize((nxt[0] - corner[0], nxt[1] - corner[1]))
        len_in = self.distance(prev, corner)
        len_out = self.distance(corner, nxt)
        if len_in < 0.001 or len_out < 0.001:
            return None

        cos_theta = max(-0.999, min(0.999, (-(in_dir[0]) * out_dir[0]) + (-(in_dir[1]) * out_dir[1])))
        theta = math.acos(cos_theta)
        trim = radius / math.tan(theta * 0.5)
        if theta < 0.05 or trim <= 0.0:
            return None

        trim = max(0.0, min(trim, min(len_in, len_out) * 0.45))
        if trim < 0.001:
            return None

        t1 = (corner[0] - in_dir[0] * trim, corner[1] - in_dir[1] * trim)
        t2 = (corner[0] + out_dir[0] * trim, corner[1] + out_dir[1] * trim)
        n1 = (-in_dir[1], in_dir[0])
        n2 = (-out_dir[1], out_dir[0])
        denom = self.cross(n1, n2)
        if abs(denom) < 0.0001:
            return t1, [t1, t2]

        t2_minus_t1 = (t2[0] - t1[0], t2[1] - t1[1])
        center_scale = self.cross(t2_minus_t1, n2) / denom
        center = (t1[0] + n1[0] * center_scale, t1[1] + n1[1] * center_scale)
        cw = self.cross(in_dir, out_dir) < 0.0
        return t1, self.sample_arc(t1, t2, center, cw, arc_steps)

    def sample_arc(self, start, end, center, cw, steps):
        radius = self.distance(start, center)
        a0 = math.atan2(start[1] - center[1], start[0] - center[0])
        a1 = math.atan2(end[1] - center[1], end[0] - center[0])
        sweep = a1 - a0
        if cw:
            if sweep >= 0.0:
                sweep -= math.tau
        elif sweep <= 0.0:
            sweep += math.tau

        return [
            (center[0] + radius * math.cos(a0 + sweep * i / steps),
             center[1] + radius * math.sin(a0 + sweep * i / steps))
            for i in range(steps + 1)
        ]

    def distance(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def normalize(self, vector):
        length = math.hypot(vector[0], vector[1])
        if length < 0.0001:
            return (0.0, 0.0)
        return (vector[0] / length, vector[1] / length)

    def cross(self, a, b):
        return (a[0] * b[1]) - (a[1] * b[0])

    def set_xy(self):
        x = self.set_x_spin.value()
        y = self.set_y_spin.value()
        self.send_command(f"SXY X{x:.3f} Y{y:.3f}")

    def send_command(self, command, log_tx=True):
        command = command.strip()
        if not command:
            return

        if not self.serial.isOpen():
            self.log_line(f"发送失败，串口未连接: {command}")
            QMessageBox.warning(self, "串口", "串口尚未连接。")
            return

        data = (command + "\r\n").encode("ascii", errors="ignore")
        self.serial.write(data)
        self.serial.flush()
        if log_tx:
            self.log_line(f"> {command}")

    def on_ready_read(self):
        self.rx_buffer.extend(bytes(self.serial.readAll()))

        while b"\n" in self.rx_buffer or b"\r" in self.rx_buffer:
            split_positions = [pos for pos in (self.rx_buffer.find(b"\n"), self.rx_buffer.find(b"\r")) if pos >= 0]
            pos = min(split_positions)
            raw = self.rx_buffer[:pos]
            self.rx_buffer = self.rx_buffer[pos + 1 :]
            line = raw.decode("ascii", errors="replace").strip()
            if line:
                self.handle_rx_line(line)

    def handle_rx_line(self, line):
        self.log_line(f"< {line}")
        match = POS_RE.match(line)
        if match:
            x = float(match.group("x"))
            y = float(match.group("y"))
            m1 = int(match.group("m1"))
            m2 = int(match.group("m2"))
            state = match.group("state")
            pen = match.group("pen")
            self.last_x = x
            self.last_y = y
            self.xy_label.setText(f"X{x:.3f}  Y{y:.3f}")
            self.motor_label.setText(f"M1 {m1}  M2 {m2}")
            self.state_label.setText(self.translate_state(state))
            if pen is not None:
                self.pen_label.setText("下笔" if pen == "1" else "抬笔")
            self.workspace.set_current(x, y)
            self.workspace.add_path_point(x, y)
            self.set_x_spin.setValue(x)
            self.set_y_spin.setValue(y)
            return

        match = POS_NO_FLOAT_RE.match(line)
        if match:
            m1 = int(match.group("m1"))
            m2 = int(match.group("m2"))
            state = match.group("state")
            pen = match.group("pen")
            self.motor_label.setText(f"M1 {m1}  M2 {m2}")
            self.state_label.setText(self.translate_state(state))
            if pen is not None:
                self.pen_label.setText("下笔" if pen == "1" else "抬笔")
            self.log_line("提示：下位机状态中的 X/Y 为空，请重新烧录新版固件。")
        elif line == "OK PEN UP":
            self.pen_label.setText("抬笔")
            self.send_next_queued_command()
        elif line == "OK PEN DOWN":
            self.pen_label.setText("下笔")
            self.send_next_queued_command()
        elif line == "RDY":
            self.state_label.setText("空闲")
            self.workspace.set_target(self.last_x, self.last_y)
            self.send_next_queued_command()
        elif line.startswith("EV SW") or line.startswith("SW "):
            self.handle_switch_status(line)
        elif line.startswith("ER"):
            self.state_label.setText("错误")
            self.queue_running = False
            self.command_queue.clear()

    def handle_switch_status(self, line):
        match = SW_RE.match(line)
        if not match:
            return
        active = []
        if match.group("a") == "1":
            active.append("PB0")
        if match.group("b") == "1":
            active.append("PA1")
        if active:
            self.state_label.setText("光电遮挡 " + "/".join(active))
        else:
            self.state_label.setText("光电未遮挡")

    def translate_state(self, state):
        states = {
            "BUSY": "运行中",
            "IDLE": "空闲",
            "RDY": "就绪",
            "ERROR": "错误",
            "STOP": "停止",
            "STOPPED": "已停止",
            "RUN": "运行中",
            "MOVING": "运行中",
        }
        return states.get(state.upper(), state)

    def log_line(self, text):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log.appendPlainText(f"{stamp}  {text}")

    def closeEvent(self, event):
        if self.serial.isOpen():
            self.status_timer.stop()
            self.serial.close()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = ScaraHost()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
