import sys
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTableWidget, QTableWidgetItem,
    QLabel, QComboBox, QMessageBox, QHeaderView
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor


# ===== КОНСТАНТЫ =====
ROW_COUNT = 19
BANDS = [160, 80, 40, 30, 20, 17, 15, 12, 10, 6]
BAUD_RATES = [19200, 9600]  # доступные скорости

CMD_START = 0xAA
CMD_END   = 0x55


# ===== ДЕФОЛТ =====
DEFAULT_RULES = [
    (1800, 1840, 160, 'C'), (1840, 2000, 160, 'P'),
    (3500, 3600, 80, 'C'),  (3600, 3800, 80, 'P'),
    (7000, 7050, 40, 'C'),  (7050, 7200, 40, 'P'),
    (10100, 10150, 30, 'C'),
    (14000, 14110, 20, 'C'), (14110, 14350, 20, 'P'),
    (18068, 18110, 17, 'C'), (18110, 18168, 17, 'P'),
    (21000, 21110, 15, 'C'), (21110, 21450, 15, 'P'),
    (24890, 24920, 12, 'C'), (24920, 24990, 12, 'P'),
    (28000, 28300, 10, 'C'), (28300, 30000, 10, 'P'),
    (50000, 50200, 6, 'C'),  (50200, 54000, 6, 'P'),
]


class BandEditor(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Icom Band Decoder Config")

        # ===== MAIN LAYOUT =====
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)   # padding окна
        layout.setSpacing(8)

        # ===== COM PANEL =====
        com_panel = QHBoxLayout()
        com_panel.setSpacing(6)

        com_panel.addWidget(QLabel("COM:"))

        self.com_box = QComboBox()
        self.com_box.setMinimumWidth(120)
        com_panel.addWidget(self.com_box)

        btn_refresh = QPushButton("↻")
        btn_refresh.setFixedWidth(40)
        btn_refresh.clicked.connect(self.refresh_ports)
        com_panel.addWidget(btn_refresh)

        # разделитель
        com_panel.addSpacing(20)

        # выбор скорости
        com_panel.addWidget(QLabel("Speed:"))

        self.baud_box = QComboBox()
        self.baud_box.setMinimumWidth(80)
        for rate in BAUD_RATES:
            self.baud_box.addItem(str(rate), rate)
        self.baud_box.setCurrentIndex(0)  # 19200 по умолчанию
        com_panel.addWidget(self.baud_box)

        com_panel.addStretch()

        layout.addLayout(com_panel)

        # ===== TABLE =====
        self.table = QTableWidget(ROW_COUNT, 4)
        self.table.setHorizontalHeaderLabels(["Start kHz (>=)", "End kHz (<)", "Band", "Mode (C-CW, P-Phone)"])
        self.table.verticalHeader().setVisible(False)

        # растягивание колонок
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)

        # убираем скроллинг
        self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        layout.addWidget(self.table)

        # ===== BUTTONS =====
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        btn_defaults = QPushButton("Defaults")
        btn_defaults.clicked.connect(self.load_defaults)
        btn_layout.addWidget(btn_defaults)

        btn_send = QPushButton("SEND")
        btn_send.clicked.connect(self.send_data)
        btn_layout.addWidget(btn_send)

        btn_layout.addStretch()

        layout.addLayout(btn_layout)

        self.setLayout(layout)

        # ===== ФИКС РАЗМЕРА (чтобы всё влезало) =====
        self.resize(600, 750)

        self.refresh_ports()
        self.load_defaults()

    # ===== COM =====
    def refresh_ports(self):
        self.com_box.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.com_box.addItem(p.device)

    # ===== DEFAULT =====
    def load_defaults(self):
        self.table.clearContents()

        for row, rule in enumerate(DEFAULT_RULES):

            # start / end
            for col in [0, 1]:
                item = QTableWidgetItem(str(rule[col]))
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(row, col, item)

            # band
            combo = QComboBox()
            for b in BANDS:
                combo.addItem(str(b), b)
            combo.setCurrentText(str(rule[2]))
            self.table.setCellWidget(row, 2, combo)

            # mode
            item = QTableWidgetItem(rule[3])
            item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(row, 3, item)

            # 30м фикс
            if rule[2] == 30:
                combo.setEnabled(False)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)

                for col in [0, 1, 3]:
                    self.table.item(row, col).setBackground(QColor(230, 230, 230))

                combo.setStyleSheet("background-color:#e6e6e6;")

    # ===== SEND =====
    def send_data(self):
        port = self.com_box.currentText()
        baud_rate = self.baud_box.currentData()

        if not port:
            QMessageBox.warning(self, "Error", "No COM port")
            return

        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))
            return

        try:
            packet = bytearray()

            # START
            packet.append(CMD_START)

            # COUNT
            packet.append(ROW_COUNT)

            # DATA (10 байт на правило!)
            for i in range(ROW_COUNT):
                start = int(self.table.item(i, 0).text())
                end = int(self.table.item(i, 1).text())

                band = self.table.cellWidget(i, 2).currentData()
                mode = self.table.item(i, 3).text()

                packet += start.to_bytes(4, 'little')
                packet += end.to_bytes(4, 'little')
                packet.append(band)
                packet.append(ord(mode))

            # CRC (XOR)
            crc = 0
            for b in packet:
                crc ^= b

            packet.append(crc)

            # END
            packet.append(CMD_END)

            print(f"SEND (baud={baud_rate}):", packet.hex())

            ser.write(packet)

            QMessageBox.information(self, "OK", f"Data sent at {baud_rate} baud")

        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

        finally:
            ser.close()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = BandEditor()
    w.show()
    sys.exit(app.exec_())