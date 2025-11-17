"""
GUI для калибровки офсетов сервоприводов в реальном времени
Требует: pip install pyserial tkinter
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time

class ServoCalibrationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Калибровка сервоприводов StackForce")
        self.root.geometry("600x700")
        
        self.serial_port = None  # Инициализация атрибута serial_port
        self.offsets = [-35, -162, -150, -15, -74, -147, -108, -23]  # Откалиброваны в IK режиме
        self.manual_enabled = tk.BooleanVar(value=False)
        self.manual_leg_vars = {}
        self.manual_event_block = False
        self.manual_x_range = (-60.0, 60.0)
        self.manual_y_range = (70.0, 150.0)
        self.leg_labels = {
            "fl": "Front Left",
            "fr": "Front Right",
            "bl": "Back Left",
            "br": "Back Right"
        }
        
        # Названия серво
        # ВНИМАНИЕ: Робот повернут на 180° относительно гироскопа
        self.servo_names = [
            "FL_FRONT (CH2)",
            "FL_REAR (CH3)",
            "FR_FRONT (CH9)",
            "FR_REAR (CH8)",
            "BL_FRONT (CH0)",
            "BL_REAR (CH1)",
            "BR_FRONT (CH11)",
            "BR_REAR (CH10)"
        ]
        
        self.servo_channels = [2, 3, 9, 8, 0, 1, 11, 10]
        
        self.create_widgets()
        
    def create_widgets(self):
        # === ПОДКЛЮЧЕНИЕ ===
        conn_frame = ttk.LabelFrame(self.root, text="Подключение", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(conn_frame, text="COM-порт:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5)
        
        ttk.Button(conn_frame, text="Обновить", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        ttk.Button(conn_frame, text="Подключить", command=self.connect).grid(row=0, column=3, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Не подключено", foreground="red")
        self.status_label.grid(row=0, column=4, padx=10)
        
        # === ОФСЕТЫ ===
        offsets_frame = ttk.LabelFrame(self.root, text="Офсеты сервоприводов", padding=10)
        offsets_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.offset_vars = []
        self.offset_scales = []
        
        for i in range(8):
            # Название
            ttk.Label(offsets_frame, text=self.servo_names[i]).grid(row=i, column=0, sticky="w", pady=5)
            
            # Слайдер с откалиброванным значением
            var = tk.IntVar(value=self.offsets[i])
            scale = ttk.Scale(offsets_frame, from_=-200, to=200, variable=var, 
                             orient="horizontal", length=250,
                             command=lambda v, idx=i: self.on_offset_change(idx, v))
            scale.grid(row=i, column=1, padx=10)
            
            # Поле ввода
            entry = ttk.Entry(offsets_frame, textvariable=var, width=8)
            entry.grid(row=i, column=2, padx=5)
            entry.bind("<Return>", lambda e, idx=i: self.on_offset_entry(idx))
            
            self.offset_vars.append(var)
            self.offset_scales.append(scale)
        
        # === КНОПКИ УПРАВЛЕНИЯ ===
        control_frame = ttk.Frame(self.root, padding=10)
        control_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Button(control_frame, text="Сброс в 0", command=self.reset_offsets).pack(side="left", padx=5)
        ttk.Button(control_frame, text="Все серво → 90°", command=self.all_neutral).pack(side="left", padx=5)
        ttk.Button(control_frame, text="🎯 Автовыравнивание", command=self.auto_level).pack(side="left", padx=5)
        ttk.Button(control_frame, text="Запросить текущие", command=self.request_offsets).pack(side="left", padx=5)
        
        # === РЕЖИМ IK ===
        ik_frame = ttk.LabelFrame(self.root, text="IK Режим", padding=10)
        ik_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Button(ik_frame, text="🔄 Переключить режим (m)", command=self.toggle_mode).pack(side="left", padx=5)
        
        ttk.Label(ik_frame, text="X:").pack(side="left", padx=5)
        self.x_var = tk.DoubleVar(value=0.0)
        ttk.Entry(ik_frame, textvariable=self.x_var, width=8).pack(side="left", padx=5)
        ttk.Button(ik_frame, text="Установить X", command=self.set_x).pack(side="left", padx=5)
        
        ttk.Label(ik_frame, text="Y:").pack(side="left", padx=5)
        self.y_var = tk.DoubleVar(value=115.0)
        ttk.Entry(ik_frame, textvariable=self.y_var, width=8).pack(side="left", padx=5)
        ttk.Button(ik_frame, text="Установить Y", command=self.set_y).pack(side="left", padx=5)

        self.build_manual_section()
        
        # === СОХРАНЕНИЕ ===
        save_frame = ttk.LabelFrame(self.root, text="Сохранение", padding=10)
        save_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Button(save_frame, text="Сохранить в quadrupedal_data.h", 
                  command=self.save_to_header).pack(side="left", padx=5)
        ttk.Button(save_frame, text="Экспорт в .txt", 
                  command=self.export_txt).pack(side="left", padx=5)
        
        # === ЛОГ ===
        log_frame = ttk.LabelFrame(self.root, text="Лог", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.log_text = tk.Text(log_frame, height=8, state="disabled")
        self.log_text.pack(fill="both", expand=True)
        
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scrollbar.set)
        
        # Обновить список портов при старте
        self.refresh_ports()

    def build_manual_section(self):
        manual_frame = ttk.LabelFrame(self.root, text="Ручное тестирование ног (stab off + manual on)", padding=10)
        manual_frame.pack(fill="both", expand=True, padx=10, pady=5)

        ttk.Label(manual_frame, text="1) На контроллере: 'stab off', затем 'manual on'.\n2) Используйте слайдеры, чтобы задать X/Y каждой ноги.").pack(anchor="w", pady=(0, 8))

        btn_frame = ttk.Frame(manual_frame)
        btn_frame.pack(fill="x", pady=(0, 10))
        ttk.Button(btn_frame, text="Manual ON", command=lambda: self.set_manual_mode(True)).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Manual OFF", command=lambda: self.set_manual_mode(False)).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Сбросить цели", command=self.reset_manual_targets).pack(side="left", padx=5)
        self.manual_status = ttk.Label(btn_frame, text="Manual: OFF", foreground="red")
        self.manual_status.pack(side="left", padx=10)

        grid = ttk.Frame(manual_frame)
        grid.pack(fill="both", expand=True)

        legs = [("fl", "Передняя левая"), ("fr", "Передняя правая"), ("bl", "Задняя левая"), ("br", "Задняя правая")]
        for idx, (leg_key, leg_title) in enumerate(legs):
            card = ttk.LabelFrame(grid, text=leg_title, padding=8)
            card.grid(row=idx // 2, column=idx % 2, sticky="nsew", padx=5, pady=5)
            grid.grid_columnconfigure(idx % 2, weight=1)

            x_var = tk.DoubleVar(value=0.0)
            y_var = tk.DoubleVar(value=110.0)
            self.manual_leg_vars[leg_key] = {"x": x_var, "y": y_var}

            ttk.Label(card, text="X (мм)").pack(anchor="w")
            tk.Scale(card, from_=self.manual_x_range[0], to=self.manual_x_range[1], orient="horizontal",
                     resolution=0.5, variable=x_var,
                     command=lambda _v, leg=leg_key: self.on_manual_slider(leg)).pack(fill="x")
            x_entry = ttk.Entry(card, textvariable=x_var, width=7)
            x_entry.pack(pady=(2, 6))
            x_entry.bind("<Return>", lambda _e, leg=leg_key: self.on_manual_entry(leg))

            ttk.Label(card, text="Y (мм)").pack(anchor="w")
            tk.Scale(card, from_=self.manual_y_range[0], to=self.manual_y_range[1], orient="horizontal",
                     resolution=0.5, variable=y_var,
                     command=lambda _v, leg=leg_key: self.on_manual_slider(leg)).pack(fill="x")
            y_entry = ttk.Entry(card, textvariable=y_var, width=7)
            y_entry.pack(pady=(2, 0))
            y_entry.bind("<Return>", lambda _e, leg=leg_key: self.on_manual_entry(leg))
    
    def log(self, message):
        """Добавить сообщение в лог"""
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"{time.strftime('%H:%M:%S')} - {message}\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")
    
    def refresh_ports(self):
        """Обновить список COM-портов"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
        self.log(f"Найдено портов: {len(ports)}")
    
    def connect(self):
        """Подключение к ESP32"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.status_label.config(text="Не подключено", foreground="red")
            self.log("Отключено")
            return
        
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Ошибка", "Выберите COM-порт")
            return
        
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)  # Ждём перезагрузку ESP32
            self.status_label.config(text=f"Подключено: {port}", foreground="green")
            self.log(f"Подключено к {port}")
            
            # Запустить поток чтения
            threading.Thread(target=self.read_serial, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось подключиться:\n{e}")
            self.log(f"ОШИБКА: {e}")
    
    def read_serial(self):
        """Чтение ответов от ESP32"""
        while self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.log(f"← {line}")
                        # Парсинг ответа на команду 'p' (вывод офсетов)
                        self.parse_offset_line(line)
            except Exception as e:
                self.log(f"ОШИБКА чтения: {e}")
                break
            time.sleep(0.05)
    
    def parse_offset_line(self, line):
        """Парсинг строки с офсетом и обновление UI"""
        import re
        # Формат: "FL_FRONT (CH11): 44"
        match = re.match(r'(\w+)\s+\(CH(\d+)\):\s*(-?\d+)', line)
        if match:
            servo_name = match.group(1)
            channel = int(match.group(2))
            offset = int(match.group(3))
            
            # Найти индекс по каналу
            try:
                index = self.servo_channels.index(channel)
                # Обновить переменную (автоматически обновит слайдер и поле)
                self.offset_vars[index].set(offset)
                self.log(f"✓ Обновлен {servo_name}: {offset}")
            except ValueError:
                pass  # Канал не в нашем списке
    
    def send_command(self, cmd):
        """Отправка команды на ESP32"""
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Предупреждение", "Сначала подключитесь к роботу")
            return False
        
        try:
            self.serial_port.write(f"{cmd}\n".encode())
            self.log(f"→ {cmd}")
            return True
        except Exception as e:
            self.log(f"ОШИБКА отправки: {e}")
            return False
    
    def on_offset_change(self, index, value):
        """Обработка изменения слайдера"""
        offset = int(float(value))
        channel = self.servo_channels[index]
        self.send_command(f"o{channel} {offset}")
    
    def on_offset_entry(self, index):
        """Обработка ввода в поле"""
        offset = self.offset_vars[index].get()
        channel = self.servo_channels[index]
        self.send_command(f"o{channel} {offset}")
    
    def reset_offsets(self):
        """Сброс всех офсетов в 0"""
        if self.send_command("r"):
            for var in self.offset_vars:
                var.set(0)
    
    def all_neutral(self):
        """Установка всех серво на 90°"""
        self.send_command("a 90")
    
    def auto_level(self):
        """Автовыравнивание робота по IMU"""
        result = messagebox.askyesno(
            "Автовыравнивание", 
            "Робот должен стоять неподвижно!\n\n"
            "Убедитесь, что:\n"
            "• Робот стоит на ровной поверхности\n"
            "• Ничто не мешает движению ног\n"
            "• Робот не касается проводами пола\n\n"
            "Начать автовыравнивание?"
        )
        if result:
            self.log(">>> Запуск автовыравнивания...")
            self.send_command("l")
    
    def toggle_mode(self):
        """Переключение между режимами калибровки и IK"""
        self.send_command("m")
    
    def set_x(self):
        """Установка X координаты в IK режиме"""
        x_value = self.x_var.get()
        self.send_command(f"x {x_value}")
    
    def set_y(self):
        """Установка Y координаты в IK режиме"""
        y_value = self.y_var.get()
        self.send_command(f"y {y_value}")

    def set_manual_mode(self, enabled: bool):
        cmd = "manual on" if enabled else "manual off"
        if self.send_command(cmd):
            self.manual_enabled.set(enabled)
            status = "ON" if enabled else "OFF"
            color = "green" if enabled else "red"
            self.manual_status.config(text=f"Manual: {status}", foreground=color)

    def reset_manual_targets(self):
        self.manual_event_block = True
        for leg_vars in self.manual_leg_vars.values():
            leg_vars["x"].set(0.0)
            leg_vars["y"].set(110.0)
        self.manual_event_block = False
        self.send_command("manual reset")

    def on_manual_slider(self, leg_key: str):
        if self.manual_event_block:
            return
        self.apply_manual_limits(leg_key)
        self.send_leg_target(leg_key)

    def on_manual_entry(self, leg_key: str):
        if self.manual_event_block:
            return
        self.apply_manual_limits(leg_key)
        self.send_leg_target(leg_key)

    def apply_manual_limits(self, leg_key: str):
        for axis, limits in (("x", self.manual_x_range), ("y", self.manual_y_range)):
            value = self.manual_leg_vars[leg_key][axis].get()
            clamped = max(limits[0], min(limits[1], value))
            if clamped != value:
                self.manual_event_block = True
                self.manual_leg_vars[leg_key][axis].set(clamped)
                self.manual_event_block = False

    def send_leg_target(self, leg_key: str):
        if not self.manual_enabled.get():
            self.log("Manual режим выключен. Нажмите Manual ON перед движением ног.")
            return
        x_value = self.manual_leg_vars[leg_key]["x"].get()
        y_value = self.manual_leg_vars[leg_key]["y"].get()
        self.send_command(f"move {leg_key} {x_value:.1f} {y_value:.1f}")
    
    def request_offsets(self):
        """Запросить текущие офсеты с робота"""
        self.send_command("p")
    
    def save_to_header(self):
        """Сохранить офсеты в файл quadrupedal_data.h"""
        filepath = filedialog.asksaveasfilename(
            defaultextension=".h",
            filetypes=[("Header files", "*.h"), ("All files", "*.*")],
            initialfile="quadrupedal_data.h"
        )
        
        if not filepath:
            return
        
        offsets = [var.get() for var in self.offset_vars]
        
        header_content = f"""// Смещения серво для калибровки (градусы)
#define SERVO_FL_FRONT_OFFSET {offsets[0]}
#define SERVO_FL_REAR_OFFSET  {offsets[1]}
#define SERVO_FR_FRONT_OFFSET {offsets[2]}
#define SERVO_FR_REAR_OFFSET  {offsets[3]}

#define SERVO_BL_FRONT_OFFSET {offsets[4]}
#define SERVO_BL_REAR_OFFSET  {offsets[5]}
#define SERVO_BR_FRONT_OFFSET {offsets[6]}
#define SERVO_BR_REAR_OFFSET  {offsets[7]}
"""
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(header_content)
            self.log(f"✓ Сохранено в {filepath}")
            messagebox.showinfo("Успех", f"Офсеты сохранены в:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось сохранить:\n{e}")
    
    def export_txt(self):
        """Экспорт офсетов в текстовый файл"""
        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            initialfile="servo_offsets.txt"
        )
        
        if not filepath:
            return
        
        offsets = [var.get() for var in self.offset_vars]
        
        content = "# Офсеты сервоприводов StackForce\n\n"
        for i, name in enumerate(self.servo_names):
            content += f"{name}: {offsets[i]}\n"
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(content)
            self.log(f"✓ Экспортировано в {filepath}")
            messagebox.showinfo("Успех", f"Офсеты экспортированы в:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось экспортировать:\n{e}")

def main():
    root = tk.Tk()
    app = ServoCalibrationGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
