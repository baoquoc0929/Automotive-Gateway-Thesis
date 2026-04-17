import socket
import threading
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import csv
from datetime import datetime

# --- CONFIGURATION ---
UDP_IP_PC = "192.168.1.100"
UDP_IP_STM32 = "192.168.1.50"
UDP_PORT = 8080

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class GatewayThesisApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Automotive Gateway - Thesis Edition")
        self.geometry("900x650") # Wide layout for Graph

        # Data Storage
        self.dist_history = [0] * 50 # Last 50 points
        self.is_logging = False

        # Network
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP_PC, UDP_PORT))

        self.setup_ui()
        
        threading.Thread(target=self.receive_data, daemon=True).start()

    def setup_ui(self):
        # Left Panel (Controls & Stats)
        self.left_frame = ctk.CTkFrame(self, width=300, corner_radius=0)
        self.left_frame.pack(side="left", fill="y", padx=0, pady=0)

        ctk.CTkLabel(self.left_frame, text="SYSTEM STATUS", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=20)
        
        # Distance Card
        self.dist_label = ctk.CTkLabel(self.left_frame, text="--- cm", font=ctk.CTkFont(size=45, weight="bold"), text_color="#2ecc71")
        self.dist_label.pack(pady=10)

        # Logging Button
        self.log_btn = ctk.CTkButton(self.left_frame, text="START LOGGING (CSV)", fg_color="#e67e22", command=self.toggle_logging)
        self.log_btn.pack(pady=20, padx=20)

        # Control Buttons
        ctk.CTkLabel(self.left_frame, text="MANUAL COMMANDS").pack(pady=10)
        for i, (txt, color) in enumerate([("SAFE", "#27ae60"), ("CAUTION", "#f1c40f"), ("WARNING", "#e67e22"), ("DANGER", "#e74c3c")]):
            ctk.CTkButton(self.left_frame, text=txt, fg_color=color, width=150, command=lambda c=str(i): self.send_command(c)).pack(pady=5)

        # BUTTON RESTORE AUTO MODE
        self.auto_btn = ctk.CTkButton(self.left_frame, 
                                      text="RESTORE AUTO MODE", 
                                      fg_color="#3498db",      # BLUE
                                      hover_color="#2980b9",
                                      width=150, 
                                      height=45, 
                                      corner_radius=10,
                                      font=ctk.CTkFont(weight="bold"),
                                      command=lambda: self.send_command('A')) # Send character 'A'
        self.auto_btn.pack(pady=15)

        # Right Panel (Graph)
        self.right_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.right_frame.pack(side="right", fill="both", expand=True, padx=20, pady=20)

        # Setup Matplotlib Figure
        self.fig = Figure(figsize=(5, 4), dpi=100, facecolor='#242424')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#1e1e1e')
        self.ax.tick_params(colors='white')
        self.ax.set_title("Real-time Distance (cm)", color='white')
        self.ax.set_ylim(0, 250)
        self.line, = self.ax.plot(self.dist_history, color='#2ecc71', linewidth=2)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def toggle_logging(self):
        self.is_logging = not self.is_logging
        self.log_btn.configure(text="STOP LOGGING" if self.is_logging else "START LOGGING (CSV)",
                               fg_color="#c0392b" if self.is_logging else "#e67e22")
        if self.is_logging:
            self.log_file = open(f"gateway_log_{datetime.now().strftime('%H%M%S')}.csv", "w", newline='')
            self.writer = csv.writer(self.log_file)
            self.writer.writerow(["Timestamp", "Distance_cm"])

    def send_command(self, cmd):
        try: self.sock.sendto(cmd.encode(), (UDP_IP_STM32, UDP_PORT))
        except: pass

    def receive_data(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                msg = data.decode('utf-8')
                if "Dist:" in msg:
                    dist = int(msg.split("Dist:")[1].split("cm")[0].strip())
                    self.after(0, lambda d=dist: self.update_data(d))
            except: pass

    def update_data(self, dist):
        self.dist_label.configure(text=f"{dist} cm")
        
        # Update history and Graph
        self.dist_history.pop(0)
        self.dist_history.append(dist)
        self.line.set_ydata(self.dist_history)
        self.canvas.draw_idle()

        # Logging
        if self.is_logging:
            self.writer.writerow([datetime.now().strftime("%H:%M:%S.%f"), dist])

if __name__ == "__main__":
    app = GatewayThesisApp()
    app.mainloop()