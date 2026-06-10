import socket
import threading
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import csv
from datetime import datetime

# --- CONFIGURATION ---
UDP_IP_PC = "192.168.1.100"
UDP_IP_STM32 = "192.168.1.50" # <-- UPDATE WITH YOUR GATEWAY IP
UDP_PORT = 8080

# --- UI THEME ---
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class GatewayThesisApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Automotive Gateway - Thesis Edition")
        self.geometry("900x650") 

        # --- DATA STORAGE ---
        self.dist_history = [0] * 50 
        self.is_logging = False

        # --- NETWORK SETUP ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP_PC, UDP_PORT))

        self.setup_ui()
        
        # Start background thread for receiving UDP data
        threading.Thread(target=self.receive_data, daemon=True).start()

    def setup_ui(self):
        # --- LEFT PANEL (CONTROLS & STATS) ---
        self.left_frame = ctk.CTkFrame(self, width=250, corner_radius=0)
        self.left_frame.pack(side="left", fill="y", padx=0, pady=0)
        self.left_frame.pack_propagate(False)

        ctk.CTkLabel(self.left_frame, text="SYSTEM STATUS", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=20)
        
        # Distance Display Card
        self.dist_label = ctk.CTkLabel(self.left_frame, text="--- cm", font=ctk.CTkFont(size=45, weight="bold"), text_color="#2ecc71")
        self.dist_label.pack(pady=10)

        # CSV Logging Button
        self.log_btn = ctk.CTkButton(self.left_frame, text="START LOGGING (CSV)", fg_color="#e67e22", command=self.toggle_logging)
        self.log_btn.pack(pady=20, padx=20)

        # Application Control Buttons
        ctk.CTkLabel(self.left_frame, text="MANUAL COMMANDS").pack(pady=10)
        for i, (txt, color) in enumerate([("SAFE", "#27ae60"), ("CAUTION", "#f1c40f"), ("WARNING", "#e67e22"), ("DANGER", "#e74c3c")]):
            ctk.CTkButton(self.left_frame, text=txt, fg_color=color, width=150, command=lambda c=str(i): self.send_command(c)).pack(pady=5)

        # Restore Auto Mode Button
        self.auto_btn = ctk.CTkButton(self.left_frame, 
                                      text="RESTORE AUTO MODE", 
                                      fg_color="#3498db", 
                                      hover_color="#2980b9",
                                      width=150, 
                                      height=45, 
                                      corner_radius=10,
                                      font=ctk.CTkFont(weight="bold"),
                                      command=lambda: self.send_command('A')) 
        self.auto_btn.pack(pady=15)

        # --- UDS DIAGNOSTIC COMMANDS ---
        ctk.CTkLabel(self.left_frame, text="UDS DIAGNOSTIC COMMANDS", font=ctk.CTkFont(weight="bold"), text_color="#9b59b6").pack(pady=(20, 5))

        # ECU Reset Button (0x11)
        self.ecu_reset_btn = ctk.CTkButton(self.left_frame, 
                                           text="ECU RESET (0x11)", 
                                           fg_color="#8e44ad", 
                                           hover_color="#9b59b6",
                                           command=self.send_uds_reset)
        self.ecu_reset_btn.pack(pady=5)

        # IO Control Button (0x2F) - Test Buzzer
        self.io_control_btn = ctk.CTkButton(self.left_frame, 
                                            text="TEST BUZZER (0x2F)", 
                                            fg_color="#c0392b", 
                                            hover_color="#e74c3c",
                                            command=self.send_uds_io_control)
        self.io_control_btn.pack(pady=5)

        # UDS Response Result Label
        self.uds_result_label = ctk.CTkLabel(self.left_frame, text="Result: ---", font=ctk.CTkFont(size=14, weight="bold"), text_color="#f1c40f")
        self.uds_result_label.pack(pady=5)

        # --- RIGHT PANEL (MATPLOTLIB GRAPH) ---
        self.right_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.right_frame.pack(side="right", fill="both", expand=True, padx=20, pady=20)

        self.fig = Figure(figsize=(5, 4), dpi=100, facecolor='#242424')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#1e1e1e')
        self.ax.tick_params(colors='white')
        self.ax.set_title("Real-time Distance (cm)", color='white')
        self.ax.set_ylim(0, 250)
        self.line, = self.ax.plot(self.dist_history, color='#2ecc71', linewidth=2)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # --- LOGIC & COMMUNICATION METHODS ---
    
    def toggle_logging(self):
        """ Toggle CSV data logging state """
        self.is_logging = not self.is_logging
        self.log_btn.configure(text="STOP LOGGING" if self.is_logging else "START LOGGING (CSV)",
                               fg_color="#c0392b" if self.is_logging else "#e67e22")
        if self.is_logging:
            self.log_file = open(f"gateway_log_{datetime.now().strftime('%H%M%S')}.csv", "w", newline='')
            self.writer = csv.writer(self.log_file)
            self.writer.writerow(["Timestamp", "Distance_cm"])

    def send_command(self, cmd):
        """ Send raw application command ('0'-'3' or 'A') """
        try: 
            self.sock.sendto(cmd.encode(), (UDP_IP_STM32, UDP_PORT))
        except Exception as e: 
            print(f"[TX Error] {e}")

    def send_uds_reset(self):
        """ Pack and send UDS ECU Reset (0x11) via DoIP """
        uds_payload = bytearray([0x11, 0x01])
        self.transmit_doip(uds_payload)

    def send_uds_io_control(self):
        """ Pack and send UDS IO Control (0x2F) via DoIP """
        uds_payload = bytearray([0x2F, 0x01])
        self.transmit_doip(uds_payload)

    def transmit_doip(self, uds_payload):
        """ Helper function to construct and send a DoIP packet """
        doip_version = 0x02             # ISO 13400-2 Standard
        inverse_version = 0xFD          # Bitwise inverse of 0x02
        payload_type = [0x80, 0x01]     # Diagnostic Message

        payload_length = len(uds_payload)
        length_bytes = payload_length.to_bytes(4, byteorder='big')

        # Combine Header + Length + Payload
        doip_packet = bytearray([doip_version, inverse_version] + payload_type) + length_bytes + uds_payload

        try: 
            self.sock.sendto(doip_packet, (UDP_IP_STM32, UDP_PORT))
            hex_array = [f"0x{b:02X}" for b in doip_packet]
            print(f"[UDS TX] Transmitted DoIP: {hex_array}")
        except Exception as e: 
            print(f"[UDS TX Error] {e}")

    def receive_data(self):
        """ Continuous loop to receive incoming UDP data from Gateway """
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                msg = data.decode('utf-8')
                
                # Branch 1: Telemetry Data (Distance)
                if "Dist:" in msg:
                    dist = int(msg.split("Dist:")[1].split("cm")[0].strip())
                    self.after(0, lambda d=dist: self.update_data(d))
                
                # Branch 2: UDS Diagnostic Response
                elif "UDS_ACK:" in msg:
                    status = msg.split("UDS_ACK:")[1].strip()
                    
                    # Decode Positive Responses
                    if status == "81":   # 0x51 (Response for 0x11 ECU Reset)
                        display_text = "ECU RESET SUCCESS!"
                        color = "#2ecc71" # Green
                    elif status == "111": # 0x6F (Response for 0x2F IO Control)
                        display_text = "BUZZER ON (2s) SUCCESS!"
                        color = "#e67e22" # Orange
                    else:
                        display_text = f"Unknown ACK: {status}"
                        color = "#e74c3c" # Red
                        
                    self.after(0, lambda t=display_text, c=color: self.uds_result_label.configure(text=t, text_color=c))
                    print(f"[UDS RX] Gateway Response: {status} -> {display_text}")

            except Exception as e: 
                pass

    def update_data(self, dist):
        """ Update UI elements (Label & Graph) with new distance """
        self.dist_label.configure(text=f"{dist} cm")
        self.dist_history.pop(0)
        self.dist_history.append(dist)
        self.line.set_ydata(self.dist_history)
        self.canvas.draw_idle()

        # Write to CSV if logging is active
        if self.is_logging:
            self.writer.writerow([datetime.now().strftime("%H:%M:%S.%f"), dist])

if __name__ == "__main__":
    app = GatewayThesisApp()
    app.mainloop()