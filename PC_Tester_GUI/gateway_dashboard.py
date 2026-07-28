import socket
import threading
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import csv
from datetime import datetime
import time

# --- CONFIGURATION ---
UDP_IP_PC = "192.168.1.100"
UDP_IP_STM32 = "192.168.1.50" # <-- UPDATE WITH YOUR GATEWAY IP
UDP_PORT = 13400

# --- UI THEME ---
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class GatewayThesisApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Automotive Gateway - Thesis Edition")
        self.geometry("900x650") 

        # --- DATA & LOGGING STORAGE ---
        self.dist_history = [0] * 50 
        self.is_logging = False
        self.log_file = None
        self.csv_writer = None
        self.log_lock = threading.Lock() # Ensure thread-safe file writing
        
        # Variables for Latency calculation
        self.last_tx_time = None
        self.last_tx_service = "Unknown"

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

        ctk.CTkLabel(self.left_frame, text="SYSTEM STATUS", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=10)
        
        # Distance Display Card
        self.dist_label = ctk.CTkLabel(self.left_frame, text="--- cm", font=ctk.CTkFont(size=45, weight="bold"), text_color="#2ecc71")
        self.dist_label.pack(pady=5)

        # CSV Logging Button
        self.log_btn = ctk.CTkButton(self.left_frame, text="START LOGGING (CSV)", fg_color="#e67e22", command=self.toggle_logging)
        self.log_btn.pack(pady=10, padx=20)

        # --- UDS IO CONTROL COMMANDS (0x2F) ---
        ctk.CTkLabel(self.left_frame, text="ACTUATOR I/O CONTROL (0x2F)", font=ctk.CTkFont(weight="bold")).pack(pady=(15, 5))
        
        button_levels = [
            ("LEVEL 0: SAFE", "#27ae60"), 
            ("LEVEL 1: CAUTION", "#f1c40f"), 
            ("LEVEL 2: WARNING", "#e67e22"), 
            ("LEVEL 3: DANGER", "#e74c3c")
        ]
        
        for i, (txt, color) in enumerate(button_levels):
            ctk.CTkButton(self.left_frame, text=txt, fg_color=color, width=170, command=lambda c=str(i): self.send_command(c)).pack(pady=2)

        # Restore Auto Mode Button (Return Control To ECU)
        self.auto_btn = ctk.CTkButton(self.left_frame, 
                                      text="RETURN CONTROL (AUTO)", 
                                      fg_color="#3498db", 
                                      hover_color="#2980b9",
                                      width=170, 
                                      height=35, 
                                      corner_radius=10,
                                      font=ctk.CTkFont(weight="bold"),
                                      command=lambda: self.send_command('A')) 
        self.auto_btn.pack(pady=(10, 15))

        # --- SYSTEM DIAGNOSTICS ---
        ctk.CTkLabel(self.left_frame, text="SYSTEM DIAGNOSTICS", font=ctk.CTkFont(weight="bold"), text_color="#9b59b6").pack(pady=(10, 5))

        # ECU Reset Button (0x11)
        self.ecu_reset_btn = ctk.CTkButton(self.left_frame, 
                                           text="HARD ECU RESET (0x11)", 
                                           fg_color="#8e44ad", 
                                           hover_color="#9b59b6",
                                           width=170,
                                           command=self.send_uds_reset)
        self.ecu_reset_btn.pack(pady=2)

        # Read Data Button (0x22)
        self.read_data_btn = ctk.CTkButton(self.left_frame, 
                                           text="READ SENSOR DATA (0x22)", 
                                           fg_color="#2980b9",
                                           hover_color="#3498db",
                                           width=170,
                                           command=self.send_uds_read_data)
        self.read_data_btn.pack(pady=2)

        # UDS Response Result Label
        self.uds_result_label = ctk.CTkLabel(self.left_frame, text="Result: ---", font=ctk.CTkFont(size=14, weight="bold"), text_color="#f1c40f", wraplength=230)
        self.uds_result_label.pack(pady=10)

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
    
    def log_to_csv(self, direction, uds_service, payload_hex, latency="---"):
        """ Centralized method to write rows securely to the CSV """
        if self.is_logging and self.csv_writer:
            with self.log_lock:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                try:
                    self.csv_writer.writerow([timestamp, direction, uds_service, payload_hex, latency])
                    self.log_file.flush() # Ensure it writes to disk immediately
                except Exception as e:
                    print(f"[Log Error] {e}")

    def toggle_logging(self):
        """ Toggle CSV data logging state and manage file locking correctly """
        self.is_logging = not self.is_logging
        self.log_btn.configure(text="STOP LOGGING" if self.is_logging else "START LOGGING (CSV)",
                               fg_color="#c0392b" if self.is_logging else "#e67e22")
        
        with self.log_lock:
            if self.is_logging:
                # START: Open a new file and write the header
                filename = f"gateway_log_{datetime.now().strftime('%H%M%S')}.csv"
                self.log_file = open(filename, "w", newline='')
                self.csv_writer = csv.writer(self.log_file)
                self.csv_writer.writerow(["Timestamp", "Direction", "UDS Service", "Payload/Data", "Latency (ms)"])
                print(f"[LOG] Started logging to {filename}")
            else:
                # STOP: Close the file properly to release it
                if self.log_file:
                    self.log_file.close()
                    self.log_file = None
                    self.csv_writer = None
                    print("[LOG] Stopped logging and saved file.")

    def send_command(self, cmd):
        """ Translate UI commands into standard UDS IO Control (0x2F) payloads """
        try: 
            if cmd == 'A' or cmd == 'a':
                uds_payload = bytearray([0x2F, 0x01, 0x02, 0x00])
                print("[TESTER] Sent UDS 0x2F: AUTO Mode (Return Control)")
            elif cmd in ['0', '1', '2', '3']:
                control_state = int(cmd)
                uds_payload = bytearray([0x2F, 0x01, 0x02, 0x03, control_state])
                print(f"[TESTER] Sent UDS 0x2F: MANUAL Mode (Level {control_state})")
            else:
                return
            
            self.transmit_doip(uds_payload)
        except Exception as e: 
            print(f"[TX Error] {e}")

    def send_uds_reset(self):
        uds_payload = bytearray([0x11, 0x01])
        self.transmit_doip(uds_payload)

    def send_uds_io_control(self):
        uds_payload = bytearray([0x2F, 0x01, 0x01, 0x03])
        self.transmit_doip(uds_payload)

    def send_uds_read_data(self):
        uds_payload = bytearray([0x22, 0x01, 0x01])
        self.transmit_doip(uds_payload)

    def transmit_doip(self, uds_payload):
        """ Construct, send, and log a DoIP packet """
        doip_version = 0x02 
        inverse_version = 0xFD 
        payload_type = [0x80, 0x01] 
        source_address = [0x0E, 0x80] 
        target_address = [0x10, 0x00] 

        diagnostic_payload = bytearray(source_address + target_address) + uds_payload
        payload_length = len(diagnostic_payload)
        length_bytes = payload_length.to_bytes(4, byteorder='big')

        doip_packet = bytearray([doip_version, inverse_version] + payload_type) + length_bytes + diagnostic_payload

        try: 
            # Capture TX time for latency calculation
            self.last_tx_time = time.time()
            self.last_tx_service = f"0x{uds_payload[0]:02X}"
            
            self.sock.sendto(doip_packet, (UDP_IP_STM32, UDP_PORT))
            
            # Format Payload for Logging
            hex_array_str = " ".join([f"{b:02X}" for b in doip_packet])
            print(f"[UDS TX] Transmitted DoIP: {hex_array_str}")
            
            # Log TX event
            self.log_to_csv("Tx", self.last_tx_service, hex_array_str, "---")
            
        except Exception as e: 
            print(f"[UDS TX Error] {e}")

    def receive_data(self):
        """ Continuous loop to receive, process, and log incoming DoIP data """
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                rx_time = time.time() # Capture RX time immediately
                
                # Check valid DoIP Diagnostic Message
                if len(data) >= 13 and data[0] == 0x02 and data[2] == 0x80 and data[3] == 0x01:
                    
                    uds_response_sid = data[12] 
                    
                    # Calculate Latency
                    latency_ms = "---"
                    if self.last_tx_time:
                        latency_ms = f"{round((rx_time - self.last_tx_time) * 1000, 2)} ms"
                        self.last_tx_time = None # Reset to avoid double counting

                    # Deduce requested service based on response SID (Response SID = Request SID + 0x40)
                    request_service = f"0x{(uds_response_sid - 0x40):02X}" if uds_response_sid >= 0x40 else f"0x{uds_response_sid:02X}"
                    
                    # Format Payload for Logging
                    hex_array_str = " ".join([f"{b:02X}" for b in data])
                    self.log_to_csv("Rx", request_service, hex_array_str, latency_ms)

                    # Process GUI Updates based on Response SID
                    if uds_response_sid == 0x51:   
                        display_text = "ECU RESET SUCCESS!"
                        color = "#2ecc71" 
                    elif uds_response_sid == 0x6F: 
                        display_text = "ACTUATOR CONTROL SUCCESS!"
                        color = "#e67e22" 
                    elif uds_response_sid == 0x62: 
                        if len(data) >= 17:
                            dist_val = (data[15] << 8) | data[16]
                            display_text = f"READ DATA 0x22 SUCCESS! Dist: {dist_val} cm"
                            color = "#3498db" 
                            self.after(0, lambda d=dist_val: self.update_data(d))
                        else:
                            continue 
                    else:
                        display_text = f"Unknown or Error ACK: {hex(uds_response_sid)}"
                        color = "#e74c3c" 
                        
                    self.after(0, lambda t=display_text, c=color: self.uds_result_label.configure(text=t, text_color=c))
                    print(f"[UDS RX] Latency: {latency_ms}ms | {hex(uds_response_sid)} -> {display_text}")
                    
            except Exception as e:
                print(f"[RX Error] {e}")

    def update_data(self, dist):
        """ Update UI graph (Logging removed from here as it's handled in receive_data) """
        self.dist_label.configure(text=f"{dist} cm")
        self.dist_history.pop(0)
        self.dist_history.append(dist)
        self.line.set_ydata(self.dist_history)
        self.canvas.draw_idle()
        
    def destroy(self):
        """ Ensure file is safely closed when the user exits the application window """
        if self.is_logging and self.log_file:
            self.log_file.close()
        super().destroy()

if __name__ == "__main__":
    app = GatewayThesisApp()
    app.mainloop()