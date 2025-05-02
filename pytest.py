"""
Python GUI for Stepper Motor Control with Position Tracking and eGun Control
Implements control for:
1. Motor on/off
2. Direction control
3. Frequency/pulse control for movement
4. Position tracking in millimeters
5. Homing with kill switch
6. eGun control for automated sampling (using pin 11)
7. Sequence programming for multiple sample positions
"""

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import serial
import serial.tools.list_ports
import threading
import time
import re

class StepperControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Stepper Motor Control with eGun")
        self.root.geometry("550x620")
        
        # Serial connection
        self.serial_port = None
        self.connected = False
        
        # Status variables
        self.limit_status = False
        self.position_mm = 0.0
        self.load_allowed = False
        self.egun_active = False
        self.sequence_running = False
        self.total_sequence_steps = 0
        self.at_load_position = False # Flag to track if LOAD command is executed
        
        # Create style for buttons
        self.style = ttk.Style()
        self.style.configure("Stop.TButton", foreground="red", font=("Arial", 10, "bold"))
        self.style.configure("eGun.TButton", foreground="blue", font=("Arial", 10, "bold"))
        self.style.configure("Sequence.TButton", foreground="green", font=("Arial", 10, "bold"))
        
        # Main frame
        main_frame = ttk.Frame(root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create connection section
        self.create_connection_section(main_frame)
        
        # Create basic control section
        self.create_control_section(main_frame)
        
        # Create movement section
        self.create_movement_section(main_frame)
        
        # Create homing section
        self.create_homing_section(main_frame)
        
        # Create eGun section
        self.create_egun_section(main_frame)
        
        # Create status section
        self.create_status_section(main_frame)
        
        # Get available ports
        self.refresh_ports()
        
        # Bind window close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_connection_section(self, parent):
        """Create the connection section"""
        frame = ttk.LabelFrame(parent, text="Connection", padding="5")
        frame.pack(fill=tk.X, pady=5)
        
        # Port selection row
        ttk.Label(frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        
        refresh_btn = ttk.Button(frame, text="Refresh", command=self.refresh_ports)
        refresh_btn.grid(row=0, column=2, padx=5, pady=5)
        
        self.connect_btn = ttk.Button(frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=5, pady=5)
    
    def create_control_section(self, parent):
        """Create the basic motor control section"""
        frame = ttk.LabelFrame(parent, text="Motor Control", padding="5")
        frame.pack(fill=tk.X, pady=5)
        
        # Motor ON/OFF buttons
        ttk.Button(frame, text="Motor ON", command=self.motor_on).grid(
            row=0, column=0, padx=10, pady=5)
        
        ttk.Button(frame, text="Motor OFF", command=self.motor_off).grid(
            row=0, column=1, padx=10, pady=5)
        
        # Direction controls
        ttk.Label(frame, text="Direction:").grid(row=1, column=0, padx=5, pady=5)
        
        ttk.Button(frame, text="Forward", command=self.direction_forward).grid(
            row=1, column=1, padx=10, pady=5)
        
        ttk.Button(frame, text="Reverse", command=self.direction_reverse).grid(
            row=1, column=2, padx=10, pady=5)
        
        # Status button
        ttk.Button(frame, text="Get Status", command=self.get_status).grid(
            row=0, column=2, padx=10, pady=5)
        
        # Stop button (emergency)
        ttk.Button(frame, text="STOP", command=self.stop, 
                   style="Stop.TButton").grid(
            row=0, column=3, rowspan=2, padx=10, pady=5, sticky="ns")
    
    def create_movement_section(self, parent):
        """Create the movement control section"""
        frame = ttk.LabelFrame(parent, text="Movement Control", padding="5")
        frame.pack(fill=tk.X, pady=5)
        
        # Frequency control
        ttk.Label(frame, text="Frequency (Hz):").grid(row=0, column=0, padx=5, pady=5)
        self.freq_var = tk.StringVar(value="1000")
        freq_entry = ttk.Entry(frame, textvariable=self.freq_var, width=8)
        freq_entry.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(frame, text="Set Frequency", command=self.set_frequency).grid(
            row=0, column=2, padx=5, pady=5)
        
        # Number of pulses (steps)
        ttk.Label(frame, text="Number of Pulses:").grid(row=1, column=0, padx=5, pady=5)
        self.pulses_var = tk.StringVar(value="1000")
        pulses_entry = ttk.Entry(frame, textvariable=self.pulses_var, width=8)
        pulses_entry.grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Button(frame, text="Move", command=self.move).grid(
            row=1, column=2, padx=5, pady=5)
        
        # Distance (millimeters)
        ttk.Label(frame, text="Distance (mm):").grid(row=2, column=0, padx=5, pady=5)
        self.mm_var = tk.StringVar(value="10")
        mm_entry = ttk.Entry(frame, textvariable=self.mm_var, width=8)
        mm_entry.grid(row=2, column=1, padx=5, pady=5)
        
        ttk.Button(frame, text="Move (mm)", command=self.move_mm).grid(
            row=2, column=2, padx=5, pady=5)
    
    def create_homing_section(self, parent):
        """Create the homing section with limit switch status"""
        frame = ttk.LabelFrame(parent, text="Homing & Position", padding="5")
        frame.pack(fill=tk.X, pady=5)
        
        # Position display
        ttk.Label(frame, text="Position:").grid(row=0, column=0, padx=5, pady=5)
        self.position_var = tk.StringVar(value="0.0 mm")
        ttk.Label(frame, textvariable=self.position_var, width=10).grid(
            row=0, column=1, padx=5, pady=5)
        
        # Position status display (HOME or LOAD)
        self.position_status_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.position_status_var).grid(
            row=0, column=2, padx=5, pady=5)
        
        # Buttons
        ttk.Button(frame, text="HOME", command=self.home).grid(
            row=1, column=0, padx=5, pady=5)
        
        self.load_btn = ttk.Button(frame, text="LOAD", command=self.load_position,
                                  state=tk.DISABLED)
        self.load_btn.grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Button(frame, text="ZERO", command=self.zero_position).grid(
            row=1, column=2, padx=5, pady=5)
            
        ttk.Button(frame, text="Get Position", command=self.get_position).grid(
            row=2, column=0, padx=5, pady=5)
        
        # Kill switch indicator
        ttk.Label(frame, text="Kill Switch:").grid(row=2, column=1, padx=5, pady=5, sticky="e")
        
        self.kill_indicator = tk.Canvas(frame, width=20, height=20, bg="gray")
        self.kill_indicator.grid(row=2, column=2, padx=5, pady=5, sticky="w")
        self.kill_indicator.create_oval(2, 2, 18, 18, fill="red", tags="indicator")
    
    def create_egun_section(self, parent):
        """Create the eGun and sequence control section"""
        frame = ttk.LabelFrame(parent, text="eGun & Sampling Control (Pin 11)", padding="5")
        frame.pack(fill=tk.X, pady=5)
        
        # eGun manual controls
        ttk.Button(frame, text="eGun ON", command=self.egun_on, 
                   style="eGun.TButton").grid(
            row=0, column=0, padx=5, pady=5)
        
        ttk.Button(frame, text="eGun OFF", command=self.egun_off,
                   style="eGun.TButton").grid(
            row=0, column=1, padx=5, pady=5)
        
        # eGun status indicator
        ttk.Label(frame, text="eGun Status:").grid(row=0, column=2, padx=5, pady=5)
        self.egun_indicator = tk.Canvas(frame, width=20, height=20, bg="gray")
        self.egun_indicator.grid(row=0, column=3, padx=5, pady=5)
        self.egun_indicator.create_oval(2, 2, 18, 18, fill="gray", tags="egun_indicator")
        
        # Sequence controls
        ttk.Button(frame, text="Program Sequence", command=self.program_sequence,
                   style="Sequence.TButton").grid(
            row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        ttk.Button(frame, text="Run Sequence", command=self.run_sequence,
                   style="Sequence.TButton").grid(
            row=1, column=2, padx=5, pady=5)
        
        ttk.Button(frame, text="Cancel Sequence", command=self.cancel_sequence,
                   style="Stop.TButton").grid(
            row=1, column=3, padx=5, pady=5)
        
        # Sequence status
        ttk.Label(frame, text="Sequence:").grid(row=2, column=0, padx=5, pady=5, sticky="e")
        self.sequence_status_var = tk.StringVar(value="Not programmed")
        ttk.Label(frame, textvariable=self.sequence_status_var).grid(
            row=2, column=1, columnspan=3, padx=5, pady=5, sticky="w")
    
    def create_status_section(self, parent):
        """Create the status log section"""
        frame = ttk.LabelFrame(parent, text="Status Log", padding="5")
        frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Status log text
        self.status_text = tk.Text(frame, height=8, width=50)
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Add a scrollbar
        scrollbar = ttk.Scrollbar(frame, orient="vertical", command=self.status_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_text.config(yscrollcommand=scrollbar.set)
    
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        available_ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = available_ports
        if available_ports:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """Connect to or disconnect from the selected port"""
        if not self.connected:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "No port selected")
                return
            
            try:
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                self.connected = True
                self.connect_btn.config(text="Disconnect")
                self.log_status(f"Connected to {port}")
                
                # Start reading thread
                self.stop_thread = False
                self.read_thread = threading.Thread(target=self.read_serial)
                self.read_thread.daemon = True
                self.read_thread.start()
                
                # Get initial status after connecting
                time.sleep(0.5)  # Short delay to ensure connection is established
                self.get_status()
                
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
        else:
            # Disconnect
            self.stop_thread = True
            if hasattr(self, 'read_thread'):
                self.read_thread.join(timeout=1.0)
            
            if self.serial_port:
                self.serial_port.close()
            
            self.connected = False
            self.connect_btn.config(text="Connect")
            self.log_status("Disconnected")
    
    def send_command(self, command):
        """Send a command to the Arduino"""
        if not self.connected or not self.serial_port:
            messagebox.showerror("Error", "Not connected to Arduino")
            return False
        
        try:
            self.serial_port.write(f"{command}\n".encode())
            self.log_status(f"Sent: {command}")
            return True
        except Exception as e:
            self.log_status(f"Error sending command: {str(e)}")
            return False
    
    def read_serial(self):
        """Thread function to read data from the Arduino"""
        while not self.stop_thread:
            if self.serial_port and self.serial_port.in_waiting > 0:
                try:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.log_status(f"Arduino: {line}")
                        self.process_arduino_message(line)
                except Exception as e:
                    self.log_status(f"Error reading: {str(e)}")
            time.sleep(0.1)
    
    def process_arduino_message(self, message):
        """Process messages from the Arduino and update UI accordingly"""
        # Check for kill switch status
        if "KILL SWITCH ACTIVATED" in message:
            self.limit_status = True
            self.root.after(0, self.update_kill_indicator)
        elif "KILL SWITCH DEACTIVATED" in message:
            self.limit_status = False
            self.root.after(0, self.update_kill_indicator)
        
        # Check for position information
        position_match = re.search(r"Current position: (-?\d+\.?\d*)mm", message)
        if position_match:
            try:
                self.position_mm = float(position_match.group(1))
                self.root.after(0, self.update_position_display)
            except ValueError:
                self.log_status("Error parsing position value")
        
        # Also check for position in status output
        position_match = re.search(r"Position: (-?\d+\.?\d*)mm", message)
        if position_match:
            try:
                self.position_mm = float(position_match.group(1))
                self.root.after(0, self.update_position_display)
            except ValueError:
                self.log_status("Error parsing position value")
                
        # Check for home/load position mentions
        if "(HOME position)" in message:
            self.root.after(0, lambda: self.position_status_var.set("(HOME)"))
            self.at_load_position = False
        elif "(LOAD position)" in message:
            self.root.after(0, lambda: self.position_status_var.set("(LOAD)"))
            self.at_load_position = True
        
        # Special case for LOAD position reached
        if "LOAD POSITION REACHED" in message:
            self.at_load_position = True
            self.root.after(0, lambda: self.position_status_var.set("(LOAD)"))
        
        # Check for load command status
        if "LOAD command is now allowed" in message or "LOAD command: ALLOWED" in message:
            self.load_allowed = True
            self.root.after(0, lambda: self.load_btn.config(state=tk.NORMAL))
        elif "LOAD command disabled" in message or "LOAD command: NOT ALLOWED" in message:
            self.load_allowed = False
            self.root.after(0, lambda: self.load_btn.config(state=tk.DISABLED))
            
        # Check for Moving to LOAD position
        if "Moving to LOAD position" in message:
            self.at_load_position = True
            
        # Check for eGun status
        if "eGun ON" in message:
            self.egun_active = True
            self.root.after(0, self.update_egun_indicator)
        elif "eGun OFF" in message:
            self.egun_active = False
            self.root.after(0, self.update_egun_indicator)
            
        # Check for sequence status
        if "Starting sequence execution" in message:
            self.sequence_running = True
            self.root.after(0, lambda: self.sequence_status_var.set("Running"))
        elif "Sequence canceled" in message or "Sequence completed" in message:
            self.sequence_running = False
            if "Sequence completed" in message:
                self.root.after(0, lambda: self.sequence_status_var.set("Completed"))
            else:
                self.root.after(0, lambda: self.sequence_status_var.set(f"Programmed ({self.total_sequence_steps} steps)"))
            
        # Check for sequence step updates
        sequence_step_match = re.search(r"Sequence step (\d+)/(\d+)", message)
        if sequence_step_match:
            current_step = int(sequence_step_match.group(1))
            total_steps = int(sequence_step_match.group(2))
            self.total_sequence_steps = total_steps
            self.root.after(0, lambda: self.sequence_status_var.set(f"Running step {current_step}/{total_steps}"))
    
    def update_kill_indicator(self):
        """Update the kill switch indicator color"""
        color = "green" if self.limit_status else "red"
        self.kill_indicator.itemconfig("indicator", fill=color)
    
    def update_egun_indicator(self):
        """Update the eGun status indicator"""
        color = "blue" if self.egun_active else "gray"
        self.egun_indicator.itemconfig("egun_indicator", fill=color)
    
    def update_position_display(self):
        """Update the position display in the UI"""
        self.position_var.set(f"{self.position_mm:.1f} mm")
        
        # Update position indicator based on position
        if abs(self.position_mm) < 0.1:
            self.position_status_var.set("(HOME)")
        elif abs(self.position_mm - 240.0) < 0.1 or self.at_load_position:
            self.position_status_var.set("(LOAD)")
        else:
            self.position_status_var.set("")
    
    def motor_on(self):
        """Turn motor ON"""
        self.send_command("ON")
    
    def motor_off(self):
        """Turn motor OFF"""
        self.send_command("OFF")
    
    def direction_forward(self):
        """Set direction to forward"""
        self.send_command("FWD")
    
    def direction_reverse(self):
        """Set direction to reverse"""
        self.send_command("REV")
    
    def set_frequency(self):
        """Set pulse frequency"""
        try:
            freq = int(self.freq_var.get())
            if freq <= 0:
                messagebox.showerror("Error", "Frequency must be positive")
                return
            self.send_command(f"FREQ:{freq}")
        except ValueError:
            messagebox.showerror("Error", "Invalid frequency value")
    
    def move(self):
        """Start movement with specified number of pulses"""
        try:
            pulses = int(self.pulses_var.get())
            if pulses <= 0:
                messagebox.showerror("Error", "Number of pulses must be positive")
                return
            self.send_command(f"MOVE:{pulses}")
        except ValueError:
            messagebox.showerror("Error", "Invalid number of pulses")
    
    def move_mm(self):
        """Move a specified distance in millimeters"""
        try:
            mm = float(self.mm_var.get())
            if mm == 0:
                messagebox.showerror("Error", "Distance cannot be zero")
                return
            self.send_command(f"MOVE_MM:{mm}")
            # Reset LOAD position when moving
            self.at_load_position = False
        except ValueError:
            messagebox.showerror("Error", "Invalid distance value")
    
    def stop(self):
        """Stop motor movement and cancel sequence if running"""
        self.send_command("STOP")
    
    def home(self):
        """Start homing sequence"""
        self.send_command("HOME")
        # Reset LOAD position flag when homing
        self.at_load_position = False
    
    def zero_position(self):
        """Set current position to zero"""
        self.send_command("ZERO")
    
    def get_position(self):
        """Get current position"""
        self.send_command("POS")
    
    def load_position(self):
        """Move to load position"""
        self.send_command("LOAD")
        # Set the LOAD position flag
        self.at_load_position = True
    
    def get_status(self):
        """Get system status"""
        self.send_command("STATUS")
    
    def egun_on(self):
        """Turn eGun ON (5V on pin 11)"""
        self.send_command("EGUN_ON")
    
    def egun_off(self):
        """Turn eGun OFF (0V on pin 11)"""
        self.send_command("EGUN_OFF")
    
    def program_sequence(self):
        """Program a sequence of sample positions"""
        # First check if we're at LOAD position
        if self.at_load_position or "(LOAD)" in self.position_status_var.get():
            pass
        else:
            if messagebox.askyesno("Confirm", "The system should be at the LOAD position (zeroed) before programming a sequence. Do you want to move to LOAD position first?"):
                self.load_position()
                # Wait until movement completes
                return
        
        # Ask for number of sample positions
        num_steps = simpledialog.askinteger("Sequence Setup", "Enter number of sample positions:", minvalue=1, maxvalue=10)
        if not num_steps:
            return
            
        self.total_sequence_steps = num_steps
        
        # Send number of steps to Arduino
        self.send_command(f"SEQ_STEPS:{num_steps}")
        
        # Configure each position
        for i in range(num_steps):
            # Create a dialog for this step
            step_dialog = tk.Toplevel(self.root)
            step_dialog.title(f"Configure Sample Position {i+1}")
            step_dialog.geometry("400x300")
            step_dialog.transient(self.root)
            step_dialog.grab_set()
            
            # Position frame
            pos_frame = ttk.Frame(step_dialog, padding="10")
            pos_frame.pack(fill=tk.BOTH, expand=True)
            
            # Get position
            ttk.Label(pos_frame, text=f"Position {i+1} (mm from LOAD):").grid(row=0, column=0, padx=5, pady=5, sticky="w")
            pos_var = tk.StringVar()
            ttk.Entry(pos_frame, textvariable=pos_var, width=10).grid(row=0, column=1, padx=5, pady=5)
            
            # Get pre-delay
            ttk.Label(pos_frame, text="Pre-eGun delay (seconds):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
            pre_var = tk.StringVar(value="1.0")
            ttk.Entry(pos_frame, textvariable=pre_var, width=10).grid(row=1, column=1, padx=5, pady=5)
            
            # Get eGun time
            ttk.Label(pos_frame, text="eGun activation time (seconds):").grid(row=2, column=0, padx=5, pady=5, sticky="w")
            egun_var = tk.StringVar(value="5.0")
            ttk.Entry(pos_frame, textvariable=egun_var, width=10).grid(row=2, column=1, padx=5, pady=5)
            
            # Get post-delay
            ttk.Label(pos_frame, text="Post-eGun delay (seconds):").grid(row=3, column=0, padx=5, pady=5, sticky="w")
            post_var = tk.StringVar(value="1.0")
            ttk.Entry(pos_frame, textvariable=post_var, width=10).grid(row=3, column=1, padx=5, pady=5)
            
            # Help text
            help_text = """
Position: ABSOLUTE distance in mm BACKWARD from LOAD position
    - All positions must be POSITIVE values
    - The system always moves BACKWARD from LOAD
    
    Example: If position 1 = 5mm and position 2 = 10mm,
    the system will first move 5mm backward from LOAD,
    then move to 10mm backward from LOAD (5mm more).

Pre-delay: Time to wait after reaching position
eGun time: Duration to keep eGun ON (pin 11 HIGH)
Post-delay: Time to wait after eGun turns OFF
            """
            help_label = ttk.Label(pos_frame, text=help_text, justify=tk.LEFT)
            help_label.grid(row=4, column=0, columnspan=2, padx=5, pady=5, sticky="w")
            
            # OK button 
            def save_position():
                try:
                    pos = float(pos_var.get())
                    pre = float(pre_var.get())
                    egun = float(egun_var.get())
                    post = float(post_var.get())
                    
                    # Send values to Arduino
                    self.send_command(f"SEQ_POS:{i}:{pos}")
                    self.send_command(f"SEQ_PRE:{i}:{pre}")
                    self.send_command(f"SEQ_EGUN:{i}:{egun}")
                    self.send_command(f"SEQ_POST:{i}:{post}")
                    
                    step_dialog.destroy()
                except ValueError:
                    messagebox.showerror("Error", "All values must be valid numbers")
            
            ttk.Button(pos_frame, text="OK", command=save_position).grid(row=5, column=0, columnspan=2, padx=5, pady=10)
            
            # Wait for this dialog to close before continuing
            self.root.wait_window(step_dialog)
        
        # Update status
        self.sequence_status_var.set(f"Programmed ({num_steps} steps)")
        messagebox.showinfo("Sequence Programmed", f"Sequence with {num_steps} positions has been programmed successfully.")
    
    def run_sequence(self):
        """Start the programmed sequence"""
        if self.total_sequence_steps <= 0:
            messagebox.showerror("Error", "No sequence has been programmed")
            return
            
        if messagebox.askyesno("Confirm", "Start the programmed sequence? Make sure samples are ready."):
            self.send_command("RUN_SEQUENCE")
    
    def cancel_sequence(self):
        """Cancel the running sequence"""
        if self.sequence_running:
            if messagebox.askyesno("Confirm", "Cancel the running sequence?"):
                self.send_command("CANCEL_SEQUENCE")
        else:
            messagebox.showinfo("Information", "No sequence is currently running")
    
    def log_status(self, message):
        """Add a message to the status log"""
        self.status_text.configure(state='normal')
        self.status_text.insert(tk.END, message + "\n")
        self.status_text.see(tk.END)
        self.status_text.configure(state='disabled')
    
    def on_closing(self):
        """Handle window closing"""
        if self.connected:
            self.toggle_connection()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = StepperControlGUI(root)
    root.mainloop()
