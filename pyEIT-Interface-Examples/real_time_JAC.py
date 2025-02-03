import serial
import time
import numpy as np
import os
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import pyeit.eit.jac as jac
import pyeit.mesh as mesh
from pyeit.eit.fem import EITForward
from pyeit.eit.interp2d import sim2pts
from pyeit.mesh.shape import thorax
import pyeit.eit.protocol as protocol
from pyeit.mesh.wrapper import PyEITAnomaly_Circle
import matplotlib.animation as animation
from timeit import default_timer as timer

# Ensure the "data" folder exists
data_folder = "./data"
os.makedirs(data_folder, exist_ok=True)

# Port Communication
port = "COM7"
baudrate = 115200
buffer_size = 1000
data_bits = 8  
stop_bits = serial.STOPBITS_ONE  
parity = serial.PARITY_ODD  

uart_delay = 0

# Frequency
fs = 2e6
f_t = 10e3

# PGA
gainDB = 60 
gainLIN = 80

# Excitation
freq = 10e3
freq_sel = 1

# Opening serial connection with the device
utom = serial.Serial(
    port=port,
    baudrate=baudrate,
    bytesize=data_bits,
    parity=parity,
    stopbits=stop_bits,
    timeout=1
)

# Measurement Definitions
UTOM_path = "./protocol"  # Define your UTOM_path here
stimulation_pattern = np.loadtxt(f"{UTOM_path}/UTOM_SW.txt", delimiter=",")

n_commands = len(stimulation_pattern)
stimulation_pattern = 2*(stimulation_pattern-1)
n_bytes = 2 * buffer_size
data = np.zeros(n_commands, dtype=np.float32)

# Set new frequency channel
command = bytearray([ord('E'), freq_sel, 0, 0, 0])
utom.write(command)

# Set new gain
command = bytearray([ord('G'), gainDB, gainLIN, 0, 0])
utom.write(command)

############################### EIT reconstruction setup ###################################################


""" 0. build mesh """
n_el = 16  # nb of electrodes
use_customize_shape = False
mesh_obj = mesh.create(n_el, h0=0.05)

# extract node, element, alpha
pts = mesh_obj.node
tri = mesh_obj.element
x, y = pts[:, 0], pts[:, 1]

""" 2. FEM simulation """
# setup EIT scan conditions
protocol_obj = protocol.create(n_el, dist_exc=1, step_meas=1, parser_meas="std")
# calculate simulated data
fwd = EITForward(mesh_obj, protocol_obj)
sim_data = fwd.solve_eit()
sim_signs = np.where(sim_data>0,1,-1)
eit = jac.JAC(mesh_obj, protocol_obj)
eit.setup(p=0.5, lamb=0.2, method="kotre", perm=1, jac_normalized=True)


####################################################################################################################

def find_harmonic(data, sampling_rate, target_frequency):
    n = len(data)
    fft_result = np.fft.fft(data)
    fft_magnitude = np.abs(fft_result)[:n // 2]  # Magnitude of FFT (positive frequencies)
    freqs = np.fft.fftfreq(n, d=1 / sampling_rate)[:n // 2]  # Frequency bins

    # Find the index of the target frequency
    delta_f = sampling_rate / n  # Frequency resolution
    target_index = int(target_frequency / delta_f)

    # Get the magnitude of the harmonic at the target frequency
    harmonic_magnitude = fft_magnitude[target_index]

    return harmonic_magnitude

def normalize_array(arr, axis=None):

    arr_max = np.max(arr, axis=axis, keepdims=True)
    return arr / arr_max

############################################## REFERENCE SAMPLE ####################################################

channel_ant = stimulation_pattern[-1, :].astype(np.uint8)


avg = 4
for i in range(n_commands):

    data[i] = 0
    
    channel = stimulation_pattern[i, :].astype(np.uint8)
    for ch in np.arange(len(channel)):
            if channel[ch] > 15:
                channel[ch] = 47-channel[ch]
        # Set new channel
    command = bytearray([ord('S')]) + bytearray(channel)
    utom.write(command)
    if(channel_ant[0] != channel[0]) & (channel_ant[1] != channel[1]):
        time.sleep(0.02)    # Start measurement  

    for j in range(avg):   
            
        # Start measurement
        command = bytearray([ord('F')]) + bytearray(channel)
        utom.write(command)
        
        raw_data = utom.read(4)
        #print(raw_data)
        # Convert binary data to floar and assign
        data[i] = (np.frombuffer(raw_data, dtype=np.float32)[0] + data[i])/avg
        
    channel_ant = np.copy(channel)
    


# End of Communication and Data Collection
# Collects data

################################ EIT reconstruction #####################################
v0 = normalize_array(np.copy(data))
#v0 = np.copy(harm_data)
ds = eit.solve(v0, v0, normalize=True)
ds_n = sim2pts(pts, tri, np.real(ds))
################################ Plotting setup/reference ###############################

# Create a grid layout with gridspec
fig = plt.figure(figsize=(15, 8))
gs = GridSpec(2, 2, width_ratios=[2, 1], height_ratios=[1, 1])

# Plot time-domain data
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(v0)
ax1.set_xlabel("Measurement")
ax1.set_ylabel("Magnitude")
ax1.set_title("Data")
ax1.legend()
ax1.grid(True)

# Plot frequency-domain data
ax2 = fig.add_subplot(gs[1, 0])
ax2.plot(v0-v0)
ax2.set_xlabel("Measurement")
ax2.set_ylabel("Magnitude")
ax2.set_title("Difference")
ax2.legend()
ax2.grid(True)

# Plot EIT reconstruction (right side)
ax3 = fig.add_subplot(gs[:, 1])  # Spans both rows
im = ax3.tripcolor(x, y, tri, ds_n, shading="flat")
for i, e in enumerate(mesh_obj.el_pos):
    ax3.annotate(str(i + 1), xy=(x[e], y[e]), color="r")
ax3.set_aspect("equal")
ax3.set_title("EIT Reconstruction")
ax3.set_xlim(-1,1)

# Interactive update for EIT plot
plt.ion()  # Turn on interactive mode
plt.tight_layout()  # Adjust layout to prevent overlaps
plt.show()
plt.pause(0.1) 

############################################## TDEIT ####################################################

while(1):
    
    avg = 4
    for i in range(n_commands):

        data[i] = 0
        
        channel = stimulation_pattern[i, :].astype(np.uint8)
        for ch in np.arange(len(channel)):
                if channel[ch] > 15:
                    channel[ch] = 47-channel[ch]
            # Set new channel
        command = bytearray([ord('S')]) + bytearray(channel)
        utom.write(command)

        if(channel_ant[0] != channel[0]) & (channel_ant[1] != channel[1]):
            time.sleep(0.02)    # Start measurement  

        for j in range(avg):       
            # Start measurement
            command = bytearray([ord('F')]) + bytearray(channel)
            utom.write(command)
            
            raw_data = utom.read(4)
            #print(raw_data)
            # Convert binary data to floar and assign
            data[i] = (np.frombuffer(raw_data, dtype=np.float32)[0] + data[i])/avg
        channel_ant = np.copy(channel)
    


    # End of Communication and Data Collection
    # Collects data

    ################################ EIT reconstruction #####################################
    v1 = normalize_array(np.copy(data))
    ax1.lines[0].set_ydata(v1)  # Update frequency-domain plot
    ax2.lines[0].set_ydata(v1-v0)  # Update frequency-domain plot

    #v1 = np.copy(harm_data)
    
    ds = eit.solve(v1, v0, normalize=True)
    
    ds_n = sim2pts(pts, tri, np.real(ds))
    # Update the tripcolor plot
    tic = timer()
    im = ax3.tripcolor(x, y, tri, ds_n, shading="flat")
    for i, e in enumerate(mesh_obj.el_pos):
        ax3.annotate(str(i + 1), xy=(x[e], y[e]), color="r")
    
    plt.draw()
    plt.pause(0.01) 
    toc = timer()
    print(toc-tic)
