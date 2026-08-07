import subprocess
import time
import os
import ctypes

VK_W = 0x57
VK_D = 0x44
VK_R = 0x52

user32 = ctypes.windll.user32

def key_down(vk): user32.keybd_event(vk, 0, 0, 0)
def key_up(vk): user32.keybd_event(vk, 0, 2, 0)

def focus_window():
    ps_cmd = '$proc = Get-Process -Name drifty -ErrorAction SilentlyContinue; if ($proc) { $c = @"\nusing System;\nusing System.Runtime.InteropServices;\npublic class K { [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h); }\n"@\n Add-Type -TypeDefinition $c -ErrorAction SilentlyContinue; [K]::SetForegroundWindow($proc.MainWindowHandle) }'
    subprocess.run(["powershell", "-Command", ps_cmd], capture_output=True)

def drive_sustained_right():
    focus_window()
    # Reset car
    key_down(VK_R); time.sleep(0.05); key_up(VK_R)
    time.sleep(0.5)

    print("Driving full throttle right for 12 seconds to test right wall bounce...")
    end_time = time.time() + 12.0
    while time.time() < end_time:
        focus_window()
        key_down(VK_W)
        key_down(VK_D)
        time.sleep(0.02)
    key_up(VK_W)
    key_up(VK_D)

if __name__ == "__main__":
    drive_sustained_right()
