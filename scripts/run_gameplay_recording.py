import subprocess
import time
import os
import ctypes
import glob
import json
from PIL import Image

# Win32 Key Codes
VK_W = 0x57
VK_A = 0x41
VK_S = 0x53
VK_D = 0x44
VK_SPACE = 0x20
VK_P = 0x50
VK_R = 0x52
VK_Q = 0x51
VK_E = 0x45
VK_F1 = 0x70
VK_F2 = 0x71
VK_F3 = 0x72

user32 = ctypes.windll.user32

def key_down(vk):
    user32.keybd_event(vk, 0, 0, 0)

def key_up(vk):
    user32.keybd_event(vk, 0, 2, 0)

def press_key(vk, key_name, event_log, video_start, duration=0.05):
    ts_start = round(time.time() - video_start, 3)
    event_log.append({
        "timestamp_s": ts_start,
        "maneuver": f"Key Press ({key_name})",
        "keys": [vk],
        "action": "start"
    })
    key_down(vk)
    time.sleep(duration)
    key_up(vk)
    ts_end = round(time.time() - video_start, 3)
    event_log.append({
        "timestamp_s": ts_end,
        "maneuver": f"Key Press ({key_name})",
        "keys": [vk],
        "action": "end"
    })

def focus_window():
    ps_cmd = '$proc = Get-Process -Name drifty -ErrorAction SilentlyContinue; if ($proc) { $c = @"\nusing System;\nusing System.Runtime.InteropServices;\npublic class K { [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h); }\n"@\n Add-Type -TypeDefinition $c -ErrorAction SilentlyContinue; [K]::SetForegroundWindow($proc.MainWindowHandle) }'
    subprocess.run(["powershell", "-Command", ps_cmd], capture_output=True)

def drive_hold(keys_down, duration_s, maneuver_name, event_log, video_start, tick_interval=0.02):
    end_time = time.time() + duration_s
    event_log.append({
        "timestamp_s": round(time.time() - video_start, 3),
        "maneuver": maneuver_name,
        "keys": keys_down,
        "action": "start"
    })
    while time.time() < end_time:
        focus_window()
        for k in keys_down:
            key_down(k)
        time.sleep(tick_interval)
    for k in keys_down:
        key_up(k)
    event_log.append({
        "timestamp_s": round(time.time() - video_start, 3),
        "maneuver": maneuver_name,
        "keys": keys_down,
        "action": "end"
    })

def main():
    os.makedirs("recording_output", exist_ok=True)
    video_path = os.path.abspath("recording_output/gameplay_full.mkv")
    if os.path.exists(video_path):
        try:
            os.remove(video_path)
        except Exception:
            pass

    print("Starting ffmpeg window recording...")
    ffmpeg_proc = subprocess.Popen([
        "ffmpeg", "-y", "-f", "gdigrab", "-framerate", "30",
        "-i", 'title=Drifty - Phase 2',
        "-c:v", "libx264", "-pix_fmt", "yuv420p",
        video_path
    ], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    video_start = time.time()
    event_log = []

    time.sleep(1.0)
    focus_window()

    print("Starting Gameplay Test Sequence...")
    # Reset car first
    press_key(VK_R, "R (Reset)", event_log, video_start, 0.1)
    time.sleep(0.5)

    # 1. Full Throttle Acceleration (W for 4s)
    print("Maneuver 1: Straight line acceleration...")
    drive_hold([VK_W], 4.0, "Straight line acceleration", event_log, video_start)
    time.sleep(0.5)

    # 2. High Speed Turn & Scandinavian Flick (W + A for 1.2s, then W + D for 1.5s)
    print("Maneuver 2: Scandinavian flick & inertia drift...")
    drive_hold([VK_W, VK_A], 1.2, "Scandinavian flick entry (Left)", event_log, video_start)
    drive_hold([VK_W, VK_D], 1.5, "Scandinavian flick flick (Right)", event_log, video_start)
    time.sleep(0.5)

    # 3. Handbrake Turn (W + A + SPACE for 1s, then W + D countersteer)
    print("Maneuver 3: Handbrake entry & slide...")
    drive_hold([VK_W, VK_A, VK_SPACE], 1.0, "Handbrake turn entry", event_log, video_start)
    drive_hold([VK_W, VK_D], 1.5, "Handbrake turn countersteer", event_log, video_start)
    time.sleep(0.5)

    # 4. Off-road grass driving
    print("Maneuver 4: Driving off-road onto grass...")
    drive_hold([VK_W, VK_D], 2.5, "Off-road grass shoulder driving", event_log, video_start)
    time.sleep(0.5)

    # 5. Enable Diagnostics and Physics Lab overlays
    print("Maneuver 5: Toggle F1 Diagnostics and drive...")
    press_key(VK_F1, "F1 (Diagnostics)", event_log, video_start, 0.1)
    time.sleep(0.5)
    drive_hold([VK_W, VK_A], 1.5, "F1 Diagnostics driving", event_log, video_start)

    print("Maneuver 6: Toggle F2 Physics Lab and drive...")
    press_key(VK_F2, "F2 (Physics Lab)", event_log, video_start, 0.1)
    time.sleep(0.5)
    drive_hold([VK_W, VK_D], 1.5, "F2 Physics Lab driving", event_log, video_start)

    print("Maneuver 7: Toggle F3 Inspector and drive...")
    press_key(VK_F3, "F3 (Inspector)", event_log, video_start, 0.1)
    time.sleep(0.5)
    drive_hold([VK_S], 1.0, "F3 Replay Inspector reverse", event_log, video_start)

    # Turn off overlays
    press_key(VK_F1, "F1 (Diagnostics)", event_log, video_start, 0.1)
    press_key(VK_F2, "F2 (Physics Lab)", event_log, video_start, 0.1)
    press_key(VK_F3, "F3 (Inspector)", event_log, video_start, 0.1)
    time.sleep(0.5)

    # Reset car to finish
    press_key(VK_R, "R (Reset)", event_log, video_start, 0.1)
    time.sleep(1.0)

    print("Terminating recording...")
    try:
        ffmpeg_proc.communicate(input=b'q\n', timeout=3)
    except Exception:
        ffmpeg_proc.terminate()
        try:
            ffmpeg_proc.wait(timeout=3)
        except Exception:
            ffmpeg_proc.kill()

    print(f"Recording finished: {video_path}")

    # 1. Extract keyframes at 2 FPS for higher temporal fidelity
    print("Extracting keyframes at 2 FPS...")
    subprocess.run([
        "ffmpeg", "-y", "-i", video_path,
        "-vf", "fps=2",
        "recording_output/frame_%03d.png"
    ], check=True)

    frames = sorted(glob.glob("recording_output/frame_*.png"))
    print(f"Extracted {len(frames)} keyframes.")

    # 2. Build frame manifest mapping frame numbers to timestamps and active maneuvers
    frame_manifest = []
    for idx, frame_path in enumerate(frames, start=1):
        timestamp_s = round((idx - 1) * 0.5, 2)
        active_maneuver = "Idle / Transition"
        for event in event_log:
            if event["action"] == "start":
                # Find matching end
                matching_end = next((e for e in event_log if e["action"] == "end" and e["maneuver"] == event["maneuver"]), None)
                end_ts = matching_end["timestamp_s"] if matching_end else event["timestamp_s"] + 5.0
                if event["timestamp_s"] <= timestamp_s <= end_ts:
                    active_maneuver = event["maneuver"]
                    break
        frame_manifest.append({
            "frame_id": f"frame_{idx:03d}.png",
            "timestamp_s": timestamp_s,
            "maneuver": active_maneuver,
            "path": frame_path
        })

    manifest_path = "recording_output/frame_manifest.json"
    with open(manifest_path, "w") as f:
        json.dump(frame_manifest, f, indent=2)
    print(f"Wrote frame manifest to {manifest_path}")

    # 3. Export full event log JSON
    log_path = "recording_output/input_event_trace.json"
    with open(log_path, "w") as f:
        json.dump(event_log, f, indent=2)
    print(f"Wrote input event trace to {log_path}")

    # 4. Compute frame-to-frame visual motion deltas
    motion_metrics = []
    for i in range(len(frames) - 1):
        try:
            img1 = Image.open(frames[i]).convert("L").resize((160, 90))
            img2 = Image.open(frames[i + 1]).convert("L").resize((160, 90))
            b1 = img1.tobytes()
            b2 = img2.tobytes()
            diff = sum(abs(a - b) for a, b in zip(b1, b2)) / len(b1)
            motion_metrics.append({
                "from_frame": os.path.basename(frames[i]),
                "to_frame": os.path.basename(frames[i + 1]),
                "timestamp_s": round((i + 1) * 0.5, 2),
                "mean_pixel_delta": round(diff, 2)
            })
        except Exception:
            pass

    metrics_path = "recording_output/motion_metrics.json"
    with open(metrics_path, "w") as f:
        json.dump(motion_metrics, f, indent=2)
    print(f"Wrote motion metrics to {metrics_path}")

if __name__ == "__main__":
    main()
