import json
import os
import sys
import argparse

MANIFEST_PATH = "recording_output/agent_review_manifest.json"
METRICS_PATH = "recording_output/motion_metrics.json"

def load_json(path):
    if not os.path.exists(path):
        return None
    with open(path, "r") as f:
        return json.load(f)

def main():
    parser = argparse.ArgumentParser(description="Query Agent Review Telemetry & Manifest Data")
    parser.add_argument("--timestamp", type=float, help="Query frame info at timestamp t (in seconds)")
    parser.add_argument("--maneuver", type=str, help="Search frames matching maneuver substring")
    parser.add_argument("--motion-spikes", dest="motion_spikes", action="store_true", help="Find frames with highest visual motion deltas")
    parser.add_argument("--summary", action="store_true", help="Print overall recording session summary")

    args = parser.parse_args()

    manifest = load_json(MANIFEST_PATH)
    metrics = load_json(METRICS_PATH)

    if manifest is None:
        print(f"Error: Manifest not found at {MANIFEST_PATH}. Run scripts/agent_record_and_review.py first.")
        sys.exit(1)

    if args.timestamp is not None:
        best_frame = min(manifest, key=lambda x: abs(x["timestamp_s"] - args.timestamp))
        print(f"=== Query Result for Timestamp {args.timestamp:.2f}s ===")
        print(json.dumps(best_frame, indent=2))
        return

    if args.maneuver is not None:
        matches = [f for f in manifest if args.maneuver.lower() in f["maneuver"].lower()]
        print(f"=== Found {len(matches)} frames matching maneuver '{args.maneuver}' ===")
        print(json.dumps(matches[:10], indent=2))
        return

    if args.motion_spikes:
        if metrics is None:
            print("Motion metrics not found.")
            return
        sorted_spikes = sorted(metrics, key=lambda x: x["mean_pixel_delta"], reverse=True)[:5]
        print("=== Top 5 Motion Delta Spikes (Peaks in Vehicle / Screen Motion) ===")
        print(json.dumps(sorted_spikes, indent=2))
        return

    print("=== Agent Review Session Summary ===")
    print(f"Total Keyframes: {len(manifest)}")
    if manifest:
        duration = manifest[-1]["timestamp_s"]
        print(f"Total Duration: {duration:.2f} seconds")
        maneuvers = sorted(list(set(f["maneuver"] for f in manifest)))
        print("Active Maneuvers Tracked:")
        for m in maneuvers:
            count = sum(1 for f in manifest if f["maneuver"] == m)
            print(f"  - {m}: {count} frames")

if __name__ == "__main__":
    main()
