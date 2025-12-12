#!/usr/bin/env python3
import sys
import time
import serial
import subprocess
import shlex
import re
import threading
import json
import os
import tempfile
from datetime import datetime
from pathlib import Path
from flask import Flask, render_template_string, jsonify, request
from flask_cors import CORS

import spotipy
from spotipy.oauth2 import SpotifyOAuth
from collections import deque

try:
    import camera
    import cv2
    DETECTION_AVAILABLE = True
except ImportError:
    print("WARNING: camp.py or cv2 not found. Object detection disabled.")
    DETECTION_AVAILABLE = False

#config
SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = 115200
TIMEOUT = 1.0

#api keys
SPOTIPY_CLIENT_ID = '2d81d1bb16e64018abdcace120144903'
SPOTIPY_CLIENT_SECRET = '788fd139086a4f4f825b32d20282810a'
SPOTIPY_REDIRECT_URI = 'http://127.0.0.1:8888/callback'

#playlist url
STUDY_PLAYLIST_URI = 'spotify:playlist:37i9dQZF1DX8NTLI2OoQ26'
RELAX_PLAYLIST_URI = 'spotify:playlist:37i9dQZF1DX4sWSpwq3LiO'

HEADLESS = os.environ.get('HEADLESS', 'false').lower() == 'true'

app = Flask(__name__)
CORS(app)

#global variables
spotify_client = None
QDBUS_CMD = None
ser_connection = None
notifications_muted = False

#states
state = {
    'active_device_id': None, 'active_device_name': None,
    'previous_device_id': None, 'previous_device_name': None,
    'active_mpris_player': None,
    'spotify_device_id': None, 'spotify_devices': [], 'spotify_now_playing': 'N/A',
    'detection_mode': 'Normal', 'detected_object': 'None',
    'last_gesture': None, 'now_playing': None,
    'status': 'Initializing...', 'logs': [],
    'temperature': { 'dht_temp': '--', 'humidity': '--', 'ambient_temp': '--', 'object_temp': '--' }
}

serial_buffer = {'in_temp_block': False, 'temp_lines': []}


#debug
def add_log(message):
    timestamp = datetime.now().strftime("%H:%M:%S")
    log_message = f"[{timestamp}] {message}"
    state['logs'].insert(0, log_message)
    if len(state['logs']) > 150: 
        state['logs'] = state['logs'][:150]
    print(log_message)

def run_cmd(cmd, timeout=10):
    try:
        p = subprocess.run(
            shlex.split(cmd) if isinstance(cmd, str) else cmd, 
            capture_output=True, text=True, timeout=timeout
        )
        return p.stdout.strip(), p.returncode == 0
    except Exception as e:
        add_log(f"CMD ERROR: {e}")
        return "", False

def get_clipboard_content():
    # Try wl-paste first (Wayland)
    out, success = run_cmd("wl-paste", timeout=2)
    if success and out:
        return out
    
    # Try xclip (X11)
    out, success = run_cmd("xclip -o -selection clipboard", timeout=2)
    if success and out:
        return out
    
    # Try xsel as another X11 alternative
    out, success = run_cmd("xsel --clipboard --output", timeout=2)
    if success and out:
        return out
    
    add_log("Could not get clipboard. Install 'wl-clipboard' (Wayland) or 'xclip' (X11)")
    add_log("  Wayland: sudo apt-get install wl-clipboard")
    add_log("  X11: sudo apt-get install xclip")
    return None

#media players and kde connect
def detect_qdbus():
    global QDBUS_CMD
    for cmd in ['qdbus6', 'qdbus-qt6', 'qdbus-qt5', 'qdbus']:
        if run_cmd([cmd, '--version'], 2)[1]:
            QDBUS_CMD = cmd
            add_log(f"D-Bus tool detected: {cmd}")
            return
    add_log("WARNING: No qdbus found. Media control will be limited.")

def diagnose_kde_connect():
    add_log("=== KDE Connect Diagnostics ===")
    
    version_out, version_ok = run_cmd("kdeconnect-cli --version", timeout=3)
    if version_ok:
        add_log(f"KDE Connect version: {version_out.splitlines()[0] if version_out else 'Unknown'}")
    else:
        add_log("ERROR: kdeconnect-cli not found or not working")
        return False
    
    # List all devices
    list_out, list_ok = run_cmd("kdeconnect-cli --list-devices", timeout=5)
    add_log(f"All devices:\n{list_out}")

    avail_out, avail_ok = run_cmd("kdeconnect-cli --list-available", timeout=5)
    add_log(f"Available devices:\n{avail_out}")
    
    # Check if any device is available
    devices = list_available_devices()
    if not devices:
        add_log("WARNING: No paired and reachable devices found!")
        add_log("Please pair your device using: kdeconnect-cli --pair -d <device_id>")
        return False
    
    for dev in devices:
        add_log(f"\nTesting device: {dev['name']} ({dev['id']})")
        
        # Test ping
        ping_out, ping_ok = run_cmd(f"kdeconnect-cli -d {dev['id']} --ping", timeout=5)
        add_log(f"  Ping test: {'OK' if ping_ok else 'FAILED'} - {ping_out}")
        
        # Check available commands
        help_out, _ = run_cmd(f"kdeconnect-cli -d {dev['id']} --help", timeout=3)
        if '--send-key' in help_out:
            add_log(f"  --send-key: Supported")
        else:
            add_log(f"  --send-key: NOT FOUND in help (may not be supported)")
        
        # Try a test key send
        test_out, test_ok = run_cmd(f"kdeconnect-cli -d {dev['id']} --send-key play", timeout=5)
        add_log(f"  Test key send: {'OK' if test_ok else 'FAILED'} - {test_out}")
    
    add_log("=== End Diagnostics ===")
    return True

def list_available_devices():
    out, s = run_cmd("kdeconnect-cli --list-available")
    devices = []
    if s:
        for line in out.splitlines():
            if ':' in line and "reachable" in line.lower():
                parts = line.split(':')
                if len(parts) >= 2:
                    name = parts[0].strip().strip('-').strip()
                    rest = parts[1]
                    # Extract device ID
                    match = re.search(r'([a-f0-9_]{16,})', rest, re.I)
                    if match:
                        devices.append({'name': name, 'id': match.group(1)})

    if not devices:
        add_log("Trying alternative device list parsing...")
        out2, s2 = run_cmd("kdeconnect-cli --list-devices")
        if s2:
            for line in out2.splitlines():
                if ':' in line:
                    match = re.search(r'([a-f0-9_]{16,})', line, re.I)
                    if match:
                        name = line.split(':')[0].strip().strip('-').strip()
                        devices.append({'name': name, 'id': match.group(1)})
    
    return devices

def control_remote_media(device_id, action, volume_delta=None):
    if not QDBUS_CMD:
        add_log("ERROR: qdbus not available for MPRIS control")
        return False
    
    # Find the MPRIS player for this device
    player = find_mpris_player_for_device(device_id)
    if not player:
        add_log(f"ERROR: No MPRIS player found for device {device_id}")
        add_log("Make sure media is playing on your device")
        return False
    
    try:
        if action == 'play':
            return send_mpris_command(player, 'PlayPause')
        
        elif action == 'next':
            return send_mpris_command(player, 'Next')
        
        elif action == 'previous':
            return send_mpris_command(player, 'Previous')
        
        elif action == 'stop':
            return send_mpris_command(player, 'Stop')
        
        elif action in ['volume_up', 'volume_down', 'mute']:
            # Get current volume
            out, success = run_cmd([
                QDBUS_CMD, player, '/org/mpris/MediaPlayer2',
                'org.freedesktop.DBus.Properties.Get',
                'org.mpris.MediaPlayer2.Player', 'Volume'
            ], timeout=3)
            
            if not success:
                add_log("WARNING: Could not get current volume from MPRIS player")
                return False
            
            try:
                current_volume = float(out.strip())
            except ValueError:
                add_log(f"WARNING: Invalid volume value: {out}")
                return False
            
            new_volume = current_volume
            
            if action == 'mute':
                new_volume = 0.0 if current_volume > 0.05 else 0.5
                add_log(f"Mute toggle: {current_volume:.2f} -> {new_volume:.2f}")
            
            elif action == 'volume_up':
                delta = volume_delta if volume_delta else 0.1
                new_volume = min(1.0, current_volume + abs(delta))
                add_log(f"Volume up: {current_volume:.2f} -> {new_volume:.2f}")
            
            elif action == 'volume_down':
                delta = volume_delta if volume_delta else 0.1
                new_volume = max(0.0, current_volume - abs(delta))
                add_log(f"Volume down: {current_volume:.2f} -> {new_volume:.2f}")
            
            success = run_cmd([
                QDBUS_CMD, player, '/org/mpris/MediaPlayer2',
                'org.freedesktop.DBus.Properties.Set',
                'org.mpris.MediaPlayer2.Player', 'Volume', str(new_volume)
            ], timeout=3)[1]
            
            if success:
                percentage = int(new_volume * 100)
                add_log(f"Volume set to {percentage}%")
            else:
                add_log("ERROR: Failed to set volume")
            
            return success
        
        else:
            add_log(f"ERROR: Unknown action '{action}'")
            return False
    
    except Exception as e:
        add_log(f"ERROR in control_remote_media: {e}")
        return False

def send_repeated_media_keys(device_id, key, count):
    """Send a media key multiple times"""
    add_log(f"Sending {count}x {key} to device")
    for _ in range(count):
        send_media_key(device_id, key)
        time.sleep(0.1) 

def get_all_mpris_players():
    players = []
    if not QDBUS_CMD:
        add_log("qdbus not available; cannot list MPRIS players.")
        return players

    out, success = run_cmd(QDBUS_CMD)
    if not success:
        add_log("qdbus failed to list services.")
        return players

    for line in out.splitlines():
        if 'org.mpris.MediaPlayer2' in line:
            service = line.strip()
            _, works = run_cmd([
                QDBUS_CMD, service, '/org/mpris/MediaPlayer2', 
                'org.freedesktop.DBus.Properties.Get', 
                'org.mpris.MediaPlayer2', 'Identity'
            ], timeout=1)
            name = service.split('.')[-1] if '.' in service else service
            players.append({'id': service, 'name': name, 'responsive': works})
    return players

def find_mpris_player_for_device(device_id=None):
    if not QDBUS_CMD: 
        return None
    
    players = get_all_mpris_players()
    if not players:
        add_log("No MPRIS players found.")
        return None

    if device_id:
        device_id_short = device_id[-8:].lower()
        for p in players:
            if device_id_short in p['id'].lower() and p['responsive']:
                add_log(f"Found matching MPRIS player: {p['id']}")
                return p['id']

    for p in players:
        if p['responsive']:
            add_log(f"Found active MPRIS player: {p['id']}")
            return p['id']
    
    add_log("Found MPRIS players, but none responsive. Returning first.")
    return players[0]['id']

def send_mpris_command(player, method):
    add_log(f"Sending MPRIS command '{method}' to player.")
    success = run_cmd([
        QDBUS_CMD, player, '/org/mpris/MediaPlayer2', 
        f'org.mpris.MediaPlayer2.Player.{method}'
    ])[1]
    if not success: 
        add_log(f"MPRIS command '{method}' failed.")
    return success

def get_now_playing(player):
    out, s = run_cmd([
        QDBUS_CMD, player, '/org/mpris/MediaPlayer2', 
        'org.freedesktop.DBus.Properties.Get', 
        'org.mpris.MediaPlayer2.Player', 'Metadata'
    ])
    if s:
        info = {'artist': 'Unknown Artist', 'title': 'Unknown Title'}
        for line in out.splitlines():
            if 'xesam:title' in line: 
                info['title'] = line.split(':', 1)[-1].strip().strip('"')
            elif 'xesam:artist' in line:
                artists = re.findall(r'string \"(.*?)\"', line)
                if artists: 
                    info['artist'] = " & ".join(artists)
        return f"{info['artist']} - {info['title']}"
    return "N/A"

def ping_device(device_id, message): 
    run_cmd(f'kdeconnect-cli -d {device_id} --ping-msg "{message}"')

def share_clipboard_to_device(device_id, content):
    cmd = f'kdeconnect-cli -d {device_id} --share-text "{content}"'
    out, success = run_cmd(cmd)
    
    if success:
        add_log("Clipboard sent via --share-text")
        return True
    
    add_log("Direct text share failed, trying file method...")
    try:
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.txt') as f:
            f.write(content)
            temp_path = f.name
        
        cmd = f'kdeconnect-cli -d {device_id} --share "{temp_path}"'
        out, success = run_cmd(cmd)
        
        try:
            os.unlink(temp_path)
        except:
            pass
        
        if success:
            add_log("Clipboard sent via temp file")
            return True
        else:
            add_log("Failed to share clipboard via both methods")
            return False
    except Exception as e:
        add_log(f"Error sharing clipboard: {e}")
        return False

def share_file_to_device(device_id, file_path):
    if not os.path.exists(file_path):
        add_log(f"File not found: {file_path}")
        return False
    
    cmd = f'kdeconnect-cli -d {device_id} --share "{file_path}"'
    out, success = run_cmd(cmd)
    
    if success:
        add_log(f"File sent: {os.path.basename(file_path)}")
        return True
    else:
        add_log(f"Failed to send file: {os.path.basename(file_path)}")
        return False

#spotify control
def init_spotify(force=False):
    global spotify_client
    if spotify_client and not force:
        return
    
    add_log("Initializing Spotify...")
    if not all([SPOTIPY_CLIENT_ID, SPOTIPY_CLIENT_SECRET]):
        add_log("Spotify credentials missing.")
        return
    
    try:
        auth = SpotifyOAuth(
            scope="user-modify-playback-state user-read-playback-state",
            client_id=SPOTIPY_CLIENT_ID, 
            client_secret=SPOTIPY_CLIENT_SECRET,
            redirect_uri=SPOTIPY_REDIRECT_URI, 
            open_browser=not HEADLESS, 
            cache_path="./.spotify_cache"
        )
        
        if HEADLESS:
            add_log("Running in HEADLESS mode - no browser will open")
            auth_url = auth.get_authorize_url()
            add_log(f"SPOTIFY AUTH URL: {auth_url}")
            print(f"\n{'='*70}")
            print(f"SPOTIFY AUTHORIZATION REQUIRED")
            print(f"Please visit this URL in a browser:")
            print(f"{auth_url}")
            print(f"{'='*70}\n")
        
        spotify_client = spotipy.Spotify(auth_manager=auth)
        spotify_client.me()
        add_log("Spotify authenticated successfully.")
    except Exception as e:
        spotify_client = None
        add_log(f"Spotify Init Error: {e}")

def toggle_notifications_muted(mute: bool):
    global notifications_muted
    _, success = run_cmd(f"dunstctl set-paused {'true' if mute else 'false'}")
    if success: 
        notifications_muted = mute
        add_log(f"Notifications {'Muted' if mute else 'Unmuted'}")
    else:
        add_log("dunstctl not available for notification control")

def activate_study_mode():
    if state['detection_mode'] == 'Study': 
        return
    add_log("Activating Study Mode...")
    state['detection_mode'] = 'Study'
    
    if spotify_client and state['spotify_device_id']:
        try:
            spotify_client.start_playback(
                device_id=state['spotify_device_id'], 
                context_uri=STUDY_PLAYLIST_URI
            )
            add_log("Started Spotify Study Playlist.")
        except Exception as e: 
            add_log(f"Spotify Error: {e}")
    
    toggle_notifications_muted(True)

def activate_relax_mode():
    if state['detection_mode'] == 'Relax': 
        return
    add_log("Activating Relax Mode...")
    state['detection_mode'] = 'Relax'
    
    if spotify_client and state['spotify_device_id']:
        try:
            spotify_client.start_playback(
                device_id=state['spotify_device_id'], 
                context_uri=RELAX_PLAYLIST_URI
            )
            add_log("Started Spotify Relax Playlist.")
        except Exception as e: 
            add_log(f"Spotify Error: {e}")
    
    toggle_notifications_muted(False)

def deactivate_modes():
    if state['detection_mode'] != 'Normal':
        add_log("Deactivating special modes...")
        state['detection_mode'] = 'Normal'
        
        if spotify_client and state['spotify_device_id']:
            try: 
                spotify_client.pause_playback(device_id=state['spotify_device_id'])
            except Exception as e: 
                add_log(f"Spotify Error: {e}")
        
        toggle_notifications_muted(False)

# =========================
# GESTURE ACTIONS
# =========================
def perform_gesture_action(gesture_id, gesture_meta=None):
    state['last_gesture'] = gesture_id
    gesture_meta = gesture_meta or {}
    
    add_log(f"Gesture {gesture_id} received: {gesture_meta}")

    if gesture_id == 3:
        add_log("HOLD Gesture: Toggling active KDE device...")
        if state['previous_device_id'] and state['active_device_id']:
            state['active_device_id'], state['previous_device_id'] = \
                state['previous_device_id'], state['active_device_id']
            state['active_device_name'], state['previous_device_name'] = \
                state['previous_device_name'], state['active_device_name']

            add_log(f"Switching to: {state['active_device_name']}")
            state['active_mpris_player'] = find_mpris_player_for_device(state['active_device_id'])

            if not state['active_mpris_player']:
                state['active_mpris_player'] = find_mpris_player_for_device()

            if state['active_mpris_player']:
                state['now_playing'] = get_now_playing(state['active_mpris_player'])

            add_log(f"Active device is now: {state['active_device_name']}")
            ping_device(state['active_device_id'], "Control Active")
        else:
            add_log("Toggle failed: Only one device has been active.")
        return
    if not state['active_device_id']:
        add_log("Gesture ignored: No active device detected.")
        return

    if gesture_id == 1:
        add_log("TAP: Play/Pause")
        control_remote_media(state['active_device_id'], 'play')

    elif gesture_id == 2:
        add_log("FLICK: Next Track")
        control_remote_media(state['active_device_id'], 'next')

    elif gesture_id == 4:
        volume_change = gesture_meta.get('volume_change')
        
        if volume_change is not None:
            try:
                vol_value = float(volume_change)
                if vol_value == 0:
                    add_log("Volume change is 0: muting")
                    control_remote_media(state['active_device_id'], 'mute')
                else:
                    delta = abs(vol_value) / 100.0
                    if vol_value > 0:
                        add_log(f"Volume Up by {delta:.2f}")
                        control_remote_media(state['active_device_id'], 'volume_up', delta)
                    else:
                        add_log(f"Volume Down by {delta:.2f} (negative value in UP gesture)")
                        control_remote_media(state['active_device_id'], 'volume_down', delta)
            except (ValueError, TypeError):
                add_log(f"Invalid volume_change value: {volume_change}, using default")
                control_remote_media(state['active_device_id'], 'volume_up', 0.1)
        else:
            add_log("UP: Volume Up (default +10%)")
            control_remote_media(state['active_device_id'], 'volume_up', 0.1)

    elif gesture_id == 5:
        volume_change = gesture_meta.get('volume_change')
        
        if volume_change is not None:
            try:
                vol_value = float(volume_change)
                if vol_value == 0:
                    add_log("Volume change is 0: muting")
                    control_remote_media(state['active_device_id'], 'mute')
                else:
                    delta = abs(vol_value) / 100.0
                    if vol_value < 0:
                        add_log(f"Volume Down by {delta:.2f}")
                        control_remote_media(state['active_device_id'], 'volume_down', delta)
                    else:
                        add_log(f"Volume Up by {delta:.2f} (positive value in DOWN gesture)")
                        control_remote_media(state['active_device_id'], 'volume_up', delta)
            except (ValueError, TypeError):
                add_log(f"Invalid volume_change value: {volume_change}, using default")
                control_remote_media(state['active_device_id'], 'volume_down', 0.1)
        else:
            add_log("DOWN: Volume Down (default -10%)")
            control_remote_media(state['active_device_id'], 'volume_down', 0.1)

    elif gesture_id == 6:
        content = get_clipboard_content()
        if content:
            add_log(f"Sharing clipboard to {state['active_device_name']}...")
            share_clipboard_to_device(state['active_device_id'], content)
        else:
            add_log("Clipboard is empty or xclip failed.")

    else:
        add_log(f"Unknown gesture ID: {gesture_id}")

# object detection
def detection_handler():
    if not DETECTION_AVAILABLE:
        add_log("Object detection disabled (missing dependencies)")
        return
    
    add_log("Preparing object detection...")
    picam2 = camp.initialize_camera()
    if picam2 is None: 
        add_log("Halting detection: camera failed.")
        return
    
    recent_labels = deque(maxlen=camp.SMOOTHING_FRAMES)
    last_printed_mode = None
    last_known_label = None

    
    try:
        while True:
            frame = picam2.capture_array()[:, :, :3]
            H, W = frame.shape[:2]
            cx, cy = W // 2, H // 2
            rw, rh = int(W * camp.roi_scale), int(H * camp.roi_scale)
            x1, y1, x2, y2 = cx - rw // 2, cy - rh // 2, cx + rw // 2, cy + rh // 2

            _, hsv = camp.preprocess(frame)
            masks = camp.get_masks(hsv)
            label, contour, centroid, area, combined_mask = camp.choose_best_contour(
                (cx, cy), (x1, y1, x2, y2), masks, camp.min_area, 
                roi_fallback_overlap=not camp.center_only_mode
            )
            
            try:
                obj_temp = float(state['temperature']['object_temp']) if state['temperature']['object_temp'] != '--' else None
                ambient_temp = float(state['temperature']['ambient_temp']) if state['temperature']['ambient_temp'] != '--' else None
                humidity = float(state['temperature']['humidity']) if state['temperature']['humidity'] != '--' else None
            except (ValueError, TypeError):
                obj_temp = None
                ambient_temp = None
                humidity = None

            if contour is not None and centroid is not None:
                ccx, ccy = centroid
                shp = camp.shape_of(contour)
                bx, by, bw, bh = cv2.boundingRect(contour)
                current_label, current_conf, _ = camp.classify_pen_book_bottle(
                    color_label=label, shape=shp, area=area, bbox=(bx, by, bw, bh), 
                    frame_size=(W, H), contour=contour,
                    obj_temp=obj_temp, ambient_temp=ambient_temp, 
                    humidity=humidity
                )
                recent_labels.append((current_label, current_conf))
                
                chosen_label, chosen_conf = current_label, current_conf
                if len(recent_labels) > 0:
                    counts = {lab: [item[0] for item in recent_labels].count(lab) for lab, _ in recent_labels}
                    best_lab = max(counts, key=counts.get)
                    if counts[best_lab] >= camp.SMOOTHING_REQUIRED:
                        chosen_label = best_lab

                state['detected_object'] = chosen_label

                current_mode = None
                if chosen_label in ("Pen", "Book"):
                    current_mode = "Study"
                elif chosen_label in ("Cup"):
                    current_mode = "Relax"
                
                if current_mode and (current_mode != last_printed_mode or chosen_label != last_known_label):
                    add_log(f"MODE: {current_mode} (Object: {chosen_label})")
                    last_printed_mode = current_mode
                    last_known_label = chosen_label
                    
                    if current_mode == "Study":
                        activate_study_mode()
                    elif current_mode == "Relax":
                        activate_relax_mode()
            else:
                recent_labels.clear()

            time.sleep(0.5)
            
    except Exception as e:
        add_log(f"Detection thread error: {e}")
        time.sleep(5)
    finally:
        picam2.stop()

# web page
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title></title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background: #ffffff;
            padding: 20px;
            color: #333;
            margin: 0;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        .header h1 {
            text-align: center;
            color: #1e40af;
            margin-bottom: 30px;
            font-size: 2em;
            font-weight: 600;
        }
        .card {
            background: white;
            border: 2px solid #3b82f6;
            border-radius: 8px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 2px 4px rgba(59, 130, 246, 0.1);
        }
        .card h2 {
            color: #1e40af;
            margin-bottom: 15px;
            font-size: 1.2em;
            font-weight: 600;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 20px;
        }
        select, button, input[type="file"] {
            width: 100%;
            padding: 10px;
            font-size: 1em;
            border: 2px solid #3b82f6;
            border-radius: 4px;
            margin-bottom: 10px;
            box-sizing: border-box;
        }
        button {
            background: #3b82f6;
            color: white;
            border: none;
            cursor: pointer;
            font-weight: 600;
        }
        button:hover {
            background: #2563eb;
        }
        .refresh-btn {
            background: #10b981;
        }
        .refresh-btn:hover {
            background: #059669;
        }
        .info-row {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #e5e7eb;
        }
        .info-row:last-child {
            border-bottom: none;
        }
        .info-label {
            font-weight: 600;
            color: #4b5563;
        }
        .info-value {
            text-align: right;
            color: #1f2937;
        }
        .logs {
            background: #f9fafb;
            border: 2px solid #3b82f6;
            color: #1f2937;
            padding: 15px;
            border-radius: 4px;
            max-height: 300px;
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 0.85em;
        }
        .gesture-legend {
            background: #eff6ff;
            border: 1px solid #3b82f6;
            padding: 12px;
            border-radius: 4px;
            margin-top: 10px;
            font-size: 0.85em;
        }
        .gesture-legend div {
            padding: 3px 0;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Multimodal Control Hub</h1>
        </div>
        
        <div class="grid">
            <div class="card">
                <h2>System Status</h2>
                <div class="info-row">
                    <span class="info-label">Mode:</span>
                    <span class="info-value" id="detectionMode">--</span>
                </div>
                <div class="info-row">
                    <span class="info-label">Detected Object:</span>
                    <span class="info-value" id="detectedObject">--</span>
                </div>
                <div class="info-row">
                    <span class="info-label">Last Gesture:</span>
                    <span class="info-value" id="lastGesture">--</span>
                </div>
                <div class="gesture-legend">
                    <strong>Gesture Map:</strong>
                    <div>1 = TAP (Play/Pause)</div>
                    <div>2 = FLICK (Next Track)</div>
                    <div>3 = HOLD (Toggle Device)</div>
                    <div>4 = UP (Volume Up)</div>
                    <div>5 = DOWN (Volume Down)</div>
                    <div>6 = CUSTOM (Send Clipboard)</div>
                </div>
            </div>

            <div class="card">
                <h2>Spotify Control</h2>
                <button class="refresh-btn" onclick="refreshSpotifyDevices()">Refresh Devices</button>
                <select id="spotifyDeviceSelect" onchange="selectSpotifyDevice()">
                    <option>-- Select Device --</option>
                </select>
                <div class="info-row">
                    <span class="info-label">Now Playing:</span>
                    <span class="info-value" id="spotifyNowPlaying">--</span>
                </div>
            </div>

            <div class="card">
                <h2>MPRIS Players</h2>
                <button class="refresh-btn" onclick="refreshMprisPlayers()">Refresh Players</button>
                <select id="mprisPlayerSelect" onchange="selectMprisPlayer()">
                    <option>-- Select Player --</option>
                </select>
                <div class="info-row">
                    <span class="info-label">Active Player:</span>
                    <span class="info-value" id="activePlayer">--</span>
                </div>
                <div class="info-row">
                    <span class="info-label">Now Playing:</span>
                    <span class="info-value" id="nowPlaying">--</span>
                </div>
            </div>

            <div class="card">
                <h2>KDE Connect Devices</h2>
                <button class="refresh-btn" onclick="refreshKdeDevices()">Refresh Devices</button>
                <select id="kdeDeviceSelect" onchange="selectKdeDevice()">
                    <option>-- Select Device --</option>
                </select>
                <div class="info-row">
                    <span class="info-label">Active:</span>
                    <span class="info-value" id="activeDevice">--</span>
                </div>
                <div class="info-row">
                    <span class="info-label">Previous:</span>
                    <span class="info-value" id="previousDevice">--</span>
                </div>
            </div>

            <div class="card">
                <h2>Actions</h2>
                <button onclick="sendClipboard()">Send Clipboard to Device</button>
                <button onclick="toggleNotifications()">Toggle Notifications Mute</button>
                <input type="file" id="fileInput" style="display:none" onchange="uploadFile()">
                <button onclick="document.getElementById('fileInput').click()">Upload File to Device</button>
                <button class="refresh-btn" onclick="runDiagnostics()">Run KDE Connect Diagnostics</button>
                <div class="info-row">
                    <span class="info-label">Notifications:</span>
                    <span class="info-value" id="notificationStatus">Active</span>
                </div>
            </div>

            <div class="card">
                <h2>Temperature Sensors</h2>
                <div class="info-row">
                    <span class="info-label">DHT Temperature:</span>
                    <span class="info-value" id="dhtTemp">--</span>
                </div>
                <div class="info-row">
                    <span class="info-label">Humidity:</span>
                    <span class="info-value" id="humidity">--</span>
                </div>
                <div class="info-row">
                    <span class="info-label">Ambient Temp:</span>
                    <span class="info-value" id="ambientTemp">--</span>
                </div>
                <div class="info-row">
                    <span class="info-label">Object Temp:</span>
                    <span class="info-value" id="objectTemp">--</span>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>System Logs</h2>
            <div class="logs" id="logs"></div>
        </div>
    </div>

    <script>
        function updateUI() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('detectionMode').textContent = data.detection_mode;
                    document.getElementById('detectedObject').textContent = data.detected_object;
                    document.getElementById('lastGesture').textContent = data.last_gesture || "None";
                    
                    document.getElementById('spotifyNowPlaying').textContent = data.spotify_now_playing;
                    document.getElementById('activeDevice').textContent = data.active_device_name || "None";
                    document.getElementById('previousDevice').textContent = data.previous_device_name || "None";
                    document.getElementById('activePlayer').textContent = 
                        data.active_mpris_player ? data.active_mpris_player.split('.').pop() : "None";
                    document.getElementById('nowPlaying').textContent = data.now_playing || "N/A";
                    
                    document.getElementById('dhtTemp').textContent = data.temperature.dht_temp + '°C';
                    document.getElementById('humidity').textContent = data.temperature.humidity + '%';
                    document.getElementById('ambientTemp').textContent = data.temperature.ambient_temp + '°C';
                    document.getElementById('objectTemp').textContent = data.temperature.object_temp + '°C';
                    
                    document.getElementById('logs').innerHTML = 
                        data.logs.map(e => `<div>${e}</div>`).join('');
                });
        }

        function refreshSpotifyDevices() {
            fetch('/api/spotify/devices/refresh')
                .then(r => r.json())
                .then(data => {
                    const sel = document.getElementById('spotifyDeviceSelect');
                    sel.innerHTML = '<option>-- Select Device --</option>';
                    data.devices.forEach(d => sel.add(new Option(d.name, d.id)));
                    if (data.selected) sel.value = data.selected;
                });
        }

        function selectSpotifyDevice() {
            fetch('/api/spotify/device/select', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    device_id: document.getElementById('spotifyDeviceSelect').value
                })
            });
        }

        function refreshMprisPlayers() {
            fetch('/api/mpris/players/refresh')
                .then(r => r.json())
                .then(data => {
                    const sel = document.getElementById('mprisPlayerSelect');
                    sel.innerHTML = '<option>-- Select Player --</option>';
                    data.players.forEach(p => {
                        let opt = new Option(p.name, p.id);
                        if (!p.responsive) opt.text += ' (unresponsive)';
                        sel.add(opt);
                    });
                    if (data.selected) sel.value = data.selected;
                });
        }

        function selectMprisPlayer() {
            fetch('/api/mpris/player/select', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    player_id: document.getElementById('mprisPlayerSelect').value
                })
            }).then(() => setTimeout(updateUI, 500));
        }

        function refreshKdeDevices() {
            fetch('/api/kde/devices/refresh')
                .then(r => r.json())
                .then(data => {
                    const sel = document.getElementById('kdeDeviceSelect');
                    sel.innerHTML = '<option>-- Select Device --</option>';
                    data.devices.forEach(d => sel.add(new Option(d.name, d.id)));
                    if (data.selected) sel.value = data.selected;
                });
        }

        function selectKdeDevice() {
            fetch('/api/kde/device/select', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    device_id: document.getElementById('kdeDeviceSelect').value
                })
            }).then(() => setTimeout(updateUI, 500));
        }

        function sendClipboard() {
            fetch('/api/kde/clipboard/send', {method: 'POST'})
                .then(r => r.json())
                .then(data => alert(data.message || 'Clipboard sent'));
        }

        function toggleNotifications() {
            fetch('/api/notifications/toggle', {method: 'POST'})
                .then(r => r.json())
                .then(data => {
                    document.getElementById('notificationStatus').textContent = 
                        data.muted ? 'Muted' : 'Active';
                });
        }

        function uploadFile() {
            const fileInput = document.getElementById('fileInput');
            const file = fileInput.files[0];
            if (!file) return;

            const formData = new FormData();
            formData.append('file', file);

            fetch('/api/kde/file/send', {
                method: 'POST',
                body: formData
            })
            .then(r => r.json())
            .then(data => alert(data.message || 'File sent'))
            .catch(e => alert('Error: ' + e));
        }

        function runDiagnostics() {
            alert('Running diagnostics... Check the System Logs section below for results.');
            fetch('/api/kde/diagnose')
                .then(r => r.json())
                .then(data => {
                    setTimeout(updateUI, 1000);
                });
        }

        setInterval(updateUI, 1500);
        window.onload = () => {
            updateUI();
            refreshSpotifyDevices();
            refreshMprisPlayers();
            refreshKdeDevices();
        };
    </script>
</body>
</html>
'''

@app.route('/')
def index(): 
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/status')
def api_status(): 
    return jsonify(state)

@app.route('/api/spotify/devices/refresh')
def api_spotify_devices_refresh():
    if not spotify_client:
        init_spotify()
    if not spotify_client:
        add_log("Spotify client unavailable for device refresh.")
        return jsonify({'devices': [], 'selected': state.get('spotify_device_id')})
    try:
        devices = spotify_client.devices()['devices']
        state['spotify_devices'] = [{'name': d['name'], 'id': d['id']} for d in devices]
        return jsonify({'devices': state['spotify_devices'], 'selected': state.get('spotify_device_id')})
    except Exception as e:
        add_log(f"Could not fetch Spotify devices: {e}")
        return jsonify({'devices': [], 'selected': state.get('spotify_device_id')})

@app.route('/api/spotify/device/select', methods=['POST'])
def api_spotify_device_select():
    state['spotify_device_id'] = request.json.get('device_id')
    dev_name = next((d['name'] for d in state['spotify_devices'] if d['id'] == state['spotify_device_id']), 'Unknown')
    add_log(f"Selected Spotify device: {dev_name}")
    return jsonify({'success': True})

@app.route('/api/mpris/players/refresh')
def api_mpris_players_refresh():
    players = get_all_mpris_players()
    return jsonify({'players': players, 'selected': state.get('active_mpris_player')})

@app.route('/api/mpris/player/select', methods=['POST'])
def api_mpris_player_select():
    player_id = request.json.get('player_id')
    if not player_id:
        return jsonify({'success': False, 'error': 'no player_id provided'}), 400
    state['active_mpris_player'] = player_id
    try:
        state['now_playing'] = get_now_playing(player_id)
    except Exception as e:
        add_log(f"Error updating now playing for selected player: {e}")
    add_log(f"Selected MPRIS player: {player_id.split('.')[-1]}")
    return jsonify({'success': True})

@app.route('/api/kde/devices/refresh')
def api_kde_devices_refresh():
    devices = list_available_devices()
    return jsonify({'devices': devices, 'selected': state.get('active_device_id')})

@app.route('/api/kde/device/select', methods=['POST'])
def api_kde_device_select():
    device_id = request.json.get('device_id')
    if not device_id:
        return jsonify({'success': False, 'error': 'no device_id provided'}), 400
    
    devs = list_available_devices()
    dev_name = next((d['name'] for d in devs if d['id'] == device_id), 'Unknown')
    
    if state.get('active_device_id') and state['active_device_id'] != device_id:
        state['previous_device_id'] = state['active_device_id']
        state['previous_device_name'] = state['active_device_name']
        add_log(f"Previous device saved: {state['previous_device_name']}")
    
    state.update({
        'active_device_id': device_id,
        'active_device_name': dev_name,
    })
    
    state['active_mpris_player'] = find_mpris_player_for_device(device_id)
    if not state['active_mpris_player']:
        add_log("Retrying player detection without device matching...")
        time.sleep(0.5)
        state['active_mpris_player'] = find_mpris_player_for_device()
    
    if state['active_mpris_player']:
        state['now_playing'] = get_now_playing(state['active_mpris_player'])
    else:
        state['now_playing'] = "N/A"
        add_log("No media player found. Start playing music on your device.")
    
    ping_device(device_id, f"Control Active: {dev_name}")
    add_log(f"KDE Active device set to: {dev_name}")
    return jsonify({'success': True})

@app.route('/api/kde/device/pair', methods=['POST'])
def api_kde_device_pair():
    data = request.json
    device_id = data.get('device_id')
    uid = data.get('uid')  
    
    if not device_id:
        return jsonify({'success': False, 'error': 'device_id required'}), 400
    
    devs = list_available_devices()
    dev = next((d for d in devs if d['id'] == device_id), None)
    
    if not dev:
        return jsonify({'success': False, 'error': 'device not found'}), 404
    
    add_log(f"Pairing with '{dev['name']}'. Please accept on your device.")
    out, success = run_cmd(f"kdeconnect-cli --pair -d {device_id}", 15)
    
    if success and uid and ser_connection:
        payload = f"[paired]UID={uid};device_id={device_id};device_name={dev['name']}[/paired]\n"
        ser_connection.write(payload.encode())
        ser_connection.flush()
        add_log("Pairing info sent to ESP32.")
    
    return jsonify({'success': success, 'device': dev})

@app.route('/api/kde/clipboard/send', methods=['POST'])
def api_kde_clipboard_send():
    if not state['active_device_id']:
        return jsonify({'success': False, 'message': 'No active device'}), 400
    
    content = get_clipboard_content()
    short = (content[:40] + "...") if len(content) > 40 else content
    if not content:
        return jsonify({'success': False, 'message': 'Clipboard is empty'}), 400
    
    success = share_clipboard_to_device(state['active_device_id'], content)
    return jsonify({
        'success': success, 
        'message': f"Clipboard sent: {content}" if success else 'Failed to send clipboard'
    })

@app.route('/api/kde/file/send', methods=['POST'])
def api_kde_file_send():
    if not state['active_device_id']:
        return jsonify({'success': False, 'message': 'No active device'}), 400
    
    if 'file' not in request.files:
        return jsonify({'success': False, 'message': 'No file provided'}), 400
    
    file = request.files['file']
    if file.filename == '':
        return jsonify({'success': False, 'message': 'No file selected'}), 400
    
    try:
        temp_dir = tempfile.gettempdir()
        temp_path = os.path.join(temp_dir, file.filename)
        file.save(temp_path)

        success = share_file_to_device(state['active_device_id'], temp_path)

        try:
            os.unlink(temp_path)
        except:
            pass
        
        return jsonify({
            'success': success,
            'message': f'File sent: {file.filename}' if success else 'Failed to send file'
        })
    except Exception as e:
        add_log(f"Error in file upload: {e}")
        return jsonify({'success': False, 'message': str(e)}), 500

@app.route('/api/notifications/toggle', methods=['POST'])
def api_notifications_toggle():
    """Toggle notification mute state"""
    global notifications_muted
    new_state = not notifications_muted
    toggle_notifications_muted(new_state)

    if state['active_device_id']:
        ping_device(state['active_device_id'], 
                   f"Notifications {'Muted' if new_state else 'Unmuted'}")
        send_media_key(state['active_device_id'], 'mute')
    
    return jsonify({'success': True, 'muted': notifications_muted})

# serial handling
def parse_message_body(body):
    result = {}
    for pair in body.split(';'):
        if '=' in pair:
            key, value = pair.split('=', 1)
            result[key.strip()] = value.strip()
    return result

def parse_temp_block(lines):
    result = {}
    for line in lines:
        line = line.strip()
        if '=' in line:
            key, value = line.split('=', 1)
            key = key.strip().upper()
            value = value.strip()

            if key in ['DHT', 'DHT_TEMP']:
                result['dht_temp'] = value
            elif key in ['HUM', 'HUMIDITY']:
                result['humidity'] = value
            elif key in ['AMB', 'AMBIENT', 'AMBIENT_TEMP']:
                result['ambient_temp'] = value
            elif key in ['OBJ', 'OBJECT', 'OBJECT_TEMP']:
                result['object_temp'] = value
    
    return result

def handle_register(uid, ser):
    add_log(f"New Tag Registration Request (UID: {uid})")
    devices = list_available_devices()
    
    if not devices:
        add_log("No KDE Connect devices available for pairing.")
        return

    print("\nReachable devices:")
    for i, dev in enumerate(devices, 1):
        print(f"  {i}) {dev['name']:<30} id={dev['id']}")

    try:
        choice = input("\nPick device number to pair (or 0 to cancel): ").strip()
        if not choice.isdigit() or not 0 < int(choice) <= len(devices):
            print("Canceled.")
            return

        dev = devices[int(choice) - 1]
        add_log(f"Pairing with '{dev['name']}'. Please accept on your device.")
        run_cmd(f"kdeconnect-cli --pair -d {dev['id']}", 15)

        payload = f"[paired]UID={uid};device_id={dev['id']};device_name={dev['name']}[/paired]\n"
        ser.write(payload.encode())
        ser.flush()
        add_log("Pairing info sent to ESP32.")

    except (KeyboardInterrupt, EOFError):
        print("\nCanceled by user.")

def handle_detected(uid, device_id, device_name):
    add_log(f"Tag Detected: {device_name} (UID: {uid})")

    if state['active_device_id'] and state['active_device_id'] != device_id:
        state['previous_device_id'] = state['active_device_id']
        state['previous_device_name'] = state['active_device_name']
        add_log(f"Previous device saved: {state['previous_device_name']}")

    state.update({
        'active_device_id': device_id,
        'active_device_name': device_name,
    })

    state['active_mpris_player'] = find_mpris_player_for_device(device_id)

    if not state['active_mpris_player']:
        add_log("Retrying player detection without device matching...")
        time.sleep(0.5)
        state['active_mpris_player'] = find_mpris_player_for_device()

    if state['active_mpris_player']:
        state['now_playing'] = get_now_playing(state['active_mpris_player'])
    else:
        state['now_playing'] = "N/A"
        add_log("No media player found. Start playing music on your device.")

    ping_device(device_id, f"Control Active: {device_name}")

def handle_removed(device_name):
    add_log(f"Tag Removed: {device_name}. Deactivating controls.")
    if state.get('active_device_id'):
        ping_device(state['active_device_id'], "Session Ended")
    state.update({
        'active_device_id': None, 'active_device_name': None,
        'active_mpris_player': None, 'now_playing': "N/A"
    })

def serial_handler():
    global ser_connection, serial_buffer
    
    try:
        with serial.Serial(SERIAL_PORT, BAUD, timeout=TIMEOUT) as ser:
            ser_connection = ser
            time.sleep(1.5)
            ser.reset_input_buffer()
            add_log("Serial connected. Listening for events...")
            state['status'] = 'Connected'

            while True:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    if line.startswith("[TEMP]") or line == "[TEMP]":
                        serial_buffer['in_temp_block'] = True
                        serial_buffer['temp_lines'] = []

                        if "[/TEMP]" in line:
                            content = line.replace("[TEMP]", "").replace("[/TEMP]", "").strip()
                            if content:
                                serial_buffer['temp_lines'].append(content)

                            temp_data = parse_temp_block(serial_buffer['temp_lines'])
                            state['temperature'].update(temp_data)
                            serial_buffer['in_temp_block'] = False
                            serial_buffer['temp_lines'] = []
                        continue
                    
                    if serial_buffer['in_temp_block']:
                        if line.endswith("[/TEMP]") or line == "[/TEMP]":
                            serial_buffer['in_temp_block'] = False
                            content = line.replace("[/TEMP]", "").strip()
                            if content:
                                serial_buffer['temp_lines'].append(content)
                            
                            temp_data = parse_temp_block(serial_buffer['temp_lines'])
                            state['temperature'].update(temp_data)
                            serial_buffer['temp_lines'] = []
                        else:
                            if line: 
                                serial_buffer['temp_lines'].append(line)
                        continue
                    
                    if line.startswith('[') and not line.startswith('[TEMP]'):
                        add_log(f"RX: {line}")

                    if line.startswith("[register]"):
                        items = parse_message_body(line[10:-11])
                        handle_register(items.get('UID'), ser)
                    
                    elif line.startswith("[detected]"):
                        items = parse_message_body(line[10:-11])
                        handle_detected(
                            items.get('UID'), 
                            items.get('device_id'), 
                            items.get('device_name')
                        )

                    elif line.startswith("[removed]"):
                        items = parse_message_body(line[9:-10])
                        handle_removed(items.get('device_name'))

                    elif line.startswith("[Gesture]"):
                        gesture_body = line[9:-10]  
                        gesture_data = parse_message_body(gesture_body)
                        
                        gesture_id = int(gesture_data.get('Gesture_id', 0))
                        
                        volume_change = gesture_data.get('volume_change')
                        if volume_change:
                            try:
                                gesture_data['volume_change'] = float(volume_change)
                            except (ValueError, TypeError):
                                pass
                        
                        perform_gesture_action(gesture_id, gesture_data)

                except (KeyboardInterrupt, SystemExit):
                    break
                except serial.SerialException as e:
                    add_log(f"Serial exception: {e}")
                    break
                except Exception as e:
                    add_log(f"Serial loop error: {e}")
                    time.sleep(1)

    except serial.SerialException as e:
        add_log(f"CRITICAL Serial error: {e}")
        state['status'] = 'Serial Error'

# main function
def main():
    print("="*70)
    print("Multimodal Control Hub v3.0 Started")
    print("="*70)
    
    detect_qdbus()
    if not run_cmd("kdeconnect-cli --version")[1]:
        print("ERROR: KDE Connect not found! Please install it.")
        print("  Ubuntu/Debian: sudo apt-get install kdeconnect")
        sys.exit(1)
    
    has_wl = run_cmd("wl-paste --version", timeout=2)[1]
    has_xclip = run_cmd("xclip -version", timeout=2)[1]
    has_xsel = run_cmd("xsel --version", timeout=2)[1]
    
    if not (has_wl or has_xclip or has_xsel):
        print("WARNING: No clipboard tool found. Clipboard sharing will not work.")
        print("  For Wayland: sudo apt-get install wl-clipboard")
        print("  For X11: sudo apt-get install xclip")
    else:
        if has_wl:
            print("✓ Clipboard: wl-clipboard (Wayland) detected")
        elif has_xclip:
            print("✓ Clipboard: xclip (X11) detected")
        elif has_xsel:
            print("✓ Clipboard: xsel (X11) detected")
    
    print("\nRunning KDE Connect diagnostics...")
    diagnose_kde_connect()

    if HEADLESS:
        add_log("Running in HEADLESS mode")
    init_spotify()
    
    threading.Thread(target=serial_handler, daemon=True).start()
    
    if DETECTION_AVAILABLE:
        threading.Thread(target=detection_handler, daemon=True).start()
    else:
        add_log("Object detection disabled")
    
    try:
        ip = run_cmd("hostname -I")[0].split()[0]
    except:
        ip = "localhost"
    
    print(f"\nWeb Interface available at: http://{ip}:5000")
    print("="*70)
    print("\nTROUBLESHOOTING TIPS:")
    print("1. Ensure your Android/iOS device is paired with KDE Connect")
    print("2. Check that both devices are on the same network")
    print("3. Open KDE Connect app on your phone and verify connection")
    print("4. Media must be PLAYING on phone for MPRIS control to work")
    print("5. Visit http://{}:5000/api/kde/diagnose for detailed diagnostics".format(ip))
    print("="*70)
    
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)

    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == "__main__":
    main()
