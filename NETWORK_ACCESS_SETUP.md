# 🌐 Network Access Setup - Complete!

## What Was Changed

I've updated the system so the terminal works on your **local network** (accessible from phone, tablet, etc.).

### Changes Made:

1. **Web UI (App.jsx)** ✅
   - Terminal URL now uses **current hostname/IP** instead of localhost
   - Automatically detects the correct URL based on where you're accessing from
   - Works on both localhost AND network IPs

2. **Terminal Server (terminal_server.py)** ✅
   - Now binds to `0.0.0.0` (all network interfaces) instead of localhost only
   - Accessible from any device on your local network
   - Added security warnings when no authentication is set

3. **Network Access Script** ✅
   - Created `scripts/show_network_access.sh` to show your network URLs

---

## 🚀 How to Use

### Step 1: Rebuild the Package

Since we modified Python files, rebuild:

```bash
cd /home/farhan/Projects/Surgical_lokalisasi_bismillah/motion_webots_farhan_coba

colcon build --packages-select bascorro_studio
source install/setup.bash
```

### Step 2: Restart Bascorro Studio

```bash
# Stop current session (Ctrl+C)

# Restart
./scripts/bascorro_studio.sh
```

### Step 3: Find Your Network IP

```bash
./scripts/show_network_access.sh
```

**Example Output:**
```
📍 Network IP Addresses:
  • 192.168.1.100

📱 From Phone/Tablet/Other Device:
  Web UI:     http://192.168.1.100:5173
  Terminal:   http://192.168.1.100:7681
  ROSBridge:  ws://192.168.1.100:9090

💻 From This Computer:
  Web UI:     http://localhost:5173
  Terminal:   http://localhost:7681
```

### Step 4: Access From Your Phone

1. **Make sure your phone is on the same WiFi network**
2. Open browser on your phone
3. Go to: `http://YOUR_IP:5173` (from show_network_access.sh)
4. Click the **Terminal** tab
5. **It should now work!** 🎉

---

## 🔍 How It Works Now

### Before (Didn't Work on Network):
```
Phone Browser → http://192.168.1.100:5173
                ↓
                Web UI loads
                ↓
                Terminal iframe tries: http://localhost:7681 ❌
                (localhost on phone = phone itself, not your computer!)
```

### After (Works on Network):
```
Phone Browser → http://192.168.1.100:5173
                ↓
                Web UI loads
                ↓
                JavaScript detects hostname: 192.168.1.100
                ↓
                Terminal iframe uses: http://192.168.1.100:7681 ✅
                (points to your computer!)
                ↓
                Terminal server listening on 0.0.0.0:7681 ✅
                (accessible from network)
```

---

## 🔒 Security Recommendations

Since the terminal is now accessible on your network:

### Option 1: Add Password Protection (Recommended)

```bash
# Before starting, set credentials:
export OP3_STUDIO_TERMINAL_CREDENTIAL="admin:your_secure_password"
./scripts/bascorro_studio.sh
```

Then when accessing from phone, you'll need to enter:
- Username: `admin`
- Password: `your_secure_password`

### Option 2: Firewall Rules (Advanced)

Restrict access to specific devices:

```bash
# Only allow from your phone's IP (example: 192.168.1.50)
sudo ufw allow from 192.168.1.50 to any port 7681
```

---

## 🧪 Testing

### Test from Computer (Local):

```bash
# Should work:
curl http://localhost:7681
```

### Test from Phone (Network):

1. Find your computer's IP: `./scripts/show_network_access.sh`
2. On phone browser, go to: `http://YOUR_IP:7681`
3. Should see the terminal interface

### Test Web UI Integration:

1. On phone, go to: `http://YOUR_IP:5173`
2. Click **Terminal** tab
3. Should load the terminal (using correct IP automatically)

---

## 📝 Troubleshooting

### Can't Access from Phone

**Check 1: Same Network**
```bash
# On computer, find IP
ip addr show

# On phone, check WiFi settings
# Should be same subnet (e.g., both 192.168.1.x)
```

**Check 2: Firewall**
```bash
# Check if port is blocked
sudo ufw status

# Allow port 7681
sudo ufw allow 7681/tcp
sudo ufw allow 5173/tcp
```

**Check 3: Server Running**
```bash
# Check if ttyd is listening on all interfaces
sudo netstat -tlnp | grep 7681

# Should show: 0.0.0.0:7681 (not 127.0.0.1:7681)
```

---

## 📊 Summary

✅ **Terminal iframe** - Now uses dynamic hostname (your IP)
✅ **Terminal server** - Binds to 0.0.0.0 (accessible on network)
✅ **Network script** - Shows your access URLs
✅ **Works on phone** - Terminal accessible from any device on your network!

---

## 🎯 Quick Start

```bash
# 1. Rebuild
colcon build --packages-select bascorro_studio
source install/setup.bash

# 2. Restart
./scripts/bascorro_studio.sh

# 3. Find your IP
./scripts/show_network_access.sh

# 4. On phone, open browser to:
#    http://YOUR_IP:5173
#    Click Terminal tab → Should work! 🎉
```

---

## 🔐 Recommended: Enable Password

For security on network:

```bash
export OP3_STUDIO_TERMINAL_CREDENTIAL="op3:robot2024"
./scripts/bascorro_studio.sh
```

Now your terminal is protected! 🔒
