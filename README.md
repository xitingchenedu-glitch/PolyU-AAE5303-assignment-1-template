# AAE5303 Environment Setup Report — Template for Students

> **Important:** Follow this structure exactly in your submission README.  
> Your goal is to demonstrate **evidence, process, problem-solving, and reflection** — not only screenshots.

---

## 1. System Information

**Laptop model:**  
_ASUS TUF Gaming F16 FX607JV_

**CPU / RAM:**  
_Intel Core i9-13980HX, 16GB RAM_

**Host OS:**  
_Windows 11_

**Linux/ROS environment type:**  
_[Choose one:]_
- [ ] Dual-boot Ubuntu
- [✓] WSL2 Ubuntu
- [ ] Ubuntu in VM (UTM/VirtualBox/VMware/Parallels)
- [ ] Docker container
- [ ] Lab PC
- [ ] Remote Linux server

---

## 2. Python Environment Check

### 2.1 Steps Taken

Describe briefly how you created/activated your Python environment:

**Tool used:**  
_venv_

**Key commands you ran:**
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Any deviations from the default instructions:**  
_None_

### 2.2 Test Results

Run these commands and paste the actual terminal output (not just screenshots):

```bash
python scripts/test_python_env.py
```

**Output:**
```
========================================
AAE5303 Environment Check (Python + ROS)
Goal: help you verify your environment and understand what each check means.                                                          
========================================

Step 1: Environment snapshot
  Why: We capture platform/Python/ROS variables to diagnose common setup mistakes (especially mixed ROS env).                         
Step 2: Python version
  Why: The course assumes Python 3.10+; older versions often break package wheels.                                                    
Step 3: Python imports (required/optional)
  Why: Imports verify packages are installed and compatible with your Python version.                                                 
Step 4: NumPy sanity checks
  Why: We run a small linear algebra operation so success means more than just `import numpy`.                                        
Step 5: SciPy sanity checks
  Why: We run a small FFT to confirm SciPy is functional (not just installed).                                                        
Step 6: Matplotlib backend check
  Why: We generate a tiny plot image (headless) to confirm plotting works on your system.                                             
Step 7: OpenCV PNG decoding (subprocess)
  Why: PNG decoding uses native code; we isolate it so corruption/codec issues cannot crash the whole report.                         
Step 8: Open3D basic geometry + I/O (subprocess)
  Why: Open3D is a native extension; ABI mismatches can segfault. Subprocess isolation turns crashes into readable failures.          
Step 9: ROS toolchain checks
  Why: The course requires ROS tooling. This check passes if ROS 2 OR ROS 1 is available (either one is acceptable).                  
  Action: building ROS 2 workspace package `env_check_pkg` (this may take 1-3 minutes on first run)...                                
  Action: running ROS 2 talker/listener for a few seconds to verify messages flow...                                                  
Step 10: Basic CLI availability
  Why: We confirm core commands exist on PATH so students can run the same commands as in the labs.                                   

=== Summary ===
✅ Environment: {
  "platform": "Linux-6.6.87.2-microsoft-standard-WSL2-x86_64-with-glibc2.35",                                                         
  "python": "3.10.12",
  "executable": "/root/PolyU-AAE5303-env-smork-test/.venv/bin/python",                                                                
  "cwd": "/root/PolyU-AAE5303-env-smork-test",
  "ros": {
    "ROS_VERSION": "2",
    "ROS_DISTRO": "humble",
    "ROS_ROOT": null,
    "ROS_PACKAGE_PATH": null,
    "AMENT_PREFIX_PATH": "/opt/ros/humble",
    "CMAKE_PREFIX_PATH": null
  }
}
✅ Python version OK: 3.10.12
✅ Module 'numpy' found (v2.2.6).
✅ Module 'scipy' found (v1.15.3).
✅ Module 'matplotlib' found (v3.10.8).
✅ Module 'cv2' found (v4.13.0).
✅ Module 'rclpy' found (vunknown).
✅ numpy matrix multiply OK.
✅ numpy version 2.2.6 detected.
✅ scipy FFT OK.
✅ scipy version 1.15.3 detected.
✅ matplotlib backend OK (Agg), version 3.10.8.
✅ OpenCV OK (v4.13.0), decoded sample image 128x128.
✅ Open3D OK (v0.19.0), NumPy 2.2.6.
✅ Open3D loaded sample PCD with 8 pts and completed round-trip I/O.                                                                  
✅ ROS 2 CLI OK: /opt/ros/humble/bin/ros2
✅ ROS 1 tools not found (acceptable if ROS 2 is installed).
✅ colcon found: /usr/bin/colcon
✅ ROS 2 workspace build OK (env_check_pkg).
✅ ROS 2 runtime OK: talker and listener exchanged messages.
✅ Binary 'python3' found at /root/PolyU-AAE5303-env-smork-test/.venv/bin/python3                                                     

All checks passed. You are ready for AAE5303 🚀
```

```bash
python scripts/test_open3d_pointcloud.py
```

**Output:**
```
ℹ️ Loading /root/PolyU-AAE5303-env-smork-test/data/sample_pointcloud.pcd ...                                                           
✅ Loaded 8 points.
   • Centroid: [0.025 0.025 0.025]
   • Axis-aligned bounds: min=[0. 0. 0.], max=[0.05 0.05 0.05]
✅ Filtered point cloud kept 7 points.
✅ Wrote filtered copy with 7 points to /root/PolyU-AAE5303-env-smork-test/data/sample_pointcloud_copy.pcd                            
   • AABB extents: [0.05 0.05 0.05]
   • OBB  extents: [0.08164966 0.07071068 0.05773503], max dim 0.0816 m                                                               
🎉 Open3D point cloud pipeline looks good.
```

**Screenshot:**  
_[Include one screenshot showing both tests passing]

<img width="2559" height="1527" alt="Python Tests Passing" src="https://github.com/user-attachments/assets/21a9dcbb-d664-4068-9e6c-9687ae297287" />

---

## 3. ROS 2 Workspace Check

### 3.1 Build the workspace

Paste the build output summary (final lines only):

```bash
source /opt/ros/humble/setup.bash
colcon build
```

**Expected output:**
```
Summary: 1 package finished [x.xx s]
```

**Your actual output:**
```
Finished <<< env_check_pkg [6.68s]                     
Summary: 1 package finished [7.02s]
```

### 3.2 Run talker and listener

Show both source commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Then run talker:**
```bash
ros2 run env_check_pkg talker.py
```

**Output (3–4 lines):**
```
[INFO] [1769145834.300857745] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #0'
[INFO] [1769145834.797247829] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #1'
[INFO] [1769145835.293450473] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #2'
```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
[INFO] [1769146773.922291076] [env_check_pkg_listener]: I heard: 'AAE5303 hello #0'
[INFO] [1769146774.427389824] [env_check_pkg_listener]: I heard: 'AAE5303 hello #1'
[INFO] [1769146774.932376277] [env_check_pkg_listener]: I heard: 'AAE5303 hello #2'
```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
_[Include one screenshot showing talker + listener running]_

<img width="2559" height="1527" alt="Talker and Listener Running" src="https://github.com/user-attachments/assets/354fe946-5709-4b14-b5b2-61b594daf0c0" />

---

## 4. Problems Encountered and How I Solved Them

> **Note:** Write 2–3 issues, even if small. This section is crucial — it demonstrates understanding and problem-solving.

### Issue 1: No executable found-ros2 run env_check_pkg talker.py

**Cause / diagnosis:**  
_ used a wrong executable name for a C++ package_

**Fix:**  
_Why no executable found ros2 run env_check_pkg talker.py?_

```bash
ros2 run env_check_pkg talker
```

**Reference:**  
_cursor AI assistant_

---

### Issue 2: No listener output displayed

**Cause / diagnosis:**  
_The listener and talker are not running simultaneously.

**Fix:**  
_Start the talker to test the listener's output_

```bash
cd /root/PolyU-AAE5303-env-smork-test && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run env_check_pkg talker
sleep 3 && source /opt/ros/humble/setup.bash && source /root/PolyU-AAE5303-env-smork-test/install/setup.bash && timeout 5 ros2 topic echo /env_check_chatter 2>&1 | head -20
source /opt/ros/humble/setup.bash && source /root/PolyU-AAE5303-env-smork-test/install/setup.bash && timeout 5 ros2 topic echo /rosout 2>&1 | grep -E "listener|I heard" | head -15
ps aux | grep -E "(talker|listener)" | grep -v grep | grep -v defunct
```

**Reference:**  
_cursor AI assistant_

---

## 5. Use of Generative AI (Required)

Choose one of the issues above and document how you used AI to solve it.

> **Goal:** Show critical use of AI, not blind copying.

### 5.1 Exact prompt you asked

**Your prompt:**
```
check whether talker and listener are running correctly 
```

### 5.2 Key helpful part of the AI's answer

**AI's response (relevant part only):**
```
Listener Status:
The listener is running, node name: /env_check_pkg_listener
Subscribed to topic: /env_check_chatter
No talker is currently running, therefore the listener will not receive messages.
Listener Output:
According to the source code, the listener will output:
On startup: "AAE5303 listener awaiting messages."
Upon receiving a message: "I heard: 'AAE5303 hello #<n>'"
How to view listener output:
Since the listener runs in the background, the output may not be visible. You can:
Start the talker to make the listener output:
ros2 run env_check_pkg talker
Then the listener will start printing the message "I heard: ...".
Re-run the listener in the foreground to view real-time output:
ros2 run env_check_pkg listener
```

### 5.3 What you changed or ignored and why

Explain briefly:
- Did the AI recommend something unsafe?
- Did you modify its solution?
- Did you double-check with official docs?

**Your explanation:**  
After checking the running status of the listener and talker, the AI ​​found the reason why the listener wasn't displaying output: the talker wasn't running. The cursor AI provided me with code to start the talker and view the real-time output; I ran this method directly and received the listener's output.

### 5.4 Final solution you applied

Show the exact command or file edit that fixed the problem:

```bash
cd /root/PolyU-AAE5303-env-smork-test && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run env_check_pkg talker
sleep 3 && source /opt/ros/humble/setup.bash && source /root/PolyU-AAE5303-env-smork-test/install/setup.bash && timeout 5 ros2 topic echo /env_check_chatter 2>&1 | head -20
source /opt/ros/humble/setup.bash && source /root/PolyU-AAE5303-env-smork-test/install/setup.bash && timeout 5 ros2 topic echo /rosout 2>&1 | grep -E "listener|I heard" | head -15
ps aux | grep -E "(talker|listener)" | grep -v grep | grep -v defunct
ros2 topic echo /rosout | grep "I heard"
```

**Why this worked:**  
The listener requires the talker to publish messages to produce output. A listener is running in the background, but no talker is running. Therefore, the talker needs to be started to allow the listener to output, and then the listener needs to be run again in the foreground to check the real-time output.

---

## 6. Reflection (3–5 sentences)

Short but thoughtful:

- What did you learn about configuring robotics environments?
- What surprised you?
- What would you do differently next time (backup, partitioning, reading error logs, asking better AI questions)?
- How confident do you feel about debugging ROS/Python issues now?

**Your reflection:**

AI can not only analyze problems and quickly provide solutions, but it can even help me run the modified code directly, greatly reducing the difficulty of debugging for coding novices.I am now more confident in learning the courses to come.

---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
_[CHEN XITING]_

**Student ID:**  
_[25091409g]_

**Date:**  
_[2026/01/25]_

---

## Submission Checklist

Before submitting, ensure you have:

- [✓] Filled in all system information
- [✓] Included actual terminal outputs (not just screenshots)
- [✓] Provided at least 2 screenshots (Python tests + ROS talker/listener)
- [✓] Documented 2–3 real problems with solutions
- [✓] Completed the AI usage section with exact prompts
- [✓] Written a thoughtful reflection (3–5 sentences)
- [✓] Signed the declaration

---

**End of Report**
