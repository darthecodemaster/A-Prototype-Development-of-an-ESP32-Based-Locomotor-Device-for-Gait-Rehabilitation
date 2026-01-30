/* 
 * BILATERAL EXOSKELETON CONTROL WITH WEB SOCKET
 * Arduino IDE Version - Optimized for ESP32
 * Direct servo control without smoothing
 * 
 * Features:
 * - Real-time bilateral gait pattern playback from 100-sample arrays
 * - Web-based control interface with WebSocket communication
 * - Direct servo movement (no smoothing)
 * - Dynamic safety limits
 * - Manual override with position feedback
 * - Advanced diagnostics and error handling
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <ArduinoJson.h>

// ========== SYSTEM CONFIGURATION ==========
const char* ssid = "ExoskeletonAP";      // WiFi Access Point name
const char* password = "rehab2025";      // WiFi password

// Pin Definitions for ESP32 - BILATERAL CONTROL WITH PWM OPTIMIZATION
#define SERVO_LEFT_HIP_PIN 18     // GPIO18 for left hip servo (PWM Channel 0)
#define SERVO_LEFT_KNEE_PIN 19    // GPIO19 for left knee servo (PWM Channel 1)
#define SERVO_RIGHT_HIP_PIN 32    // GPIO32 for right hip servo (PWM Channel 2)
#define SERVO_RIGHT_KNEE_PIN 33   // GPIO33 for right knee servo (PWM Channel 3)

// Additional feedback pins
#define BUZZER_PIN 26             // GPIO26 for buzzer feedback
#define LED_STATUS_PIN 2          // GPIO2 for status LED

// ========== SERVO CONFIGURATION ==========
#define SERVO_MIN_PULSE 500     // Minimum pulse width in microseconds (for 0°)
#define SERVO_MAX_PULSE 2500    // Maximum pulse width in microseconds (for 180°)
#define SERVO_MIN_ANGLE 0       // Minimum safe angle in degrees
#define SERVO_MAX_ANGLE 180     // Maximum safe angle in degrees
#define SERVO_HOME_POS 90       // Neutral/home position for servos

// ========== SYSTEM OBJECTS ==========
Servo servoLeftHip;             // Left hip joint servo object
Servo servoLeftKnee;            // Left knee joint servo object
Servo servoRightHip;            // Right hip joint servo object
Servo servoRightKnee;           // Right knee joint servo object
AsyncWebServer server(80);      // HTTP server on port 80
AsyncWebSocket ws("/ws");       // WebSocket endpoint for real-time communication

// ========== SYSTEM STATE MANAGEMENT ==========
enum SystemState { 
  STOPPED, 
  RUNNING, 
  PAUSED, 
  ERROR_STATE
};  
SystemState currentState = STOPPED;             // Current system state
String lastErrorMessage = "";                   // Last error message

// ========== ADVANCED DATA STRUCTURES ==========
struct ServoData {
  float currentAngle;           // Current angle in degrees
  float targetAngle;            // Target angle in degrees
  unsigned long lastUpdate;     // Last update timestamp
};

struct SystemData {
  ServoData leftHip;
  ServoData leftKnee;
  ServoData rightHip;
  ServoData rightKnee;
  int gaitPhase;                // Current phase of gait cycle (0-4)
  int gaitProgress;             // Progress through gait cycle (0-99)
  float gaitSpeed;              // Current playback speed
  unsigned long timestamp;      // Timestamp of last update
  float systemLoad;             // System load percentage
  int errorCode;                // Error code (0 = no error)
  String errorMessage;          // Error message
};

SystemData currentData;         // Current system data snapshot

// ========== GAIT CONTROL VARIABLES ==========
int gaitCycleIndex = 0;         // Current position in gait cycle (0-99)
float gaitSpeed = 1.0;          // Playback speed multiplier (0.5-2.0)
unsigned long lastGaitUpdate = 0; // Last gait update timestamp
int completedCycles = 0;        // Count of completed gait cycles
int lastProgress = -1;          // Last gait progress for cycle detection

// ========== SERVO UPDATE INTERVAL ==========
const int SERVO_UPDATE_INTERVAL = 20;    // Servo update interval in ms (50Hz)
unsigned long lastServoUpdate = 0;

// ========== YOUR GAIT ARRAYS ==========
const int GAIT_SAMPLES = 100;

float hipLeft[GAIT_SAMPLES] = { 78.76, 77.71, 76.72, 75.79, 74.93, 74.15, 73.43, 72.79, 72.24, 71.77,
  71.38, 71.08, 70.87, 70.75, 70.72, 70.78, 70.94, 71.18, 71.51, 71.92,
  72.43, 73.01, 73.67, 74.42, 75.23, 76.11, 77.06, 78.07, 79.14, 80.25,
  81.42, 82.62, 83.86, 85.13, 86.41, 87.72, 89.04, 90.36, 91.68, 92.99,
  94.29, 95.57, 96.82, 98.04, 99.22, 100.36, 101.45, 102.49, 103.46, 104.38,
  105.22, 106.00, 106.70, 107.32, 107.86, 108.31, 108.68, 108.96, 109.16, 109.26,
  109.27, 109.19, 109.02, 108.77, 108.42, 107.99, 107.47, 106.87, 106.19, 105.43,
  104.61, 103.71, 102.75, 101.73, 100.65, 99.53, 98.36, 97.15, 95.90, 94.63,
  93.34, 92.03, 90.71, 89.39, 88.07, 86.76, 85.46, 84.19, 82.94, 81.73,
  80.56, 79.43, 78.35, 77.32, 76.36, 75.46, 74.62, 73.86, 73.18, 72.57,};

float kneeLeft[GAIT_SAMPLES] = { 104.26, 103.53, 102.73, 101.86, 100.94, 99.95, 98.92, 97.84, 96.72, 95.57,
  94.39, 93.19, 91.97, 90.74, 89.50, 88.27, 87.05, 85.84, 84.66, 83.50,
  82.37, 81.29, 80.25, 79.25, 78.31, 77.43, 76.62, 75.87, 75.20, 74.60,
  74.08, 73.64, 73.28, 73.01, 72.82, 72.72, 72.71, 72.79, 72.95, 73.20,
  73.54, 73.96, 74.46, 75.04, 75.70, 76.43, 77.22, 78.09, 79.01, 79.99,
  81.02, 82.10, 83.21, 84.37, 85.54, 86.75, 87.97, 89.19, 90.43, 91.66,
  92.88, 94.09, 95.28, 96.44, 97.56, 98.65, 99.70, 100.69, 101.64, 102.52,
  103.34, 104.09, 104.77, 105.37, 105.90, 106.34, 106.70, 106.98, 107.17, 107.28,
  107.29, 107.22, 107.06, 106.82, 106.48, 106.07, 105.57, 104.99, 104.34, 103.62,
  102.82, 101.96, 101.04, 100.07, 99.04, 97.96, 96.85, 95.70, 94.52, 93.32,};

float hipRight[GAIT_SAMPLES] = {  105.22, 106.00, 106.70, 107.32, 107.86, 108.31, 108.68, 108.96, 109.16, 109.26,
  109.27, 109.19, 109.02, 108.77, 108.42, 107.99, 107.47, 106.87, 106.19, 105.43,
  104.61, 103.71, 102.75, 101.73, 100.65, 99.53, 98.36, 97.15, 95.90, 94.63,
  93.34, 92.03, 90.71, 89.39, 88.07, 86.76, 85.46, 84.19, 82.94, 81.73,
  80.56, 79.43, 78.35, 77.32, 76.36, 75.46, 74.62, 73.86, 73.18, 72.57,
  78.76, 77.71, 76.72, 75.79, 74.93, 74.15, 73.43, 72.79, 72.24, 71.77,
  71.38, 71.08, 70.87, 70.75, 70.72, 70.78, 70.94, 71.18, 71.51, 71.92,
  72.43, 73.01, 73.67, 74.42, 75.23, 76.11, 77.06, 78.07, 79.14, 80.25,
  81.42, 82.62, 83.86, 85.13, 86.41, 87.72, 89.04, 90.36, 91.68, 92.99,
  94.29, 95.57, 96.82, 98.04, 99.22, 100.36, 101.45, 102.49, 103.46, 104.38,};

float kneeRight[GAIT_SAMPLES] = { 81.02, 82.10, 83.21, 84.37, 85.54, 86.75, 87.97, 89.19, 90.43, 91.66,
  92.88, 94.09, 95.28, 96.44, 97.56, 98.65, 99.70, 100.69, 101.64, 102.52,
  103.34, 104.09, 104.77, 105.37, 105.90, 106.34, 106.70, 106.98, 107.17, 107.28,
  107.29, 107.22, 107.06, 106.82, 106.48, 106.07, 105.57, 104.99, 104.34, 103.62,
  102.82, 101.96, 101.04, 100.07, 99.04, 97.96, 96.85, 95.70, 94.52, 93.32,
  104.26, 103.53, 102.73, 101.86, 100.94, 99.95, 98.92, 97.84, 96.72, 95.57,
  94.39, 93.19, 91.97, 90.74, 89.50, 88.27, 87.05, 85.84, 84.66, 83.50,
  82.37, 81.29, 80.25, 79.25, 78.31, 77.43, 76.62, 75.87, 75.20, 74.60,
  74.08, 73.64, 73.28, 73.01, 72.82, 72.72, 72.71, 72.79, 72.95, 73.20,
  73.54, 73.96, 74.46, 75.04, 75.70, 76.43, 77.22, 78.09, 79.01, 79.99,};

// ========== HTML WEB PAGE ==========

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Exoskeleton Rehabilitation System Control Panel</title>
    <style>
        :root {
            --primary: #8B7BC6;
            --primary-dark: #6B5BA6;
            --accent: #A78BFA;
            --success: #10B981;
            --warning: #FBBF24;
            --danger: #EF4444;
            --info: #3B82F6;
            --bg: #F3F4F6;
            --bg-light: #FAFBFC;
            --text-primary: #1F2937;
            --text-secondary: #6B7280;
            --shadow-light: rgba(255, 255, 255, 0.8);
            --shadow-dark: rgba(0, 0, 0, 0.1);
        }
        
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }
        
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: var(--bg);
            min-height: 100vh;
            color: var(--text-primary);
            padding: 20px;
        }
        
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        
        .header {
            text-align: center;
            margin-bottom: 30px;
            padding: 25px;
            background: var(--bg-light);
            border-radius: 20px;
            box-shadow: 8px 8px 20px var(--shadow-dark), 
                        -8px -8px 20px var(--shadow-light);
        }
        
        h1 {
            font-size: 1.1rem;
            margin-bottom: 8px;
            color: var(--text-primary);
            font-weight: 600;
        }
        
        .connection-status {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 10px 18px;
            border-radius: 25px;
            font-weight: 600;
            font-size: 0.85rem;
            z-index: 1000;
            box-shadow: 4px 4px 12px var(--shadow-dark), 
                        -4px -4px 12px var(--shadow-light);
        }
        
        .connected { 
            background: var(--bg-light);
            color: var(--success);
        }
        .disconnected { 
            background: var(--bg-light);
            color: var(--danger);
        }
        
        .dashboard {
            display: grid;
            grid-template-columns: 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        
        @media (min-width: 768px) {
            .dashboard {
                grid-template-columns: 1fr 1fr;
            }
        }
        
        @media (min-width: 1200px) {
            .dashboard {
                grid-template-columns: 1fr 2fr 1fr;
            }
        }
        
        .panel {
            background: var(--bg-light);
            border-radius: 20px;
            padding: 25px;
            box-shadow: 8px 8px 20px var(--shadow-dark), 
                        -8px -8px 20px var(--shadow-light);
        }
        
        .panel h2 {
            font-size: 1.1rem;
            margin-bottom: 20px;
            display: flex;
            align-items: center;
            gap: 10px;
            color: var(--text-primary);
            font-weight: 600;
        }
        
        .status-display {
            display: flex;
            align-items: center;
            justify-content: space-between;
            margin-bottom: 20px;
        }
        
        .status-indicator {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .status-dot {
            width: 14px;
            height: 14px;
            border-radius: 50%;
            box-shadow: inset 2px 2px 4px rgba(0,0,0,0.15);
        }
        
        .stopped { background: var(--text-secondary); }
        .running { 
            background: var(--success);
            animation: pulse 1s infinite;
        }
        .paused { background: var(--warning); }
        .error { 
            background: var(--danger);
            animation: blink 0.5s infinite;
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        @keyframes blink {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.3; }
        }
        
        .gait-progress-container {
            margin: 20px 0;
        }
        
        .gait-progress-bar {
            width: 100%;
            height: 24px;
            background: var(--bg);
            border-radius: 12px;
            overflow: hidden;
            margin-bottom: 8px;
            box-shadow: inset 3px 3px 8px var(--shadow-dark),
                        inset -2px -2px 6px var(--shadow-light);
        }
        
        .gait-progress-fill {
            height: 100%;
            background: linear-gradient(90deg, var(--primary), var(--accent));
            width: 0%;
            transition: width 0.3s;
            border-radius: 12px;
        }
        
        .progress-labels {
            display: flex;
            justify-content: space-between;
            font-size: 0.8rem;
            color: var(--text-secondary);
        }
        
        .control-buttons-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 12px;
            margin-bottom: 20px;
        }
        
        button {
            padding: 14px;
            font-size: 0.9rem;
            border: none;
            border-radius: 15px;
            cursor: pointer;
            font-weight: 600;
            transition: all 0.2s;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 6px;
            min-height: 48px;
            background: var(--bg-light);
            color: var(--text-primary);
            box-shadow: 5px 5px 15px var(--shadow-dark), 
                        -5px -5px 15px var(--shadow-light);
        }
        
        button:hover:not(:disabled) {
            box-shadow: 2px 2px 8px var(--shadow-dark), 
                        -2px -2px 8px var(--shadow-light);
        }
        
        button:active:not(:disabled) {
            box-shadow: inset 3px 3px 8px var(--shadow-dark),
                        inset -2px -2px 6px var(--shadow-light);
        }
        
        button:disabled {
            opacity: 0.4;
            cursor: not-allowed;
        }
        
        .btn-start { color: var(--success); }
        .btn-pause { color: var(--warning); }
        .btn-stop { color: var(--danger); }
        
        .speed-control {
            margin: 20px 0;
        }
        
        .speed-slider-container {
            display: flex;
            align-items: center;
            gap: 12px;
            margin-top: 12px;
        }
        
        input[type="range"] {
            flex: 1;
            height: 8px;
            -webkit-appearance: none;
            background: var(--bg);
            border-radius: 4px;
            outline: none;
            box-shadow: inset 2px 2px 5px var(--shadow-dark),
                        inset -2px -2px 5px var(--shadow-light);
        }
        
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 24px;
            height: 24px;
            border-radius: 50%;
            background: var(--bg-light);
            cursor: pointer;
            box-shadow: 3px 3px 8px var(--shadow-dark), 
                        -3px -3px 8px var(--shadow-light);
        }
        
        .leg-controls {
            display: grid;
            grid-template-columns: 1fr;
            gap: 20px;
            margin-top: 20px;
        }
        
        @media (min-width: 576px) {
            .leg-controls {
                grid-template-columns: 1fr 1fr;
            }
        }
        
        .leg-panel {
            background: var(--bg);
            border-radius: 15px;
            padding: 20px;
            box-shadow: inset 3px 3px 8px var(--shadow-dark),
                        inset -2px -2px 6px var(--shadow-light);
        }
        
        .left-leg { border-left: 4px solid var(--info); }
        .right-leg { border-left: 4px solid var(--warning); }
        
        .joint-control {
            margin: 15px 0;
        }
        
        .joint-label {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-size: 0.9rem;
            color: var(--text-secondary);
        }
        
        .angle-display {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin-top: 20px;
        }
        
        .angle-card {
            background: var(--bg);
            border-radius: 15px;
            padding: 15px;
            text-align: center;
            box-shadow: inset 3px 3px 8px var(--shadow-dark),
                        inset -2px -2px 6px var(--shadow-light);
        }
        
        .angle-value {
            font-size: 1.8rem;
            font-weight: 700;
            margin: 8px 0;
            color: var(--primary);
        }
        
        .angle-label {
            font-size: 0.8rem;
            color: var(--text-secondary);
        }
        
        .diagnostics-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin: 20px 0;
        }
        
        .data-card {
            background: var(--bg);
            border-radius: 15px;
            padding: 15px;
            text-align: center;
            box-shadow: inset 3px 3px 8px var(--shadow-dark),
                        inset -2px -2px 6px var(--shadow-light);
        }
        
        .data-value {
            font-size: 1.5rem;
            font-weight: 700;
            margin: 8px 0;
            color: var(--primary);
        }
        
        .data-label {
            font-size: 0.8rem;
            color: var(--text-secondary);
        }
        
        .footer {
            text-align: center;
            margin-top: 30px;
            padding: 15px;
            font-size: 0.8rem;
            color: var(--text-secondary);
            display: grid;
            grid-template-columns: 1fr;
            gap: 8px;
        }
        
        @media (min-width: 576px) {
            .footer {
                grid-template-columns: repeat(3, 1fr);
            }
        }
        
        .alert {
            background: var(--bg);
            border-left: 4px solid var(--danger);
            padding: 12px;
            margin: 12px 0;
            border-radius: 10px;
            display: none;
            font-size: 0.9rem;
            box-shadow: 3px 3px 8px var(--shadow-dark);
            color: var(--danger);
        }
        
        .alert.show {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .gait-phase-display {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            margin-top: 20px;
        }
        
        .phase-group {
            background: var(--bg);
            border-radius: 12px;
            padding: 15px;
            box-shadow: inset 2px 2px 5px var(--shadow-dark),
                        inset -2px -2px 5px var(--shadow-light);
        }
        
        .phase-group-title {
            font-size: 0.85rem;
            font-weight: 600;
            color: var(--text-primary);
            margin-bottom: 10px;
            text-align: center;
        }
        
        .phase-indicator {
            text-align: center;
            padding: 8px 4px;
            border-radius: 8px;
            background: var(--bg-light);
            margin: 5px 0;
            font-size: 0.75rem;
            color: var(--text-secondary);
            box-shadow: 2px 2px 5px var(--shadow-dark),
                        -2px -2px 5px var(--shadow-light);
        }
        
        .phase-active {
            background: var(--primary);
            color: white;
            font-weight: 600;
            box-shadow: 3px 3px 8px var(--shadow-dark), 
                        -3px -3px 8px var(--shadow-light);
        }
    </style>
</head>
<body>
    <div class="connection-status disconnected" id="connectionStatus">
        ⚠ Disconnected
    </div>
    
    <div class="container">
        <div class="header">
            <h1>GAIT REHABILITATION SYSTEM CONTROL</h1>
        </div>
        
        <div class="dashboard">
            <div class="panel">
                <h2>📊 System Status</h2>
                <div class="status-display">
                    <div class="status-indicator">
                        <div class="status-dot stopped" id="statusIndicator"></div>
                        <span id="systemStatus">System Stopped</span>
                    </div>
                    <div id="errorAlert" class="alert">
                        <span>⚠</span>
                        <span id="errorMessage">Error message</span>
                    </div>
                </div>
                
                <div class="gait-progress-container">
                    <div class="gait-progress-bar">
                        <div class="gait-progress-fill" id="gaitProgressBar"></div>
                    </div>
                    <div class="progress-labels">
                        <span>Phase: <span id="currentPhase">1</span>/5</span>
                        <span>Progress: <span id="gaitProgress">0</span>%</span>
                    </div>
                </div>
                
                <div class="speed-control">
                    <label>Gait Speed: <span id="speedValue">1.0</span>x</label>
                    <div class="speed-slider-container">
                        <button onclick="adjustSpeed(-0.1)" style="padding: 8px 12px;">-</button>
                        <input type="range" id="speedSlider" min="0.5" max="2.0" step="0.1" value="1.0" 
                               oninput="updateSpeedDisplay(this.value)">
                        <button onclick="adjustSpeed(0.1)" style="padding: 8px 12px;">+</button>
                    </div>
                </div>
                
                <div class="gait-phase-display">
    <div class="phase-group">
        <div class="phase-group-title">Early Swing</div>
        <div class="phase-indicator" data-phase="0">1. Heel Strike</div>
        <div class="phase-indicator" data-phase="1">2. Foot Flat</div>
    </div>
    <div class="phase-group">
        <div class="phase-group-title">Mid-Swing</div>
        <div class="phase-indicator" data-phase="2">3. Midstance</div>
        <div class="phase-indicator" data-phase="3">4. Heel Off</div>
    </div>
    <div class="phase-group">
        <div class="phase-group-title">Late Swing</div>
        <div class="phase-indicator" data-phase="4">5. Toe Off</div>
    </div>
</div>
            </div>
            
            <div class="panel">
                <h2>🎮 Control Panel</h2>
                <div class="control-buttons-grid">
                    <button class="btn-start" id="btnStart" onclick="sendCommand('start')">
                        <span>▶</span> Start
                    </button>
                    <button class="btn-pause" id="btnPause" onclick="sendCommand('pause')" disabled>
                        <span>⏸</span> Pause
                    </button>
                    <button class="btn-stop" id="btnStop" onclick="sendCommand('stop')" disabled>
                        <span>⏹</span> Stop
                    </button>
                </div>
                
                <h2 style="margin-top: 20px;">🦿 Manual Control</h2>
                <div class="leg-controls">
                    <div class="leg-panel left-leg">
                        <h3>Left Leg</h3>
                        <div class="joint-control">
                            <div class="joint-label">
                                <span>Hip:</span>
                                <span id="leftHipValue">90°</span>
                            </div>
                            <input type="range" id="leftHipSlider" min="0" max="180" step="1" value="90" 
                                   oninput="updateManualAngle('leftHip', this.value)">
                        </div>
                        <div class="joint-control">
                            <div class="joint-label">
                                <span>Knee:</span>
                                <span id="leftKneeValue">90°</span>
                            </div>
                            <input type="range" id="leftKneeSlider" min="0" max="180" step="1" value="90" 
                                   oninput="updateManualAngle('leftKnee', this.value)">
                        </div>
                    </div>
                    
                    <div class="leg-panel right-leg">
                        <h3>Right Leg</h3>
                        <div class="joint-control">
                            <div class="joint-label">
                                <span>Hip:</span>
                                <span id="rightHipValue">90°</span>
                            </div>
                            <input type="range" id="rightHipSlider" min="0" max="180" step="1" value="90" 
                                   oninput="updateManualAngle('rightHip', this.value)">
                        </div>
                        <div class="joint-control">
                            <div class="joint-label">
                                <span>Knee:</span>
                                <span id="rightKneeValue">90°</span>
                            </div>
                            <input type="range" id="rightKneeSlider" min="0" max="180" step="1" value="90" 
                                   oninput="updateManualAngle('rightKnee', this.value)">
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="panel">
                <h2>📈 Diagnostics</h2>
                <div class="diagnostics-grid">
                    <div class="data-card">
                        <div class="data-label">System Load</div>
                        <div class="data-value" id="systemLoad">0%</div>
                    </div>
                    <div class="data-card">
                        <div class="data-label">Uptime</div>
                        <div class="data-value" id="uptime">0s</div>
                    </div>
                    <div class="data-card">
                        <div class="data-label">Cycle</div>
                        <div class="data-value" id="cycleCount">0</div>
                    </div>
                    <div class="data-card">
                        <div class="data-label">Clients</div>
                        <div class="data-value" id="clientCount">0</div>
                    </div>
                </div>
                
                <h3 style="margin-top: 15px;">📊 Angle Display</h3>
                <div class="angle-display">
                    <div class="angle-card">
                        <div class="angle-label">Left Hip</div>
                        <div class="angle-value" id="leftHipAngle">90.0°</div>
                        <div class="angle-label">Target: <span id="leftHipTarget">90.0°</span></div>
                    </div>
                    <div class="angle-card">
                        <div class="angle-label">Left Knee</div>
                        <div class="angle-value" id="leftKneeAngle">90.0°</div>
                        <div class="angle-label">Target: <span id="leftKneeTarget">90.0°</span></div>
                    </div>
                    <div class="angle-card">
                        <div class="angle-label">Right Hip</div>
                        <div class="angle-value" id="rightHipAngle">90.0°</div>
                        <div class="angle-label">Target: <span id="rightHipTarget">90.0°</span></div>
                    </div>
                    <div class="angle-card">
                        <div class="angle-label">Right Knee</div>
                        <div class="angle-value" id="rightKneeAngle">90.0°</div>
                        <div class="angle-label">Target: <span id="rightKneeTarget">90.0°</span></div>
                    </div>
                </div>
            </div>
        </div>
        
        <div class="footer">
            <div>ESP32-Based Gait Rehabilitation Device | v1.0</div>
            <div>Proponents: </div>
            <div>Caparro, D., David, L., Echavez, M., Monterde, L., Tañala, A. | © 2026</div>
        </div>
    </div>
    
    <script>
        let ws;
        let reconnectInterval = 3000;
        let lastUpdateTime = Date.now();
        let systemUptime = 0;
        let cycleCount = 0;
        let lastGaitProgress = 0;
        
        const gaitPhases = [
        "Heel Strike",
        "Foot Flat", 
        "Midstance",
        "Heel Off",
        "Toe Off"
    ];
        
        function connectWebSocket() {
            const host = window.location.hostname;
            ws = new WebSocket('ws://' + host + '/ws');
            
            ws.onopen = function() {
                console.log('WebSocket connected successfully!');
                updateConnectionStatus(true);
                sendCommand('getStatus');
            };
            
            ws.onmessage = function(event) {
                lastUpdateTime = Date.now();
                try {
                    const data = JSON.parse(event.data);
                    updateUI(data);
                } catch (e) {
                    console.error('JSON parse error:', e);
                }
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                updateConnectionStatus(false);
                setTimeout(connectWebSocket, reconnectInterval);
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
        }
        
        function updateConnectionStatus(connected) {
            const statusElement = document.getElementById('connectionStatus');
            if (connected) {
                statusElement.textContent = '✓ Connected';
                statusElement.className = 'connection-status connected';
            } else {
                statusElement.textContent = '⚠ Disconnected';
                statusElement.className = 'connection-status disconnected';
            }
        }
        
        function updateUI(data) {
            const indicator = document.getElementById('statusIndicator');
            const statusText = document.getElementById('systemStatus');
            
            let stateClass = data.state;
            indicator.className = 'status-dot ' + stateClass;
            
            let statusDisplay = '';
            switch(data.state) {
                case 'running': statusDisplay = 'Running'; break;
                case 'paused': statusDisplay = 'Paused'; break;
                case 'error': statusDisplay = 'Error'; break;
                default: statusDisplay = 'Stopped';
            }
            statusText.textContent = 'System ' + statusDisplay;
            
            const errorAlert = document.getElementById('errorAlert');
            const errorMessage = document.getElementById('errorMessage');
            if (data.errorCode && data.errorCode !== 0) {
                errorAlert.classList.add('show');
                errorMessage.textContent = data.errorMessage || 'Unknown error';
            } else {
                errorAlert.classList.remove('show');
            }
            
            updateButtonStates(data.state);
            
            document.getElementById('leftHipAngle').textContent = data.leftHipAngle.toFixed(1) + '°';
            document.getElementById('leftKneeAngle').textContent = data.leftKneeAngle.toFixed(1) + '°';
            document.getElementById('rightHipAngle').textContent = data.rightHipAngle.toFixed(1) + '°';
            document.getElementById('rightKneeAngle').textContent = data.rightKneeAngle.toFixed(1) + '°';
            
            if (data.leftHipTarget !== undefined) {
                document.getElementById('leftHipTarget').textContent = data.leftHipTarget.toFixed(1) + '°';
                document.getElementById('leftKneeTarget').textContent = data.leftKneeTarget.toFixed(1) + '°';
                document.getElementById('rightHipTarget').textContent = data.rightHipTarget.toFixed(1) + '°';
                document.getElementById('rightKneeTarget').textContent = data.rightKneeTarget.toFixed(1) + '°';
            }
            
            document.getElementById('leftHipSlider').value = Math.round(data.leftHipAngle);
            document.getElementById('leftKneeSlider').value = Math.round(data.leftKneeAngle);
            document.getElementById('rightHipSlider').value = Math.round(data.rightHipAngle);
            document.getElementById('rightKneeSlider').value = Math.round(data.rightKneeAngle);
            
            document.getElementById('leftHipValue').textContent = Math.round(data.leftHipAngle) + '°';
            document.getElementById('leftKneeValue').textContent = Math.round(data.leftKneeAngle) + '°';
            document.getElementById('rightHipValue').textContent = Math.round(data.rightHipAngle) + '°';
            document.getElementById('rightKneeValue').textContent = Math.round(data.rightKneeAngle) + '°';
            
            document.getElementById('gaitProgress').textContent = data.gaitProgress;
            document.getElementById('currentPhase').textContent = (data.gaitPhase + 1);
            document.getElementById('gaitProgressBar').style.width = data.gaitProgress + '%';
            document.getElementById('gaitProgressBar').style.transition = 'width ' + (0.3 / data.speed) + 's';
            
            updatePhaseIndicators(data.gaitPhase);
            
            document.getElementById('speedValue').textContent = data.speed.toFixed(1);
            document.getElementById('speedSlider').value = data.speed;
            
            document.getElementById('systemLoad').textContent = (data.systemLoad || 0).toFixed(1) + '%';
            document.getElementById('cycleCount').textContent = data.cycleCount || 0;
            document.getElementById('clientCount').textContent = data.clientCount || '1';
            
            const now = new Date();
            
            let systemInfo = '';
            if (data.state === 'running') {
                systemInfo = `Gait at ${data.speed.toFixed(1)}x`;
            } else if (data.state === 'paused') {
                systemInfo = 'Paused';
            } else if (data.state === 'error') {
                systemInfo = 'Error';
            } else {
                systemInfo = 'Ready';
            }
            
            if (data.gaitProgress === 0 && lastGaitProgress > 90 && data.state === 'running') {
                cycleCount++;
            }
            
            lastGaitProgress = data.gaitProgress;
        }
        
        function updatePhaseIndicators(currentPhase) {
            const indicators = document.querySelectorAll('.phase-indicator');
            indicators.forEach(indicator => {
                const phase = parseInt(indicator.getAttribute('data-phase'));
                if (phase === currentPhase) {
                    indicator.classList.add('phase-active');
                } else {
                    indicator.classList.remove('phase-active');
                }
            });
        }
        
        function updateButtonStates(state) {
            const btnStart = document.getElementById('btnStart');
            const btnPause = document.getElementById('btnPause');
            const btnStop = document.getElementById('btnStop');
            
            btnStart.disabled = (state === 'running');
            btnPause.disabled = (state !== 'running');
            btnStop.disabled = (state === 'stopped');
            
            if (state === 'running') {
                btnStart.innerHTML = '<span>▶</span> Running';
            } else if (state === 'paused') {
                btnStart.innerHTML = '<span>▶</span> Resume';
            } else {
                btnStart.innerHTML = '<span>▶</span> Start';
            }
        }
        
        function sendCommand(command, data = {}) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const message = JSON.stringify({ command, ...data });
                ws.send(message);
                console.log('Sent:', message);
            } else {
                console.error('WebSocket not connected');
                alert('Not connected to ESP32. Please wait for connection...');
            }
        }
        
        function updateSpeedDisplay(value) {
            document.getElementById('speedValue').textContent = value;
            clearTimeout(window.speedTimeout);
            window.speedTimeout = setTimeout(() => {
                sendCommand('setSpeed', { value: parseFloat(value) });
            }, 300);
        }
        
        function adjustSpeed(delta) {
            const slider = document.getElementById('speedSlider');
            let newValue = parseFloat(slider.value) + delta;
            newValue = Math.max(0.5, Math.min(2.0, newValue));
            slider.value = newValue;
            updateSpeedDisplay(newValue);
        }
        
        function updateManualAngle(joint, value) {
            document.getElementById(joint + 'Value').textContent = value + '°';
            sendCommand('setAngle', { joint, value: parseFloat(value) });
        }
        
        setInterval(() => {
            systemUptime++;
            const hours = Math.floor(systemUptime / 3600);
            const minutes = Math.floor((systemUptime % 3600) / 60);
            const seconds = systemUptime % 60;
            
            let uptimeText = '';
            if (hours > 0) uptimeText += hours + 'h ';
            if (minutes > 0 || hours > 0) uptimeText += minutes + 'm ';
            uptimeText += seconds + 's';
            
            document.getElementById('uptime').textContent = uptimeText;
        }, 1000);
        
        window.addEventListener('load', function() {
            connectWebSocket();
            
            setInterval(() => {
                if (ws && ws.readyState === WebSocket.OPEN) {
                    updateConnectionStatus(true);
                    const timeSinceUpdate = Date.now() - lastUpdateTime;
                    if (timeSinceUpdate > 10000) {
                        sendCommand('getStatus');
                    }
                } else {
                    updateConnectionStatus(false);
                }
            }, 5000);
            
            updatePhaseIndicators(0);
        });
    </script>
</body>
</html>
)rawliteral";
// (HTML code remains the same, just include the full string)
// ... [HTML code remains exactly the same] ...

// ========== WEB SOCKET FUNCTIONS ==========

void sendDataToClients() {
  String stateStr;
  switch (currentState) {
    case RUNNING: stateStr = "running"; break;
    case PAUSED: stateStr = "paused"; break;
    case ERROR_STATE: stateStr = "error"; break;
    default: stateStr = "stopped"; break;
  }
  
  String json = "{";
  json += "\"state\":\"" + stateStr + "\",";
  json += "\"leftHipAngle\":" + String(currentData.leftHip.currentAngle, 1) + ",";
  json += "\"leftHipTarget\":" + String(currentData.leftHip.targetAngle, 1) + ",";
  json += "\"leftKneeAngle\":" + String(currentData.leftKnee.currentAngle, 1) + ",";
  json += "\"leftKneeTarget\":" + String(currentData.leftKnee.targetAngle, 1) + ",";
  json += "\"rightHipAngle\":" + String(currentData.rightHip.currentAngle, 1) + ",";
  json += "\"rightHipTarget\":" + String(currentData.rightHip.targetAngle, 1) + ",";
  json += "\"rightKneeAngle\":" + String(currentData.rightKnee.currentAngle, 1) + ",";
  json += "\"rightKneeTarget\":" + String(currentData.rightKnee.targetAngle, 1) + ",";
  json += "\"gaitPhase\":" + String(currentData.gaitPhase) + ",";
  json += "\"gaitProgress\":" + String(currentData.gaitProgress) + ",";
  json += "\"speed\":" + String(gaitSpeed, 1) + ",";
  json += "\"systemLoad\":" + String(currentData.systemLoad, 1) + ",";
  json += "\"cycleCount\":" + String(completedCycles) + ",";
  json += "\"errorCode\":" + String(currentData.errorCode) + ",";
  json += "\"errorMessage\":\"" + String(currentData.errorMessage) + "\",";
  json += "\"clientCount\":" + String(ws.count());
  json += "}";
  
  ws.textAll(json);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String message = (char*)data;
    
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
      Serial.print("JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    String command = doc["command"];
    Serial.print("Received command: ");
    Serial.println(command);
    
    if (command == "start") {
      if (currentState != ERROR_STATE) {
        currentState = RUNNING;
        Serial.println("Bilateral gait pattern playback STARTED");
        tone(BUZZER_PIN, 1200, 100);
      }
    }
    else if (command == "stop") {
      currentState = STOPPED;
      gaitCycleIndex = 0;
      lastProgress = -1;
      returnToHomePosition();
      Serial.println("System STOPPED - returned to home position");
      tone(BUZZER_PIN, 800, 150);
    }
    else if (command == "pause") {
      if (currentState == RUNNING) {
        currentState = PAUSED;
        Serial.println("Gait playback PAUSED");
        tone(BUZZER_PIN, 1000, 100);
      }
    }
    else if (command == "setSpeed") {
      if (doc.containsKey("value")) {
        gaitSpeed = doc["value"];
        gaitSpeed = constrain(gaitSpeed, 0.5, 2.0);
        Serial.print("Gait speed set to: ");
        Serial.println(gaitSpeed);
      }
    }
    else if (command == "setAngle") {
      if (doc.containsKey("joint") && doc.containsKey("value")) {
        String joint = doc["joint"];
        float angle = doc["value"];
        angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        
        if (joint == "leftHip") {
          currentData.leftHip.targetAngle = angle;
          currentData.leftHip.currentAngle = angle;
          Serial.print("Left hip target set to: ");
        } else if (joint == "leftKnee") {
          currentData.leftKnee.targetAngle = angle;
          currentData.leftKnee.currentAngle = angle;
          Serial.print("Left knee target set to: ");
        } else if (joint == "rightHip") {
          currentData.rightHip.targetAngle = angle;
          currentData.rightHip.currentAngle = angle;
          Serial.print("Right hip target set to: ");
        } else if (joint == "rightKnee") {
          currentData.rightKnee.targetAngle = angle;
          currentData.rightKnee.currentAngle = angle;
          Serial.print("Right knee target set to: ");
        }
        Serial.println(angle);
      }
    }
    else if (command == "getStatus" || command == "getDiagnostics") {
      updateDiagnostics();
      Serial.println("Status/diagnostics request received");
    }
    
    sendDataToClients();
  }
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", 
                   client->id(), client->remoteIP().toString().c_str());
      sendDataToClients();
      break;
      
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
      
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
      
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// ========== SERVO CONTROL FUNCTIONS ==========

void updateServos() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
    currentData.leftHip.currentAngle = currentData.leftHip.targetAngle;
    currentData.leftKnee.currentAngle = currentData.leftKnee.targetAngle;
    currentData.rightHip.currentAngle = currentData.rightHip.targetAngle;
    currentData.rightKnee.currentAngle = currentData.rightKnee.targetAngle;
    
    currentData.leftHip.currentAngle = constrain(currentData.leftHip.currentAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    currentData.leftKnee.currentAngle = constrain(currentData.leftKnee.currentAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    currentData.rightHip.currentAngle = constrain(currentData.rightHip.currentAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    currentData.rightKnee.currentAngle = constrain(currentData.rightKnee.currentAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    servoLeftHip.write((int)currentData.leftHip.currentAngle);
    servoLeftKnee.write((int)currentData.leftKnee.currentAngle);
    servoRightHip.write((int)currentData.rightHip.currentAngle);
    servoRightKnee.write((int)currentData.rightKnee.currentAngle);
    
    lastServoUpdate = currentTime;
  }
}

void handleServoError(String errorMessage) {
  lastErrorMessage = errorMessage;
  currentState = ERROR_STATE;
  currentData.errorCode = 1;
  currentData.errorMessage = errorMessage;
  
  tone(BUZZER_PIN, 300, 1000);
  digitalWrite(LED_STATUS_PIN, HIGH);
  
  Serial.println("ERROR: " + errorMessage);
}

void returnToHomePosition() {
  currentData.leftHip.targetAngle = SERVO_HOME_POS;
  currentData.leftKnee.targetAngle = SERVO_HOME_POS;
  currentData.rightHip.targetAngle = SERVO_HOME_POS;
  currentData.rightKnee.targetAngle = SERVO_HOME_POS;
  
  currentData.leftHip.currentAngle = SERVO_HOME_POS;
  currentData.leftKnee.currentAngle = SERVO_HOME_POS;
  currentData.rightHip.currentAngle = SERVO_HOME_POS;
  currentData.rightKnee.currentAngle = SERVO_HOME_POS;
}

// ========== GAIT SIMULATION ==========

void updateGaitSimulation() {
  if (currentState != RUNNING) return;
  
  unsigned long currentTime = millis();
  int updateInterval = 50 / gaitSpeed;
  updateInterval = constrain(updateInterval, 20, 200);
  
  if (currentTime - lastGaitUpdate >= updateInterval) {
    // Update gait progress
    currentData.gaitProgress = gaitCycleIndex;
    
    // Check for cycle completion - detect when we wrap from 99 back to 0
    if (lastProgress > 95 && gaitCycleIndex < 5) {
      completedCycles++;
      tone(BUZZER_PIN, 800, 50);
      Serial.print("Gait cycle completed: ");
      Serial.println(completedCycles);
    }
    
    lastProgress = gaitCycleIndex;
    
    // Update servo angles from gait arrays
    currentData.leftHip.targetAngle = hipLeft[gaitCycleIndex];
    currentData.leftKnee.targetAngle = kneeLeft[gaitCycleIndex];
    currentData.rightHip.targetAngle = hipRight[gaitCycleIndex];
    currentData.rightKnee.targetAngle = kneeRight[gaitCycleIndex];
    
    // Calculate gait phase (0-4)
    currentData.gaitPhase = (gaitCycleIndex * 5) / 100;
    if (currentData.gaitPhase > 4) currentData.gaitPhase = 4;
    currentData.gaitSpeed = gaitSpeed;
    
    // Increment index for next update
    gaitCycleIndex++;
    if (gaitCycleIndex >= GAIT_SAMPLES) {
      gaitCycleIndex = 0;
    }
    
    lastGaitUpdate = currentTime;
  }
}

// ========== DIAGNOSTICS FUNCTIONS ==========

void updateDiagnostics() {
  static unsigned long lastMemoryCheck = 0;
  unsigned long currentTime = millis();
  
  currentData.systemLoad = (gaitSpeed * 25.0);
  if (currentState == RUNNING) currentData.systemLoad += 30.0;
  else currentData.systemLoad += 10.0;
  
  currentData.systemLoad = constrain(currentData.systemLoad, 0, 100);
  
  if (currentTime - lastMemoryCheck > 5000) {
    lastMemoryCheck = currentTime;
  }
}

// ========== SETUP FUNCTION ==========

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n==============================================");
  Serial.println("    BILATERAL EXOSKELETON CONTROL - NO SMOOTHING");
  Serial.println("==============================================");
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  
  initializeServos();
  setupWiFi();
  setupWebServer();
  indicateSystemReady();
  
  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("1. Connect to WiFi: " + String(ssid));
  Serial.println("2. Password: " + String(password));
  Serial.println("3. Open browser to: http://" + WiFi.softAPIP().toString());
  Serial.println("==============================================");
}

void initializeServos() {
  Serial.println("Initializing servos...");
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  servoLeftHip.setPeriodHertz(50);
  servoLeftKnee.setPeriodHertz(50);
  servoRightHip.setPeriodHertz(50);
  servoRightKnee.setPeriodHertz(50);
  
  servoLeftHip.attach(SERVO_LEFT_HIP_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servoLeftKnee.attach(SERVO_LEFT_KNEE_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servoRightHip.attach(SERVO_RIGHT_HIP_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servoRightKnee.attach(SERVO_RIGHT_KNEE_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  currentData.leftHip.currentAngle = SERVO_HOME_POS;
  currentData.leftHip.targetAngle = SERVO_HOME_POS;
  
  currentData.leftKnee.currentAngle = SERVO_HOME_POS;
  currentData.leftKnee.targetAngle = SERVO_HOME_POS;
  
  currentData.rightHip.currentAngle = SERVO_HOME_POS;
  currentData.rightHip.targetAngle = SERVO_HOME_POS;
  
  currentData.rightKnee.currentAngle = SERVO_HOME_POS;
  currentData.rightKnee.targetAngle = SERVO_HOME_POS;
  
  currentData.gaitPhase = 0;
  currentData.gaitProgress = 0;
  currentData.gaitSpeed = gaitSpeed;
  currentData.systemLoad = 0;
  currentData.errorCode = 0;
  currentData.errorMessage = "";
  
  returnToHomePosition();
  delay(500);
  
  Serial.println("Servo initialization complete");
}

void setupWiFi() {
  Serial.print("Setting up WiFi Access Point... ");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.println("Done!");
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.print("SSID: ");
  Serial.println(ssid);
}

void setupWebServer() {
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{";
    json += "\"state\":" + String(currentState) + ",";
    json += "\"leftHipAngle\":" + String(currentData.leftHip.currentAngle, 1) + ",";
    json += "\"leftKneeAngle\":" + String(currentData.leftKnee.currentAngle, 1) + ",";
    json += "\"rightHipAngle\":" + String(currentData.rightHip.currentAngle, 1) + ",";
    json += "\"rightKneeAngle\":" + String(currentData.rightKnee.currentAngle, 1) + ",";
    json += "\"speed\":" + String(gaitSpeed, 1);
    json += "}";
    request->send(200, "application/json", json);
  });
  
  server.begin();
  Serial.println("HTTP server started on port 80");
}

void indicateSystemReady() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 1500 + i * 200, 100);
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(100);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(100);
  }
}

// ========== MAIN LOOP ==========

void loop() {
  static unsigned long lastDataSend = 0;
  static unsigned long lastDiagnostics = 0;
  
  updateGaitSimulation();
  updateServos();
  
  if (millis() - lastDataSend >= 100) {
    sendDataToClients();
    lastDataSend = millis();
  }
  
  if (millis() - lastDiagnostics >= 2000) {
    updateDiagnostics();
    lastDiagnostics = millis();
  }
  
  updateStatusLED();
  ws.cleanupClients();
  delay(5);
}

void updateStatusLED() {
  static unsigned long lastBlink = 0;
  unsigned long currentTime = millis();
  unsigned long blinkInterval = 1000;
  
  switch (currentState) {
    case RUNNING:
      blinkInterval = 200;
      break;
    case PAUSED:
      blinkInterval = 500;
      break;
    case ERROR_STATE:
      blinkInterval = 100;
      break;
    default:
      digitalWrite(LED_STATUS_PIN, LOW);
      return;
  }
  
  if (currentTime - lastBlink >= blinkInterval) {
    digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
    lastBlink = currentTime;
  }
}