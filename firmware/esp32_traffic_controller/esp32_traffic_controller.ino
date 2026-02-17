/*
  Lifeline Traffic — ESP32 final (Galaxy Lite UI, embedded HTML)
  - Debounce, stuck-detection, Basic Auth, SoftAP fallback
  - Auto-phase: Green -> Yellow -> Red -> All-Red -> Next-Yellow -> Next-Green
  - UI polling 1000 ms
  - Embedded lightweight Galaxy-themed HTML (single-file)
*/

#define LED_BUILTIN 2

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <MFRC522.h>

// ====== CONFIG - change these before upload ======
const char* STA_SSID     = "Test";
const char* STA_PASSWORD = "123456789";

const char* HTTP_USER = "admin";
const char* HTTP_PASS = "1234";

const char* AP_SSID     = "TrafficAP";
const char* AP_PASSWORD = "traffic123";
// ===================================================

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// IR pins (unchanged)
#define IR_NORTH 34
#define IR_EAST  35
#define IR_WEST  36

// Traffic lights mapping: [direction][{RED,YELLOW,GREEN}]
int traffic[3][3] = {
  {13, 12, 14},  // North
  {27, 26, 25},  // East
  {33, 32, 23}   // West
};
const char* directions[3] = {"North","East","West"};

// ===== RFID =====
#define RFID_SS   5
#define RFID_RST  4   // IMPORTANT: NOT 22
MFRC522 rfid(RFID_SS, RFID_RST);

// RFID UIDs
byte northTag[4] = {0xF2, 0x76, 0x64, 0x06};
byte westTag[4]  = {0x1E, 0x46, 0xD1, 0x06};

// Server
WebServer server(80);

// State
int currentLight = 0;
unsigned long lastChange = 0;
int phase = 0; // 0=Green,1=Yellow,2=Red,3=All-Red,4=Next-Yellow
bool emergencyMode = false;
int emergencyActive = -1;

bool irMode = false;
int irActive = -1;
unsigned long irActivationTime = 0;
#define IR_HOLD_TIME 5000UL

// Durations (ms)
unsigned long greenTimeMs[3]  = {5000UL,5000UL,5000UL};
unsigned long yellowTimeMs[3] = {2000UL,2000UL,2000UL};
unsigned long redTimeMs[3]    = {1000UL,1000UL,1000UL};

const unsigned long ALL_RED_MS = 1500UL;

// UI helper
int remainingSeconds = 0;

// IR debounce / stuck detection
const int IR_SAMPLE_COUNT = 6;
const unsigned long IR_SAMPLE_TOTAL_MS = 40UL;
const unsigned long SENSOR_STUCK_THRESHOLD = 30000UL; // 30s

unsigned long lastIRChangeTime[3] = {0,0,0};
int lastIRRaw[3] = {HIGH, HIGH, HIGH};

// WiFi timeout
const unsigned long WIFI_CONNECT_TIMEOUT = 10000UL; // 10s

// ---------- Helpers ----------
void setTrafficSingle(int index,int state){
  // state: 0=GREEN,1=YELLOW,2=RED
  digitalWrite(traffic[index][0], LOW);
  digitalWrite(traffic[index][1], LOW);
  digitalWrite(traffic[index][2], LOW);
  if(state == 0) digitalWrite(traffic[index][2], HIGH);
  else if(state == 1) digitalWrite(traffic[index][1], HIGH);
  else digitalWrite(traffic[index][0], HIGH);
}

void setAllRedPins(){
  for(int i=0;i<3;i++){
    digitalWrite(traffic[i][0], HIGH);
    digitalWrite(traffic[i][1], LOW);
    digitalWrite(traffic[i][2], LOW);
  }
}

void showAllOnOLED(int active, const String &ph, int countdown){
  static unsigned long lastOLED = 0;
  unsigned long now = millis();
  if(now - lastOLED < 400) return;
  lastOLED = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  for(int i=0;i<3;i++){
    display.setCursor(0, i*18);
    display.print(directions[i]); display.print(": ");
    if(i==active) { display.print(ph); display.print(" "); display.print(countdown); display.print("s"); }
    else display.print("Red");
  }
  display.setCursor(0,54);
  display.print("IR N:"); display.print(digitalRead(IR_NORTH)==LOW?"D":"C");
  display.print(" E:"); display.print(digitalRead(IR_EAST)==LOW?"D":"C");
  display.print(" W:"); display.print(digitalRead(IR_WEST)==LOW?"D":"C");
  display.display();
}

void showEmergencyOLED(int active){
  static unsigned long lastOLED = 0;
  unsigned long now = millis();
  if(now - lastOLED < 400) return;
  lastOLED = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("Emergency Ambulance");
  for(int i=0;i<3;i++){
    display.setCursor(0,(i+1)*15);
    display.print(directions[i]); display.print(": ");
    display.print(i==active?"Green":"Red");
  }
  display.display();
}

void showIROLED(int active){
  static unsigned long lastOLED = 0;
  unsigned long now = millis();
  if(now - lastOLED < 400) return;
  lastOLED = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("IR Sensor Activated");
  for(int i=0;i<3;i++){
    display.setCursor(0,(i+1)*15);
    display.print(directions[i]); display.print(": ");
    display.print(i==active?"Green":"Red");
  }
  unsigned long remaining = (IR_HOLD_TIME - (millis() - irActivationTime)) / 1000;
  display.setCursor(0,54);
  display.print("Hold: ");
  display.print(remaining);
  display.print("s");
  display.display();
}

// Debounced sampling (returns true if sensor majority low)
bool stableDigitalRead(int pin){
  int countLow = 0;
  unsigned long step = IR_SAMPLE_TOTAL_MS / (IR_SAMPLE_COUNT > 0 ? IR_SAMPLE_COUNT : 1);
  for(int i=0;i<IR_SAMPLE_COUNT;i++){
    if(digitalRead(pin) == LOW) countLow++;
    delay(step);
  }
  return (countLow >= (IR_SAMPLE_COUNT * 2 / 3));
}

// IR checking with debounce + stuck protection
void checkIRSensors(){
  if(emergencyMode) return;

  if(irMode){
    if(millis() - irActivationTime >= IR_HOLD_TIME){
      irMode = false;
      irActive = -1;
      lastChange = millis(); // resume auto timings
    }
    return;
  }

  bool north = stableDigitalRead(IR_NORTH);
  bool east  = stableDigitalRead(IR_EAST);
  bool west  = stableDigitalRead(IR_WEST);

  int rawVals[3] = { digitalRead(IR_NORTH), digitalRead(IR_EAST), digitalRead(IR_WEST) };
  for(int i=0;i<3;i++){
    if(rawVals[i] != lastIRRaw[i]){
      lastIRRaw[i] = rawVals[i];
      lastIRChangeTime[i] = millis();
    }
  }

  if(millis() - lastIRChangeTime[0] > SENSOR_STUCK_THRESHOLD) north = false;
  if(millis() - lastIRChangeTime[1] > SENSOR_STUCK_THRESHOLD) east  = false;
  if(millis() - lastIRChangeTime[2] > SENSOR_STUCK_THRESHOLD) west  = false;

  if(north || east || west){
    irMode = true;
    if(north) irActive = 0;
    else if(east) irActive = 1;
    else if(west) irActive = 2;
    irActivationTime = millis();

    for(int i=0;i<3;i++){
      if(i==irActive) setTrafficSingle(i,0); else setTrafficSingle(i,2);
    }
  }
}

// ---------- NEW AutoTraffic with Next-Yellow step ----------
void autoTraffic(){
  unsigned long now = millis();
  unsigned long duration;

  switch(phase){
    case 0: duration = greenTimeMs[currentLight]; break;                    // GREEN
    case 1: duration = yellowTimeMs[currentLight]; break;                   // YELLOW
    case 2: duration = redTimeMs[currentLight]; break;                      // RED
    case 3: duration = ALL_RED_MS; break;                                    // ALL-RED
    case 4: duration = yellowTimeMs[(currentLight+1)%3]; break;             // NEXT-YELLOW (next dir)
    default: duration = greenTimeMs[currentLight]; break;
  }

  if(now - lastChange >= duration){
    phase++;
    if(phase > 4){
      phase = 0;
      // after finishing phase 4, next is phase 0 (green) for currentLight (which was advanced when entering phase 4)
    } else if(phase == 4){
      // advance direction when entering phase 4 (next-yellow)
      currentLight = (currentLight + 1) % 3;
    }
    lastChange = now;
  }

  // Apply lights according to phase
  if(phase == 0){
    for(int i=0;i<3;i++) setTrafficSingle(i, i==currentLight ? 0 : 2); // GREEN
  } else if(phase == 1){
    for(int i=0;i<3;i++) setTrafficSingle(i, i==currentLight ? 1 : 2); // YELLOW
  } else if(phase == 2){
    for(int i=0;i<3;i++) setTrafficSingle(i, 2); // RED
  } else if(phase == 3){
    setAllRedPins(); // ALL-RED
  } else if(phase == 4){
    for(int i=0;i<3;i++) setTrafficSingle(i, i==currentLight ? 1 : 2); // NEXT-YELLOW (currentLight already advanced)
  }

  long rem = (long)duration - (long)(now - lastChange);
  if(rem < 0) rem = 0;
  remainingSeconds = (int)((rem + 999) / 1000);

  String phName = (phase==0?"Green":(phase==1?"Yellow":(phase==2?"Red":(phase==3?"All-Red":"Next-Yellow"))));
  showAllOnOLED(currentLight, phName, remainingSeconds);
}

// Emergency handling
void emergencyTraffic(){
  for(int i=0;i<3;i++) setTrafficSingle(i, (i==emergencyActive ? 0 : 2));
  showEmergencyOLED(emergencyActive);
}

// IR traffic display
void irTraffic(){
  showIROLED(irActive);
}

bool matchUID(byte *uid, byte *target){
  for(int i=0;i<4;i++){
    if(uid[i] != target[i]) return false;
  }
  return true;
}

void checkRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  Serial.print("RFID UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Match NORTH
  if (rfid.uid.size == 4 &&
      rfid.uid.uidByte[0] == 0xF2 &&
      rfid.uid.uidByte[1] == 0x76 &&
      rfid.uid.uidByte[2] == 0x64 &&
      rfid.uid.uidByte[3] == 0x06) {

    emergencyMode = true;
    emergencyActive = 0; // North
    irMode = false;
    irActive = -1;
    Serial.println("🚑 NORTH EMERGENCY");
  }

  // Match WEST
  else if (rfid.uid.size == 4 &&
           rfid.uid.uidByte[0] == 0x1E &&
           rfid.uid.uidByte[1] == 0x46 &&
           rfid.uid.uidByte[2] == 0xD1 &&
           rfid.uid.uidByte[3] == 0x06) {

    emergencyMode = true;
    emergencyActive = 2; // West
    irMode = false;
    irActive = -1;
    Serial.println("🚑 WEST EMERGENCY");
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ========== Embedded lightweight Galaxy UI (LITE) ==========
String htmlPage(){
  String page = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lifeline Traffic</title>
<style>
:root{--bg1:#070217;--bg2:#1b0831;--muted:#d6cee8;--accent:#a57bff}
*{box-sizing:border-box;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial}
html,body{height:100%;margin:0;background:radial-gradient(800px 400px at 10% 20%, rgba(165,123,255,0.08), transparent 15%), linear-gradient(180deg,var(--bg1),var(--bg2));color:var(--muted)}
.container{max-width:1100px;margin:24px auto;display:grid;grid-template-columns:320px 1fr;gap:18px;padding:12px}
.card{background:rgba(255,255,255,0.02);padding:14px;border-radius:12px;border:1px solid rgba(255,255,255,0.03)}
.brand{display:flex;align-items:center;gap:10px}
.logo{width:44px;height:44;border-radius:10px;background:linear-gradient(135deg,#7a3bff,#3b7aff);display:flex;align-items:center;justify-content:center;color:#fff;font-weight:700}
.stats{margin-top:12px;display:flex;flex-direction:column;gap:10px}
.stat{display:flex;justify-content:space-between;padding:8px;background:rgba(0,0,0,0.12);border-radius:8px}
.controls{margin-top:12px;display:flex;flex-direction:column;gap:8px}
.btn{background:linear-gradient(90deg,#7a4bff,#3b8dff);color:white;border:none;padding:9px;border-radius:10px;cursor:pointer;font-weight:600}
.ghost{background:transparent;border:1px solid rgba(255,255,255,0.06);color:var(--muted)}
.inputs{display:flex;gap:6px;margin-top:8px}
.input{width:60px;padding:6px;border-radius:8px;border:1px solid rgba(255,255,255,0.03);background:transparent;color:var(--muted)}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}
.signal{display:flex;flex-direction:column;align-items:center;padding:10px;border-radius:10px;background:rgba(255,255,255,0.01)}
.light{width:52px;height:52px;border-radius:50%;opacity:0.22;margin:6px 0;transition:all .12s}
.red{background:radial-gradient(circle at 30% 30%, #ff7b7b,#6f0000)}
.yellow{background:radial-gradient(circle at 30% 30%, #ffeaa7,#b38f00)}
.green{background:radial-gradient(circle at 30% 30%, #9affd7,#004d2e)}
.active{opacity:1;box-shadow:0 8px 24px rgba(165,123,255,0.12)}
.status{margin-top:12px;padding:10px;border-radius:8px;background:linear-gradient(90deg, rgba(165,123,255,0.04), rgba(59,141,255,0.02));display:flex;justify-content:space-between;align-items:center}
.note{font-size:12px;color:rgba(255,255,255,0.6);text-align:center;margin-top:10px}
.footer{margin-top:10px;font-size:12px;color:rgba(255,255,255,0.5);text-align:center}
@media(max-width:900px){.container{grid-template-columns:1fr}.grid{grid-template-columns:1fr}}
</style>
</head><body>
<div class="container">
  <div class="card">
    <div class="brand"><div class="logo">TL</div><div><div style="font-weight:700">Lifeline Traffic</div><div style="font-size:13px;color:rgba(255,255,255,0.6)">Galaxy Lite</div></div></div>
    <div class="stats">
      <div class="stat"><div>Mode</div><div id="modeVal">Auto</div></div>
      <div class="stat"><div>Active</div><div id="activeDir">North</div></div>
      <div class="stat"><div>Timer</div><div id="timerVal">--s</div></div>
    </div>

    <div class="controls">
      <div style="display:flex;gap:8px">
        <button class="btn" onclick="emergency(0)">🚑 North</button>
        <button class="btn" onclick="emergency(1)">🚑 East</button>
        <button class="btn" onclick="emergency(2)">🚑 West</button>
      </div>
      <div style="display:flex;gap:8px;margin-top:8px">
        <button class="btn ghost" onclick="resetAuto()">Reset</button>
      </div>

      <div style="margin-top:10px">
        <div style="font-size:13px;margin-bottom:6px">Set all timings (s)</div>
        <div class="inputs">
          <input id="gAll" class="input" type="number" placeholder="G">
          <input id="yAll" class="input" type="number" placeholder="Y">
          <input id="rAll" class="input" type="number" placeholder="R">
          <button class="btn" onclick="applyAll()">Apply</button>
        </div>
      </div>
    </div>

    <div class="note">Control endpoints are protected with Basic Auth. Status polling is public.</div>
    <div class="footer">© Lifeline • Galaxy Lite</div>
  </div>

  <div class="card">
    <div style="display:flex;justify-content:space-between;align-items:center">
      <div style="font-weight:700">Intersection Dashboard</div>
      <div style="font-size:13px;color:rgba(255,255,255,0.6)">Realtime</div>
    </div>

    <div class="grid" style="margin-top:12px">
      <div class="signal">
        <div style="font-weight:700">North</div>
        <div class="light red" id="red0"></div>
        <div class="light yellow" id="yellow0"></div>
        <div class="light green" id="green0"></div>
        <div style="display:flex;gap:6px;margin-top:8px">
          <input id="g0" class="input" type="number" placeholder="G">
          <input id="y0" class="input" type="number" placeholder="Y">
          <input id="r0" class="input" type="number" placeholder="R">
        </div>
        <button class="btn" style="margin-top:8px" onclick="applyDir(0)">Apply North</button>
      </div>

      <div class="signal">
        <div style="font-weight:700">East</div>
        <div class="light red" id="red1"></div>
        <div class="light yellow" id="yellow1"></div>
        <div class="light green" id="green1"></div>
        <div style="display:flex;gap:6px;margin-top:8px">
          <input id="g1" class="input" type="number" placeholder="G">
          <input id="y1" class="input" type="number" placeholder="Y">
          <input id="r1" class="input" type="number" placeholder="R">
        </div>
        <button class="btn" style="margin-top:8px" onclick="applyDir(1)">Apply East</button>
      </div>

      <div class="signal">
        <div style="font-weight:700">West</div>
        <div class="light red" id="red2"></div>
        <div class="light yellow" id="yellow2"></div>
        <div class="light green" id="green2"></div>
        <div style="display:flex;gap:6px;margin-top:8px">
          <input id="g2" class="input" type="number" placeholder="G">
          <input id="y2" class="input" type="number" placeholder="Y">
          <input id="r2" class="input" type="number" placeholder="R">
        </div>
        <button class="btn" style="margin-top:8px" onclick="applyDir(2)">Apply West</button>
      </div>
    </div>

    <div class="status" style="margin-top:12px">
      <div>
        <div id="statusText">Loading…</div>
        <div style="font-size:13px;color:rgba(255,255,255,0.8)" id="timerText">—</div>
      </div>
      <div style="text-align:right">
        <div style="font-weight:700">IR Sensors</div>
        <div id="irStatus" style="font-size:13px;color:rgba(255,255,255,0.8)">N: — | E: — | W: —</div>
      </div>
    </div>

  </div>
</div>

<script>
async function update(){
  try{
    const res = await fetch('/status');
    const d = await res.json();

    ['red','yellow','green'].forEach(c=>{
      for(let i=0;i<3;i++){
        document.getElementById(c+i).className = 'light ' + c;
      }
    });

    for(let i=0;i<3;i++){
      document.getElementById('g'+i).value = Math.round(d.durations[i].green/1000);
      document.getElementById('y'+i).value = Math.round(d.durations[i].yellow/1000);
      document.getElementById('r'+i).value = Math.round(d.durations[i].red/1000);
    }

    document.getElementById('irStatus').innerText = 'N: ' + (d.irSensors[0] ? 'Detected' : 'Clear') + ' | E: ' + (d.irSensors[1] ? 'Detected' : 'Clear') + ' | W: ' + (d.irSensors[2] ? 'Detected' : 'Clear');

    if(d.emergency){
      document.getElementById('modeVal').innerText = 'Emergency';
      const a = d.emergencyActive;
      document.getElementById('activeDir').innerText = d.directions[a];
      document.getElementById('timerVal').innerText = '';
      document.getElementById('green'+a).classList.add('active');
      document.getElementById('statusText').innerText = '🚑 Emergency — ' + d.directions[a] + ' Green';
      document.getElementById('timerText').innerText = '';
    } else if(d.irMode){
      document.getElementById('modeVal').innerText = 'IR Priority';
      document.getElementById('activeDir').innerText = d.directions[d.irActive];
      document.getElementById('timerVal').innerText = '';
      document.getElementById('green'+d.irActive).classList.add('active');
      document.getElementById('statusText').innerText = '🚗 IR — ' + d.directions[d.irActive] + ' Green';
      document.getElementById('timerText').innerText = 'IR hold: ' + d.irRemaining + 's';
    } else {
      document.getElementById('modeVal').innerText = 'Auto';
      document.getElementById('activeDir').innerText = d.directions[d.currentLight];
      document.getElementById('timerVal').innerText = d.remainingSeconds + 's';
      const id = (d.phase==0?'green':(d.phase==1?'yellow':(d.phase==2?'red':(d.phase==3?'red':'yellow')))) + d.currentLight;
      document.getElementById(id).classList.add('active');
      document.getElementById('statusText').innerText = 'Auto — ' + d.directions[d.currentLight] + ' (' + (d.phase==0?'Green':(d.phase==1?'Yellow':(d.phase==2?'Red':(d.phase==3?'All-Red':'Next-Yellow')))) + ')';
      document.getElementById('timerText').innerText = 'Time left: ' + d.remainingSeconds + 's';
    }
  } catch(e){
    document.getElementById('statusText').innerText = 'No response';
  }
}

function applyDir(i){
  let g = document.getElementById('g'+i).value || 0;
  let y = document.getElementById('y'+i).value || 0;
  let r = document.getElementById('r'+i).value || 0;
  fetch('/settimes?dir='+i+'&g='+g+'&y='+y+'&r='+r).then(()=>setTimeout(update,300));
}

function applyAll(){
  let g = document.getElementById('gAll').value || 0;
  let y = document.getElementById('yAll').value || 0;
  let r = document.getElementById('rAll').value || 0;
  fetch('/setall?g='+g+'&y='+y+'&r='+r).then(()=>setTimeout(update,300));
}

function emergency(dir){
  fetch('/traffic'+(dir+1)).then(()=>setTimeout(update,200));
}
function resetAuto(){ fetch('/reset').then(()=>setTimeout(update,250)); }

setInterval(update,1000);
window.onload = update;
</script>
</body></html>
)rawliteral";
  return page;
}

// ========== /status builder ==========
String getStatus(){
  String s = "{";
  if (emergencyMode && emergencyActive >= 0) {
    s += "\"emergency\":true,";
    s += "\"emergencyActive\":"+String(emergencyActive)+",";
  } else if (irMode && irActive >= 0) {
    s += "\"emergency\":false,";
    s += "\"irMode\":true,";
    s += "\"irActive\":"+String(irActive)+",";
    long irRemaining = (long)(IR_HOLD_TIME - (millis() - irActivationTime)) / 1000;
    if(irRemaining < 0) irRemaining = 0;
    s += "\"irRemaining\":"+String(irRemaining)+",";
  } else {
    s += "\"emergency\":false,";
    s += "\"irMode\":false,";
    s += "\"currentLight\":"+String(currentLight)+",";
    s += "\"phase\":"+String(phase)+",";
    s += "\"remainingSeconds\":"+String(remainingSeconds)+",";
  }

  s += "\"durations\":[";
  for(int i=0;i<3;i++){
    s += "{\"green\":"+String(greenTimeMs[i])+",\"yellow\":"+String(yellowTimeMs[i])+",\"red\":"+String(redTimeMs[i])+"}";
    if(i<2) s += ",";
  }
  s += "],";

  s += "\"irSensors\":[" + String((digitalRead(IR_NORTH)==LOW)?1:0) + "," + String((digitalRead(IR_EAST)==LOW)?1:0) + "," + String((digitalRead(IR_WEST)==LOW)?1:0) + "],";

  s += "\"directions\":[\"North\",\"East\",\"West\"]";
  s += "}";
  return s;
}

// ========== Handlers ==========
void handleSetTimes(){
  if(!server.hasArg("dir")) { server.send(400,"text/plain","Missing dir"); return; }
  int dir = server.arg("dir").toInt();
  if(dir < 0 || dir > 2) { server.send(400,"text/plain","Invalid dir"); return; }
  if(server.hasArg("g")) greenTimeMs[dir] = (unsigned long)server.arg("g").toInt() * 1000UL;
  if(server.hasArg("y")) yellowTimeMs[dir] = (unsigned long)server.arg("y").toInt() * 1000UL;
  if(server.hasArg("r")) redTimeMs[dir] = (unsigned long)server.arg("r").toInt() * 1000UL;

  unsigned long now = millis();
  unsigned long curDur = (phase==0?greenTimeMs[currentLight]:(phase==1?yellowTimeMs[currentLight]:redTimeMs[currentLight]));
  if(now - lastChange > curDur) lastChange = now;

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleSetAll(){
  if(server.hasArg("g")){
    unsigned long g = (unsigned long)server.arg("g").toInt() * 1000UL;
    for(int i=0;i<3;i++) greenTimeMs[i] = g;
  }
  if(server.hasArg("y")){
    unsigned long y = (unsigned long)server.arg("y").toInt() * 1000UL;
    for(int i=0;i<3;i++) yellowTimeMs[i] = y;
  }
  if(server.hasArg("r")){
    unsigned long r = (unsigned long)server.arg("r").toInt() * 1000UL;
    for(int i=0;i<3;i++) redTimeMs[i] = r;
  }

  unsigned long now = millis();
  unsigned long curDur = (phase==0?greenTimeMs[currentLight]:(phase==1?yellowTimeMs[currentLight]:redTimeMs[currentLight]));
  if(now - lastChange > curDur) lastChange = now;

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleEmergency1(){ emergencyMode=true; emergencyActive=0; irMode=false; irActive=-1; server.send(200,"text/html",htmlPage()); }
void handleEmergency2(){ emergencyMode=true; emergencyActive=1; irMode=false; irActive=-1; server.send(200,"text/html",htmlPage()); }
void handleEmergency3(){ emergencyMode=true; emergencyActive=2; irMode=false; irActive=-1; server.send(200,"text/html",htmlPage()); }

void handleReset(){ emergencyMode=false; emergencyActive=-1; irMode=false; irActive=-1; lastChange=millis(); phase=0; server.send(200,"text/html",htmlPage()); }

bool checkAuth(){
  if(!server.authenticate(HTTP_USER, HTTP_PASS)){
    server.requestAuthentication();
    return false;
  }
  return true;
}

// ========== Setup ==========
void setup(){
  Serial.begin(115200);

  // IR pins unchanged
  pinMode(IR_NORTH, INPUT);
  pinMode(IR_EAST, INPUT);
  pinMode(IR_WEST, INPUT);

  // Traffic pins
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      pinMode(traffic[i][j], OUTPUT);
      digitalWrite(traffic[i][j], LOW);
    }
  }

  pinMode(LED_BUILTIN, OUTPUT);

  // OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("SSD1306 init failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,20); display.print("Smart Traffic");
    display.setCursor(0,35); display.print("Controller");
    display.display();
    delay(700);
  }

// ===== RFID INIT =====
SPI.begin(18, 19, 23, RFID_SS);   // SCK, MISO, MOSI, SS
rfid.PCD_Init();
Serial.println("RFID initialized");

  // WiFi connect with timeout and AP fallback
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long t0 = millis();
  while(WiFi.status() != WL_CONNECTED){
    delay(200);
    Serial.print(".");
    if(millis() - t0 > WIFI_CONNECT_TIMEOUT){
      Serial.println("\nWiFi connect timeout, starting AP mode");
      WiFi.mode(WIFI_AP);
      WiFi.softAP(AP_SSID, AP_PASSWORD);
      Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
      break;
    }
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println(); Serial.print("IP: "); Serial.println(WiFi.localIP());
  }

  // Web handlers
  server.on("/", [](){ if(!checkAuth()) return; server.send(200,"text/html",htmlPage()); });
  server.on("/traffic1", [](){ if(!checkAuth()) return; handleEmergency1(); });
  server.on("/traffic2", [](){ if(!checkAuth()) return; handleEmergency2(); });
  server.on("/traffic3", [](){ if(!checkAuth()) return; handleEmergency3(); });
  server.on("/reset", [](){ if(!checkAuth()) return; handleReset(); });

  server.on("/status", [](){ server.send(200, "application/json", getStatus()); });
  server.on("/settimes", [](){ if(!checkAuth()) return; handleSetTimes(); });
  server.on("/setall", [](){ if(!checkAuth()) return; handleSetAll(); });

  server.begin();
  lastChange = millis();
  Serial.println("Server started");
}

// ========== Loop ==========
void loop() {
  server.handleClient();

  checkRFID();      // 

  checkIRSensors();

  if (emergencyMode && emergencyActive >= 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    emergencyTraffic();
  } else if (irMode && irActive >= 0) {
    digitalWrite(LED_BUILTIN, LOW);
    irTraffic();
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    autoTraffic();
  }
}