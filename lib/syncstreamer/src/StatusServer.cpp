#include "StatusServer.h"
#include "SyncController.h"
#include "JitterBuffer.h"
#include "OffsetEstimator.h"
#include <Arduino.h>
#include <WiFi.h>
#include <ElegantOTA.h>
#include <esp_mac.h>
#include <stdio.h>
#include <string.h>

// ── MQTT ──────────────────────────────────────────────────────────────────────
static WiFiClient    s_wifi_client;
static PubSubClient  s_mqtt(s_wifi_client);
static char          s_node_id[20];
static char          s_mqtt_topic[48];
static uint32_t      s_last_mqtt_ms = 0;

// ── Config adjustable via /api/config ────────────────────────────────────────
static volatile uint32_t s_target_fill_ms = JB_TARGET_MS;   // mirrors JB_TARGET_MS

// ── Dashboard HTML ────────────────────────────────────────────────────────────
static const char PAGE_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SyncStreamer</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Courier New',monospace;padding:1.2rem}
h1{color:#58a6ff;border-bottom:1px solid #30363d;padding-bottom:.5rem;margin-bottom:1.2rem;font-size:1.4rem;letter-spacing:.05em}
.badge{display:inline-block;padding:.25rem .7rem;border-radius:20px;font-weight:bold;font-size:.85rem;margin-left:.6rem;vertical-align:middle}
.b-idle{background:#30363d;color:#8b949e}
.b-acquiring{background:#5a4000;color:#e3b341}
.b-locked{background:#0d3621;color:#3fb950}
.b-recovering{background:#5a2e00;color:#ffa657}
.b-reacquiring{background:#4d1c1c;color:#f85149}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:.8rem;margin-bottom:1rem}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:.9rem;border-top-width:3px}
.card h2{font-size:.72rem;text-transform:uppercase;letter-spacing:.12em;margin-bottom:.6rem;opacity:.8}
.c1{border-top-color:#58a6ff}.c1 h2{color:#58a6ff}
.c2{border-top-color:#3fb950}.c2 h2{color:#3fb950}
.c3{border-top-color:#ffa657}.c3 h2{color:#ffa657}
.c4{border-top-color:#d2a8ff}.c4 h2{color:#d2a8ff}
.row{display:flex;justify-content:space-between;padding:.18rem 0;border-bottom:1px solid #21262d;font-size:.81rem}
.row:last-child{border-bottom:none}.val{font-weight:bold}
.ok{color:#3fb950}.warn{color:#e3b341}.err{color:#f85149}
.bar-bg{background:#21262d;border-radius:4px;height:8px;margin-top:.5rem;overflow:hidden}
.bar-fg{height:8px;border-radius:4px;transition:width .5s ease}
.cfg{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:.9rem;margin-bottom:.8rem;border-top:3px solid #e3b341}
.cfg h2{color:#e3b341;font-size:.72rem;text-transform:uppercase;letter-spacing:.12em;margin-bottom:.7rem}
label{font-size:.82rem;margin-right:1rem;display:inline-flex;align-items:center;gap:.3rem}
select,input[type=number]{background:#0d1117;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;padding:.25rem .4rem}
input[type=number]{width:80px}
.btn{border:none;border-radius:5px;padding:.38rem .9rem;cursor:pointer;font-size:.82rem;margin-right:.4rem;color:#fff;font-family:inherit}
.apply{background:#238636}.apply:hover{background:#2ea043}
.ota{background:#1f6feb}.ota:hover{background:#388bfd}
#ts{font-size:.7rem;color:#484f58;margin-top:.5rem}
</style></head>
<body>
<h1>&#x1F50A; SyncStreamer <span id="statebadge" class="badge b-idle">IDLE</span>
  <span id="duckbadge"></span>
  <span style="font-size:.55em;color:#484f58;float:right;margin-top:.4rem" id="ipspan"></span>
</h1>
<div class="grid">
  <div class="card c1"><h2>&#x1F4F6; Network</h2><div id="d_net"></div></div>
  <div class="card c2"><h2>&#x1F9EE; Buffer</h2><div id="d_buf"></div></div>
  <div class="card c3"><h2>&#x23F1; PLL / Rate</h2><div id="d_pll"></div></div>
  <div class="card c4"><h2>&#x1F4E1; Stats</h2><div id="d_stat"></div></div>
</div>
<div class="cfg">
  <h2>&#x2699; Configuration</h2>
  <form id="cfgform">
    <label>Mode
      <select name="mode" id="c_mode">
        <option value="music">Music</option>
        <option value="tts">TTS</option>
      </select>
    </label>
    <label>Target fill <input type="number" name="target_fill_ms" id="c_fill" min="20" max="500">ms</label>
    <button class="btn apply" type="submit">Apply</button>
    <button class="btn ota" type="button" onclick="location='/update'">&#x1F504; OTA Update</button>
  </form>
</div>
<div id="ts">Connecting...</div>
<script>
const STATES={0:'IDLE',1:'ACQUIRING',2:'LOCKED',3:'RECOVERING',4:'REACQUIRING'};
const BCLASS={0:'b-idle',1:'b-acquiring',2:'b-locked',3:'b-recovering',4:'b-reacquiring'};
function row(l,v,c=''){return`<div class="row"><span>${l}</span><span class="val ${c}">${v}</span></div>`}
function bar(pct,col){return`<div class="bar-bg"><div class="bar-fg" style="width:${Math.min(Math.max(pct,0),100)}%;background:${col}"></div></div>`}
function rclr(rssi){return rssi>=-65?'ok':rssi>=-80?'warn':'err'}
function update(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    // Header
    const sb=document.getElementById('statebadge');
    sb.textContent=STATES[d.state]??d.state;
    sb.className='badge '+(BCLASS[d.state]??'b-idle');
    document.getElementById('ipspan').textContent=d.ip;
    const db=document.getElementById('duckbadge');
    db.innerHTML=d.ducked?'<span class="badge" style="background:#4d1c1c;color:#f85149">DUCK</span>':'';
    // Network
    document.getElementById('d_net').innerHTML=
      row('SSID',d.ssid)+row('RSSI',d.rssi+' dBm',rclr(d.rssi))+row('Uptime',d.uptime_s+'s');
    // Buffer
    const fp=Math.round(d.fill_ms/(d.target_ms||60)*100);
    const bc=d.state===2?'#3fb950':d.state===3?'#ffa657':'#e3b341';
    document.getElementById('d_buf').innerHTML=
      row('Fill',d.fill_ms+' ms')+
      row('Target',d.target_ms+' ms')+
      row('Seq gaps',d.seq_gaps,d.seq_gaps?'warn':'')+
      bar(fp,bc);
    // PLL
    const ac=Math.abs(d.rate_ppm)<50?'ok':Math.abs(d.rate_ppm)<150?'warn':'err';
    document.getElementById('d_pll').innerHTML=
      row('Rate PPM',(d.rate_ppm>=0?'+':'')+d.rate_ppm.toFixed(1),ac)+
      row('Filtered err',d.filtered_err.toFixed(1)+' fr')+
      row('Offset',d.offset_us+'&thinsp;µs')+
      row('Mode',d.mode.toUpperCase());
    // Stats
    document.getElementById('d_stat').innerHTML=
      row('Occupancy',d.occ_frames+' fr')+
      row('Stalled',d.stalled?'YES':'no',d.stalled?'err':'');
    // Config
    document.getElementById('c_mode').value=d.mode;
    document.getElementById('c_fill').value=d.target_ms;
    document.getElementById('ts').textContent='Updated: '+new Date().toLocaleTimeString();
  }).catch(()=>{document.getElementById('ts').textContent='\u26A0 Connection lost';});
}
document.getElementById('cfgform').addEventListener('submit',e=>{
  e.preventDefault();
  fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:new URLSearchParams(new FormData(e.target)).toString()}).then(()=>update());
});
update();setInterval(update,2000);
</script></body></html>
)rawhtml";

// ── Init ──────────────────────────────────────────────────────────────────────
void status_server_init(AsyncWebServer* server)
{
    // Build node-id and MQTT topic from MAC address.
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(s_node_id,  sizeof(s_node_id),  "%02x%02x%02x", mac[3], mac[4], mac[5]);
    snprintf(s_mqtt_topic, sizeof(s_mqtt_topic), "speaker/%s/status", s_node_id);

    // MQTT
    s_mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    s_mqtt.setBufferSize(256);

    // Routes
    server->on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", reinterpret_cast<const char*>(PAGE_HTML));
    });

    server->on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        const char* state_name[] = {"idle","acquiring","locked","recovering","reacquiring"};
        const char* mode_str = (g_mode == MODE_TTS) ? "tts" : "music";
        int  sn = (int)g_state;
        if (sn < 0 || sn > 4) sn = 0;

        char buf[512];
        snprintf(buf, sizeof(buf),
            "{"
            "\"state\":%d,\"ip\":\"%s\",\"ssid\":\"%s\","
            "\"rssi\":%d,\"uptime_s\":%lu,"
            "\"fill_ms\":%u,\"target_ms\":%u,\"occ_frames\":%u,"
            "\"rate_ppm\":%.2f,\"filtered_err\":%.2f,"
            "\"offset_us\":%lld,\"seq_gaps\":%u,"
            "\"ducked\":%s,\"stalled\":%s,\"mode\":\"%s\""
            "}",
            sn,
            WiFi.localIP().toString().c_str(),
            WiFi.SSID().c_str(),
            WiFi.RSSI(),
            (unsigned long)(esp_timer_get_time() / 1000000ULL),
            jb_occupancy_ms(),
            s_target_fill_ms,
            jb_occupancy_frames(),
            (float)g_rate_ppm,
            (float)g_filtered_err,
            g_offset_us,
            g_seq_gaps,
            g_ducked  ? "true" : "false",
            jb_stalled() ? "true" : "false",
            mode_str
        );
        req->send(200, "application/json", buf);
    });

    server->on("/api/config", HTTP_POST,
        [](AsyncWebServerRequest* req) {
            if (req->hasParam("target_fill_ms", true)) {
                uint32_t v = (uint32_t)req->getParam("target_fill_ms", true)->value().toInt();
                if (v >= 20 && v <= 500) {
                    s_target_fill_ms = v;
                    log_i("StatusServer: target_fill_ms=%u", v);
                }
            }
            if (req->hasParam("mode", true)) {
                String m = req->getParam("mode", true)->value();
                g_mode = (m == "tts") ? MODE_TTS : MODE_MUSIC;
                log_i("StatusServer: mode=%s", m.c_str());
            }
            req->send(200, "text/plain", "OK");
        }
    );

    ElegantOTA.begin(server);
    server->begin();
    log_i("StatusServer: listening — node_id=%s mqtt_topic=%s", s_node_id, s_mqtt_topic);
}

// ── loop ──────────────────────────────────────────────────────────────────────
void status_server_loop(AsyncWebServer* /*server*/)
{
    ElegantOTA.loop();

    // MQTT reconnect + publish every MQTT_INTERVAL_MS.
    if (!s_mqtt.connected()) {
        char client_id[24];
        snprintf(client_id, sizeof(client_id), "syncstreamer-%s", s_node_id);
        s_mqtt.connect(client_id);  // non-blocking attempt; ignore failure
    }
    s_mqtt.loop();

    uint32_t now = millis();
    if (now - s_last_mqtt_ms >= MQTT_INTERVAL_MS && s_mqtt.connected()) {
        s_last_mqtt_ms = now;

        const char* state_name[] = {"idle","acquiring","locked","recovering","reacquiring"};
        int sn = (int)g_state;
        if (sn < 0 || sn > 4) sn = 0;

        char payload[256];
        snprintf(payload, sizeof(payload),
            "{\"state\":\"%s\",\"fill_ms\":%u,\"rate_ppm\":%.2f,"
            "\"offset_us\":%lld,\"seq_gaps\":%u,\"rssi\":%d,\"uptime_s\":%lu}",
            state_name[sn],
            jb_occupancy_ms(),
            (float)g_rate_ppm,
            g_offset_us,
            g_seq_gaps,
            WiFi.RSSI(),
            (unsigned long)(esp_timer_get_time() / 1000000ULL)
        );
        s_mqtt.publish(s_mqtt_topic, payload);
    }
}
