#include "StatusServer.h"
#include <Arduino.h>
#include <WiFi.h>
#include <stdio.h>

// ── HTML page (served once; JS polls /api/status every 2 s) ──────────────────
static const char PAGE_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SyncStreamer</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:monospace;padding:1rem}
h1{color:#58a6ff;border-bottom:1px solid #30363d;padding-bottom:.5rem;margin-bottom:1rem;font-size:1.3rem}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(270px,1fr));gap:.8rem;margin-bottom:1rem}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:.9rem;border-top-width:3px}
.card h2{font-size:.75rem;text-transform:uppercase;letter-spacing:.12em;margin-bottom:.6rem}
.net{border-top-color:#58a6ff}.net h2{color:#58a6ff}
.tim{border-top-color:#3fb950}.tim h2{color:#3fb950}
.str{border-top-color:#f78166}.str h2{color:#f78166}
.syn{border-top-color:#d2a8ff}.syn h2{color:#d2a8ff}
.aud{border-top-color:#ffa657}.aud h2{color:#ffa657}
.pkt{border-top-color:#79c0ff}.pkt h2{color:#79c0ff}
.row{display:flex;justify-content:space-between;padding:.18rem 0;border-bottom:1px solid #21262d;font-size:.82rem}
.row:last-child{border-bottom:none}.val{font-weight:bold}
.ok{color:#3fb950}.warn{color:#e3b341}.err{color:#f85149}
.bar-bg{background:#21262d;border-radius:3px;height:5px;margin-top:.5rem}
.bar-fg{height:5px;border-radius:3px;transition:width .4s}
.cfg{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:.9rem;margin-bottom:.8rem;border-top:3px solid #e3b341}
.cfg h2{color:#e3b341;font-size:.75rem;text-transform:uppercase;letter-spacing:.12em;margin-bottom:.7rem}
label{font-size:.82rem;margin-right:1.2rem}
input[type=number]{background:#0d1117;color:#c9d1d9;border:1px solid #30363d;border-radius:4px;padding:.25rem .4rem;width:75px;margin:0 .3rem}
.btn{border:none;border-radius:5px;padding:.35rem .8rem;cursor:pointer;font-size:.82rem;margin-right:.4rem;color:#fff}
.apply{background:#238636}.apply:hover{background:#2ea043}
.ota{background:#1f6feb}.ota:hover{background:#388bfd}
#ts{font-size:.72rem;color:#484f58;margin-top:.5rem}
</style></head>
<body>
<h1>&#x1F50A; SyncStreamer &nbsp;<span id="ip" style="font-size:.65em;color:#484f58"></span></h1>
<div class="grid">
  <div class="card net"><h2>Network</h2><div id="d_net"></div></div>
  <div class="card tim"><h2>Time / NTP</h2><div id="d_time"></div></div>
  <div class="card str"><h2>Stream</h2><div id="d_str"></div></div>
  <div class="card syn"><h2>Sync</h2><div id="d_syn"></div></div>
  <div class="card aud"><h2>Audio</h2><div id="d_aud"></div></div>
  <div class="card pkt"><h2>Packets</h2><div id="d_pkt"></div></div>
</div>
<div class="cfg">
  <h2>Configuration</h2>
  <form id="cfgform">
    <label>Startup fill<input type="number" name="startup_min_ms" id="c_stup" min="100" max="2000">ms</label>
    <label>Duck level<input type="number" name="duck_level_pct" id="c_duck" min="0" max="50">%</label>
    <button class="btn apply" type="submit">Apply</button>
    <button class="btn ota" type="button" onclick="location='/update'">&#x1F504; OTA Update</button>
  </form>
</div>
<div id="ts">Connecting...</div>
<script>
function row(l,v,c=''){return`<div class="row"><span>${l}</span><span class="val ${c}">${v}</span></div>`}
function bar(pct,col='#58a6ff'){return`<div class="bar-bg"><div class="bar-fg" style="width:${Math.min(Math.max(pct,0),100)}%;background:${col}"></div></div>`}
function cls(v,ok,warn){return v<=ok?'ok':v<=warn?'warn':'err'}
function update(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    document.getElementById('ip').textContent='['+d.wifi.ip+']';
    document.getElementById('d_net').innerHTML=
      row('SSID',d.wifi.ssid)+
      row('RSSI',d.wifi.rssi+' dBm',cls(-d.wifi.rssi,70,85))+
      row('IP',d.wifi.ip);
    document.getElementById('d_time').innerHTML=
      row('NTP synced',d.ntp.synced?'YES':'NO',d.ntp.synced?'ok':'err')+
      row('Last sync',d.ntp.last_sync_age_sec<4294967295?d.ntp.last_sync_age_sec+'s ago':'never',
          cls(d.ntp.last_sync_age_sec,60,300));
    const fill=d.stream.fill_ms;
    document.getElementById('d_str').innerHTML=
      row('State','<b>'+d.stream.state+'</b>')+
      row('Stream ID',d.stream.stream_id)+
      row('Buffer fill',fill+' ms')+bar(fill/120*100,'#f78166');
    const fe=Math.abs(d.sync.filtered_us);
    document.getElementById('d_syn').innerHTML=
      row('Phase error',d.sync.phase_us+' µs')+
      row('Filtered',d.sync.filtered_us+' µs',cls(fe,500,2000))+
      row('Slips +/&minus;',d.sync.slips_insert+' / '+d.sync.slips_drop);
    const dc=d.audio.ducked;
    document.getElementById('d_aud').innerHTML=
      row('Volume',d.audio.volume_pct+'%')+
      row('Duck state',dc?'DUCKED':'normal',dc?'warn':'')+
      row('Samples out',d.audio.samples_written.toLocaleString())+
      bar(d.audio.volume_pct,'#ffa657');
    document.getElementById('d_pkt').innerHTML=
      row('RX packets',d.packets.rx_ok)+
      row('Parse errors',d.packets.parse_err,d.packets.parse_err?'err':'')+
      row('JB push ok',d.packets.jb_push_ok)+
      row('JB late drops',d.packets.jb_drop_late,d.packets.jb_drop_late?'warn':'')+
      row('Underruns',d.packets.sched_underrun,d.packets.sched_underrun?'warn':'');
    document.getElementById('c_stup').value=d.config.startup_min_ms;
    document.getElementById('c_duck').value=d.config.duck_level_pct;
    document.getElementById('ts').textContent='Updated: '+new Date().toLocaleTimeString();
  }).catch(()=>{document.getElementById('ts').textContent='&#x26A0; Connection lost';});
}
document.getElementById('cfgform').addEventListener('submit',e=>{
  e.preventDefault();
  const fd=new FormData(e.target);
  fetch('/api/config',{method:'POST',body:new URLSearchParams(fd)}).then(()=>update());
});
update();setInterval(update,2000);
</script>
</body></html>
)rawhtml";

// ── Begin ─────────────────────────────────────────────────────────────────────
void StatusServer::begin(AsyncWebServer*   server,
                          NTPSync*          ntp,
                          JitterBuffer*     jbuf,
                          NetworkReceiver*  net,
                          AudioOutput*      audio,
                          PlaybackScheduler* sched,
                          SyncController*   sync)
{
    _ntp   = ntp;
    _jbuf  = jbuf;
    _net   = net;
    _audio = audio;
    _sched = sched;
    _sync  = sync;

    server->on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", PAGE_HTML);
    });

    server->on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
        _handleStatus(req);
    });

    server->on("/api/config", HTTP_POST, [this](AsyncWebServerRequest* req) {
        _handleConfig(req);
    });

    log_i("StatusServer: routes registered");
}

// ── /api/status ───────────────────────────────────────────────────────────────
void StatusServer::_handleStatus(AsyncWebServerRequest* req)
{
    const SyncStatus& ss = _sync->getStatus();

    const char* stateStr = "FILLING";
    switch (_sched->getState()) {
        case SchedulerState::PLAYING:   stateStr = "PLAYING";   break;
        case SchedulerState::RESETTING: stateStr = "RESETTING"; break;
        default: break;
    }

    char buf[768];
    snprintf(buf, sizeof(buf),
        "{"
        "\"wifi\":{\"rssi\":%d,\"ip\":\"%s\",\"ssid\":\"%s\"},"
        "\"ntp\":{\"synced\":%s,\"last_sync_age_sec\":%u},"
        "\"stream\":{\"state\":\"%s\",\"stream_id\":%u,\"fill_ms\":%lld},"
        "\"sync\":{\"phase_us\":%lld,\"filtered_us\":%lld,"
                  "\"slips_insert\":%d,\"slips_drop\":%d},"
        "\"audio\":{\"volume_pct\":%d,\"ducked\":%s,\"samples_written\":%u},"
        "\"packets\":{\"rx_ok\":%u,\"parse_err\":%u,\"push_fail\":%u,"
                     "\"jb_push_ok\":%u,\"jb_drop_late\":%u,"
                     "\"jb_drop_dup\":%u,\"jb_drop_overflow\":%u,"
                     "\"jb_underrun\":%u,\"sched_underrun\":%u},"
        "\"config\":{\"startup_min_ms\":%u,\"duck_level_pct\":%d}"
        "}",
        WiFi.RSSI(),
        WiFi.localIP().toString().c_str(),
        WiFi.SSID().c_str(),
        _ntp->isSynced() ? "true" : "false",
        _ntp->getLastSyncAgeSec(),
        stateStr,
        _sched->getCurrentStreamId(),
        _jbuf->getFillLevelMicros() / 1000LL,
        ss.phaseErrorUs,
        ss.filteredErrorUs,
        ss.totalSlipsInserted,
        ss.totalSlipsDropped,
        _audio->getCurrentVolumePercent(),
        _audio->isDucked() ? "true" : "false",
        _audio->getSamplesWritten(),
        _net->statPacketsReceived,
        _net->statParseErrors,
        _net->statPushFailed,
        _jbuf->statPushOk,
        _jbuf->statDropLate,
        _jbuf->statDropDuplicate,
        _jbuf->statDropOverflow,
        _jbuf->statUnderrun,
        _sched->statUnderruns,
        _sched->getStartupMinMs(),
        _audio->getDuckLevelPercent()
    );

    req->send(200, "application/json", buf);
}

// ── /api/config ───────────────────────────────────────────────────────────────
void StatusServer::_handleConfig(AsyncWebServerRequest* req)
{
    if (req->hasParam("startup_min_ms", true)) {
        uint32_t v = (uint32_t)req->getParam("startup_min_ms", true)->value().toInt();
        if (v >= 100 && v <= 2000) {
            _sched->setStartupMinMs(v);
            log_i("StatusServer: startup_min_ms set to %u", v);
        }
    }
    if (req->hasParam("duck_level_pct", true)) {
        int v = req->getParam("duck_level_pct", true)->value().toInt();
        if (v >= 0 && v <= 50) {
            _audio->setDuckLevelPercent(v);
            log_i("StatusServer: duck_level_pct set to %d", v);
        }
    }
    req->send(200, "text/plain", "OK");
}
