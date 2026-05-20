#pragma once
#include <stdint.h>
#include <ESPAsyncWebServer.h>
#include "NTPSync.h"
#include "JitterBuffer.h"
#include "NetworkReceiver.h"
#include "AudioOutput.h"
#include "PlaybackScheduler.h"
#include "SyncController.h"

// ============================================================
// StatusServer — async web status/config UI + ElegantOTA entry
//
// Routes:
//   GET  /           colourful single-page status dashboard
//   GET  /api/status JSON snapshot (fetched every 2 s by page JS)
//   POST /api/config update runtime parameters
//   GET  /update     ElegantOTA firmware update (registered externally)
//
// Call begin() after AsyncWebServer is created; the server's
// begin() call is made in main.cpp after all routes are set up.
// ============================================================

class StatusServer {
public:
    void begin(AsyncWebServer*   server,
               NTPSync*          ntp,
               JitterBuffer*     jbuf,
               NetworkReceiver*  net,
               AudioOutput*      audio,
               PlaybackScheduler* sched,
               SyncController*   sync);

private:
    NTPSync*           _ntp   = nullptr;
    JitterBuffer*      _jbuf  = nullptr;
    NetworkReceiver*   _net   = nullptr;
    AudioOutput*       _audio = nullptr;
    PlaybackScheduler* _sched = nullptr;
    SyncController*    _sync  = nullptr;

    void _handleStatus(AsyncWebServerRequest* req);
    void _handleConfig(AsyncWebServerRequest* req);
};
