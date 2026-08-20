#ifndef PORTAL_HTML_H
#define PORTAL_HTML_H

#include <Arduino.h>

// Single-page config portal, served straight from flash with send_P().
// Everything is inline - the AP has no internet, so an external stylesheet or
// font would just hang the page. Light theme on purpose: this gets read on a
// phone in direct sunlight.
static const char PORTAL_INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AWS Setup</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font:16px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
       background:#eef1f5;color:#16202c;padding:16px;max-width:560px;margin:0 auto}
  header{margin-bottom:20px}
  h1{font-size:20px;letter-spacing:-.01em}
  .sub{font:600 14px ui-monospace,Menlo,Consolas,monospace;color:#5a6b7d;margin-top:2px}
  .card{background:#fff;border:1px solid #d5dde6;border-radius:12px;
        padding:16px;margin-bottom:14px}
  .card.primary{border:2px solid #1668d6;box-shadow:0 2px 10px rgba(22,104,214,.12)}
  h2{font-size:13px;text-transform:uppercase;letter-spacing:.06em;
     color:#5a6b7d;margin-bottom:12px}
  .card.primary h2{color:#1668d6}
  label{display:block;font-size:14px;font-weight:600;margin:12px 0 5px}
  label:first-of-type{margin-top:0}
  input{width:100%;padding:12px;font-size:17px;border:1px solid #b9c5d2;
        border-radius:8px;background:#fff;color:#16202c;font-family:inherit}
  input:focus{outline:2px solid #1668d6;outline-offset:-1px;border-color:#1668d6}
  .bigrow{display:flex;align-items:center;gap:12px}
  .bigrow input{font-size:34px;font-weight:700;text-align:center;padding:14px;
                letter-spacing:-.02em}
  .unit{font-size:17px;font-weight:600;color:#5a6b7d;white-space:nowrap}
  .hint{font-size:13px;color:#5a6b7d;margin-top:8px}
  button{width:100%;padding:13px;font-size:16px;font-weight:600;
         border:0;border-radius:8px;cursor:pointer;margin-top:12px;
         font-family:inherit;background:#1668d6;color:#fff}
  button:active{transform:translateY(1px)}
  button.ghost{background:#fff;color:#16202c;border:1px solid #b9c5d2}
  button.warn{background:#fff;color:#b3261e;border:1px solid #e5b4b0}
  dl{display:grid;grid-template-columns:auto 1fr;gap:7px 14px;font-size:14px}
  dt{color:#5a6b7d}
  dd{text-align:right;font-weight:600;
     font-family:ui-monospace,Menlo,Consolas,monospace;overflow-wrap:anywhere}
  dd.bad{color:#b3261e}
  dd.good{color:#146c43}
  #toast{position:fixed;left:50%;transform:translateX(-50%);bottom:-90px;
         background:#16202c;color:#fff;padding:12px 20px;border-radius:8px;
         font-size:14px;font-weight:600;transition:bottom .25s;
         max-width:90vw;text-align:center}
  #toast.show{bottom:20px}
  #toast.err{background:#b3261e}
</style>
</head>
<body>

<header>
  <h1>Automatic Weather Station</h1>
  <div class="sub" id="h-station">connecting...</div>
</header>

<section class="card primary">
  <h2>Update interval</h2>
  <form id="f-interval">
    <div class="bigrow">
      <input type="number" id="interval" min="1" max="60" step="1" required>
      <span class="unit">minutes</span>
    </div>
    <p class="hint">How often the station wakes to measure, log and transmit.
       Between bursts it sleeps to save battery.</p>
    <button type="submit">Save interval</button>
  </form>
</section>

<section class="card">
  <h2>Station</h2>
  <form id="f-station">
    <label for="station">Station code</label>
    <input type="text" id="station" maxlength="20">
    <label for="clock">Set clock (EAT)</label>
    <input type="text" id="clock" placeholder="YYYY-MM-DD HH:MM:SS">
    <p class="hint">Leave the clock blank to keep the current time. It is set
       automatically from the internet whenever WiFi is available.</p>
    <button type="submit">Save station settings</button>
  </form>
</section>

<section class="card">
  <h2>Status</h2>
  <dl>
    <dt>Time</dt><dd id="s-time">-</dd>
    <dt>Clock source</dt><dd id="s-clock">-</dd>
    <dt>Next reading</dt><dd id="s-next">-</dd>
    <dt>WiFi</dt><dd id="s-wifi">-</dd>
    <dt>Queued uploads</dt><dd id="s-queue">-</dd>
    <dt>Air temp</dt><dd id="s-temp">-</dd>
    <dt>Humidity</dt><dd id="s-hum">-</dd>
    <dt>Wind avg / gust</dt><dd id="s-wind">-</dd>
    <dt>Rain</dt><dd id="s-rain">-</dd>
    <dt>Battery</dt><dd id="s-batt">-</dd>
    <dt>Solar</dt><dd id="s-solar">-</dd>
    <dt>Uptime</dt><dd id="s-up">-</dd>
  </dl>
</section>

<section class="card">
  <h2>More</h2>
  <button class="ghost" id="b-wifi">Configure WiFi network</button>
  <button class="ghost" id="b-upload">Upload queued readings now</button>
  <button class="warn" id="b-restart">Restart station</button>
  <button id="b-done">Done - sleep now</button>
</section>

<div id="toast"></div>

<script>
var $ = function(id){ return document.getElementById(id); };
var touched = {};
var toastTimer;

function toast(msg, isErr){
  var t = $('toast');
  t.textContent = msg;
  t.className = 'show' + (isErr ? ' err' : '');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(function(){ t.className = ''; }, 2600);
}

// Don't let a status refresh overwrite a field the user is editing.
function mark(id){
  $(id).addEventListener('input', function(){ touched[id] = true; });
}
mark('interval'); mark('station'); mark('clock');

function fmtDur(s){
  if (s < 0) return 'now';
  if (s < 60) return s + 's';
  var m = Math.floor(s / 60), h = Math.floor(m / 60);
  if (h > 0) return h + 'h ' + (m % 60) + 'm';
  return m + 'm ' + (s % 60) + 's';
}

function set(id, txt, cls){
  var el = $(id);
  el.textContent = txt;
  el.className = cls || '';
}

function refresh(){
  fetch('/api/status').then(function(r){ return r.json(); }).then(function(d){
    $('h-station').textContent = d.station;
    if (!touched['interval']) $('interval').value = d.intervalMin;
    if (!touched['station'])  $('station').value  = d.station;

    set('s-time', d.time, d.clockValid ? '' : 'bad');
    set('s-clock', d.clockSource, d.clockValid ? 'good' : 'bad');
    set('s-next', fmtDur(d.nextCycleSec));
    if (d.wifi.connected) set('s-wifi', d.wifi.ssid + '  ' + d.wifi.ip, 'good');
    else set('s-wifi', 'not connected', 'bad');
    set('s-queue', d.queueDepth + (d.queueDepth === 1 ? ' record' : ' records'),
        d.queueDepth > 0 ? 'bad' : 'good');

    if (d.haveReading){
      set('s-temp',  d.temp.toFixed(1) + ' C');
      set('s-hum',   d.humidity.toFixed(0) + ' %');
      set('s-wind',  d.windMS.toFixed(1) + ' / ' + d.gustMS.toFixed(1) + ' m/s');
      set('s-rain',  d.rain.toFixed(1) + ' mm');
      set('s-batt',  d.battV.toFixed(2) + ' V  ' + d.battI.toFixed(0) + ' mA');
      set('s-solar', d.solarV.toFixed(2) + ' V  ' + d.solarI.toFixed(0) + ' mA');
    } else {
      ['s-temp','s-hum','s-wind','s-rain','s-batt','s-solar'].forEach(function(i){
        set(i, 'no reading yet');
      });
    }
    set('s-up', fmtDur(d.uptimeSec));
  }).catch(function(){ /* mid-handoff or busy; the next tick retries */ });
}

function post(path, body, okMsg){
  return fetch(path, {
    method: 'POST',
    headers: {'Content-Type':'application/x-www-form-urlencoded'},
    body: body || ''
  }).then(function(r){ return r.json(); }).then(function(d){
    toast(d.message || okMsg, !d.ok);
    if (d.ok) { touched = {}; refresh(); }
    return d;
  }).catch(function(){ toast('Station did not respond', true); });
}

$('f-interval').addEventListener('submit', function(e){
  e.preventDefault();
  post('/api/settings', 'interval=' + encodeURIComponent($('interval').value),
       'Interval saved');
});

$('f-station').addEventListener('submit', function(e){
  e.preventDefault();
  var b = 'station=' + encodeURIComponent($('station').value) +
          '&clock=' + encodeURIComponent($('clock').value);
  post('/api/settings', b, 'Saved').then(function(){ $('clock').value = ''; });
});

$('b-upload').addEventListener('click', function(){
  toast('Uploading...');
  post('/api/upload', '', 'Upload finished');
});

$('b-wifi').addEventListener('click', function(){
  if (!confirm('The setup network will restart so you can pick a WiFi network. '
             + 'Rejoin AWS-Config afterwards. Continue?')) return;
  post('/api/wifi', '', 'Switching to WiFi setup...');
});

$('b-restart').addEventListener('click', function(){
  if (!confirm('Restart the station now?')) return;
  post('/api/restart', '', 'Restarting...');
});

$('b-done').addEventListener('click', function(){
  post('/api/done', '', 'Closing portal - station going to sleep');
});

refresh();
setInterval(refresh, 3000);
</script>
</body>
</html>
)HTMLPAGE";

#endif
