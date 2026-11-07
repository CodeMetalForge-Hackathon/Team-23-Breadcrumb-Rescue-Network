// Captive-portal pages served by B1 (soft-AP "RESCUE-TEAM-NEARBY").
// Placeholder pages: the teammate may replace this file with the final design (same two names).
// PAGE_SENT: "%ID%" is replaced with the SOS id; the page polls /status?id= until the rescuers ACK.
// No external resources: the phone has no internet while on this Wi-Fi.
#pragma once

static const char PAGE_FORM[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rescue team nearby</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;padding:14px;background:#111;color:#fff;font-size:18px}
h1{font-size:1.5em;color:#ff5a4f;margin:.1em 0 .2em}
p{margin:.3em 0 .8em;color:#ccc}
.q{margin:18px 0 6px;font-weight:700;font-size:1.05em}
.g{display:flex;flex-wrap:wrap;gap:8px}
.g input{position:absolute;opacity:0;width:1px;height:1px}
.g label{flex:1 1 30%;min-width:88px;text-align:center;padding:14px 6px;border:2px solid #555;border-radius:12px;background:#222;font-size:1.05em;user-select:none}
.g input:checked+label{background:#ffd54f;color:#000;border-color:#ffd54f;font-weight:800}
textarea{width:100%;box-sizing:border-box;font-size:1em;padding:10px;border-radius:10px;border:2px solid #555;background:#222;color:#fff}
small{color:#aaa}
.send{display:block;width:100%;margin:10px 0;font-size:1.4em;padding:18px;border:0;border-radius:14px;background:#e53935;color:#fff;font-weight:800}
.top{margin-top:4px}
</style></head><body>
<h1>&#x1F198; Rescue team nearby</h1>
<p>Tap what fits, then SEND. Skip anything you don't know.</p>
<form method="POST" action="/sos">
<button class="send top" type="submit">SEND SOS NOW</button>

<div class="q">How many people are with you?</div>
<div class="g">
<input type="radio" name="p" id="p1" value="1"><label for="p1">Just me</label>
<input type="radio" name="p" id="p2" value="2"><label for="p2">2</label>
<input type="radio" name="p" id="p3" value="3"><label for="p3">3</label>
<input type="radio" name="p" id="p4" value="4"><label for="p4">4 or more</label>
</div>

<div class="q">Is anyone hurt?</div>
<div class="g">
<input type="radio" name="inj" id="i0" value="0"><label for="i0">No one hurt</label>
<input type="radio" name="inj" id="i1" value="1"><label for="i1">Hurt, can move</label>
<input type="radio" name="inj" id="i2" value="2"><label for="i2">Trapped / can't move</label>
</div>

<div class="q">Anything urgent? (tap all that apply)</div>
<div class="g">
<input type="checkbox" name="u" id="u1" value="bleeding"><label for="u1">Heavy bleeding</label>
<input type="checkbox" name="u" id="u2" value="breathing"><label for="u2">Hard to breathe</label>
<input type="checkbox" name="u" id="u3" value="unconscious"><label for="u3">Someone unconscious</label>
</div>

<div class="q">Danger near you?</div>
<div class="g">
<input type="checkbox" name="h" id="h1" value="smoke"><label for="h1">Smoke</label>
<input type="checkbox" name="h" id="h2" value="fire"><label for="h2">Fire</label>
<input type="checkbox" name="h" id="h3" value="water"><label for="h3">Water</label>
<input type="checkbox" name="h" id="h4" value="gas"><label for="h4">Gas smell</label>
<input type="checkbox" name="h" id="h5" value="collapse"><label for="h5">Ceiling unstable</label>
</div>

<div class="q">Where are you? (optional)</div>
<textarea name="msg" rows="2" maxlength="40" placeholder="e.g. 2nd floor, under stairs"></textarea>
<small>Can't type? Tap &#x1F3A4; on the keyboard and speak.</small>

<button class="send" type="submit">SEND SOS</button>
</form></body></html>)HTML";

static const char PAGE_SENT[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Message sent</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#111;color:#eee;text-align:center}
h1{color:#4caf50;font-size:1.6em}
a{color:#8ab4f8;font-size:1.1em}
.wait{color:#aaa;font-style:italic}
body.ok{background:#0d3b1e}
body.ok #s{font-size:1.4em;font-weight:700;color:#fff}
</style></head><body>
<h1 id="h">&#x2705; Message sent</h1>
<p id="s">Your message is on its way to the rescue team.</p>
<p id="w" class="wait">Waiting for the rescuers to confirm&hellip;</p>
<p>Stay where you are if it is safe. Keep this phone on.</p>
<p><a href="/">Send another message</a></p>
<script>
var id="%ID%";
function poll(){
  fetch("/status?id="+id,{cache:"no-store"}).then(function(r){return r.json()}).then(function(d){
    if(d.ack){
      document.getElementById("h").innerHTML="&#x1F691; Received by rescuers";
      document.getElementById("s").textContent=d.reply||"Help is on the way";
      document.getElementById("w").style.display="none";
      document.body.className="ok";
    } else setTimeout(poll,2000);
  }).catch(function(){setTimeout(poll,3000)});
}
setTimeout(poll,2000);
</script>
</body></html>)HTML";
