// Captive-portal pages served by B1 (soft-AP "RESCUE-TEAM-NEARBY").
// Placeholder pages: the teammate may replace this file with the final design (same two names).
// No external resources: the phone has no internet while on this Wi-Fi.
#pragma once

static const char PAGE_FORM[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rescue team nearby</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#111;color:#eee}
h1{font-size:1.5em;color:#ff5a4f;margin:.2em 0}
p{line-height:1.4}
label{display:block;margin:14px 0 6px;font-weight:600}
input,select,textarea{width:100%;box-sizing:border-box;font-size:1.1em;padding:10px;border-radius:8px;border:1px solid #555;background:#222;color:#eee}
button{margin-top:20px;width:100%;font-size:1.3em;padding:16px;border:0;border-radius:10px;background:#e53935;color:#fff;font-weight:700}
small{color:#aaa}
</style></head><body>
<h1>&#x1F198; Rescue team nearby</h1>
<p>You are connected to a rescue relay. Send a short message and it will reach the rescue team.</p>
<form method="POST" action="/sos">
<label for="p">How many people are with you?</label>
<input id="p" name="p" type="number" min="1" max="99" value="1" inputmode="numeric">
<label for="inj">Is anyone injured?</label>
<select id="inj" name="inj"><option value="no">No</option><option value="yes">Yes</option></select>
<label for="msg">What do you need? Where are you?</label>
<textarea id="msg" name="msg" rows="3" maxlength="70" placeholder="e.g. Leg trapped under beam, 2nd floor"></textarea>
<small>Keep it short (70 characters).</small>
<button type="submit">SEND SOS</button>
</form></body></html>)HTML";

static const char PAGE_SENT[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Message sent</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#111;color:#eee;text-align:center}
h1{color:#4caf50;font-size:1.6em}
a{color:#8ab4f8;font-size:1.1em}
</style></head><body>
<h1>&#x2705; Message sent</h1>
<p>Your message is on its way to the rescue team.</p>
<p>Stay where you are if it is safe. Keep this phone on.</p>
<p><a href="/">Send another message</a></p>
</body></html>)HTML";
