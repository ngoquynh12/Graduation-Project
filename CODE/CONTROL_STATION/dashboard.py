from flask import Flask, render_template_string
import json
import time

app = Flask(__name__)

JSON_FILE = "/tmp/lora_latest.json"
MARKER_FILE = "/tmp/marker_latest.json"

HTML = """
<!doctype html>
<html>
<head>
<meta http-equiv="refresh" content="1">
<title>Rescue Robot Dashboard</title>

<style>
body{
    margin:0;
    background:#0b1020;
    color:white;
    font-family:Segoe UI,Arial;
}

.header{
    padding:22px 35px;
    background:#111936;
    font-size:30px;
    font-weight:bold;
    color:#00ff9d;
    box-shadow:0 2px 10px #000;
}

.grid{
    display:grid;
    grid-template-columns:repeat(3,1fr);
    gap:22px;
    padding:30px;
}

.card{
    background:#151f3d;
    border-radius:18px;
    padding:22px;
    box-shadow:0 0 18px #0008;
    min-height:190px;
}

.card h2{
    margin-top:0;
    color:#00d9ff;
}

.value{
    font-size:34px;
    font-weight:bold;
    color:#00ff9d;
}

.bad{
    color:#ff4d6d;
    font-weight:bold;
}

.warn{
    color:#ffd166;
    font-weight:bold;
}

.small{
    color:#aab3d1;
    font-size:16px;
    margin-top:8px;
}

.marker-row{
    margin-bottom:14px;
}

.ok{
    color:#00ff9d;
    font-weight:bold;
}
</style>
</head>

<body>

<div class="header">🚨 Rescue Robot Monitoring Dashboard</div>

<div class="grid">

    <div class="card">
        <h2>📷 Camera</h2>
        <div>Person: <span class="{{ 'bad' if d.cp else '' }}">{{ d.cp }}</span></div>
        <div>Dog: {{ d.cd }}</div>
        <div>Cat: {{ d.cc }}</div>
    </div>

    <div class="card">
        <h2>🎤 Microphone</h2>
        <div>Person: <span class="{{ 'bad' if d.mp else '' }}">{{ d.mp }}</span></div>
        <div>Dog: {{ d.md }}</div>
    </div>

    <div class="card">
        <h2>📡 Radar</h2>
        <div class="value {{ 'bad' if d.r else '' }}">
            {{ 'DETECTED' if d.r else 'CLEAR' }}
        </div>
        <div class="small">Movement: {{ d.d }} m</div>
    </div>

    <div class="card">
        <h2>📈 Vibration</h2>
        <div class="value {{ 'warn' if d.s == 'VIBRATION' else '' }}">
            {{ d.s }}
        </div>
        <div class="small">MMI: {{ d.m }}</div>
        <div class="small">PGA: {{ d.p }}</div>
    </div>

    <div class="card">
        <h2>📍 NRF Markers</h2>

        {% for i in [1,2,3] %}
        {% set key = "marker_" ~ i %}
        {% set mk = marker.get(key, {}) %}
        {% set link = mk.get("link", "LOST") %}
        <div class="marker-row">
            <b>Marker {{ i }}:</b>
            <span class="{{ 'ok' if link == 'OK' else 'bad' }}">{{ link }}</span>
            <div class="small">
                Status: {{ mk.get("status", "NO DATA") }} |
                Battery: {{ mk.get("battery", 0.0) }}V |
                Count: {{ mk.get("counter", 0) }}
            </div>
        </div>
        {% endfor %}
    </div>

    <div class="card">
        <h2>⏱ Update</h2>
        <div class="value">{{ update_time }}</div>
        <div class="small">Auto refresh every 1 second</div>
    </div>

</div>

</body>
</html>
"""

class Obj:
    def __init__(self, data):
        self.cp = data.get("cp", 0)
        self.cd = data.get("cd", 0)
        self.cc = data.get("cc", 0)

        self.mp = data.get("mp", 0)
        self.md = data.get("md", 0)

        self.r = data.get("r", 0)
        self.d = data.get("d", 0.0)

        self.m = data.get("m", "I")
        self.p = data.get("p", 0.0)

        self.s = data.get("s", "NO DATA")

@app.route("/")
def home():
    try:
        with open(JSON_FILE, "r") as f:
            data = json.load(f)
    except:
        data = {}

    try:
        with open(MARKER_FILE, "r") as f:
            marker = json.load(f)
    except:
        marker = {}

    return render_template_string(
        HTML,
        d=Obj(data),
        marker=marker,
        update_time=time.strftime("%H:%M:%S")
    )

app.run(host="0.0.0.0", port=5000)