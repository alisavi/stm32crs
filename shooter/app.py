#!/bin/python3
from flask import Flask, render_template, send_file
import random
import json

x = 1920 // 2
y = 1080 // 2

app = Flask(__name__)

@app.route('/')
def index():
    return render_template("responce.html")

@app.route("/js/responce.js")
def send_script():
    return send_file("js/responce.js")

@app.route("/img/<image>")
def get_image(image):
    return send_file(f"img/{image}")

@app.route('/get_data')
def data():
    global x, y
    #x += random.randint(-32, 32)
    #y += random.randint(-32, 32)
    x = (x+20) % 1920
    y = (y+3) % 720
    #x = max(0, x)
    #x = min(x, 1920 - 1)
    #y = max(0, y)
    #y = min(y, 720 - 1)
    a = {"x": x, "y": y, "press": 1, "hold": 0} # if random.randint(0, 100000) >= 99999 else 0
    return json.dumps(a)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)