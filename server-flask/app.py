from flask import Flask, jsonify, request
import subprocess
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

def communicate_with_robot(command):
    try:
        result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode != 0:
            return jsonify({"error": result.stderr}), 400
        return jsonify({"message": result.stdout}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/robot/identify/physical/<int:robotId>', methods=['POST'])
def identifyRobot(robotId):
    if request.method == 'POST':
        print("Identifying robot", robotId)
        command = f"ros2 topic pub /robot{robotId}_command std_msgs/String 'data: start' --once"
        return communicate_with_robot(command)


@app.route('/robot/start-mission/physical/<int:robotId>', methods=['POST'])
def startMission(robotId):
    if request.method == 'POST':
        print("Starting mission", robotId)
        command = f"ros2 topic pub /robot{robotId}_command std_msgs/String 'data: start' --once"
        return communicate_with_robot(command)

@app.route('/robot/stop-mission/physical/<int:robotId>', methods=['POST'])
def stopMission(robotId):
    if request.method == 'POST':
        print("Stopping mission", robotId)
        command = f"ros2 topic pub /robot{robotId}_command std_msgs/String 'data: stop' --once"
        return communicate_with_robot(command)


if __name__ == '__main__':
    print("Starting server")
    app.run(host='0.0.0.0', debug=True, port=5000)