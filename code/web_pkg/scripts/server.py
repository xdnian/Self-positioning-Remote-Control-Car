#!/usr/bin/env python

from flask import Flask, Response, request, jsonify
import rospy
from std_msgs.msg import String

app = Flask(__name__, static_folder='static', static_url_path='')

@app.route('/test')
def test():
    return "Hello World!"

def cam_talker(data):
    if not rospy.is_shutdown():
        # rospy.loginfo(data)
        cam_act_pub.publish(data)

@app.route('/camera_rotate', methods=['POST'])
def camera_rotate():
    data = request.form['action']
    # print data
    try:
        cam_talker(data)
    except rospy.ROSInterruptException:
        pass
    return 'success'

@app.route('/car_info', methods=['GET'])
def car_info():
    global Coord, Coll
    return_data = {'coordinate': Coord, 'collision': Coll}
    print return_data
    return jsonify(return_data)

def updateCoord(data):
    global Coord
    if not rospy.is_shutdown():
        data = map(int, data.data.split())
        Coord = data

def updateColl(data):
    global Coll
    if not rospy.is_shutdown():
        data = map(int, data.data.split())
        Coll = data

if __name__ == '__main__':
    global Coord, Coll

    Coord = None
    Coll = None

    cam_act_pub = rospy.Publisher('cam_rotate', String, queue_size=10)
    coord_sub = rospy.Subscriber("/coord", String, updateCoord)
    coll_sub = rospy.Subscriber("/sonar", String, updateColl)

    rospy.init_node('image_bridge', anonymous=True)

    app.run(host='0.0.0.0', port=2333, debug=True)
