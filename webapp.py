#-------------------------------------------------------#
from flask import Flask, render_template,request
from flask_socketio import SocketIO, send
import eventlet
# eventlet.monkey_patch()


app = Flask(__name__)
app.config['SECRET_KEY'] = 'key'

socketio = SocketIO(app, async_mode='eventlet')


@socketio.on('message')
def handleMessage(msg):
    print('Message: ' + msg)
    send(msg, broadcast=True)


@app.route('/')
def hello_world():
    return render_template('index.html')


def webappRun():
    print("webapp_working")
    socketio.run(app, debug=False)


# if __name__ == '__main__':
#     webappRun()
#-------------------------------------------------------#