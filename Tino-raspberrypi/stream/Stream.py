import io
from picamera2 import Picamera2  
from flask import Flask, Response

app = Flask(__name__)

def generate_frames():
    picam2 = Picamera2() 
    camera_config = picam2.create_video_configuration({"size": (800,600)})
    picam2.configure(camera_config)
    picam2.start()
    
    picam2.set_controls({"FrameRate": 20})
    
    stream = io.BytesIO()
    
    while True:
        picam2.capture_file(stream, format="jpeg")
        stream.seek(0)
        frame = stream.read()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        stream.seek(0)
        stream.truncate()

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)