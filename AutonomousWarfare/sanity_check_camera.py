from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2(1)
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Capture an image
picam2.capture_file("test_image.jpg")

picam2.stop()