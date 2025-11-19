import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer


class CameraHandler(BaseHTTPRequestHandler):
    def get_image(self):
        # Capture image from camera
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        cap.release()
        if ret:
            return frame
        else:
            return None

    def do_GET(self):
        if self.path == "/":
            frame = self.get_image()
            if frame is not None:
                # Encode frame as JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                _, encoded_img = cv2.imencode(".jpg", frame, encode_param)

                # Send response
                self.send_response(200)
                self.send_header("Content-type", "image/jpeg")
                self.send_header("Content-length", str(len(encoded_img)))
                self.end_headers()
                self.wfile.write(encoded_img.tobytes())
            else:
                self.send_error(500, "Failed to capture image")
        else:
            self.send_error(404)


def run_server(port=8000):
    server_address = ("0.0.0.0", port)
    httpd = HTTPServer(server_address, CameraHandler)
    print(f"Camera server running on port {port}")
    httpd.serve_forever()


if __name__ == "__main__":
    run_server()
