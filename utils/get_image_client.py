import urllib.request


def get_image(ip="localhost", port=8001):
    url = f"http://{ip}:{port}/"
    try:
        with urllib.request.urlopen(url) as response:
            image_data = response.read()
            return image_data
    except Exception as e:
        print(f"Error fetching image: {e}")


if __name__ == "__main__":
    import cv2
    import numpy as np
    # keep getting image and show
    while True:
        image = get_image(ip="sparks-beta.local")  # Default to localhost and port 8000
        if image:
            # Convert byte data to numpy array
            nparr = np.frombuffer(image, np.uint8)
            img_np = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            # resize for display
            img_np = cv2.resize(img_np, (1280, 720))
            cv2.imshow("Image", img_np)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        