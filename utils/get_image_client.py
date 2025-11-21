import urllib.request


def get_image(ip="localhost", port=8000):
    url = f"http://{ip}:{port}/"
    try:
        with urllib.request.urlopen(url) as response:
            image_data = response.read()
            with open("captured_image.jpg", "wb") as f:
                f.write(image_data)
            print("Image saved as captured_image.jpg")
    except Exception as e:
        print(f"Error fetching image: {e}")
