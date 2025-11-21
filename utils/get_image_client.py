import urllib.request
import argparse


def main():
    parser = argparse.ArgumentParser(description="Fetch image from camera server")
    parser.add_argument(
        "--ip", default="localhost", help="IP address of the camera server"
    )
    parser.add_argument(
        "--port", type=int, default=8001, help="Port of the camera server"
    )
    args = parser.parse_args()

    url = f"http://{args.ip}:{args.port}/"
    try:
        with urllib.request.urlopen(url) as response:
            image_data = response.read()
            with open("captured_image.jpg", "wb") as f:
                f.write(image_data)
            print("Image saved as captured_image.jpg")
    except Exception as e:
        print(f"Error fetching image: {e}")


if __name__ == "__main__":
    main()
