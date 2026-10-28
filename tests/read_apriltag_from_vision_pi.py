import json
import socket
import time


VISION_PI_IP = "10.255.2.97"
VISION_PI_PORT = 5005


def main():
    print(f"Connecting to vision Pi at {VISION_PI_IP}:{VISION_PI_PORT}...")

    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((VISION_PI_IP, VISION_PI_PORT))
            file = sock.makefile("r")

            print("Connected to vision Pi.")
            print("Waiting for AprilTag data...")
            print()

            for line in file:
                data = json.loads(line)

                if not data.get("seen", False):
                    print("No tag seen")
                    continue

                print(
                    f"ID {data['tag_id']} | "
                    f"pixel=({data['pixel_x']:.1f}, {data['pixel_y']:.1f}) | "
                    f"pixel_error=({data['pixel_error_x']:.1f}, {data['pixel_error_y']:.1f}) | "
                    f"camera XYZ=({data['x_m']:.4f}, {data['y_m']:.4f}, {data['z_m']:.4f}) m | "
                    f"straight={data['straight_distance_m']:.4f} m | "
                    f"tag-plane lateral={data['tag_plane_lateral_m']:.4f} m | "
                    f"margin={data['margin']:.1f}"
                )

        except ConnectionRefusedError:
            print("Connection refused. Is apriltag_server.py running on the vision Pi?")
        except TimeoutError:
            print("Connection timed out.")
        except OSError as e:
            print(f"Connection error: {e}")
        except KeyboardInterrupt:
            print("Stopping receiver.")
            break

        print("Retrying in 2 seconds...")
        time.sleep(2)


if __name__ == "__main__":
    main()
