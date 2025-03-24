from dynamixel_helper import DynamixelCtrlU2D2, OperatingModes, Motors
import argparse


def main():
    parser = argparse.ArgumentParser(description="Serial Data Logger")
    parser.add_argument(
        "-m", "--motor", type=str, default="/dev/ttyUSB1", help="Motor port"
    )
    parser.add_argument("-i", "--motor-id", type=int, default=4, help="Motor id")
    args = parser.parse_args()

    dyn = DynamixelCtrlU2D2(
        port=args.motor,
        operating_mode=OperatingModes.EXTENDED_POSITION,
        motor=Motors.X_SERIES,
    )
    dyn.add_motor(args.motor_id, center=True)
    step = 1
    origin = dyn.get_goal(id=args.motor_id)
    while True:
        try:
            user_input = input("> ").strip()

            if user_input == "":
                origin += step
                dyn.set_goal(id=args.motor_id, pos=origin, block_thread=True)
                print(f"Moved to {origin}")
            elif user_input:
                value = int(user_input)
                if -10000 <= value <= 10000:
                    step = value
                    print(f"Step updated to {step}")
                else:
                    print("Enter a number between 0 and 100.")
            else:
                print("Invalid input. Enter a number or press Enter.")

        except KeyboardInterrupt:
            print("\nExiting...")
            break
    # df.to_csv("motor_log.csv", index=False)


if __name__ == "__main__":
    main()
