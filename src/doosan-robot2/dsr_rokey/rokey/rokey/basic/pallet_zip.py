import rclpy
import DR_init
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
ON, OFF = 1, 0
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            DR_MV_MOD_REL,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)
    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)
    def pick_and_place(src, dest):
        """Performs pick-and-place from src to dest."""
        movel(src, vel=VELOCITY, acc=ACC)
        movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
        grip()
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        movel(dest, vel=VELOCITY, acc=ACC)
        movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
        release()
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    # Define coordinates
    positions = [
        posx(502.04, -150.91, 200, 154.5, 179.02, 154.04),
        posx(450.80, -150.91, 200, 154.5, 179.02, 154.04),
        posx(399.55, -150.91, 200, 154.5, 179.02, 154.04),
        posx(502.04, -99.67, 200, 154.5, 179.02, 154.04),
        posx(450.80, -99.67, 200, 154.5, 179.02, 154.04),
        posx(399.55, -99.67, 200, 154.5, 179.02, 154.04),
        posx(502.04, -48.43, 200, 154.5, 179.02, 154.04),
        posx(450.80, -48.43, 200, 154.5, 179.02, 154.04),
        posx(399.55, -48.43, 200, 154.5, 179.02, 154.04),
    ]
    destinations = [
        posx(501.04, 0.00, 200, 154.5, 179.02, 154.04),
        posx(449.80, 0.00, 200, 154.5, 179.02, 154.04),
        posx(398.55, 0.00, 200, 154.5, 179.02, 154.04),
        posx(501.04, 50.00, 200, 154.5, 179.02, 154.04),
        posx(449.80, 50.00, 200, 154.5, 179.02, 154.04),
        posx(398.55, 50.00, 200, 154.5, 179.02, 154.04),
        posx(501.04, 100.00, 200, 154.5, 179.02, 154.04),
        posx(449.80, 100.00, 200, 154.5, 179.02, 154.04),
        posx(398.55, 100.00, 200, 154.5, 179.02, 154.04),
    ]
    down = posx(0, 0, -163, 0, 0, 0)
    up = posx(0, 0, 163, 0, 0, 0)
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    release()
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
    # Perform pick-and-place
    for src, dest in zip(positions, destinations):
        pick_and_place(src, dest)
    # Perform place-back operation
    for dest, src in zip(destinations, positions):
        pick_and_place(dest, src)
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
if __name__ == "__main__":
    main() 