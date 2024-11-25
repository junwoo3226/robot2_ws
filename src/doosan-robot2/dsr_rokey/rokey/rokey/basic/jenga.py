import rclpy
import DR_init
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 200, 200
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
            amovel,
            wait,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            get_current_posx,
            set_desired_force,
            trans
        )
        from DSR_ROBOT2 import (
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_FC_MOD_REL,
            DR_TOOL,
            DR_QSTOP
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    def cgrip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def place(point):
        """Performs pick-and-place from src to dest."""
        movej(point, vel=VELOCITY, acc=ACC)
        movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
#########################################힘제어##################################
        print("컴플라이언스 온")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(axis=DR_AXIS_Z, min=9, ref=DR_TOOL) == 0: #-1이 거짓, 0이 참
                release_compliance_ctrl()
                movel(up, vel=200, acc=200)
                break
        release()
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
##########################################해제###################################

    def wait_block():
        movel(grip_place, vel=200, acc=200)
        wait(0.3)
        movel(grip_down1, vel=200, acc=200, mod=DR_MV_MOD_REL)
        movel(grip_down2, vel=20, acc=20, mod=DR_MV_MOD_REL)
        wait(0.3)
        while True:
            cgrip()
            wait(1)
            if(get_digital_input(1)):
                # wait(0.5)
                print("잡음")
                break

            release()
            wait(0.5)

    z = 340
    down = posx(0, 0, -z, 0, 0, 0)
    up = posx(0, 0, z, 0, 0, 0)

    grip_place = posx(508.77, 289.97, 250, 31.9, -180, -60)
    grip_down1 = posx(0, 0, -235, 0, 0, 0)
    grip_down2 = posx(0, 0, -8, 0, 0, 0)

    ######################################
    # 도착 위치 좌표값
    count = 0
    movej([0, 0, 66.8, 0, 110, 0], vel=VELOCITY, acc=ACC)
    release()
    while rclpy.ok():
        count += 1
        wait(0.3)
        wait_block()
        if count % 2 == 0:
            point = posj(0, 0, 67, 0, 110, 0)
            place(point)
        else:
            point = posj(0, 0, 67, 0, 110, -90)
            place(point)
        print(count)
        z -= 15
        down = posx(0, 0, -z, 0, 0, 0)
        up = posx(0, 0, z, 0, 0, 0)
        if count == 18:
            break
    movej([0, 3, 66.8, 0, 110, 0], vel=VELOCITY, acc=ACC)
    release()


if __name__ == "__main__":
    main()