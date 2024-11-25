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
            amovel,
            wait,
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            DR_FC_MOD_REL,
            get_current_posx,
            DR_TOOL,
            DR_QSTOP,
            drl_script_stop
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
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
    down = posx(0, 0, -112, 0, 0, 0)
    down_s = posx(0, 0, -48, 0, 0, 0)
    down_m = posx(0, 0, -38, 0, 0, 0)
    down_l = posx(0, 0, -28, 0, 0, 0)
    up = posx(0, 0, 163, 0, 0, 0)
    sup = posx(0,0,15,0,0,0)
    sdown = posx(0,0,-61,0,0,0)
    ssdown = posx(0,0,-33,0,0,0)
    s_cnt=0
    m_cnt=0
    l_cnt=0
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
    def cgrip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def pick_and_place(src, dest, counters):
        """Performs pick-and-place from src to dest."""
        movel(src, vel=200, acc=200)
        cgrip()
        movel(down, vel=200, acc=200, mod=DR_MV_MOD_REL)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        amovel(sdown, vel=10, acc=10, mod=DR_MV_MOD_REL)
        while rclpy.ok():
            if check_force_condition(axis=DR_AXIS_Z, min=30, ref=DR_TOOL) == 0: #-1이 거짓, 0이 참
                drl_script_stop(stop_mode = DR_QSTOP)
                height = get_current_posx()
                break
        release_compliance_ctrl()
        movel(sup, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        release()
        wait(0.3)
        movel(ssdown, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        grip()
        movel(up, vel=200, acc=200, mod=DR_MV_MOD_REL)
        print(height[0][2])
        # 작은거 57 중간 67 큰거 77
        if (50 < height[0][2] < 60):
            movel(destinations[0 + counters[0]], vel=200, acc=200)
            movel(down, vel=200, acc=200, mod=DR_MV_MOD_REL)
            movel(down_s, vel=20, acc=20, mod=DR_MV_MOD_REL)
            counters[0]+=1
            print("작")
        elif (60 < height[0][2] < 70):
            movel(destinations[3+counters[1]], vel=200, acc=200)
            movel(down, vel=200, acc=200, mod=DR_MV_MOD_REL)
            movel(down_m, vel=20, acc=20, mod=DR_MV_MOD_REL)
            counters[1]+=1
            print("중")
        elif (70 < height[0][2] < 80):
            movel(destinations[6+counters[2]], vel=200, acc=200)
            movel(down, vel=200, acc=200, mod=DR_MV_MOD_REL)
            movel(down_l, vel=20, acc=20, mod=DR_MV_MOD_REL)
            counters[2]+=1
            print("큼")
        else:
            print("규격 외")
        release()
        movel(up, vel=200, acc=200, mod=DR_MV_MOD_REL)



    

    def back_place(src, dest):  
        down = posx(0, 0, -163, 0, 0, 0)
        up = posx(0, 0, 163, 0, 0, 0)
        """Performs pick-and-place from src to dest."""
        movel(src, vel=VELOCITY, acc=ACC)
        movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
        cgrip()
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        movel(dest, vel=VELOCITY, acc=ACC)
        movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
        release()
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        release()
        movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)

    release()
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
    counters = [s_cnt, m_cnt, l_cnt]
    for src, dest in zip(positions, destinations):
        pick_and_place(src, dest, counters)
      # Perform place-back operation
    for dest, src in zip(destinations, positions):
        back_place(dest, src)


if __name__ == "__main__":
    main() 