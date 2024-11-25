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
    
    # 첫 팔레트 위치 저장
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

    # 옮겨야하는 위치 저장
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
    # 각 이동할 위치 선언
    down = posx(0, 0, -112, 0, 0, 0)
    down_s = posx(0, 0, -48, 0, 0, 0)
    down_m = posx(0, 0, -38, 0, 0, 0)
    down_l = posx(0, 0, -28, 0, 0, 0)
    up = posx(0, 0, 163, 0, 0, 0)
    sup = posx(0,0,15,0,0,0)
    sdown = posx(0,0,-61,0,0,0)
    ssdown = posx(0,0,-33,0,0,0)
    
    # 스몰 미둠 라지 카운트 선언
    s_cnt=0
    m_cnt=0
    l_cnt=0


    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
    
    # 그립 놓기
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)
    
    # 그립 잡기
    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)
    
    # wait_digital_input(1)가 없어 블록없어도 그립되는 함수
    def cgrip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
    
    # 메인으로 실행 할 함수
    def pick_and_place(src, dest, counters):
        """Performs pick-and-place from src to dest."""
        # 첫번째 위치 이동
        movel(src, vel=VELOCITY, acc=ACC)
        # 그립 닫고
        cgrip()
        # 내려가기
        movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
        # 협착 방지 함수
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        # 비동기 이동
        amovel(sdown, vel=10, acc=10, mod=DR_MV_MOD_REL)
        # 내려갈때 힘이 느껴지면 멈추고 블록 높이값 가져오기
        while rclpy.ok():
            if check_force_condition(axis=DR_AXIS_Z, min=30, ref=DR_TOOL) == 0: 
                drl_script_stop(stop_mode = DR_QSTOP)
                height = get_current_posx()
                break
        # 협착 방지 취소
        release_compliance_ctrl()
        wait(2)
        movel(sup, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        release()
        wait(2)
        movel(ssdown, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        grip()
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        print(height[0][2])
        # 작은거 57 중간 67 큰거 77
        if (50 < height[0][2] < 60):
            movel(destinations[0 + counters[0]], vel=VELOCITY, acc=ACC)
            movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
            movel(down_s, vel=20, acc=20, mod=DR_MV_MOD_REL)
            counters[0]+=1
            print("작")
        elif (60 < height[0][2] < 70):
            movel(destinations[3+counters[1]], vel=VELOCITY, acc=ACC)
            movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
            movel(down_m, vel=20, acc=20, mod=DR_MV_MOD_REL)
            counters[1]+=1
            print("중")
        elif (70 < height[0][2] < 80):
            movel(destinations[6+counters[2]], vel=VELOCITY, acc=ACC)
            movel(down, vel=100, acc=100, mod=DR_MV_MOD_REL)
            movel(down_l, vel=20, acc=20, mod=DR_MV_MOD_REL)
            counters[2]+=1
            print("큼")
        else:
            print("규격 외")
        release()
        movel(up, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    
    # 그립 열기
    release()

    # 로봇 초기 위치
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
    
    # 3 X 3 모두 실행
    counters = [s_cnt, m_cnt, l_cnt]
    for src, dest in zip(positions, destinations):
        pick_and_place(src, dest, counters)


if __name__ == "__main__":
    main()