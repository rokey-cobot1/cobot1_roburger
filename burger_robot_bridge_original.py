#!/usr/bin/env python3
"""
햄버거 제조 로봇 브릿지 (사용자 동작 시퀀스 그대로 사용)
- Firebase에서 주문 받기 (/burger_order)
- 사용자의 move_periodic.py 동작을 그대로 실행
- 상태를 Firebase로 전송 (/robot_status_update)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # [추가] 멀티스레드 실행기
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from sensor_msgs.msg import JointState
# 기존 import 아래에 추가하세요
from dsr_msgs2.srv import DrlPause, DrlResume, DrlStop, SetRobotControl, GetRobotState
import json
import time
import threading
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"#본인 TCP 이름 설정
ROBOT_TCP = "GripperDA_v1" #본인 그리퍼 이름 설정

class BurgerRobotBridge(Node):
    def __init__(self):
        super().__init__('burger_robot_bridge')
        
        self.get_logger().info('🍔🤖 햄버거 제조 로봇 브릿지 시작!')
        # [중요] 콜백 그룹 생성 (병렬 실행을 허용)
        self.callback_group = ReentrantCallbackGroup()

        # ==========================================================
        # [추가] 여기가 빠져있어서 에러가 났습니다. 서비스 클라이언트 생성
        # ==========================================================
        # 주의: '/dsr01/drl/drl_pause' 경로는 로봇 설정에 따라 다를 수 있으나 보통 표준입니다.
        self.cli_pause = self.create_client(DrlPause, '/dsr01/drl/drl_pause', callback_group=self.callback_group)
        self.cli_resume = self.create_client(DrlResume, '/dsr01/drl/drl_resume', callback_group=self.callback_group)
        self.cli_stop = self.create_client(DrlStop, '/dsr01/drl/drl_stop', callback_group=self.callback_group)
        # ==========================================================
        self.cli_get_state = self.create_client(GetRobotState, '/dsr01/system/get_robot_state', callback_group=self.callback_group)
        self.cli_set_control = self.create_client(SetRobotControl, '/dsr01/system/set_robot_control', callback_group=self.callback_group)
        # [추가] 1초마다 로봇 상태를 감시하는 타이머 생성
        # 이 타이머가 'error_callback' 역할을 대신 확실하게 수행합니다.
        self.create_timer(1.0, self.monitor_robot_state, callback_group=self.callback_group)
        
        # 두산 로봇 API import
        self.init_robot_api()
        
        # 1. 주문 구독 (콜백 그룹 적용)
        self.order_subscription = self.create_subscription(
            String,
            '/burger_order',
            self.order_callback,
            10,
            callback_group=self.callback_group # [추가] 병렬 실행 허용
        )

        self.error_sub = self.create_subscription(
            String, 
            '/dsr01/error',
            self.error_callback,
            10,
            callback_group=self.callback_group
        )
        # 2. 긴급 정지 구독 (콜백 그룹 적용)
        self.stop_subscription = self.create_subscription(
            String,
            '/robot_stop',
            self.stop_callback,
            10,
            callback_group=self.callback_group # [추가] 병렬 실행 허용
        )
        # [추가] 복구 명령 구독
        self.recovery_subscription = self.create_subscription(
            String,
            '/robot_recovery',
            self.recovery_callback,
            10,
            callback_group=self.callback_group
        )

        # 상태 발행
        self.status_publisher = self.create_publisher(
            String,
            '/robot_status_update',
            10
        )

        # [추가] 조그 명령 구독 (웹에서 조그 버튼 눌렀을 때)
        self.jog_subscription = self.create_subscription(
            String,
            '/burger_jog',
            self.jog_callback,
            10,
            callback_group=self.callback_group
        )

        # 두산 로봇 관절 상태 구독
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/dsr01/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 두산 로봇 에러 구독
        self.error_sub = self.create_subscription(
            String,
            '/dsr01/error',
            self.error_callback,
            10,
            callback_group=self.callback_group
        )
        
        
        # 현재 상태
        self.current_order = None
        self.robot_ready = True
        self.joint_positions = None

        self.stop_event = threading.Event()
        self.is_paused = False
        
        # [추가] 작업 취소 여부를 확인하는 플래그
        self.mission_cancelled = False

        # [추가됨] 긴급 정지 플래그 (True면 모든 동작 중단)
        self.stop_processing = False
        self.paused = False  # 일시 정지 상태 플래그
        # 로봇 초기 설정
        self.setup_robot()
        
        self.get_logger().info('✅ 브릿지 초기화 완료! 주문 대기 중...')
    
    def init_robot_api(self):
        """두산 로봇 API 초기화"""
        try:
            from DSR_ROBOT2 import (
                movej, movel, movejx, move_spiral,
                posj, posx, 
                set_digital_output, wait,
                set_velj, set_accj, set_velx, set_accx,
                set_singular_handling,
                drl_script_stop,
                drl_script_pause,   
                drl_script_resume,  
                get_robot_state, 
                get_last_alarm,   
                DR_MV_MOD_REL, DR_MV_MOD_ABS, 
                DR_MV_RA_DUPLICATE, DR_AVOID, DR_MV_APP_NONE,
                DR_AXIS_Z, DR_BASE, DR_TOOL,
                DR_QSTOP, DR_SSTOP,
                set_tool, set_tcp 
            )

            self.DR_BASE = DR_BASE  
            # Tool과 TCP 설정
            self.set_tool = set_tool
            self.set_tcp = set_tcp

            self.raw_movejx = movejx
            self.raw_movej = movej
            # 함수들을 self에 저장
            self.movej = self.create_safe_wrapper(movej)
            self.movel = self.create_safe_wrapper(movel)
            self.movejx = self.create_safe_wrapper(movejx)
            self.move_spiral = self.create_safe_wrapper(move_spiral)
            self.wait = self.create_safe_wrapper(wait) # wait도 멈춰야 함
            self.posj = posj
            self.posx = posx
            self.set_digital_output = set_digital_output
            self.set_velj = set_velj
            self.set_accj = set_accj
            self.set_velx = set_velx
            self.set_accx = set_accx
            self.set_singular_handling = set_singular_handling

            # [추가됨] 정지 함수 매핑
            self.drl_script_stop = drl_script_stop
            self.drl_script_pause = drl_script_pause   # <--- self에 연결
            self.drl_script_resume = drl_script_resume
            self.DR_QSTOP = DR_QSTOP # 퀵 스톱 (즉시 정지)
            self.get_robot_state = get_robot_state
            self.get_last_alarm = get_last_alarm
            # 상수들
            self.DR_MV_MOD_REL = DR_MV_MOD_REL
            self.DR_MV_MOD_ABS = DR_MV_MOD_ABS
            self.DR_MV_RA_DUPLICATE = DR_MV_RA_DUPLICATE
            self.DR_AVOID = DR_AVOID
            self.DR_MV_APP_NONE = DR_MV_APP_NONE
            self.DR_AXIS_Z = DR_AXIS_Z
            
            # 그리퍼 제어 상수
            self.ON = 1
            self.OFF = 0
            
            self.get_logger().info('✅ 두산 로봇 API 로드 성공!')
            
        except Exception as e:
            self.get_logger().error(f'❌ 로봇 API 로드 실패: {e}')
            raise

    def create_safe_wrapper(self, func):
        """
        로봇 동작 함수 실행 전에 '일시정지/취소' 여부를 먼저 확인하는 래퍼 함수
        """
        def wrapper(*args, **kwargs):
            # 1. 동작 실행 전 검문: 멈춰야 하는지 확인
            
            # Paused 상태면 여기서 무한 대기함
            if self.check_and_wait():
                return 
            
            # 2. 문제 없으면 원래 로봇 동작 실행
            return func(*args, **kwargs)
        
        return wrapper
    
    def setup_robot(self):
        """로봇 초기 설정 (move_periodic.py와 동일)"""
        try:
            # [추가] Tool과 TCP 설정 적용
            self.get_logger().info(f'🛠️ Tool 설정: {ROBOT_TOOL}')
            self.set_tool(ROBOT_TOOL)
            
            self.get_logger().info(f'📍 TCP 설정: {ROBOT_TCP}')
            self.set_tcp(ROBOT_TCP)

            self.set_singular_handling(self.DR_AVOID)
            self.set_velj(60.0)
            self.set_accj(100.0)
            self.set_velx(250.0, 80.625)
            self.set_accx(1000.0, 322.5)
            
            self.get_logger().info('✅ 로봇 초기 설정 완료')
        except Exception as e:
            self.get_logger().error(f'❌ 로봇 설정 실패: {e}')
    
   
    def stop_callback(self, msg):
        self.get_logger().warn(f"🚨 긴급 정지 명령 수신: {msg.data}")
        
        if "stop" in msg.data.lower():
            self.get_logger().warn("⏸️ 로봇 일시 정지 (Pause) 실행!")
            
            try:
                # 1. 로봇 동작 일시 정지
                # self.drl_script_pause()
                if self.cli_pause.service_is_ready():
                    req = DrlPause.Request()
                    self.cli_pause.call_async(req) # <--- 기다리지 않음!
                
            
                self.is_paused = True
                
                # 2. 상태 업데이트 (UI 팝업 띄우기 위함)
                self.update_status('paused')
                
            except Exception as e:
                self.get_logger().error(f"❌ 일시 정지 실패: {e}")

    # [추가] 복구 명령 콜백
    def recovery_callback(self, msg):
        """복구 명령 처리 (resume 또는 home)"""
        cmd = msg.data.lower()
        self.get_logger().info(f'🔄 복구 명령 수신: {cmd}')

        if cmd == 'resume':
            if self.is_paused:
                self.get_logger().info('▶️ 작업 재개')
                # Pause를 서비스로 걸었으면 Resume도 서비스로 푸는 게 확실합니다.
                if self.cli_resume.service_is_ready():
                    req = DrlResume.Request()
                    self.cli_resume.call_async(req)

                self.drl_script_resume()
                # 2. 상태 플래그 변경
                self.is_paused = False
                self.stop_event.set()# 대기 중인 스레드를 풀어줌
                self.update_status("processing")
                

        elif cmd == 'home':
            self.get_logger().info('🏠 홈 복귀 명령 수신: 상태를 확인하고 복귀 절차를 시작합니다.')
            
            self.mission_cancelled = True 
            self.stop_event.set() # 대기 중인 스레드가 있다면 깨움
            self.is_paused = False

            threading.Thread(target=self.execute_home_sequence).start()

    # [추가] 홈 복귀 통합 처리 함수
    def execute_home_sequence(self):
        """
        상황에 맞춰 로봇을 정지/리셋하고 홈으로 이동시킵니다.
        """
        self.update_status('recovering') # 웹에 '복구 중...' 표시
        
     
        current_state = 0
        try:
            if hasattr(self, 'get_robot_state'):
                current_state = self.get_robot_state()
        except:
            pass
            
        # 에러 상태인지 체크 (3:빨강, 5:노랑, 6:비상버튼)
        is_error = current_state in [3, 5, 6]

        if is_error:
            self.get_logger().warn(f"🛠️ 비상 상태(State {current_state}) 감지! 맞춤형 리셋을 시작합니다.")
            
            success = False
            
            # 최대 3번 시도
            for attempt in range(1, 4):
                self.get_logger().info(f"🔄 복구 시도 {attempt}/3 ...")
                
                
                reset_cmd = 4 # 기본값
                
                if current_state == 5:
                    self.get_logger().info("🟡 상태: 노란불(Safe Stop) -> 명령: 2번 (Reset Safe Stop)")
                    reset_cmd = 2 
                elif current_state == 3:
                    self.get_logger().info("🔴 상태: 빨간불(Safe Off) -> 명령: 3번 (Reset Safe Off)")
                    reset_cmd = 3
                else:
                    self.get_logger().info("⚪ 기타 상태 -> 명령: 4번 (Safety Reset)")
                    reset_cmd = 4 # 그 외(비상버튼 등)는 일반 리셋 시도

                # 1. 리셋 명령 전송
                if self.cli_set_control.service_is_ready():
                    req = SetRobotControl.Request()
                    req.robot_control = reset_cmd
                    future = self.cli_set_control.call_async(req)
                    while not future.done(): time.sleep(0.1)
                
                time.sleep(1.0) # 리셋 적용 대기

                # 2. 서보 켜기 (Servo On) - 리셋 후엔 시동을 걸어야 함
                if self.cli_set_control.service_is_ready():
                    self.get_logger().info("🔌 시동 켜기 (Servo On)...")
                    req = SetRobotControl.Request()
                    req.robot_control = 2 # 2번은 Servo On 명령이기도 함 (중의적 의미)
                    future = self.cli_set_control.call_async(req)
                    while not future.done(): time.sleep(0.1)

                # 3. 진짜 켜졌는지 확인 
                self.get_logger().info("⏳ 정상화 확인 중...")
                for _ in range(50):
                    s = self.get_robot_state() if hasattr(self, 'get_robot_state') else 0
                    if s == 1: # 정상 대기 상태
                        success = True
                        break
                    time.sleep(0.1)
                
                if success:
                    self.get_logger().info("✅ 복구 성공! 로봇이 정상화되었습니다.")
                    break
                else:
                    self.get_logger().warn("⚠️ 복구 실패. 다시 시도합니다...")
                    if hasattr(self, 'get_robot_state'):
                        current_state = self.get_robot_state()
                    time.sleep(1.0)
            
            if not success:
                self.get_logger().error("❌ 최종 복구 실패. 수동 조치가 필요합니다.")
                self.mission_cancelled = True
                self.stop_event.set()
                threading.Thread(target=self.handle_collision_recovery).start()
                return

        else:
            # 에러 없는 단순 정지 상황
            self.get_logger().info("⏸️ 일반 정지 상태. 하던 동작만 멈춥니다.")
            try:
                self.drl_script_stop(self.DR_QSTOP)
            except:
                pass
            time.sleep(0.5)
     
        self.go_home()

        final_state = 0
        if hasattr(self, 'get_robot_state'):
            final_state = self.get_robot_state()
        
        # 만약 홈으로 가다가 또 멈췄다면(State 3, 5, 6), 'Ready'를 띄우면 안 됨!
        if final_state in [3, 5, 6, 8]:
            self.get_logger().error(f"❌ 홈 이동 중 다시 에러 발생! (State: {final_state})")
            self.mission_cancelled = True
            self.stop_event.set()
            # 다시 에러 상태로 돌려보냄 (웹에서 빨간창 유지)
            threading.Thread(target=self.handle_collision_recovery).start()
            return # 여기서 함수 종료 (Ready로 안 넘어감)
        # 4. 상태 초기화
        self.mission_cancelled = False
        self.robot_ready = True
        self.current_order = None
        self.is_paused = False
        
        self.update_status('ready')
        self.get_logger().info("✨ 복구 완료. 다음 주문 대기 중.")
    # =========================================================
    # [추가] 홈 위치 이동 함수
    # =========================================================
    def go_home(self):
        self.get_logger().info("🏠 홈 위치(초기 위치)로 이동합니다...")
        
        try:
            # 1. 그리퍼/툴 먼저 초기화 (안전을 위해)
            self.girriper_fir() 
            self.set_digital_output(2, self.ON)
            
            self.get_logger().info("🐢 안전을 위해 저속으로 이동합니다.")
            
            # 관절 속도/가속도 (기존 60/100 -> 30/30)
            self.set_velj(30.0)
            self.set_accj(30.0)
            
            # 작업 속도/가속도 (기존 300/1000 -> 50/50)
            self.set_velx(50.0, 20.0)
            self.set_accx(50.0, 20.0)
            
            time.sleep(0.5) # 설정 적용 대기
            # 2. 홈 위치로 이동 (관절 이동 movej 사용 권장)
            
            self.raw_movejx(
                self.posx([367.00, 6.60, 423.0, 90.00, 180.00, 90.00]), 
                ref=0, 
                mod=self.DR_MV_MOD_ABS, 
                ra=self.DR_MV_RA_DUPLICATE, 
                sol=2,             # <--- 만약 계속 덜컹거리면 이 sol 값을 바꿔봐야 함
                vel=50.0, acc=50.0 
            )
            
            self.get_logger().info("✅ 홈 위치 도착 완료")
            # 3. 속도 원상 복구 (다음 주문을 위해)
            time.sleep(0.5)
            self.set_velj(60.0)
            self.set_accj(100.0)
            self.set_velx(250.0, 80.625)
            self.set_accx(1000.0, 322.5)
            self.get_logger().info("🚀 정상 속도로 복귀했습니다.")
            
        except Exception as e:
            self.get_logger().error(f"❌ 홈 이동 실패: {e}")
    
    def jog_callback(self, msg):
        """
        웹에서 보내온 조그 명령을 처리합니다.
        """
        try:
            cmd = json.loads(msg.data)
            self.get_logger().info(f"🕹️ 조그 명령 수신: {cmd}")

            # 로봇이 조리 중이면 거부 (안전)
            if self.current_order is not None:
                self.get_logger().warn("⚠️ 조리 중에는 조그 명령을 수행할 수 없습니다.")
                return

            jog_type = cmd.get('type')  # joint, task, grip, align
            mode = cmd.get('mode', 'rel') # rel(증분), abs(절대)
            
            # 1. 그리퍼 제어
            if jog_type == 'grip':
                action = cmd.get('cmd')
                if action == 'catch':
                    self.jipjip() 
                elif action == 'release':
                    self.all_open_ing()
            
            # 2. 관절 이동 (Joint)
            elif jog_type == 'joint':
                index = int(cmd.get('index', 0)) 
                value = float(cmd.get('value', 0.0))
                
                # 현재 위치 가져오기
                current_posj = self.posj([0,0,0,0,0,0])
                if hasattr(self, 'get_current_posj'):
                    current_posj = self.get_current_posj()
                
                target_pos = list(current_posj)
                if mode == 'rel':
                    target_pos[index] += value
                else:
                    target_pos[index] = value
                
                # 이동 실행 (안전 속도 30)
                self.movej(self.posj(target_pos), vel=30.0, acc=30.0)

            # 3. 좌표 이동 (Task/Linear)
            elif jog_type == 'task':
                index = int(cmd.get('index', 0))
                value = float(cmd.get('value', 0.0))
                
                if mode == 'rel':
                    delta = [0.0] * 6
                    delta[index] = value
                    self.movel(self.posx(delta), vel=50.0, acc=50.0, mod=self.DR_MV_MOD_REL, ref=self.DR_BASE)
                else:
                    if hasattr(self, 'get_current_posx'):
                        current_posx = self.get_current_posx()[0]
                        target_pos = list(current_posx)
                        target_pos[index] = value
                        self.movel(self.posx(target_pos), vel=50.0, acc=50.0, mod=self.DR_MV_MOD_ABS)

            # 4. Z축 정렬
            elif jog_type == 'align':
                from DSR_ROBOT2 import parallel_axis, DR_AXIS_Z, DR_BASE
                vect = [0, 0, -1]
                parallel_axis(vect, DR_AXIS_Z, DR_BASE)
                self.get_logger().info("✅ Z축 수직 정렬 완료")

        except Exception as e:
            self.get_logger().error(f"❌ 조그 실행 실패: {e}")
            
    def check_and_wait(self):
        """
        일시정지 상태라면, 명령이 올 때까지 스레드를 재우고 대기함.
        """
        if self.is_paused:
            self.get_logger().info("⏳ 일시정지! 복구 명령(Resume/Home) 대기 중...")
            
          
            # CPU를 쓰지 않으므로 다른 콜백(recovery_callback)이 정상 작동합니다.
            self.stop_event.wait() 
            
            # 대기가 풀리면(resume 또는 home 명령을 받으면) 여기로 내려옵니다.
            self.stop_event.clear() # 다음번 대기를 위해 이벤트 초기화
            self.get_logger().info("🚦 대기 해제! 다음 동작 진행...")

            time.sleep(1.0)

        # 취소(Home) 명령이었다면 True 반환하여 작업 중단 유도
        if self.mission_cancelled:
            return True
    
        return False

    def joint_state_callback(self, msg):
        """로봇 관절 상태 수신"""
        self.joint_positions = msg.position
    
    def error_callback(self, msg):
        """로봇 에러(외력 충돌 등) 수신 시 실행"""
        self.get_logger().warn(f"🚨 에러 신호 감지: {msg.data}")

        # 작업을 즉시 중단하도록 플래그 설정
        self.mission_cancelled = True 
        self.stop_event.set() # 대기 중인 스레드 해제

        # 별도 스레드에서 상태 확인 및 복구 로직 실행 (콜백 안에서 blocking 방지)
        threading.Thread(target=self.handle_collision_recovery).start()

    def monitor_robot_state(self):
        """
 
        """

        # 1. 로봇 상태 가져오기
        current_state = 0
     
        try:
      
            if hasattr(self, 'get_robot_state'):
                current_state = self.get_robot_state()
        except Exception:
            pass

        is_emergency = False
        
        if current_state in [3, 5, 6,8]:
            self.get_logger().error(f"🚨 [비상 감지] 로봇 상태 이상! (State Code: {current_state})")
            is_emergency = True

        # ----------------------------------------------------------------
        # 2. 복구 모드 진입
        # ----------------------------------------------------------------
        if is_emergency and not self.mission_cancelled:
            self.get_logger().error("💥 물리적 충돌 또는 비상 정지가 확인되었습니다.")
            
            self.mission_cancelled = True
            self.stop_event.set()
            
            # 웹에 상태 알림 및 복구 대기 (자동 리셋 X, 사용자 확인 대기)
            threading.Thread(target=self.handle_collision_recovery).start()
       

    def _check_state_result(self, future):
        """상태 조회 결과 처리"""
        try:
            result = future.result()
            state = result.robot_state
            
            
            # 방법: 직접 import해둔 self.get_last_alarm 사용
            alarm_info = self.get_last_alarm()
            
            # 알람이 있고(0이 아님), 아직 처리가 안 된 상태라면
            if alarm_info and alarm_info != 0:
                # 이미 취소 처리가 된 상태면 패스
                if self.mission_cancelled:
                    return

                self.get_logger().error(f"🚨 [비상 감지] 로봇이 멈췄습니다! 알람 코드: {alarm_info}")
                self.get_logger().error(f"   ㄴ 현재 로봇 상태(State): {state}")

                # 작업 취소 및 안전 정지 리셋 로직 실행
                self.mission_cancelled = True
                self.stop_event.set()
                
                # 복구(Reset) 시도
                self.recover_collision()

        except Exception as e:
            self.get_logger().warn(f"상태 모니터링 실패: {e}")

    def recover_collision(self):
        """안전 정지 리셋 요청"""
        if self.cli_set_control.service_is_ready():
            self.get_logger().info("🛠️ 자동 복구: 안전 정지(Protective Stop) 리셋 요청 중...")
            req = SetRobotControl.Request()
            req.robot_control = 4  # Reset Safety
            self.cli_set_control.call_async(req) 

    def handle_collision_recovery(self):
        """
        [수정됨] 충돌 감지 시 웹에 알리기만 하고 대기 (사용자 입력을 기다림)
        """
        # 1. 웹에 '에러 발생' 상태 전송 (경고창 띄움)
        self.get_logger().error("🚨 충돌 감지! 사용자 입력을 기다립니다.")
        self.update_status('error_collision') 
        
        # 2. 로봇 상태 로그 출력 (확인용)
        if self.cli_get_state.service_is_ready():
            req = GetRobotState.Request()
            future = self.cli_get_state.call_async(req)
            # 상태 확인만 하고 리셋은 하지 않음
        
     


    def order_callback(self, msg):
        """새 주문 수신"""
        try:

            self.stop_processing = False

            order = json.loads(msg.data)
            order_id = order.get('order_id')
            burger = order.get('burger', {})
            burger_name = burger.get('name', '알 수 없음')
            
            self.get_logger().info(f'📥 새 주문 수신: {burger_name} (ID: {order_id})')
            
            if not self.robot_ready:
                self.get_logger().warn('⚠️ 로봇이 바쁩니다. 주문 대기 중...')
                return
            
            # 주문 처리 시작
            self.current_order = order
            self.process_order(order_id, burger)
            
        except Exception as e:
            self.get_logger().error(f'❌ 주문 처리 오류: {e}')
    
    
    # =========================================================
    # 유틸리티 함수 (그리퍼 및 도구 제어)
    # =========================================================

    def gripper_initial(self):
        self.get_logger().info("Action: Gripper Initial")
        self.set_digital_output(2, self.ON)
        self.set_digital_output(1, self.ON)
        self.wait(2.00)

    def girriper_fir(self):
        self.get_logger().info("Action: Gripper/Output Reset (All OFF)")
        self.set_digital_output(1, self.OFF)
        self.set_digital_output(2, self.OFF)
        self.set_digital_output(3, self.OFF)

    def all_open_ing(self):
        self.get_logger().info("Action: All Open")
        self.set_digital_output(2, self.ON)
        self.wait(0.50)
        self.set_digital_output(2, self.OFF)

    def jipjip(self):
        self.get_logger().info("Action: JipJip (Gripper ON)")
        self.set_digital_output(2, self.ON)
        self.wait(0.50)

    def bread(self):
        self.get_logger().info("Action: Bread Tool")
        self.set_digital_output(3, self.ON)
        self.set_digital_output(2, self.OFF)
        self.wait(1.00)

    def sauce_close_ing(self):
        self.get_logger().info("Action: Sauce Close")
        self.set_digital_output(3, self.ON)
        self.wait(1.00)

    def spatura_open_ing(self):
        self.get_logger().info("Action: Spatula Open")
        self.set_digital_output(2, self.ON)
        self.wait(0.50)

    def spatura_close_ing(self):
        self.get_logger().info("Action: Spatula Close")
        self.set_digital_output(2, self.OFF)
        self.wait(0.50)
        self.set_digital_output(1, self.ON)
        self.wait(1.00)
        self.set_digital_output(3, self.OFF)

    # =========================================================
    # 동작 함수 (Motion Functions)
    # =========================================================

    def jipcat(self):
        self.get_logger().info("Function: jipcat")
        self.movejx(self.posx([400.98, -160.00, 95.00, 0.00, 90.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.all_open_ing()
        self.movel(self.posx([511.00, -160.00, 95.00, 180.00, -90.00, -180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.spatura_close_ing()
        self.wait(1.00)
        self.movejx(self.posx([400.98, -160.00, 97.00, 0.00, 90.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)

    def peti(self):
        self.get_logger().info("Function: peti")
        self.movejx(self.posx([580.00, -212.00, 215.00, 90.00, 121.35, 0.04]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([0.00, 165.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, -5.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, 0.00, 130.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([580.00, -100.00, 330.00, 90.00, 121.35, 0.04]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([580.00, -100.00, 340.00, 90.00, 146.00, 0.04]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, 0.00, 50.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([580.00, -30.00, 370.00, 90.00, 146.00, 0.04]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, -50.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)

    def cheese(self):
        self.get_logger().info("Function: cheese")
        self.movejx(self.posx([440.00, -212.00, 215.00, 90.00, 121.35, 0.04]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([0.00, 165.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, -5.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, 0.00, 130.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([140.00, 0.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([580.00, -60.00, 315.00, 90.00, 138.38, 0.04]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, -80.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)

    def won(self):
        self.get_logger().info("Function: won")
        self.movejx(self.posx([580.00, -212.00, 215.00, 90.00, 121.35, 0.04]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([0.00, 165.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, -5.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, 0.00, 130.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movejx(self.posx([167.00, -420.00, 300.00, 180.00, -121.35, -180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([167.00, -420.00, 312.00, 180.00, -140.37, -180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([-180.00, 0.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movejx(self.posx([167.00, -427.00, 300.00, 180.00, -121.35, -180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)

    def witch(self):
        self.get_logger().info("Function: witch")
        self.movejx(self.posx([400.98, -155.00, 90.00, 0.00, 90.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([511.00, -155.00, 90.00, 180.00, -90.00, -180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.girriper_fir()
        self.spatura_open_ing()
        self.movejx(self.posx([400.98, -155.00, 90.00, 0.00, 90.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.girriper_fir()

    def sauce(self):
        self.get_logger().info("Function: sauce")
        self.movejx(self.posx([400.98, -155.74, 198.17, 180.00, -90.00, 90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movejx(self.posx([650.58, -155.74, 198.17, 180.00, -90.00, 90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.sauce_close_ing()
        self.movel(self.posx([0.00, 0.00, 100.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([355.00, -155.00, 298.15, 180.00, -90.00, 90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movejx(self.posx([166.00, -425.00, 145.00, 180.00, -90.00, 90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.jipjip()
      
        self.get_logger().info("Action: Spiral Motion")
        self.move_spiral(rev=2.00, rmax=20.00, lmax=1.00, time=3.00, axis=self.DR_AXIS_Z, ref=0)
        
        self.set_digital_output(2, self.OFF)
        self.set_digital_output(3, self.OFF)
        self.movejx(self.posx([550.58, -155.74, 298.17, 0.00, 90.00, -90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        if self.check_cancellation(): return # [추가]
        # if self.check_stop(): return
        if self.check_and_wait(): return # 취소(Home)면 함수 종료
        self.movel(self.posx([100.00, 0.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, 0.00, -100.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.all_open_ing()
        self.movel(self.posx([-100.00, 0.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.girriper_fir()

    def tomato(self):
        self.get_logger().info("Function: tomato")
        self.movejx(self.posx([531.00, 19.50, 100.00, 180.00, -90.00, 90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.gripper_initial()
        self.movejx(self.posx([631.00, 19.50, 100.00, 180.00, -90.00, 90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([0.00, 0.00, 100.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([-100.00, 0.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)

        self.movejx(self.posx([505.00, -22.00, 415.00, 90.00, 115.00, -90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([505.50, 42.00, 368.00, 90.00, 115.00, -90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.set_digital_output(3, self.ON)
        self.wait(1.00)
        self.movel(self.posx([0.00, 0.00, 100.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, -100.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        
        self.movejx(self.posx([171.00, -395.00, 145.00, 180.00, -90.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([161.00, -395.00, 145.00, 0.00, 113.00, 180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.set_digital_output(3, self.OFF)
        self.wait(1.00)
        self.movel(self.posx([141.00, -395.00, 166.00, 0.00, 118.00, -180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([121.00, -395.00, 185.00, 0.00, 121.00, -180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, 0.00, 80.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movejx(self.posx([379.00, -22.00, 415.00, 90.00, 115.00, -90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([379.00, 42.00, 368.00, 90.00, 115.00, -90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.set_digital_output(3, self.ON)
        self.wait(1.00)
        self.movel(self.posx([0.00, 0.00, 100.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, -100.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        
        self.movejx(self.posx([171.00, -395.00, 145.00, 180.00, -90.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([156.00, -395.00, 160.00, 0.00, 113.00, 180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.set_digital_output(3, self.OFF)
        self.wait(1.00)
        self.movel(self.posx([138.00, -395.00, 177.00, 0.00, 118.00, -180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([121.00, -395.00, 192.00, 0.00, 121.00, -180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, 0.00, 80.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)

        self.movejx(self.posx([505.00, -22.00, 415.00, 90.00, 115.00, -90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
 
        self.bread()
        self.movejx(self.posx([263.00, -170.00, 97.00, 90.00, 90.00, -90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([263.00, -100.00, 97.00, 90.00, 90.00, -90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.set_digital_output(1, self.OFF)
        self.wait(1.00)
        self.movel(self.posx([0.00, 0.00, 100.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        
        self.movejx(self.posx([171.00, -400.00, 150.00, 180.00, -90.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.set_digital_output(3, self.OFF)
        self.set_digital_output(2, self.ON)
        self.movel(self.posx([180.00, -400.00, 190.00, 0.00, 117.00, 180.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([101.00, -400.00, 210.00, 180.00, -115.50, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([0.00, 0.00, 50.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.set_digital_output(2, self.OFF)
        self.gripper_initial()


        self.movejx(self.posx([435.96, -70.01, 271.15, 0.00, 110.00, -90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movejx(self.posx([631.00, 19.50, 200.00, 180.00, -90.00, 90.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_ABS, ra=self.DR_MV_RA_DUPLICATE, sol=2)
        self.movel(self.posx([0.00, 0.00, -100.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.movel(self.posx([-100.00, 0.00, 0.00, 0.00, 0.00, 0.00]), radius=0.00, ref=0, mod=self.DR_MV_MOD_REL, ra=self.DR_MV_RA_DUPLICATE)
        self.girriper_fir()


    
    def process_order(self, order_id, burger):
        """주문 처리 - move_periodic.py의 동작 시퀀스 그대로 실행"""

        self.mission_cancelled = False 
        self.stop_event.clear()
        self.robot_ready = False
        burger_name = burger.get('name', '알 수 없음')
        
        try:
            # 상태: 조리 시작
            self.update_status('cooking', order_id)
            self.get_logger().info(f'🍳 {burger_name} 조리 시작!')
            
 
            
            self.get_logger().info("--- 로봇 동작 시작 ---")
            # 순차적 실행 (while loop 1회)
            if self.check_cancellation(): return # [추가]
            # if self.check_stop(): return
            if self.check_and_wait(): return # 취소(Home)면 함수 종료
            self.girriper_fir()
            
            if self.check_cancellation(): return # [추가]
            # if self.check_stop(): return
            if self.check_and_wait(): return # 취소(Home)면 함수 종료
            self.jipcat()
            
            if self.check_cancellation(): return # [추가]
            # if self.check_stop(): return
            if self.check_and_wait(): return # 취소(Home)면 함수 종료
            self.peti()
            
            if self.check_cancellation(): return # [추가]
            # if self.check_stop(): return
            if self.check_and_wait(): return # 취소(Home)면 함수 종료
            self.cheese()
            
            if self.check_cancellation(): return # [추가]
            # if self.check_stop(): return
            if self.check_and_wait(): return # 취소(Home)면 함수 종료
            self.won()
            
            if self.check_cancellation(): return # [추가]
            # if self.check_stop(): return
            if self.check_and_wait(): return # 취소(Home)면 함수 종료
            self.witch()
            
            if self.check_cancellation(): return # [추가]
            # if self.check_stop(): return
            if self.check_and_wait(): return # 취소(Home)면 함수 종료
            self.sauce()

            if self.check_cancellation(): return # [추가]
            # if self.check_stop(): return
            if self.check_and_wait(): return # 취소(Home)면 함수 종료
            self.tomato()
            
            self.get_logger().info("--- 로봇 동작 완료 ---")
            
            # ========================================
            
            # 상태: 완료
            self.update_status('completed', order_id)
            self.get_logger().info(f'✅ {burger_name} 조리 완료!')
            
        except Exception as e:
            self.get_logger().error(f'❌ 조리 실패: {e}')
            self.update_status('error', order_id)
        finally:
            self.robot_ready = True
            self.current_order = None
        #=========================================================================================
    def check_cancellation(self):
        """작업이 취소되었는지 확인하고, 일시정지 상태면 대기"""
        # 1. 일시정지 상태라면 여기서 무한 대기
        if self.is_paused:
            self.get_logger().info("⏸️ 일시정지 대기 중...")
            self.stop_event.wait() # resume이나 home 명령이 올 때까지 여기서 멈춤
        
        # 2. 대기가 풀렸을 때, '홈(취소)' 명령이었는지 확인
        if self.mission_cancelled:
            self.get_logger().info("🛑 작업이 취소되어 시퀀스를 종료합니다.")
            return True # 취소됨
            
        return False # 계속 진행
    def update_status(self, status, order_id=None):
        """상태를 Firebase로 전송"""
        msg = String()
        status_data = {
            'status': status,
            'timestamp': time.time()
        }
        if order_id:
            status_data['order_id'] = order_id
        
        msg.data = json.dumps(status_data)
        self.status_publisher.publish(msg)
        
        self.get_logger().info(f'📤 상태 전송: {status}')
    
    

def main(args=None):
    rclpy.init(args=args)
    
    # DR_init 설정
    node = rclpy.create_node("burger_robot_bridge", namespace=ROBOT_ID)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    bridge = BurgerRobotBridge()
    executor = MultiThreadedExecutor() # 멀티스레드 실행기 생성
    executor.add_node(bridge)
    try:
        # bridge = BurgerRobotBridge()
        
        print('\n' + '='*60)
        print('🍔🤖 햄버거 제조 로봇 브릿지 실행 중!')
        print('='*60)
        print('✅ 사용자의 동작 시퀀스를 그대로 사용합니다')
        print('✅ 주문을 기다리고 있습니다...')
        print('✅ 웹에서 주문하면 자동으로 로봇이 작동합니다!')
        print('Ctrl+C로 종료')
        print('='*60 + '\n')
        
        # rclpy.spin(bridge)
        executor.spin()
    except KeyboardInterrupt:
        print('\n👋 브릿지 종료')
    except Exception as e:
        print(f'❌ 브릿지 오류: {e}')
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
