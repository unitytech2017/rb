/**
 * 4DOF 2족보행 로봇 확장 블록
 * Robotbit(PCA9685) 기반 서보 제어
 * 
 * 관절 매핑:
 *   S1 = 왼발목 (좌우 무게중심 이동)
 *   S2 = 왼힙  (다리 전후 스윙)
 *   S3 = 오른힙 (다리 전후 스윙)
 *   S4 = 오른발목 (좌우 무게중심 이동)
 */

//% color="#7B68EE" weight=10 icon="\uf544"
//% groups='["초기화", "관절 제어", "동작 프리미티브", "보행 패턴"]'
namespace biped {

    // ── PCA9685 하드웨어 제어 (robotbit 핵심 코드) ──

    const PCA9685_ADDR = 0x40
    const MODE1 = 0x00
    const PRESCALE = 0xFE
    const LED0_ON_L = 0x06

    let _initialized = false

    function i2cWrite(addr: number, reg: number, value: number): void {
        const buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cRead(addr: number, reg: number): number {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE)
        return pins.i2cReadNumber(addr, NumberFormat.UInt8BE)
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15) return
        const buf = pins.createBuffer(5)
        buf[0] = LED0_ON_L + 4 * channel
        buf[1] = on & 0xff
        buf[2] = (on >> 8) & 0xff
        buf[3] = off & 0xff
        buf[4] = (off >> 8) & 0xff
        pins.i2cWriteBuffer(PCA9685_ADDR, buf)
    }

    function initPCA9685(): void {
        i2cWrite(PCA9685_ADDR, MODE1, 0x00)
        // 50Hz 설정
        let prescaleval = 25000000
        prescaleval /= 4096
        prescaleval /= 50
        prescaleval -= 1
        const oldmode = i2cRead(PCA9685_ADDR, MODE1)
        i2cWrite(PCA9685_ADDR, MODE1, (oldmode & 0x7F) | 0x10)
        i2cWrite(PCA9685_ADDR, PRESCALE, prescaleval)
        i2cWrite(PCA9685_ADDR, MODE1, oldmode)
        control.waitMicros(5000)
        i2cWrite(PCA9685_ADDR, MODE1, oldmode | 0xa1)
        for (let idx = 0; idx < 16; idx++) {
            setPwm(idx, 0, 0)
        }
        _initialized = true
    }

    function setServo(channel: number, degree: number): void {
        if (!_initialized) initPCA9685()
        degree = Math.constrain(degree, 0, 180)
        // 0° → 600us, 180° → 2400us (50Hz = 20000us 주기)
        const us = degree * 1800 / 180 + 600
        const value = us * 4096 / 20000
        setPwm(channel + 7, 0, value)
    }

    // ── 내부 상태 ──

    // 관절별 서보 채널 (S1~S4 → PCA9685 채널)
    const CH_LEFT_ANKLE = 1   // S1
    const CH_LEFT_HIP = 2     // S2
    const CH_RIGHT_HIP = 3    // S3
    const CH_RIGHT_ANKLE = 4  // S4

    // 관절별 보정값 (캘리브레이션)
    let _offsetLeftAnkle = 0
    let _offsetLeftHip = 0
    let _offsetRightHip = 0
    let _offsetRightAnkle = 0

    // 현재 각도 저장
    let _curLeftAnkle = 90
    let _curLeftHip = 90
    let _curRightHip = 90
    let _curRightAnkle = 90

    // 동작 속도 (ms)
    let _speed = 500

    // ── Enum 정의 ──

    export enum Joint {
        //% block="왼발목"
        LeftAnkle = 1,
        //% block="왼힙"
        LeftHip = 2,
        //% block="오른힙"
        RightHip = 3,
        //% block="오른발목"
        RightAnkle = 4
    }

    export enum LeanDirection {
        //% block="왼쪽"
        Left = 0,
        //% block="오른쪽"
        Right = 1
    }

    export enum Leg {
        //% block="왼다리"
        Left = 0,
        //% block="오른다리"
        Right = 1
    }

    export enum SwingDirection {
        //% block="앞"
        Forward = 0,
        //% block="뒤"
        Backward = 1
    }

    export enum WalkDirection {
        //% block="왼쪽"
        Left = 0,
        //% block="오른쪽"
        Right = 1
    }

    export enum Speed {
        //% block="느리게"
        Slow = 700,
        //% block="보통"
        Normal = 500,
        //% block="빠르게"
        Fast = 300
    }

    // ── 내부 유틸 ──

    function applyJoint(joint: Joint, degree: number): void {
        degree = Math.constrain(degree, 0, 180)
        switch (joint) {
            case Joint.LeftAnkle:
                _curLeftAnkle = degree
                setServo(CH_LEFT_ANKLE, degree + _offsetLeftAnkle)
                break
            case Joint.LeftHip:
                _curLeftHip = degree
                setServo(CH_LEFT_HIP, degree + _offsetLeftHip)
                break
            case Joint.RightHip:
                _curRightHip = degree
                setServo(CH_RIGHT_HIP, degree + _offsetRightHip)
                break
            case Joint.RightAnkle:
                _curRightAnkle = degree
                setServo(CH_RIGHT_ANKLE, degree + _offsetRightAnkle)
                break
        }
    }

    function applyPose(la: number, lh: number, rh: number, ra: number): void {
        applyJoint(Joint.LeftAnkle, la)
        applyJoint(Joint.LeftHip, lh)
        applyJoint(Joint.RightHip, rh)
        applyJoint(Joint.RightAnkle, ra)
    }

    // ═══════════════════════════════════════════
    //  초기화 블록
    // ═══════════════════════════════════════════

    /**
     * 로봇을 초기화합니다. 모든 관절을 90°(홈)로 설정합니다.
     */
    //% blockId=biped_init
    //% block="로봇 초기화"
    //% group="초기화" weight=100
    export function initialize(): void {
        if (!_initialized) initPCA9685()
        _offsetLeftAnkle = 0
        _offsetLeftHip = 0
        _offsetRightHip = 0
        _offsetRightAnkle = 0
        _speed = 500
        goHome()
    }

    /**
     * 관절 보정값을 설정합니다.
     * 로봇이 홈 포지션(90°)에서 똑바로 서지 않을 때 미세 조정합니다.
     * @param joint 보정할 관절
     * @param offset 보정값 (-30 ~ +30)
     */
    //% blockId=biped_calibrate
    //% block="보정 %joint 보정값 %offset"
    //% group="초기화" weight=99
    //% offset.min=-30 offset.max=30 offset.defl=0
    export function calibrate(joint: Joint, offset: number): void {
        offset = Math.constrain(offset, -30, 30)
        switch (joint) {
            case Joint.LeftAnkle: _offsetLeftAnkle = offset; break
            case Joint.LeftHip: _offsetLeftHip = offset; break
            case Joint.RightHip: _offsetRightHip = offset; break
            case Joint.RightAnkle: _offsetRightAnkle = offset; break
        }
        // 보정 후 현재 각도 다시 적용
        goHome()
    }

    // ═══════════════════════════════════════════
    //  1층: 관절 제어 (저수준)
    // ═══════════════════════════════════════════

    /**
     * 개별 관절의 각도를 설정합니다.
     * @param joint 제어할 관절
     * @param degree 각도 (0~180)
     */
    //% blockId=biped_set_joint
    //% block="관절 %joint 각도 %degree"
    //% group="관절 제어" weight=90
    //% degree.min=0 degree.max=180 degree.defl=90
    export function setJoint(joint: Joint, degree: number): void {
        applyJoint(joint, degree)
    }

    /**
     * 4개 관절의 각도를 한 번에 설정합니다.
     * @param la 왼발목 각도
     * @param lh 왼힙 각도
     * @param rh 오른힙 각도
     * @param ra 오른발목 각도
     */
    //% blockId=biped_set_pose
    //% block="포즈 왼발목 %la 왼힙 %lh 오른힙 %rh 오른발목 %ra"
    //% group="관절 제어" weight=89
    //% la.min=0 la.max=180 la.defl=90
    //% lh.min=0 lh.max=180 lh.defl=90
    //% rh.min=0 rh.max=180 rh.defl=90
    //% ra.min=0 ra.max=180 ra.defl=90
    export function setPose(la: number, lh: number, rh: number, ra: number): void {
        applyPose(la, lh, rh, ra)
    }

    /**
     * 모든 관절을 홈 포지션(90°)으로 복귀합니다.
     */
    //% blockId=biped_go_home
    //% block="홈으로"
    //% group="관절 제어" weight=88
    export function goHome(): void {
        applyPose(90, 90, 90, 90)
    }

    /**
     * 지정한 시간(ms) 동안 대기합니다.
     * @param ms 대기 시간 (밀리초)
     */
    //% blockId=biped_wait
    //% block="대기 %ms ms"
    //% group="관절 제어" weight=87
    //% ms.defl=500
    export function wait(ms: number): void {
        basic.pause(ms)
    }

    // ═══════════════════════════════════════════
    //  2층: 동작 프리미티브 (중간)
    // ═══════════════════════════════════════════

    /**
     * 동작 속도를 설정합니다.
     * @param speed 속도 단계
     */
    //% blockId=biped_set_speed
    //% block="속도 %speed"
    //% group="동작 프리미티브" weight=80
    export function setSpeed(speed: Speed): void {
        _speed = speed
    }

    /**
     * 동작 속도를 직접 설정합니다 (ms).
     * @param ms 동작 간 대기시간 (밀리초)
     */
    //% blockId=biped_set_speed_ms
    //% block="속도 %ms ms"
    //% group="동작 프리미티브" weight=79
    //% ms.min=100 ms.max=2000 ms.defl=500
    //% advanced=true
    export function setSpeedMs(ms: number): void {
        _speed = Math.constrain(ms, 100, 2000)
    }

    /**
     * 발목을 이용해 무게중심을 이동합니다.
     * @param dir 기울일 방향
     * @param angle 기울기 각도 (기본 30)
     */
    //% blockId=biped_lean
    //% block="기울이기 %dir 각도 %angle"
    //% group="동작 프리미티브" weight=78
    //% angle.min=5 angle.max=60 angle.defl=30
    export function lean(dir: LeanDirection, angle: number): void {
        angle = Math.constrain(angle, 5, 60)
        if (dir == LeanDirection.Left) {
            // 양쪽 발목을 같은 방향으로 기울여 왼쪽으로 무게중심 이동
            applyJoint(Joint.LeftAnkle, 90 - angle)
            applyJoint(Joint.RightAnkle, 90 - angle)
        } else {
            applyJoint(Joint.LeftAnkle, 90 + angle)
            applyJoint(Joint.RightAnkle, 90 + angle)
        }
        basic.pause(_speed)
    }

    /**
     * 힙을 이용해 다리를 앞뒤로 스윙합니다.
     * @param leg 움직일 다리
     * @param dir 스윙 방향
     * @param angle 스윙 각도 (기본 30)
     */
    //% blockId=biped_swing
    //% block="스윙 %leg %dir 각도 %angle"
    //% group="동작 프리미티브" weight=77
    //% angle.min=5 angle.max=60 angle.defl=30
    export function swing(leg: Leg, dir: SwingDirection, angle: number): void {
        angle = Math.constrain(angle, 5, 60)
        let target = 90
        if (dir == SwingDirection.Forward) {
            target = 90 + angle
        } else {
            target = 90 - angle
        }
        if (leg == Leg.Left) {
            applyJoint(Joint.LeftHip, target)
        } else {
            applyJoint(Joint.RightHip, target)
        }
        basic.pause(_speed)
    }

    // ═══════════════════════════════════════════
    //  3층: 보행 패턴 (고수준)
    // ═══════════════════════════════════════════

    /**
     * 앞으로 걷습니다.
     * @param steps 걸음 수 (기본 3)
     */
    //% blockId=biped_walk_forward
    //% block="전진 %steps 걸음"
    //% group="보행 패턴" weight=70
    //% steps.min=1 steps.max=10 steps.defl=3
    export function walkForward(steps: number): void {
        const leanAngle = 30
        const swingAngle = 30
        for (let i = 0; i < steps; i++) {
            // 1. 왼쪽으로 기울이기 (오른발 들림)
            applyPose(90 - leanAngle, _curLeftHip, _curRightHip, 90 - leanAngle)
            basic.pause(_speed)
            // 2. 오른힙 앞으로 스윙
            applyJoint(Joint.RightHip, 90 + swingAngle)
            basic.pause(_speed)
            // 3. 무게중심 복귀
            applyJoint(Joint.LeftAnkle, 90)
            applyJoint(Joint.RightAnkle, 90)
            basic.pause(_speed)
            // 4. 오른쪽으로 기울이기 (왼발 들림)
            applyPose(90 + leanAngle, _curLeftHip, _curRightHip, 90 + leanAngle)
            basic.pause(_speed)
            // 5. 왼힙 앞으로 스윙
            applyJoint(Joint.LeftHip, 90 + swingAngle)
            basic.pause(_speed)
            // 6. 홈으로 복귀
            goHome()
            basic.pause(_speed)
        }
    }

    /**
     * 뒤로 걷습니다.
     * @param steps 걸음 수 (기본 3)
     */
    //% blockId=biped_walk_backward
    //% block="후진 %steps 걸음"
    //% group="보행 패턴" weight=69
    //% steps.min=1 steps.max=10 steps.defl=3
    export function walkBackward(steps: number): void {
        const leanAngle = 30
        const swingAngle = 30
        for (let i = 0; i < steps; i++) {
            applyPose(90 - leanAngle, _curLeftHip, _curRightHip, 90 - leanAngle)
            basic.pause(_speed)
            applyJoint(Joint.RightHip, 90 - swingAngle)
            basic.pause(_speed)
            applyJoint(Joint.LeftAnkle, 90)
            applyJoint(Joint.RightAnkle, 90)
            basic.pause(_speed)
            applyPose(90 + leanAngle, _curLeftHip, _curRightHip, 90 + leanAngle)
            basic.pause(_speed)
            applyJoint(Joint.LeftHip, 90 - swingAngle)
            basic.pause(_speed)
            goHome()
            basic.pause(_speed)
        }
    }

    /**
     * 제자리에서 회전합니다.
     * @param dir 회전 방향
     * @param steps 걸음 수 (기본 3)
     */
    //% blockId=biped_turn
    //% block="회전 %dir %steps 걸음"
    //% group="보행 패턴" weight=68
    //% steps.min=1 steps.max=10 steps.defl=3
    export function turn(dir: WalkDirection, steps: number): void {
        const leanAngle = 30
        const swingAngle = 30
        for (let i = 0; i < steps; i++) {
            if (dir == WalkDirection.Left) {
                // 왼쪽 회전: 오른발 앞 + 왼발 뒤
                applyPose(90 - leanAngle, _curLeftHip, _curRightHip, 90 - leanAngle)
                basic.pause(_speed)
                applyJoint(Joint.RightHip, 90 + swingAngle)
                basic.pause(_speed)
                applyJoint(Joint.LeftAnkle, 90)
                applyJoint(Joint.RightAnkle, 90)
                basic.pause(_speed)
                applyPose(90 + leanAngle, _curLeftHip, _curRightHip, 90 + leanAngle)
                basic.pause(_speed)
                applyJoint(Joint.LeftHip, 90 - swingAngle)
                basic.pause(_speed)
                goHome()
                basic.pause(_speed)
            } else {
                // 오른쪽 회전: 왼발 앞 + 오른발 뒤
                applyPose(90 + leanAngle, _curLeftHip, _curRightHip, 90 + leanAngle)
                basic.pause(_speed)
                applyJoint(Joint.LeftHip, 90 + swingAngle)
                basic.pause(_speed)
                applyJoint(Joint.LeftAnkle, 90)
                applyJoint(Joint.RightAnkle, 90)
                basic.pause(_speed)
                applyPose(90 - leanAngle, _curLeftHip, _curRightHip, 90 - leanAngle)
                basic.pause(_speed)
                applyJoint(Joint.RightHip, 90 - swingAngle)
                basic.pause(_speed)
                goHome()
                basic.pause(_speed)
            }
        }
    }

    /**
     * 발차기 동작을 합니다.
     * @param leg 킥할 다리
     */
    //% blockId=biped_kick
    //% block="킥 %leg"
    //% group="보행 패턴" weight=67
    export function kick(leg: Leg): void {
        const leanAngle = 35
        if (leg == Leg.Right) {
            // 왼쪽 기울이고 오른발 킥
            applyPose(90 - leanAngle, 90, 90, 90 - leanAngle)
            basic.pause(_speed)
            applyJoint(Joint.RightHip, 90 + 50)
            basic.pause(Math.floor(_speed / 2))
            applyJoint(Joint.RightHip, 90)
            basic.pause(_speed)
        } else {
            // 오른쪽 기울이고 왼발 킥
            applyPose(90 + leanAngle, 90, 90, 90 + leanAngle)
            basic.pause(_speed)
            applyJoint(Joint.LeftHip, 90 + 50)
            basic.pause(Math.floor(_speed / 2))
            applyJoint(Joint.LeftHip, 90)
            basic.pause(_speed)
        }
        goHome()
        basic.pause(_speed)
    }

    /**
     * 인사 동작을 합니다.
     */
    //% blockId=biped_bow
    //% block="인사"
    //% group="보행 패턴" weight=66
    export function bow(): void {
        // 양쪽 힙을 앞으로 숙이기
        applyPose(90, 90 + 40, 90 + 40, 90)
        basic.pause(_speed * 2)
        goHome()
        basic.pause(_speed)
    }
}
