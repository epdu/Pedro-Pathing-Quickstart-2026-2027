package org.firstinspires.ftc.teamcode.pedro;

import android.annotation.SuppressLint;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RTPAxon {
    private final DcMotorEx servoEncoder;
    private final CRServo servo;
    private boolean rtp;
    private double power;
    private double maxPower;
    private Direction direction;
    private double previousAngle;
    private double totalRotation;
    private double targetRotation;

    // PID 参数与状态
    private double kP = 0.015;
    private double kI = 0.0005;
    private double kD = 0.0025;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double maxIntegralSum = 100.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    public RTPAxon(CRServo servo, DcMotorEx encoder) {
        this(servo, encoder, Direction.FORWARD);
    }

    public RTPAxon(CRServo servo, DcMotorEx encoder, Direction direction) {
        this.rtp = true;
        this.servo = servo;
        this.servoEncoder = encoder;
        this.direction = direction;
        initialize();
    }

    private void initialize() {
        servo.setPower(0);
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {
        }

        do {
            STARTPOS = getCurrentAngle();
            if (Math.abs(STARTPOS) > 1) {
                previousAngle = STARTPOS;
            } else {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException ignored) {
                }
            }
            ntry++;
        } while (Math.abs(previousAngle) < 0.2 && (ntry < 25));

        totalRotation = 0;
        homeAngle = previousAngle;
        integralSum = 0.0;
        lastError = 0.0;
        maxIntegralSum = 100.0;
        maxPower = 0.5;
        cliffs = 0;
        pidTimer.reset();
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    public double getPower() {
        return power;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setRtp(boolean rtp) {
        this.rtp = rtp;
        if (rtp) {
            resetPID();
        }
    }

    public boolean getRtp() {
        return rtp;
    }

    public void setKP(double kP) { this.kP = kP; }
    public void setKI(double kI) { this.kI = kI; resetIntegral(); }
    public void setKD(double kD) { this.kD = kD; }
    public void setPidCoeffs(double kP, double kI, double kD) {
        setKP(kP);
        setKI(kI);
        setKD(kD);
    }

    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }

    public double getTotalRotation() { return totalRotation; }
    public double getTargetRotation() { return targetRotation; }

    public void changeTargetRotation(double change) {
        targetRotation += change;
    }

    // 绝对目标角度设定（不重置积分项，适合每帧连续跟踪目标点）
    public void trackTargetRotation(double targetDegrees) {
        this.targetRotation = targetDegrees;
    }

    // 阶跃目标角度设定（重置 PID 状态，适合定点切换）
    public void setTargetRotation(double targetDegrees) {
        this.targetRotation = targetDegrees;
        resetPID();
    }

    public double getCurrentAngle() {
        if (servoEncoder == null) return 0;
        // 注意：根据你们的齿轮减速比 (GEAR_RATIO)，如果 encoder 是直连炮台则乘以 GEAR_RATIO
        return (servoEncoder.getCurrentPosition() / 8192.0) * (direction == Direction.REVERSE ? -360.0 : 360.0) * TurretConstants.GEAR_RATIO;
    }

    public boolean isAtTarget() {
        return isAtTarget(3.0);
    }

    public boolean isAtTarget(double toleranceDegrees) {
        return Math.abs(targetRotation - totalRotation) < toleranceDegrees;
    }

    public double getError() {
        return targetRotation - totalRotation;
    }

    public void forceResetTotalRotation() {
        totalRotation = 0;
        previousAngle = getCurrentAngle();
        resetPID();
    }

    public void resetPID() {
        resetIntegral();
        lastError = 0;
        pidTimer.reset();
    }

    public void resetIntegral() {
        integralSum = 0;
    }

    public synchronized void update() {
        double currentAngle = getCurrentAngle();
        double angleDifference = currentAngle - previousAngle;

        if (angleDifference > 180) {
            angleDifference -= 360;
            cliffs--;
        } else if (angleDifference < -180) {
            angleDifference += 360;
            cliffs++;
        }

        totalRotation = currentAngle - homeAngle + cliffs * 360.0;
        previousAngle = currentAngle;

        if (!rtp) return;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (dt <= 0.0001 || dt > 1.0) {
            return;
        }

        double error = targetRotation - totalRotation;

        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        final double INTEGRAL_DEADZONE = 2.0;
        if (Math.abs(error) < INTEGRAL_DEADZONE) {
            integralSum *= 0.95;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);

        final double DEADZONE = 0.5;
        if (Math.abs(error) > DEADZONE) {
            double clampedPower = Math.min(maxPower, Math.abs(output)) * Math.signum(output);
            setPower(clampedPower);
        } else {
            setPower(0);
        }
    }

    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                "Current Angle: %.2f | Total: %.2f | Target: %.2f | Power: %.3f | Err: %.2f",
                getCurrentAngle(), totalRotation, targetRotation, power, getError()
        );
    }

    @com.bylazar.configurables.annotations.Configurable
    @TeleOp(name = "AAAA Dual Turret Master-Follower PID Tuner", group = "test")
    public static class CRAxonTestRev extends LinearOpMode {

        // 支持在 Panels 网页端 Configurables 面板实时修改
        public static double kP = 0.010;
        public static double kI = 0.0000;
        public static double kD = 0.00044;
        public static double maxPower = 0.5;
        public static double targetDeg = 0.0;
        public static boolean reverseFollower = false; // 网页端一键切换副舵机正反转

        private double lastKP = kP, lastKI = kI, lastKD = kD, lastTargetDeg = targetDeg;

        @Override
        public void runOpMode() {
            TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

            CRServo masterServo = hardwareMap.get(CRServo.class, "servoTurretArmL");
            CRServo followerServo = hardwareMap.get(CRServo.class, "servoTurretArmR");
            DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, "RBMotor");

            // 仅创建主 PID 控制器
            RTPAxon masterAxon = new RTPAxon(masterServo, encoder);
            masterAxon.setMaxPower(maxPower);
            masterAxon.setPidCoeffs(kP, kI, kD);

            panelsTelemetry.debug("Status", "Master-Follower Tuner Ready");
            panelsTelemetry.update(telemetry);

            waitForStart();

            while (opModeIsActive()) {
                // 1. 实时同步网页端修改的 PID 参数
                if (kP != lastKP || kI != lastKI || kD != lastKD) {
                    masterAxon.setPidCoeffs(kP, kI, kD);
                    lastKP = kP;
                    lastKI = kI;
                    lastKD = kD;
                }
                masterAxon.setMaxPower(maxPower);

                // 2. 手柄快捷阶跃测试
                if (gamepad1.dpadUpWasPressed()) targetDeg += 15.0;
                else if (gamepad1.dpadDownWasPressed()) targetDeg -= 15.0;
                else if (gamepad1.dpadRightWasPressed()) targetDeg = 45.0;
                else if (gamepad1.dpadLeftWasPressed()) targetDeg = -45.0;
                else if (gamepad1.aWasPressed()) targetDeg = 0.0;
                else if (gamepad1.bWasPressed()) {
                    masterAxon.forceResetTotalRotation();
                    targetDeg = 0.0;
                }

                targetDeg = Math.max(-90.0, Math.min(90.0, targetDeg));

                if (targetDeg != lastTargetDeg) {
                    masterAxon.setTargetRotation(targetDeg);
                    lastTargetDeg = targetDeg;
                }

                // 3. 主控制器计算 + 副舵机纯功率跟随
                masterAxon.update();
                double followerPower = masterAxon.getPower() * (reverseFollower ? -1.0 : 1.0);
                followerServo.setPower(followerPower);

                // 4. Panels 实时图表与数据输出
                panelsTelemetry.debug("Turret Target (deg)", targetDeg);
                panelsTelemetry.debug("Turret Current (deg)", masterAxon.getTotalRotation());
                panelsTelemetry.debug("Turret Error (deg)", masterAxon.getError());
                panelsTelemetry.debug("Master Power", masterAxon.getPower());

                telemetry.addData("Target / Current", "%.1f° / %.1f° (Err: %.1f°)",
                        targetDeg, masterAxon.getTotalRotation(), masterAxon.getError());
                telemetry.addData("PID", "P=%.4f | I=%.5f | D=%.5f", kP, kI, kD);
                panelsTelemetry.update(telemetry);
            }
        }
    }


//    @TeleOp(name = "Cont. Rotation Axon REV Test", group = "test")
//    public static class CRAxonTestRev extends LinearOpMode {
//        @Override
//        public void runOpMode() {
//            TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//            CRServo crservo = hardwareMap.crservo.get("servoTurretArmL");
//            DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, "RBMotor");
//            RTPAxon servo = new RTPAxon(crservo, encoder);
//
//            waitForStart();
//            while (opModeIsActive()) {
//                servo.update();
//                if (gamepad1.dpadUpWasPressed()) servo.changeTargetRotation(15);
//                if (gamepad1.dpadDownWasPressed()) servo.changeTargetRotation(-15);
//                if (gamepad1.aWasPressed()) servo.setTargetRotation(0);
//
//                panelsTelemetry.debug("Axon Log", servo.log());
//                panelsTelemetry.update(telemetry);
//            }
//        }
//    }
}
