
package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.pedro.HardwareDecodeRobotBioBuzzSeason.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Configurable
@TeleOp(name = "AAAAAA BioBuzz TeleOp")
public class TeleOpDecodeRobotRedBioBuzzSeason extends LinearOpMode {

    public static boolean ENABLE_CAMERA_LIVE_VIEW = true;

    // 红方实测基准坐标（蓝方自动中心对称镜像）
    public static double RED_BASE_GOAL_X = 58.0;
    public static double RED_BASE_GOAL_Y = 62.0;
    public static double RED_BASE_START_X = 58.0;
    public static double RED_BASE_START_Y = 8.0;
    public static double RED_BASE_START_HEADING_DEG = 180.0;

    // 炮台物理安装偏心、行程/限速与双阈值迟滞防抖参数
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;
    public static double TURRET_MAX_ROTATION_DEG = 100.0;
    public static double TURRET_MAX_SPEED_DEG_PER_S = 540.0;

    public static double MIN_SHOOT_RANGE_DISENGAGE_IN = 35.0;
    public static double MIN_SHOOT_RANGE_REENGAGE_IN  = 38.0;
    public static double ENTRY_HALF_WIDTH_DISENGAGE   = 8.5;
    public static double ENTRY_HALF_WIDTH_REENGAGE    = 7.2;

    // 全向底盘与航向锁定参数
    public static double HEADING_KP = 0.95;
    public static double HEADING_DEADBAND_DEG = 1.0;
    public static double SETTLE_SPEED_DEG_PER_S = 30.0;
    public static double STICK_DEADBAND = 0.01;

    public float DriveTrains_ReducePOWER = 1.0f;
    public float DriveTrains_smoothTurn = 1.0f;
    public float intakePowerShoot = 0.90f;
    public float intakePowerIntake = 0.95f;
    public float intakePowerDump = -0.65f;
    public float intakePowerOff = 0.0f;

    public static final double blockageblockposition = 0.10;
    public static final double blockagereleaseposition = 0.23;
    public static final double HoodArmfarposition = 0.30;
    public static final double HoodArmcloseposition = 0.35;

    public static boolean AUTO_RANGE_SHOOT_ENABLED = true;
    public static double toleranceforShoot = 100.0;
    public static double TURRET_LOOKAHEAD_S = 0.08;
    public static double TURRET_TRIM_LIMIT_DEG = 15.0;
    public static double SHOT_FLIGHT_TIME_S = 0.0;

    private final HardwareDecodeRobotBioBuzzSeason robot = new HardwareDecodeRobotBioBuzzSeason();
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    private Follower follower;
    private final ElapsedTime imuResetTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime turretRateTimer = new ElapsedTime();

    // 状态标志与缓存
    private boolean isFarMode = false;
    private boolean isShooterAtSpeed = false;
    private boolean shooterIsOn = false;
    private boolean intakeIsOn = false;
    private boolean fireRequested = false;
    private boolean imuResetInCooldown = false;
    private static final long IMU_RESET_COOLDOWN_MS = 300;

    private double currentGoalDistanceIn = 0.0;
    private double dynamicTargetVelo = 1050.0;
    private double dynamicTargetHood = 0.50;

    private boolean turretAutoAim = true;
    private boolean isInTurretDeadZone = false;
    private boolean wasAboveMinRange = false;
    private boolean wasWithinEntryAngle = false;
    private boolean turretReachable = true;
    private boolean wasTurretReachable = true;
    private double turretOverLimitDeg = 0.0;
    private double turretManualTrimDeg = 0.0;
    private double turretTargetDeg = 0.0;
    private double smoothedTurretTargetDeg = 0.0;

    private double holdHeading = 0.0;
    private boolean holdHeadingSet = false;
    private boolean turnedSinceHold = false;

    // 网页端 PID 热更新对比缓存
    private double lastTurretKP = -1, lastTurretKI = -1, lastTurretKD = -1, lastTurretMaxPower = -1;
    private double lastShooterKP = -1, lastShooterKI = -1, lastShooterKD = -1, lastShooterKF = -1;

    private double cachedShooterVeloL = 0.0;
    private double cachedShooterVeloR = 0.0;
    private String lastLedColor = "";

    @Override
    public void runOpMode() {
        // 1. 硬件初始化（内部自动加载唯一的炮台 PID 与飞轮 PIDF 参数）
        robot.init(hardwareMap);
        robot.alliance = Alliance.RED; // 红方基准（比赛分到蓝方改为 Alliance.BLUE 即可）
        robot.initVision(ENABLE_CAMERA_LIVE_VIEW);

        // 2. 初始化 Pedro Pathing Follower 并设定对称起步坐标
        follower = Constants.create(hardwareMap);
        robot.clearBulkCache();
        Pose startPose = robot.toAlliancePose(RED_BASE_START_X, RED_BASE_START_Y, RED_BASE_START_HEADING_DEG);
        follower.setPose(startPose);
        follower.update();

        holdHeading = follower.pose().heading();
        holdHeadingSet = true;

        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        telemetry.addData("Status", "Unified TeleOp Ready | Alliance: " + robot.alliance.name());
        telemetry.update();

        waitForStart();
        loopTimer.reset();
        turretRateTimer.reset();

        while (opModeIsActive()) {
            double loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();
            robot.clearBulkCache();

            cachedShooterVeloL = Math.abs(robot.MasterShooterMotorL.getVelocity());
            cachedShooterVeloR = Math.abs(robot.SlaveShooterMotorR.getVelocity());

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            follower.update();
            updateDrivetrain_FieldCentric();
            handleHeadingAndPositionReset();

            if (currentGamepad2.x && !previousGamepad2.x) {
                isFarMode = true;
            } else if (currentGamepad2.b && !previousGamepad2.b) {
                isFarMode = false;
            }

            syncShooterPIDFFromPanels();
            updateIntake();
            updateShooter();
            checkShooterVelocity();
            updateBlockage();
            updateHood();
            updateLEDs();

            updateTurretControl();
            robot.updateTurretAxons();
            updateHiveStatus();

            updateAllTelemetry(loopTimeMs);
            telemetry.update();
        }
    }

    private void syncShooterPIDFFromPanels() {
        if (ShooterPIDFConfig.kP != lastShooterKP
                || ShooterPIDFConfig.kI != lastShooterKI
                || ShooterPIDFConfig.kD != lastShooterKD
                || ShooterPIDFConfig.kF != lastShooterKF) {
            robot.applyShooterPIDF();
            lastShooterKP = ShooterPIDFConfig.kP;
            lastShooterKI = ShooterPIDFConfig.kI;
            lastShooterKD = ShooterPIDFConfig.kD;
            lastShooterKF = ShooterPIDFConfig.kF;
        }
    }

    private void updateTurretControl() {
        if (TURRET_KP != lastTurretKP || TURRET_KI != lastTurretKI || TURRET_KD != lastTurretKD) {
            robot.axonTurretArmL.setPidCoeffs(TURRET_KP, TURRET_KI, TURRET_KD);
            lastTurretKP = TURRET_KP;
            lastTurretKI = TURRET_KI;
            lastTurretKD = TURRET_KD;
        }
        if (TURRET_MAX_POWER != lastTurretMaxPower) {
            robot.axonTurretArmL.setMaxPower(TURRET_MAX_POWER);
            lastTurretMaxPower = TURRET_MAX_POWER;
        }

        if (currentGamepad2.back && !previousGamepad2.back) {
            turretAutoAim = !turretAutoAim;
        }

        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
            turretManualTrimDeg += 2.0;
        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            turretManualTrimDeg -= 2.0;
        }
        turretManualTrimDeg = Range.clip(turretManualTrimDeg, -TURRET_TRIM_LIMIT_DEG, TURRET_TRIM_LIMIT_DEG);

        Pose p = follower.pose();
        Velocity v = follower.velocity();
        double h = p.heading();
        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();

        double pivotX = p.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h);
        double pivotY = p.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h);

        double targetX = robot.toAllianceX(RED_BASE_GOAL_X);
        double targetY = robot.toAllianceY(RED_BASE_GOAL_Y);
        if (SHOT_FLIGHT_TIME_S > 0) {
            targetX -= v.vx * SHOT_FLIGHT_TIME_S;
            targetY -= v.vy * SHOT_FLIGHT_TIME_S;
        }
        double dx = targetX - pivotX;
        double dy = targetY - pivotY;
        double rangeSq = dx * dx + dy * dy;

        // 直接调用硬件类的统一插值计算方法
        currentGoalDistanceIn = Math.sqrt(rangeSq);
        dynamicTargetHood = robot.getDynamicHoodPosition(currentGoalDistanceIn);
        dynamicTargetVelo = robot.getDynamicShooterVelocity(currentGoalDistanceIn);

        boolean inFrontOfOpening = robot.isInFrontOfGoal(dy);
        double minRangeLimit = wasAboveMinRange ? MIN_SHOOT_RANGE_DISENGAGE_IN : MIN_SHOOT_RANGE_REENGAGE_IN;
        boolean aboveMinRange = (rangeSq >= minRangeLimit * minRangeLimit);
        wasAboveMinRange = aboveMinRange;

        double maxTanLimit = wasWithinEntryAngle? (ENTRY_HALF_WIDTH_DISENGAGE / 12.04)
                : (ENTRY_HALF_WIDTH_REENGAGE / 12.04);
        boolean withinEntryAngle = Math.abs(dx) <= Math.abs(dy) * maxTanLimit;
        wasWithinEntryAngle = withinEntryAngle;
        boolean isValidShootingZone = inFrontOfOpening && aboveMinRange && withinEntryAngle;
        isInTurretDeadZone = !isValidShootingZone;
        double desiredTargetDeg;
        if (!turretAutoAim || !inFrontOfOpening || !withinEntryAngle) {
            desiredTargetDeg = 0.0;
            turretReachable = true;
            turretOverLimitDeg = 0.0;
        } else if (!aboveMinRange) {
            desiredTargetDeg = turretTargetDeg;
            turretReachable = true;
            turretOverLimitDeg = 0.0;
        } else {
            double relAngle = Math.atan2(dy, dx) - h;
            while (relAngle >  Math.PI) relAngle -= 2 * Math.PI;
            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
            double bearingRate = (rangeSq > 1e-6) ? (dy * v.vx - dx * v.vy) / rangeSq : 0.0;
            double turretRate  = bearingRate - v.omega;
            double baseTargetDeg = Math.toDegrees(relAngle + turretRate * TURRET_LOOKAHEAD_S) + turretManualTrimDeg;
            double candidateDeg = baseTargetDeg;
            if (candidateDeg - currentTurretDeg > 180.0
                    && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
                candidateDeg -= 360.0;
            } else if (candidateDeg - currentTurretDeg < -180.0
                    && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
                candidateDeg += 360.0;
            }
            turretReachable = Math.abs(candidateDeg) <= TURRET_MAX_ROTATION_DEG;
            if (!turretReachable) {
                turretOverLimitDeg = (candidateDeg > TURRET_MAX_ROTATION_DEG)
                        ? (candidateDeg - TURRET_MAX_ROTATION_DEG)
                        : (candidateDeg + TURRET_MAX_ROTATION_DEG);
            } else {
                turretOverLimitDeg = 0.0;
            }
            if (wasTurretReachable && !turretReachable) {
                gamepad1.rumble(300);
            }
            wasTurretReachable = turretReachable;
            desiredTargetDeg = Range.clip(candidateDeg, -TURRET_MAX_ROTATION_DEG, TURRET_MAX_ROTATION_DEG);
        }
        turretTargetDeg = desiredTargetDeg;
        double dt = turretRateTimer.seconds();
        turretRateTimer.reset();
        if (TURRET_MAX_SPEED_DEG_PER_S > 0 && dt > 0.0 && dt < 0.5) {
            double maxStep = TURRET_MAX_SPEED_DEG_PER_S * dt;
            smoothedTurretTargetDeg += Range.clip(turretTargetDeg - smoothedTurretTargetDeg, -maxStep, maxStep);
        } else {
            smoothedTurretTargetDeg = turretTargetDeg;
        }
        robot.axonTurretArmL.trackTargetRotation(smoothedTurretTargetDeg);
        telemetry.addData("Turret Target (deg)", smoothedTurretTargetDeg);
        telemetry.addData("Turret Actual (deg)", currentTurretDeg);
        telemetry.addData("Turret Error (deg)", smoothedTurretTargetDeg - currentTurretDeg);
        telemetry.addData("Turret Servo Power", robot.axonTurretArmL.getPower());
        telemetry.addData("Turret Valid Zone", isValidShootingZone ? 1 : 0);
        telemetry.addData("Turret Reachable", turretReachable ? 1 : 0);
        if (!turretReachable) {
            telemetry.addData("TURN ROBOT WARNING", "Turn %.0f deg %s!",
                    Math.abs(turretOverLimitDeg), turretOverLimitDeg > 0 ? "LEFT" : "RIGHT");
        }
    }
    public void updateDrivetrain_FieldCentric() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * DriveTrains_smoothTurn;
        Pose pose = follower.pose();
        Velocity vel = follower.velocity();
        double botHeading = pose.heading();
        double spinSpeed = Math.abs(vel.omega);
        boolean turning = Math.abs(gamepad1.right_stick_x) > STICK_DEADBAND;
        boolean settled = spinSpeed < Math.toRadians(SETTLE_SPEED_DEG_PER_S);
        if (turning) {
            turnedSinceHold = true;
            if (!settled) {
                holdHeadingSet = false;
            }
        }
        if (settled && turnedSinceHold) {
            holdHeading = botHeading;
            holdHeadingSet = true;
            turnedSinceHold = false;
        }
        if (!turning && holdHeadingSet) {
            double headingError = Math.atan2(Math.sin(holdHeading - botHeading), Math.cos(holdHeading - botHeading));
            if (Math.abs(headingError) > Math.toRadians(HEADING_DEADBAND_DEG)) {
                rx = -HEADING_KP * headingError;
            }
        }
        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;
        robot.leftFrontMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
        robot.leftRearMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
        robot.rightFrontMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
        robot.rightRearMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
    }
    private void handleHeadingAndPositionReset() {
        if (imuResetInCooldown && imuResetTimer.milliseconds() > IMU_RESET_COOLDOWN_MS) {
            imuResetInCooldown = false;
        }
        if (currentGamepad1.back && currentGamepad1.start) {
            Pose resetPose = robot.toAlliancePose(RED_BASE_START_X, RED_BASE_START_Y, RED_BASE_START_HEADING_DEG);
            follower.setPose(resetPose);
            holdHeading = resetPose.heading();
            holdHeadingSet = true;
            gamepad1.rumble(300);
        } else if (currentGamepad1.back && !previousGamepad1.back) {
            if (robot.finalAutoX != 0 || robot.finalAutoY != 0) {
                follower.setPose(new Pose(robot.finalAutoX, robot.finalAutoY, robot.finalAutoHeading));
                holdHeading = robot.finalAutoHeading;
                holdHeadingSet = true;
            }
        } else if (currentGamepad1.start && !previousGamepad1.start && !imuResetInCooldown) {
            double homeHeadingRad = Math.toRadians(robot.toAllianceHeadingDeg(RED_BASE_START_HEADING_DEG));
            follower.setHeading(homeHeadingRad);
            holdHeading = homeHeadingRad;
            holdHeadingSet = true;
            imuResetInCooldown = true;
            imuResetTimer.reset();
            gamepad1.rumble(200);
        }
    }
    public void updateIntake() {
        if (currentGamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger <= 0.1) {
            robot.BlockageArmL.setPosition(blockageblockposition);
            robot.BlockageArmR.setPosition(blockageblockposition);
            robot.IntakeMotorL.setPower(intakePowerIntake);
            robot.IntakeMotorR.setPower(intakePowerIntake);
            intakeIsOn = true;
        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            robot.IntakeMotorL.setPower(intakePowerDump);
            robot.IntakeMotorR.setPower(intakePowerDump);
            intakeIsOn = true;
        } else if (currentGamepad1.y && !previousGamepad1.y) {
            stopIntake();
        }
    }
    private void stopIntake() {
        robot.IntakeMotorL.setPower(intakePowerOff);
        robot.IntakeMotorR.setPower(intakePowerOff);
        intakeIsOn = false;
        isShooterAtSpeed = false;
        fireRequested = false;
        if (isFarMode) {
            startShooterIdleFar();
        } else {
            startShooterIdle();
        }
    }
    public void updateShooter() {
        if (currentGamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger <= 0.1) {
            if (isFarMode) {
                startShooterFar();
            } else {
                startShooter();
            }
        }
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            fireRequested = true;
            robot.BlockageArmL.setPosition(blockagereleaseposition);
            robot.BlockageArmR.setPosition(blockagereleaseposition);
            robot.IntakeMotorL.setPower(intakePowerShoot);
            robot.IntakeMotorR.setPower(intakePowerShoot);
            intakeIsOn = true;
        }
        if (currentGamepad1.b && !previousGamepad1.b) {
            stopShooter();
            robot.BlockageArmL.setPosition(blockageblockposition);
            robot.BlockageArmR.setPosition(blockageblockposition);
        }
        if (currentGamepad1.a && !previousGamepad1.a) {
            stopIntakeAndShooter();
        }
        if (shooterIsOn && !AUTO_RANGE_SHOOT_ENABLED) {
            double manualTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
            robot.shooterL.setVelocity(Math.abs(manualTarget));
            robot.shooterR.setVelocity(Math.abs(manualTarget));
        }
    }
    private void startShooterIdle() {
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        robot.setHoodPosition(0.5);
        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeed));
        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeed));
    }
    private void startShooterIdleFar() {
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeedfar));
        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeedfar));
    }
    private void startShooter() {
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        if (AUTO_RANGE_SHOOT_ENABLED) {
            robot.setHoodPosition(dynamicTargetHood);
            robot.shooterL.setVelocity(Math.abs(dynamicTargetVelo));
            robot.shooterR.setVelocity(Math.abs(dynamicTargetVelo));
        } else {
            robot.setHoodPosition(0.50);
            robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
            robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
        }
        shooterIsOn = true;
    }
    private void startShooterFar() {
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEEDfar));
        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEEDfar));
        shooterIsOn = true;
    }
    private void stopShooter() {
        if (isFarMode) {
            startShooterIdleFar();
        } else {
            startShooterIdle();
        }
        intakeIsOn = false;
        isShooterAtSpeed = false;
        fireRequested = false;
        shooterIsOn = false;
    }
    private void stopIntakeAndShooter() {
        robot.MasterShooterMotorL.setVelocity(0);
        robot.SlaveShooterMotorR.setVelocity(0);
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        intakeIsOn = false;
        isShooterAtSpeed = false;
        fireRequested = false;
        shooterIsOn = false;
    }
    private void checkShooterVelocity() {
        double activeTarget = AUTO_RANGE_SHOOT_ENABLED
                ? dynamicTargetVelo
                : (isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED);
        isShooterAtSpeed = Math.abs(cachedShooterVeloL - activeTarget) <= toleranceforShoot;
        if (!isShooterAtSpeed) {
            fireRequested = false;
        }
    }
    private void updateHood() {
        if (currentGamepad2.start && !previousGamepad2.start) {
            AUTO_RANGE_SHOOT_ENABLED = !AUTO_RANGE_SHOOT_ENABLED;
        }
        if (AUTO_RANGE_SHOOT_ENABLED && shooterIsOn) {
            robot.setHoodPosition(dynamicTargetHood);
            double targetV = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : dynamicTargetVelo;
            robot.shooterL.setVelocity(Math.abs(targetV));
            robot.shooterR.setVelocity(Math.abs(targetV));
        } else if (!AUTO_RANGE_SHOOT_ENABLED) {
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                robot.setHoodPosition(HoodArmfarposition);
            } else if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                robot.setHoodPosition(HoodArmcloseposition);
            } else if (currentGamepad2.y && !previousGamepad2.y) {
                robot.setHoodPosition(0.50);
            }
        }
    }
    private void updateBlockage() {
        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
            robot.BlockageArmL.setPosition(blockageblockposition);
            robot.BlockageArmR.setPosition(blockageblockposition);
        } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
            robot.BlockageArmL.setPosition(blockagereleaseposition);
            robot.BlockageArmR.setPosition(blockagereleaseposition);
        }
    }
    private void updateLEDs() {
        if (isShooterAtSpeed && !isInTurretDeadZone && turretReachable && robot.axonTurretArmL.isAtTarget(4.0)) {
            setLEDColor("GREEN");
        } else {
            setLEDColor("RED");
        }
    }
    private void setLEDColor(String color) {
        if (color.equals(lastLedColor)) return;
        try {
            if ("GREEN".equals(color)) {
                if (robot.greenLED != null) robot.greenLED.setState(false);
                if (robot.redLED != null) robot.redLED.setState(true);
            } else if ("RED".equals(color)) {
                if (robot.greenLED != null) robot.greenLED.setState(true);
                if (robot.redLED != null) robot.redLED.setState(false);
            }
            lastLedColor = color;
        } catch (Exception e) {
            telemetry.addData("LED Error", e.getMessage());
        }
    }
    private void updateHiveStatus() {
        if (robot.myAprilTagProcessor == null) return;
        robot.hiveScorable = false;
        robot.hiveClusterName = "";
        robot.hiveRoll = 0.0;
        robot.hivePercentFound = 0.0;
        for (AprilTagDetection d : robot.myAprilTagProcessor.getDetections()) {
            double rawRoll = (d.ftcPose != null) ? d.ftcPose.roll : 0.0;
            if (d instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) d;
                String name = (cluster.metadata != null && cluster.metadata.name != null)
                        ? cluster.metadata.name : "Unknown_Cluster";
                boolean allianceMatch = name.startsWith(robot.getAllianceTagPrefix());
                if (!allianceMatch || Math.abs(rawRoll) >= 90.0 || cluster.percentClusterFound < 0.5) continue;
                robot.hiveScorable = true;
                robot.hiveClusterName = name;
                robot.hiveRoll = rawRoll;
                robot.hivePercentFound = cluster.percentClusterFound;
                break;
            } else if (d instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) d;
                String name = (single.metadata != null && single.metadata.name != null)
                        ? single.metadata.name : ("ID_" + single.id);
                robot.hiveScorable = true;
                robot.hiveClusterName = name;
                robot.hiveRoll = rawRoll;
                robot.hivePercentFound = 1.0;
                break;
            }
        }
    }
    private void updateAllTelemetry(double loopTimeMs) {
        Pose pose = follower.pose();
        double activeTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
        telemetry.addData("Loop Time (ms)", loopTimeMs);
        telemetry.addData("Shooter Target", activeTarget);
        telemetry.addData("Shooter Left Velo", cachedShooterVeloL);
        telemetry.addData("Shooter Right Velo", cachedShooterVeloR);
        telemetry.addData("Hive Scorable", robot.hiveScorable ? 1 : 0);
        telemetry.addData("Goal Distance (in)", currentGoalDistanceIn);
        telemetry.addData("Dynamic Velo Target", dynamicTargetVelo);
        telemetry.addData("Dynamic Hood Target", dynamicTargetHood);
        telemetry.addData("Alliance", robot.alliance.name());
        telemetry.addData("Shoot Mode", isFarMode ? "FAR" : "NEAR");
        telemetry.addData("Pose (X/Y/H)", "%.1f / %.1f / %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Hold Heading", "%.1f° (Active:%b)", Math.toDegrees(holdHeading), holdHeadingSet);
        telemetry.addData("Hive Cluster", robot.hiveClusterName);
    }
}


//package org.firstinspires.ftc.teamcode.pedro;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.JoinedTelemetry;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.pedropathing.math.Velocity;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
//import org.firstinspires.ftc.teamcode.pedro.HardwareDecodeRobotBioBuzzSeason.ShooterPIDFConfig;
//@Configurable
//@TeleOp(name = "AAAAAA BioBuzz TeleOp (Red-Base Auto Symmetry + Hysteresis)")
//public class TeleOpDecodeRobotRedBioBuzzSeason extends LinearOpMode {
//
//    // 调试/比赛摄像头预览开关（正式比赛建议改为 false 节省 CPU）
//    public static boolean ENABLE_CAMERA_LIVE_VIEW = true;
//
//    // 红方实测基准坐标（当 robot.alliance = Alliance.BLUE 时，对称引擎自动镜像为蓝方对应坐标）
//    public static double RED_BASE_GOAL_X = 58.0;            // 红方目标 X（蓝方自动镜像为 83.5）
//    public static double RED_BASE_GOAL_Y = 62.0;            // 红方目标 Y（蓝方自动镜像为 79.5）
//    public static double RED_BASE_START_X = 58.0;           // 红方起始 X（蓝方自动镜像为 83.5）
//    public static double RED_BASE_START_Y = 8.0;            // 红方起始 Y（蓝方自动镜像为 133.5）
//    public static double RED_BASE_START_HEADING_DEG = 180.0;// 红方起始朝向（蓝方自动镜像为 0.0°）
//
//    // 1. 炮台物理安装偏心、行程/限速与双阈值迟滞防抖参数
//    public static double TURRET_OFFSET_X = 0.0;             // 炮台旋转轴相对机器人中心的前后偏心（英寸，+向前，-向后）
//    public static double TURRET_OFFSET_Y = 0.0;             // 炮台旋转轴相对机器人中心的左右偏心（英寸，+向左，-向右）
//    public static double TURRET_MAX_ROTATION_DEG = 120.0;   // 臨時功能測試 （±120°）
//    //    public static double TURRET_MAX_ROTATION_DEG = 270.0;   // 最大允许旋转角度（±270°）
//    public static double TURRET_MAX_SPEED_DEG_PER_S = 540.0;// 目标角度最大变化速率（度/秒，0=不限速）
//
//    // 双阈值迟滞防抖（Hysteresis）：防止机器人在边界线附近走位时炮台反复切换
//    public static double MIN_SHOOT_RANGE_DISENGAGE_IN = 35.0; // 距离小于 35" 时进入近距离保持区
//    public static double MIN_SHOOT_RANGE_REENGAGE_IN  = 38.0; // 距离需退回 38" 以上才恢复动态追踪
//    public static double ENTRY_HALF_WIDTH_DISENGAGE   = 8.5;  // 纵深 12.04" 对应的退出半宽（±35.2°）
//    public static double ENTRY_HALF_WIDTH_REENGAGE    = 7.2;  // 纵深 12.04" 对应的重新进入半宽（±30.9°）
//
//    // 2. 全向底盘与航向锁定参数
//    public static double HEADING_KP = 0.95;                 // 航向锁定比例增益（若自转方向相反改为负数）
//    public static double HEADING_DEADBAND_DEG = 1.0;        // 航向死区（1° 以内不修正）
//    public static double SETTLE_SPEED_DEG_PER_S = 30.0;     // 松开摇杆后等待角速度降到此值以下再锁定航向
//    public static double STICK_DEADBAND = 0.01;             // 旋转摇杆死区
//
//    public float DriveTrains_ReducePOWER = 1.0f;
//    public float DriveTrains_smoothTurn = 1.0f;
//    public float intakePowerShoot = 0.90f;
//    public float intakePowerIntake = 0.95f;
//    public float intakePowerDump = -0.65f;
//    public float intakePowerOff = 0.0f;
//
//    public static final double blockageblockposition = 0.10;
//    public static final double blockagereleaseposition = 0.23;
////    public static final double blockagereleaseposition = 0.18;
//    public static final double HoodArmfarposition = 0.30;
//    public static final double HoodArmcloseposition = 0.35;
//
//    private static final double Far_SHOOTER_TARGET_Velocity = 1400;
//    private static double Med_SHOOTER_TARGET_SPEED = 1050;
//    public static double toleranceforShoot = 100;
//
//
////    public static class ShooterPIDFConfig {
////        public static double kP = 300.0;
////        public static double kI = 0.0;
////        public static double kD = 0.0;
////        public static double kF = 15.0;
////        public static double targetSPEED = Med_SHOOTER_TARGET_SPEED;
////        public static double flyWheelIdleSpeed = Med_SHOOTER_TARGET_SPEED * 0.6;
////        public static double targetSPEEDfar = Far_SHOOTER_TARGET_Velocity;
////        public static double flyWheelIdleSpeedfar = Far_SHOOTER_TARGET_Velocity * 0.6;
////    }
//
//    // 基于距离的自动仰角 (Hood) 与飞轮转速线性插值表（单位：英寸）
//    public static boolean AUTO_RANGE_SHOOT_ENABLED = true;
//    public static double SHOOTER_VELO_TRIM = 0.0;
//    public static double HOOD_POS_TRIM = 0.0;
//
//    private static final double[] DIST_TABLE_IN = { 35.0,  60.0,  85.0,  110.0, 140.0 };
//    private static final double[] HOOD_TABLE    = { 0.35,  0.42,  0.48,  0.53,  0.58  };
//    private static final double[] VELO_TABLE    = { 950.0, 1050.0, 1180.0, 1320.0, 1450.0 };
//
//    private double currentGoalDistanceIn = 0.0;
//    private double dynamicTargetVelo = 1050.0;
//    private double dynamicTargetHood = 0.50;
//
//    private final HardwareDecodeRobotBioBuzzSeason robot = new HardwareDecodeRobotBioBuzzSeason();
//    private final Gamepad currentGamepad1 = new Gamepad();
//    private final Gamepad currentGamepad2 = new Gamepad();
//    private final Gamepad previousGamepad1 = new Gamepad();
//    private final Gamepad previousGamepad2 = new Gamepad();
//
//    private Follower follower;
//    private final ElapsedTime imuResetTimer = new ElapsedTime();
//    private final ElapsedTime loopTimer = new ElapsedTime();
//    private final ElapsedTime turretRateTimer = new ElapsedTime();
//
//    // 状态标志与每帧缓存
//    private boolean isFarMode = false;
//    private boolean isShooterAtSpeed = false;
//    private boolean shooterIsOn = false;
//    private boolean intakeIsOn = false;
//    private boolean fireRequested = false;
//    private boolean imuResetInCooldown = false;
//    private static final long IMU_RESET_COOLDOWN_MS = 300;
//
//    // 炮台控制、迟滞状态与超限位变量
//    private boolean turretAutoAim = true;
//    private boolean isInTurretDeadZone = false;
//    private boolean wasAboveMinRange = false;
//    private boolean wasWithinEntryAngle = false;
//    private boolean turretReachable = true;
//    private boolean wasTurretReachable = true;
//    private double turretOverLimitDeg = 0.0;
//    private double turretManualTrimDeg = 0.0;
//    private double turretTargetDeg = 0.0;
//    private double smoothedTurretTargetDeg = 0.0;
//    // 航向锁定状态变量
//    private double holdHeading = 0.0;
//    private boolean holdHeadingSet = false;
//    private boolean turnedSinceHold = false;
//
//    // 支持在 Panels 网页端实时修改的炮台 PID 参数
////    public static double TURRET_KP = 0.018;
////    public static double TURRET_KI = 0.0000;
////    public static double TURRET_KD = 0.0016;
////    public static double TURRET_MAX_POWER = 0.5;
//    // 直接引用 Hardware 中的唯一基准参数作为初始值（仍支持 Panels 网页端实时微调并写回）
//    public static double TURRET_KP = HardwareDecodeRobotBioBuzzSeason.TURRET_KP;
//    public static double TURRET_KI = HardwareDecodeRobotBioBuzzSeason.TURRET_KI;
//    public static double TURRET_KD = HardwareDecodeRobotBioBuzzSeason.TURRET_KD;
//    public static double TURRET_MAX_POWER = HardwareDecodeRobotBioBuzzSeason.TURRET_MAX_POWER;
//
//
//    private double lastTurretKP = -1;
//    private double lastTurretKI = -1;
//    private double lastTurretKD = -1;
//    private double lastTurretMaxPower = -1;
//    // 缓存上一帧的飞轮 PIDF 参数，仅在 Panels 网页端数值变化时才触发下发
//    private double lastShooterKP = -1;
//    private double lastShooterKI = -1;
//    private double lastShooterKD = -1;
//    private double lastShooterKF = -1;
//
//    public static double TURRET_LOOKAHEAD_S = 0.08;
//    public static double TURRET_TRIM_LIMIT_DEG = 15.0;
//    public static double SHOT_FLIGHT_TIME_S = 0.0;
//    // 缓存速度与 LED 防抖
//    private double cachedShooterVeloL = 0.0;
//    private double cachedShooterVeloR = 0.0;
//    private String lastLedColor = "";
//
//    @Override
//    public void runOpMode() {
//
//
//        // 1. 绑定底层电机、舵机、LED 并开启 Bulk Caching
//        robot.init(hardwareMap);
//        robot.alliance = Alliance.RED; // 红方基准（蓝方改为 Alliance.BLUE）
//
//        // 2. 启动视觉与飞轮 PIDF
//        robot.initVision(ENABLE_CAMERA_LIVE_VIEW);
////        initShooterPIDF();
//        syncShooterPIDFFromPanels();
//
//        // 3. 创建 Pedro Pathing Follower（内部完成 Pinpoint 唯一一次硬件配置与 IMU 校准）
//        follower = Constants.create(hardwareMap);
//
//        // 4. 清空一次缓存后，再设定本场比赛的真实起始坐标与朝向（不会被覆盖）
//        robot.clearBulkCache();
//        Pose startPose = robot.toAlliancePose(RED_BASE_START_X, RED_BASE_START_Y, RED_BASE_START_HEADING_DEG);
//        follower.setPose(startPose);
//        follower.update();
//
//        // 5. 同步保存航向锁定基准角
//        holdHeading = follower.pose().heading();
//        holdHeadingSet = true;
//
////        robot.init(hardwareMap);
////
////        // =========================================================================
////        // 【赛前联盟选择】红方半场调试设 Alliance.RED；比赛分到蓝方改为 Alliance.BLUE 即可
////        // =========================================================================
////        robot.alliance = Alliance.RED;
//////        robot.alliance = Alliance.BLUE;
////
////        robot.initVision(ENABLE_CAMERA_LIVE_VIEW);
////        initShooterPIDF();
////
////        if (robot.axonTurretArmL != null) {
////            robot.axonTurretArmL.setTargetRotation(0.0);
////        }
////
////        follower = Constants.create(hardwareMap);
////        Pose startPose = robot.toAlliancePose(RED_BASE_START_X, RED_BASE_START_Y, RED_BASE_START_HEADING_DEG);
////        follower.setPose(startPose);
////        follower.update();
////
////        // 初始化航向锁定基准角
////        holdHeading = follower.pose().heading();
////        holdHeadingSet = true;
//
//        // 启用 JoinedTelemetry 实现手机端与 Panels 网页端双端自动同步
//        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
//        telemetry.addData("Status", "Ready | Alliance: " + robot.alliance.name());
//        telemetry.addData("Start Pose", "(%.1f, %.1f, %.0f°)",
//                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
//        telemetry.update();
//
//        waitForStart();
//        loopTimer.reset();
//        turretRateTimer.reset();
//
//        while (opModeIsActive()) {
//            // 1. 计算单帧 Loop Time 并刷新 REV Hub 批量缓存
//            double loopTimeMs = loopTimer.milliseconds();
//            loopTimer.reset();
//            robot.clearBulkCache();
//
//            // 2. 本轮主循环单次读取左右飞轮速度并缓存
//            cachedShooterVeloL = Math.abs(robot.MasterShooterMotorL.getVelocity());
//            cachedShooterVeloR = Math.abs(robot.SlaveShooterMotorR.getVelocity());
//
//            // 3. 更新手柄边沿状态
//            previousGamepad1.copy(currentGamepad1);
//            previousGamepad2.copy(currentGamepad2);
//            currentGamepad1.copy(gamepad1);
//            currentGamepad2.copy(gamepad2);
//
//            // 4. 更新机器人定位与底盘控制（含航向锁定）
//            follower.update();
//            updateDrivetrain_FieldCentric();
//            handleHeadingAndPositionReset();
//
//            // 5. 远近射模式切换（Gamepad2 X 切远射 / B 切近射）
//            if (currentGamepad2.x && !previousGamepad2.x) {
//                isFarMode = true;
//            } else if (currentGamepad2.b && !previousGamepad2.b) {
//                isFarMode = false;
//            }
//
//            // 6. 机构控制（全非阻塞边沿触发）
//            // 6. 机构控制（含飞轮 PIDF 网页端实时热更新）
//            syncShooterPIDFFromPanels();
//            updateIntake();
//            updateShooter();
//            checkShooterVelocity();
//            updateBlockage();
//            updateHood();
//            updateLEDs();
//
//            // 7. 炮台偏心解算、双阈值迟滞防抖、近距离保持当前角与闭环追踪
//            updateTurretControl();
//            robot.updateTurretAxons();
//            updateHiveStatus();
//
//            // 8. 统一双端刷新遥测
//            updateAllTelemetry(loopTimeMs);
//            telemetry.update();
//        }
//    }
//
//    private void updateTurretControl() {
//        // 1. 实时同步 Panels 网页端修改的炮台 PID 参数，并回写到 Hardware 全局基准
//        if (TURRET_KP != lastTurretKP || TURRET_KI != lastTurretKI || TURRET_KD != lastTurretKD) {
//            robot.axonTurretArmL.setPidCoeffs(TURRET_KP, TURRET_KI, TURRET_KD);
//            HardwareDecodeRobotBioBuzzSeason.TURRET_KP = TURRET_KP;
//            HardwareDecodeRobotBioBuzzSeason.TURRET_KI = TURRET_KI;
//            HardwareDecodeRobotBioBuzzSeason.TURRET_KD = TURRET_KD;
//            lastTurretKP = TURRET_KP;
//            lastTurretKI = TURRET_KI;
//            lastTurretKD = TURRET_KD;
//        }
//        if (TURRET_MAX_POWER != lastTurretMaxPower) {
//            robot.axonTurretArmL.setMaxPower(TURRET_MAX_POWER);
//            HardwareDecodeRobotBioBuzzSeason.TURRET_MAX_POWER = TURRET_MAX_POWER;
//            lastTurretMaxPower = TURRET_MAX_POWER;
//        }
//
//// 2. Gamepad2 Back 键切换自动跟踪 / 手动回中模式
//        if (currentGamepad2.back && !previousGamepad2.back) {
//            turretAutoAim = !turretAutoAim;
//        }
//// 3. Gamepad2 左右方向键提供 ±2° 实时微调补偿
//        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
//            turretManualTrimDeg += 2.0;
//        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
//            turretManualTrimDeg -= 2.0;
//        }
//        turretManualTrimDeg = Range.clip(turretManualTrimDeg, -TURRET_TRIM_LIMIT_DEG, TURRET_TRIM_LIMIT_DEG);
//        Pose p = follower.pose();
//        Velocity v = follower.velocity();
//        double h = p.heading();
//        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
//// 4. 计算炮台旋转轴在场地上的真实偏心坐标 (pivotX, pivotY)
//        double pivotX = p.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h);
//        double pivotY = p.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h);
//        double targetX = robot.toAllianceX(RED_BASE_GOAL_X);
//        double targetY = robot.toAllianceY(RED_BASE_GOAL_Y);
//        if (SHOT_FLIGHT_TIME_S > 0) {
//            targetX -= v.vx * SHOT_FLIGHT_TIME_S;
//            targetY -= v.vy * SHOT_FLIGHT_TIME_S;
//        }
//        double dx = targetX - pivotX;
//        double dy = targetY - pivotY;
//        double rangeSq = dx * dx + dy * dy;
//        currentGoalDistanceIn = Math.sqrt(rangeSq);
//        dynamicTargetHood = interpolateTable(DIST_TABLE_IN, HOOD_TABLE, currentGoalDistanceIn) + HOOD_POS_TRIM;
//        dynamicTargetVelo = interpolateTable(DIST_TABLE_IN, VELO_TABLE, currentGoalDistanceIn) + SHOOTER_VELO_TRIM;
//// 5. 双阈值迟滞防抖（Hysteresis）有效射击扇形环带判断
//        boolean inFrontOfOpening = robot.isInFrontOfGoal(dy);
//        double minRangeLimit = wasAboveMinRange ? MIN_SHOOT_RANGE_DISENGAGE_IN : MIN_SHOOT_RANGE_REENGAGE_IN;
//        boolean aboveMinRange = (rangeSq >= minRangeLimit * minRangeLimit);
//        wasAboveMinRange = aboveMinRange;
//        double maxTanLimit = wasWithinEntryAngle
//                ? (ENTRY_HALF_WIDTH_DISENGAGE / 12.04)
//                : (ENTRY_HALF_WIDTH_REENGAGE / 12.04);
//        boolean withinEntryAngle = Math.abs(dx) <= Math.abs(dy) * maxTanLimit;
//        wasWithinEntryAngle = withinEntryAngle;
//        boolean isValidShootingZone = inFrontOfOpening && aboveMinRange && withinEntryAngle;
//        isInTurretDeadZone = !isValidShootingZone;
//// 6. 分情况决定目标角度（结合近距离 Hold Last Angle 机制）
//        double desiredTargetDeg;
//        if (!turretAutoAim || !inFrontOfOpening || !withinEntryAngle) {
//// 情况 A：手动关闭自瞄、位于容器背面、或两侧大偏角超出有效张角 -> 平滑回正到 0°
//            desiredTargetDeg = 0.0;
//            turretReachable = true;
//            turretOverLimitDeg = 0.0;
//        } else if (!aboveMinRange) {
//// 情况 B：位于容器正面且张角合法，但太靠近目标 (<35") -> 保持进入盲区前的最后一个合法角度（Hold Last Angle）
//            desiredTargetDeg = turretTargetDeg;
//            turretReachable = true;
//            turretOverLimitDeg = 0.0;
//        } else {
//// 情况 C：完全处于有效射击扇形环带内 -> 执行实时动态跟踪与前馈补偿
//            double relAngle = Math.atan2(dy, dx) - h;
//            while (relAngle >  Math.PI) relAngle -= 2 * Math.PI;
//            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
//            double bearingRate = (rangeSq > 1e-6) ? (dy * v.vx - dx * v.vy) / rangeSq : 0.0;
//            double turretRate  = bearingRate - v.omega;
//            double baseTargetDeg = Math.toDegrees(relAngle + turretRate * TURRET_LOOKAHEAD_S)
//                    + turretManualTrimDeg;
//            double candidateDeg = baseTargetDeg;
//            if (candidateDeg - currentTurretDeg > 180.0
//                    && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
//                candidateDeg -= 360.0;
//            } else if (candidateDeg - currentTurretDeg < -180.0
//                    && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
//                candidateDeg += 360.0;
//            }
//// 超限位检测与单次手柄震动提醒
//            turretReachable = Math.abs(candidateDeg) <= TURRET_MAX_ROTATION_DEG;
//            if (!turretReachable) {
//                turretOverLimitDeg = (candidateDeg > TURRET_MAX_ROTATION_DEG)
//                        ? (candidateDeg - TURRET_MAX_ROTATION_DEG)
//                        : (candidateDeg + TURRET_MAX_ROTATION_DEG);
//            } else {
//                turretOverLimitDeg = 0.0;
//            }
//            if (wasTurretReachable && !turretReachable) {
//                gamepad1.rumble(300);
//            }
//            wasTurretReachable = turretReachable;
//            desiredTargetDeg = Range.clip(candidateDeg, -TURRET_MAX_ROTATION_DEG, TURRET_MAX_ROTATION_DEG);
//        }
//        turretTargetDeg = desiredTargetDeg;
//// 7. 目标角度变化率平滑限速
//        double dt = turretRateTimer.seconds();
//        turretRateTimer.reset();
//        if (TURRET_MAX_SPEED_DEG_PER_S > 0 && dt > 0.0 && dt < 0.5) {
//            double maxStep = TURRET_MAX_SPEED_DEG_PER_S * dt;
//            double delta = Range.clip(turretTargetDeg - smoothedTurretTargetDeg, -maxStep, maxStep);
//            smoothedTurretTargetDeg += delta;
//        } else {
//            smoothedTurretTargetDeg = turretTargetDeg;
//        }
//// 8. 下发平滑后的目标角度给主 Axon 连续旋转舵机
//        robot.axonTurretArmL.trackTargetRotation(smoothedTurretTargetDeg);
//// 9. 纯数字遥测输出（Panels Graph 可直接画图）
//        telemetry.addData("Turret Target (deg)", smoothedTurretTargetDeg);
//        telemetry.addData("Turret Actual (deg)", currentTurretDeg);
//        telemetry.addData("Turret Error (deg)", smoothedTurretTargetDeg - currentTurretDeg);
//        telemetry.addData("Turret Servo Power", robot.axonTurretArmL.getPower());
//        telemetry.addData("Turret Valid Zone", isValidShootingZone ? 1 : 0);
//        telemetry.addData("Turret Reachable", turretReachable ? 1 : 0);
//        if (!turretReachable) {
//            telemetry.addData("TURN ROBOT WARNING", "Turn %.0f deg %s!",
//                    Math.abs(turretOverLimitDeg), turretOverLimitDeg > 0 ? "LEFT" : "RIGHT");
//        }
//    }
//    public void updateDrivetrain_FieldCentric() {
//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x * DriveTrains_smoothTurn;
//        Pose pose = follower.pose();
//        Velocity vel = follower.velocity();
//        double botHeading = pose.heading();
//        double spinSpeed = Math.abs(vel.omega);
//        boolean turning = Math.abs(gamepad1.right_stick_x) > STICK_DEADBAND;
//        boolean settled = spinSpeed < Math.toRadians(SETTLE_SPEED_DEG_PER_S);
//        if (turning) {
//            turnedSinceHold = true;
//            if (!settled) {
//                holdHeadingSet = false;
//            }
//        }
//        if (settled && turnedSinceHold) {
//            holdHeading = botHeading;
//            holdHeadingSet = true;
//            turnedSinceHold = false;
//        }
//        if (!turning && holdHeadingSet) {
//            double headingError = Math.atan2(Math.sin(holdHeading - botHeading), Math.cos(holdHeading - botHeading));
//            if (Math.abs(headingError) > Math.toRadians(HEADING_DEADBAND_DEG)) {
//                rx = -HEADING_KP * headingError;
//            }
//        }
//        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
//        double frontLeftPower  = (rotY + rotX + rx) / denominator;
//        double backLeftPower   = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower  = (rotY + rotX - rx) / denominator;
//        robot.leftFrontMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
//        robot.leftRearMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
//        robot.rightFrontMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
//        robot.rightRearMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
//    }
//    private void handleHeadingAndPositionReset() {
//        if (imuResetInCooldown && imuResetTimer.milliseconds() > IMU_RESET_COOLDOWN_MS) {
//            imuResetInCooldown = false;
//        }
//// 1. Back + Start 同时按下：自动重置回当前联盟的预设起始位姿
//        if (currentGamepad1.back && currentGamepad1.start) {
//            Pose resetPose = robot.toAlliancePose(RED_BASE_START_X, RED_BASE_START_Y, RED_BASE_START_HEADING_DEG);
//            follower.setPose(resetPose);
//            holdHeading = resetPose.heading();
//            holdHeadingSet = true;
//            gamepad1.rumble(300);
//        }
//// 2. 单独按 Back：继承自动阶段结束坐标
//        else if (currentGamepad1.back && !previousGamepad1.back) {
//            if (robot.finalAutoX != 0 || robot.finalAutoY != 0) {
//                follower.setPose(new Pose(robot.finalAutoX, robot.finalAutoY, robot.finalAutoHeading));
//                holdHeading = robot.finalAutoHeading;
//                holdHeadingSet = true;
//            }
//        }
//// 3. 单独按 Start（无 Back）：自动重置为当前联盟的基准车头朝向（红方 180°，蓝方 0°）
//        else if (currentGamepad1.start && !previousGamepad1.start && !imuResetInCooldown) {
//            double homeHeadingRad = Math.toRadians(robot.toAllianceHeadingDeg(RED_BASE_START_HEADING_DEG));
//            follower.setHeading(homeHeadingRad);
//            holdHeading = homeHeadingRad;
//            holdHeadingSet = true;
//            imuResetInCooldown = true;
//            imuResetTimer.reset();
//            gamepad1.rumble(200);
//        }
//    }
////    public void initShooterPIDF() {
////        robot.shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        PIDFCoefficients pidf = new PIDFCoefficients(
////                ShooterPIDFConfig.kP,
////                ShooterPIDFConfig.kI,
////                ShooterPIDFConfig.kD,
////                ShooterPIDFConfig.kF
////        );
////        robot.shooterL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
////        robot.shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
////        robot.shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////    }
//    /**
//     * 实时检测 Panels 网页端是否修改了 ShooterPIDFConfig，若有变化则热更新到左右飞轮电机
//     */
//    private void syncShooterPIDFFromPanels() {
//        if (ShooterPIDFConfig.kP != lastShooterKP
//                || ShooterPIDFConfig.kI != lastShooterKI
//                || ShooterPIDFConfig.kD != lastShooterKD
//                || ShooterPIDFConfig.kF != lastShooterKF) {
//
//            PIDFCoefficients updatedPidf = new PIDFCoefficients(
//                    ShooterPIDFConfig.kP,
//                    ShooterPIDFConfig.kI,
//                    ShooterPIDFConfig.kD,
//                    ShooterPIDFConfig.kF
//            );
//            robot.shooterL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, updatedPidf);
//            robot.shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, updatedPidf);
//
//            lastShooterKP = ShooterPIDFConfig.kP;
//            lastShooterKI = ShooterPIDFConfig.kI;
//            lastShooterKD = ShooterPIDFConfig.kD;
//            lastShooterKF = ShooterPIDFConfig.kF;
//        }
//    }
//
//    public void updateIntake() {
//        if (currentGamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger <= 0.1) {
//            robot.BlockageArmL.setPosition(blockageblockposition);
//            robot.BlockageArmR.setPosition(blockageblockposition);
//            robot.IntakeMotorL.setPower(intakePowerIntake);
//            robot.IntakeMotorR.setPower(intakePowerIntake);
//            intakeIsOn = true;
//        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
//            robot.IntakeMotorL.setPower(intakePowerDump);
//            robot.IntakeMotorR.setPower(intakePowerDump);
//            intakeIsOn = true;
//        } else if (currentGamepad1.y && !previousGamepad1.y) {
//            stopIntake();
//        }
//    }
//    private void stopIntake() {
//        robot.IntakeMotorL.setPower(intakePowerOff);
//        robot.IntakeMotorR.setPower(intakePowerOff);
//        intakeIsOn = false;
//        isShooterAtSpeed = false;
//        fireRequested = false;
//        if (isFarMode) {
//            startShooterIdleFar();
//        } else {
//            startShooterIdle();
//        }
//    }
//    public void updateShooter() {
//        if (currentGamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger <= 0.1) {
//            if (isFarMode) {
//                startShooterFar();
//            } else {
//                startShooter();
//            }
//        }
//        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
//            fireRequested = true;
//            robot.BlockageArmL.setPosition(blockagereleaseposition);
//            robot.BlockageArmR.setPosition(blockagereleaseposition);
//            robot.IntakeMotorL.setPower(intakePowerShoot);
//            robot.IntakeMotorR.setPower(intakePowerShoot);
//            intakeIsOn = true;
//        }
//        if (currentGamepad1.b && !previousGamepad1.b) {
//            stopShooter();
//            robot.BlockageArmL.setPosition(blockageblockposition);
//            robot.BlockageArmR.setPosition(blockageblockposition);
//        }
//        if (currentGamepad1.a && !previousGamepad1.a) {
//            stopIntakeAndShooter();
//        }
//        // 若飞轮正在运行且处于手动固定转速模式，允许网页端直接修改 targetSPEED 实时调速
//        if (shooterIsOn && !AUTO_RANGE_SHOOT_ENABLED) {
//            double manualTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
//            robot.shooterL.setVelocity(Math.abs(manualTarget));
//            robot.shooterR.setVelocity(Math.abs(manualTarget));
//        }
//    }
//    private void startShooterIdle() {
//        robot.IntakeMotorL.setPower(0);
//        robot.IntakeMotorR.setPower(0);
//        robot.setHoodPosition(0.5);
//        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeed));
//        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeed));
//    }
//    private void startShooterIdleFar() {
//        robot.IntakeMotorL.setPower(0);
//        robot.IntakeMotorR.setPower(0);
//        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeedfar));
//        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeedfar));
//    }
//    private void startShooter() {
//        robot.IntakeMotorL.setPower(0);
//        robot.IntakeMotorR.setPower(0);
//        if (AUTO_RANGE_SHOOT_ENABLED) {
//            robot.setHoodPosition(dynamicTargetHood);
//            robot.shooterL.setVelocity(Math.abs(dynamicTargetVelo));
//            robot.shooterR.setVelocity(Math.abs(dynamicTargetVelo));
//        } else {
//            robot.setHoodPosition(0.50);
//            robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
//            robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
//        }
//        shooterIsOn = true;
//    }
//    private void startShooterFar() {
//        robot.IntakeMotorL.setPower(0);
//        robot.IntakeMotorR.setPower(0);
//        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEEDfar));
//        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEEDfar));
//        shooterIsOn = true;
//    }
//    private void stopShooter() {
//        if (isFarMode) {
//            startShooterIdleFar();
//        } else {
//            startShooterIdle();
//        }
//        intakeIsOn = false;
//        isShooterAtSpeed = false;
//        fireRequested = false;
//        shooterIsOn = false;
//    }
//    private void stopIntakeAndShooter() {
//        robot.MasterShooterMotorL.setVelocity(0);
//        robot.SlaveShooterMotorR.setVelocity(0);
//        robot.IntakeMotorL.setPower(0);
//        robot.IntakeMotorR.setPower(0);
//        intakeIsOn = false;
//        isShooterAtSpeed = false;
//        fireRequested = false;
//        shooterIsOn = false;
//    }
//    private void checkShooterVelocity() {
//        double activeTarget = AUTO_RANGE_SHOOT_ENABLED
//                ? dynamicTargetVelo
//                : (isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED);
//        if (Math.abs(cachedShooterVeloL - activeTarget) <= toleranceforShoot) {
//            isShooterAtSpeed = true;
//        } else {
//            isShooterAtSpeed = false;
//            fireRequested = false;
//        }
//    }
//    private void updateHood() {
//        if (currentGamepad2.start && !previousGamepad2.start) {
//            AUTO_RANGE_SHOOT_ENABLED = !AUTO_RANGE_SHOOT_ENABLED;
//        }
//        if (AUTO_RANGE_SHOOT_ENABLED && shooterIsOn) {
//            robot.setHoodPosition(dynamicTargetHood);
//            double targetV = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : dynamicTargetVelo;
//            robot.shooterL.setVelocity(Math.abs(targetV));
//            robot.shooterR.setVelocity(Math.abs(targetV));
//        } else if (!AUTO_RANGE_SHOOT_ENABLED) {
//            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
//                robot.setHoodPosition(HoodArmfarposition);
//            } else if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
//                robot.setHoodPosition(HoodArmcloseposition);
//            } else if (currentGamepad2.y && !previousGamepad2.y) {
//                robot.setHoodPosition(0.50);
//            }
//        }
//    }
//    private void updateBlockage() {
//        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
//            robot.BlockageArmL.setPosition(blockageblockposition);
//            robot.BlockageArmR.setPosition(blockageblockposition);
//        } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
//            robot.BlockageArmL.setPosition(blockagereleaseposition);
//            robot.BlockageArmR.setPosition(blockagereleaseposition);
//        }
//    }
//    private void updateLEDs() {
//        if (isShooterAtSpeed && !isInTurretDeadZone && turretReachable && robot.axonTurretArmL.isAtTarget(4.0)) {
//            setLEDColor("GREEN");
//        } else {
//            setLEDColor("RED");
//        }
//    }
//    private void setLEDColor(String color) {
//        if (color.equals(lastLedColor)) return;
//        try {
//            if ("GREEN".equals(color)) {
//                if (robot.greenLED != null) robot.greenLED.setState(false);
//                if (robot.redLED != null) robot.redLED.setState(true);
//            } else if ("RED".equals(color)) {
//                if (robot.greenLED != null) robot.greenLED.setState(true);
//                if (robot.redLED != null) robot.redLED.setState(false);
//            }
//            lastLedColor = color;
//        } catch (Exception e) {
//            telemetry.addData("LED Error", e.getMessage());
//        }
//    }
//    private void updateHiveStatus() {
//        if (robot.myAprilTagProcessor == null) return;
//        robot.hiveScorable = false;
//        robot.hiveClusterName = "";
//        robot.hiveRoll = 0.0;
//        robot.hivePercentFound = 0.0;
//        for (AprilTagDetection d : robot.myAprilTagProcessor.getDetections()) {
//            double rawRoll = (d.ftcPose != null) ? d.ftcPose.roll : 0.0;
//            if (d instanceof AprilTagClusterDetection) {
//                AprilTagClusterDetection cluster = (AprilTagClusterDetection) d;
//                String name = (cluster.metadata != null && cluster.metadata.name != null)
//                        ? cluster.metadata.name : "Unknown_Cluster";
//                boolean allianceMatch = name.startsWith(robot.getAllianceTagPrefix());
//                if (!allianceMatch || Math.abs(rawRoll) >= 90.0 || cluster.percentClusterFound < 0.5) continue;
//                robot.hiveScorable = true;
//                robot.hiveClusterName = name;
//                robot.hiveRoll = rawRoll;
//                robot.hivePercentFound = cluster.percentClusterFound;
//                break;
//            } else if (d instanceof AprilTagSingleDetection) {
//                AprilTagSingleDetection single = (AprilTagSingleDetection) d;
//                String name = (single.metadata != null && single.metadata.name != null)
//                        ? single.metadata.name : ("ID_" + single.id);
//                robot.hiveScorable = true;
//                robot.hiveClusterName = name;
//                robot.hiveRoll = rawRoll;
//                robot.hivePercentFound = 1.0;
//                break;
//            }
//        }
//    }
//    private double interpolateTable(double[] xTable, double[] yTable, double x) {
//        int n = xTable.length;
//        if (x <= xTable[0]) return yTable[0];
//        if (x >= xTable[n - 1]) return yTable[n - 1];
//        for (int i = 0; i < n - 1; i++) {
//            if (x >= xTable[i] && x <= xTable[i + 1]) {
//                double ratio = (x - xTable[i]) / (xTable[i + 1] - xTable[i]);
//                return yTable[i] + ratio * (yTable[i + 1] - yTable[i]);
//            }
//        }
//        return yTable[n - 1];
//    }
//    private void updateAllTelemetry(double loopTimeMs) {
//        Pose pose = follower.pose();
//        double activeTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
//// 纯数字序列（可直接用于 Panels Graph 折线图）
//        telemetry.addData("Loop Time (ms)", loopTimeMs);
//        telemetry.addData("Shooter Target", activeTarget);
//        telemetry.addData("Shooter Left Velo", cachedShooterVeloL);
//        telemetry.addData("Shooter Right Velo", cachedShooterVeloR);
//        telemetry.addData("Hive Scorable", robot.hiveScorable ? 1 : 0);
//        telemetry.addData("Goal Distance (in)", currentGoalDistanceIn);
//        telemetry.addData("Dynamic Velo Target", dynamicTargetVelo);
//        telemetry.addData("Dynamic Hood Target", dynamicTargetHood);
//        telemetry.addData("Blocker L Pos", robot.BlockageArmL.getPosition());
//        telemetry.addData("Blocker R Pos", robot.BlockageArmR.getPosition());
//
//// 状态摘要文字
//        telemetry.addData("Alliance", robot.alliance.name());
//        telemetry.addData("Shoot Mode", isFarMode ? "FAR" : "NEAR");
//        telemetry.addData("Pose (X/Y/H)", "%.1f / %.1f / %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
//        telemetry.addData("Hold Heading", "%.1f° (Active:%b)", Math.toDegrees(holdHeading), holdHeadingSet);
//        telemetry.addData("Hive Cluster", robot.hiveClusterName);
//    }
//}