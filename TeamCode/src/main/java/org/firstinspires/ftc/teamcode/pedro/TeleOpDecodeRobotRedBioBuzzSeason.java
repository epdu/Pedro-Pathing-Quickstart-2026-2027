package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Configurable
@TeleOp(name = "AAAAAA BioBuzz TeleOp (Red-Base Auto Symmetry)")
public class TeleOpDecodeRobotRedBioBuzzSeason extends LinearOpMode {

    // 调试/比赛摄像头预览开关（正式比赛建议改为 false 节省 CPU）
    public static boolean ENABLE_CAMERA_LIVE_VIEW = true;

    // 红方实测基准坐标（当 robot.alliance = Alliance.BLUE 时，对称引擎自动镜像为蓝方对应坐标）
    public static double RED_BASE_GOAL_X = 58.0;            // 红方目标 X（蓝方自动镜像为 83.5）
    public static double RED_BASE_GOAL_Y = 62.0;            // 红方目标 Y（蓝方自动镜像为 79.5）
    public static double RED_BASE_START_X = 58.0;           // 红方起始 X（蓝方自动镜像为 83.5）
    public static double RED_BASE_START_Y = 8.0;            // 红方起始 Y（蓝方自动镜像为 133.5）
    public static double RED_BASE_START_HEADING_DEG = 180.0;// 红方起始朝向（蓝方自动镜像为 0.0°）

    // 1. 炮台物理安装偏心与行程/限速参数
    public static double TURRET_OFFSET_X = 0.0;             // 炮台旋转轴相对机器人中心的前后偏心（英寸，+向前，-向后）
    public static double TURRET_OFFSET_Y = 0.0;             // 炮台旋转轴相对机器人中心的左右偏心（英寸，+向左，-向右）
    public static double TURRET_MAX_ROTATION_DEG = 270.0;   // 最大允许旋转角度（±270°）
    public static double TURRET_MAX_SPEED_DEG_PER_S = 540.0;// 目标角度最大变化速率（度/秒，0=不限速）
    public static double MIN_SHOOT_RANGE_IN = 35.0;         // 最小有效射程（英寸，防止仰角过陡撞顶棚或下沿）

    // 2. 全向底盘与航向锁定参数
    public static double HEADING_KP = 0.95;                 // 航向锁定比例增益（若自转方向相反改为负数）
    public static double HEADING_DEADBAND_DEG = 1.0;        // 航向死区（1° 以内不修正）
    public static double SETTLE_SPEED_DEG_PER_S = 30.0;     // 松开摇杆后等待角速度降到此值以下再锁定航向
    public static double STICK_DEADBAND = 0.01;             // 旋转摇杆死区

    public float DriveTrains_ReducePOWER = 1.0f;
    public float DriveTrains_smoothTurn = 1.0f;
    public float intakePowerShoot = 0.90f;
    public float intakePowerIntake = 0.95f;
    public float intakePowerDump = -0.65f;
    public float intakePowerOff = 0.0f;

    public static final double blockageblockposition = 0.10;
    public static final double blockagereleaseposition = 0.18;
    public static final double HoodArmfarposition = 0.30;
    public static final double HoodArmcloseposition = 0.35;

    private static final double Far_SHOOTER_TARGET_Velocity = 1400;
    private static double Med_SHOOTER_TARGET_SPEED = 1050;
    public static double toleranceforShoot = 100;

    @Configurable
    public static class ShooterPIDFConfig {
        public static double kP = 300.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 15.0;
        public static double targetSPEED = Med_SHOOTER_TARGET_SPEED;
        public static double flyWheelIdleSpeed = Med_SHOOTER_TARGET_SPEED * 0.6;
        public static double targetSPEEDfar = Far_SHOOTER_TARGET_Velocity;
        public static double flyWheelIdleSpeedfar = Far_SHOOTER_TARGET_Velocity * 0.6;
    }

    // 基于距离的自动仰角 (Hood) 与飞轮转速线性插值表（单位：英寸）
    public static boolean AUTO_RANGE_SHOOT_ENABLED = true;
    public static double SHOOTER_VELO_TRIM = 0.0;
    public static double HOOD_POS_TRIM = 0.0;

    private static final double[] DIST_TABLE_IN = { 35.0,  60.0,  85.0,  110.0, 140.0 };
    private static final double[] HOOD_TABLE    = { 0.35,  0.42,  0.48,  0.53,  0.58  };
    private static final double[] VELO_TABLE    = { 950.0, 1050.0, 1180.0, 1320.0, 1450.0 };

    private double currentGoalDistanceIn = 0.0;
    private double dynamicTargetVelo = 1050.0;
    private double dynamicTargetHood = 0.50;

    private final HardwareDecodeRobotBioBuzzSeason robot = new HardwareDecodeRobotBioBuzzSeason();
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    private Follower follower;
    private final ElapsedTime imuResetTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime turretRateTimer = new ElapsedTime();

    // 状态标志与每帧缓存
    private boolean isFarMode = false;
    private boolean isShooterAtSpeed = false;
    private boolean shooterIsOn = false;
    private boolean intakeIsOn = false;
    private boolean fireRequested = false;
    private boolean imuResetInCooldown = false;
    private static final long IMU_RESET_COOLDOWN_MS = 300;

    // 炮台控制与超限位变量
    private boolean turretAutoAim = true;
    private boolean isInTurretDeadZone = false;
    private boolean turretReachable = true;
    private boolean wasTurretReachable = true;
    private double turretOverLimitDeg = 0.0;
    private double turretManualTrimDeg = 0.0;
    private double turretTargetDeg = 0.0;
    private double smoothedTurretTargetDeg = 0.0;

    // 航向锁定状态变量
    private double holdHeading = 0.0;
    private boolean holdHeadingSet = false;
    private boolean turnedSinceHold = false;

    // 支持在 Panels 网页端实时修改的炮台 PID 参数
    public static double TURRET_KP = 0.0069;
    public static double TURRET_KI = 0.0000;
    public static double TURRET_KD = 0.0002;
    public static double TURRET_MAX_POWER = 0.5;

    private double lastTurretKP = -1;
    private double lastTurretKI = -1;
    private double lastTurretKD = -1;
    private double lastTurretMaxPower = -1;
    public static double TURRET_LOOKAHEAD_S = 0.08;
    public static double TURRET_TRIM_LIMIT_DEG = 15.0;
    public static double SHOT_FLIGHT_TIME_S = 0.0;

    // 缓存速度与 LED 防抖
    private double cachedShooterVeloL = 0.0;
    private double cachedShooterVeloR = 0.0;
    private String lastLedColor = "";

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // =========================================================================
        // 【赛前联盟选择】红方半场调试设 Alliance.RED；比赛分到蓝方改为 Alliance.BLUE 即可
        // =========================================================================
        robot.alliance = Alliance.RED;
//        robot.alliance = Alliance.BLUE;
        robot.initVision(ENABLE_CAMERA_LIVE_VIEW);
        initShooterPIDF();

        if (robot.axonTurretArmL != null) {
            robot.axonTurretArmL.setTargetRotation(0.0);
        }

        follower = Constants.create(hardwareMap);
        // 自动根据 robot.alliance 生成对应联盟的起始坐标与朝向
        Pose startPose = robot.toAlliancePose(RED_BASE_START_X, RED_BASE_START_Y, RED_BASE_START_HEADING_DEG);
        follower.setPose(startPose);
        follower.update();

        // 初始化航向锁定基准角
        holdHeading = follower.pose().heading();
        holdHeadingSet = true;

        // 启用 JoinedTelemetry 实现手机端与 Panels 网页端双端自动同步
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        telemetry.addData("Status", "Ready | Alliance: " + robot.alliance.name());
        telemetry.addData("Start Pose", "(%.1f, %.1f, %.0f°)",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.update();

        waitForStart();
        loopTimer.reset();
        turretRateTimer.reset();

        while (opModeIsActive()) {
            // 1. 计算单帧 Loop Time 并刷新 REV Hub 批量缓存
            double loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();
            robot.clearBulkCache();

            // 2. 本轮主循环单次读取左右飞轮速度并缓存
            cachedShooterVeloL = Math.abs(robot.MasterShooterMotorL.getVelocity());
            cachedShooterVeloR = Math.abs(robot.SlaveShooterMotorR.getVelocity());

            // 3. 更新手柄边沿状态
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // 4. 更新机器人定位与底盘控制（含航向锁定）
            follower.update();
            updateDrivetrain_FieldCentric();
            handleHeadingAndPositionReset();

            // 5. 远近射模式切换（Gamepad2 X 切远射 / B 切近射）
            if (currentGamepad2.x && !previousGamepad2.x) {
                isFarMode = true;
            } else if (currentGamepad2.b && !previousGamepad2.b) {
                isFarMode = false;
            }

            // 6. 机构控制（全非阻塞边沿触发）
            updateIntake();
            updateShooter();
            checkShooterVelocity();
            updateBlockage();
            updateHood();
            updateLEDs();

            // 7. 炮台偏心解算、对称目标获取、有效射击扇形区判断与闭环追踪
            updateTurretControl();
            robot.updateTurretAxons();
            updateHiveStatus();

            // 8. 统一双端刷新遥测
            updateAllTelemetry(loopTimeMs);
            telemetry.update();
        }
    }

    private void updateTurretControl() {
        // 1. 实时同步 Panels 网页端修改的炮台 PID 参数
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
// 2. Gamepad2 Back 键切换自动跟踪 / 手动回中模式
        if (currentGamepad2.back && !previousGamepad2.back) {
            turretAutoAim = !turretAutoAim;
        }
// 3. Gamepad2 左右方向键提供 ±2° 实时微调补偿
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
// 4. 计算炮台旋转轴在场地上的真实偏心坐标 (pivotX, pivotY)
        double pivotX = p.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h);
        double pivotY = p.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h);
// 通过对称引擎自动获取当前联盟的目标点坐标
        double targetX = robot.toAllianceX(RED_BASE_GOAL_X);
        double targetY = robot.toAllianceY(RED_BASE_GOAL_Y);
        if (SHOT_FLIGHT_TIME_S > 0) {
            targetX -= v.vx * SHOT_FLIGHT_TIME_S;
            targetY -= v.vy * SHOT_FLIGHT_TIME_S;
        }
        double dx = targetX - pivotX;
        double dy = targetY - pivotY;
        double rangeSq = dx * dx + dy * dy;
        currentGoalDistanceIn = Math.sqrt(rangeSq);
        dynamicTargetHood = interpolateTable(DIST_TABLE_IN, HOOD_TABLE, currentGoalDistanceIn) + HOOD_POS_TRIM;
        dynamicTargetVelo = interpolateTable(DIST_TABLE_IN, VELO_TABLE, currentGoalDistanceIn) + SHOOTER_VELO_TRIM;
// 5. 有效射击扇形环带判断（基于 12.04 in. 真实纵深，红蓝自动适配正反面）
        boolean inFrontOfOpening = robot.isInFrontOfGoal(dy);
        boolean aboveMinRange = (rangeSq >= MIN_SHOOT_RANGE_IN * MIN_SHOOT_RANGE_IN);
        boolean withinEntryAngle = Math.abs(dx) <= Math.abs(dy) * (8.5 / 12.04);
        boolean isValidShootingZone = inFrontOfOpening && aboveMinRange && withinEntryAngle;
        isInTurretDeadZone = !isValidShootingZone;
        double desiredTargetDeg;
        if (isInTurretDeadZone || !turretAutoAim) {
            desiredTargetDeg = 0.0;
            turretReachable = true;
            turretOverLimitDeg = 0.0;
        } else {
            double relAngle = Math.atan2(dy, dx) - h;
            while (relAngle >  Math.PI) relAngle -= 2 * Math.PI;
            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
            double bearingRate = (rangeSq > 1e-6) ? (dy * v.vx - dx * v.vy) / rangeSq : 0.0;
            double turretRate  = bearingRate - v.omega;
            double baseTargetDeg = Math.toDegrees(relAngle + turretRate * TURRET_LOOKAHEAD_S)
                    + turretManualTrimDeg;
            double candidateDeg = baseTargetDeg;
            if (candidateDeg - currentTurretDeg > 180.0
                    && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
                candidateDeg -= 360.0;
            } else if (candidateDeg - currentTurretDeg < -180.0
                    && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
                candidateDeg += 360.0;
            }
// 6. 超限位检测与单次手柄震动提醒
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
// 7. 目标角度变化率平滑限速
        double dt = turretRateTimer.seconds();
        turretRateTimer.reset();
        if (TURRET_MAX_SPEED_DEG_PER_S > 0 && dt > 0.0 && dt < 0.5) {
            double maxStep = TURRET_MAX_SPEED_DEG_PER_S * dt;
            double delta = Range.clip(turretTargetDeg - smoothedTurretTargetDeg, -maxStep, maxStep);
            smoothedTurretTargetDeg += delta;
        } else {
            smoothedTurretTargetDeg = turretTargetDeg;
        }
// 8. 下发平滑后的目标角度给主 Axon 连续旋转舵机
        robot.axonTurretArmL.trackTargetRotation(smoothedTurretTargetDeg);
// 9. 纯数字遥测输出（Panels Graph 可直接画图）
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
// 1. Back + Start 同时按下：自动重置回当前联盟的预设起始位姿
        if (currentGamepad1.back && currentGamepad1.start) {
            Pose resetPose = robot.toAlliancePose(RED_BASE_START_X, RED_BASE_START_Y, RED_BASE_START_HEADING_DEG);
            follower.setPose(resetPose);
            holdHeading = resetPose.heading();
            holdHeadingSet = true;
            gamepad1.rumble(300);
        }
// 2. 单独按 Back：继承自动阶段结束坐标
        else if (currentGamepad1.back && !previousGamepad1.back) {
            if (robot.finalAutoX != 0 || robot.finalAutoY != 0) {
                follower.setPose(new Pose(robot.finalAutoX, robot.finalAutoY, robot.finalAutoHeading));
                holdHeading = robot.finalAutoHeading;
                holdHeadingSet = true;
            }
        }
// 3. 单独按 Start（无 Back）：自动重置为当前联盟的基准车头朝向（红方 180°，蓝方 0°）
        else if (currentGamepad1.start && !previousGamepad1.start && !imuResetInCooldown) {
            double homeHeadingRad = Math.toRadians(robot.toAllianceHeadingDeg(RED_BASE_START_HEADING_DEG));
            follower.setHeading(homeHeadingRad);
            holdHeading = homeHeadingRad;
            holdHeadingSet = true;
            imuResetInCooldown = true;
            imuResetTimer.reset();
            gamepad1.rumble(200);
        }
    }
    public void initShooterPIDF() {
        robot.shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(
                ShooterPIDFConfig.kP,
                ShooterPIDFConfig.kI,
                ShooterPIDFConfig.kD,
                ShooterPIDFConfig.kF
        );
        robot.shooterL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        robot.shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        robot.shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if (Math.abs(cachedShooterVeloL - activeTarget) <= toleranceforShoot) {
            isShooterAtSpeed = true;
        } else {
            isShooterAtSpeed = false;
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
// 自动匹配当前联盟的 AprilTag 前缀（"R" 或 "B"）
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
    private double interpolateTable(double[] xTable, double[] yTable, double x) {
        int n = xTable.length;
        if (x <= xTable[0]) return yTable[0];
        if (x >= xTable[n - 1]) return yTable[n - 1];
        for (int i = 0; i < n - 1; i++) {
            if (x >= xTable[i] && x <= xTable[i + 1]) {
                double ratio = (x - xTable[i]) / (xTable[i + 1] - xTable[i]);
                return yTable[i] + ratio * (yTable[i + 1] - yTable[i]);
            }
        }
        return yTable[n - 1];
    }
    private void updateAllTelemetry(double loopTimeMs) {
        Pose pose = follower.pose();
        double activeTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
// 纯数字序列（可直接用于 Panels Graph 折线图）
        telemetry.addData("Loop Time (ms)", loopTimeMs);
        telemetry.addData("Shooter Target", activeTarget);
        telemetry.addData("Shooter Left Velo", cachedShooterVeloL);
        telemetry.addData("Shooter Right Velo", cachedShooterVeloR);
        telemetry.addData("Hive Scorable", robot.hiveScorable ? 1 : 0);
        telemetry.addData("Goal Distance (in)", currentGoalDistanceIn);
        telemetry.addData("Dynamic Velo Target", dynamicTargetVelo);
        telemetry.addData("Dynamic Hood Target", dynamicTargetHood);
// 状态摘要文字
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
//import com.pedropathing.api.PoseFactory;
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
//
//@Configurable
//@TeleOp(name = "AAAAAA turret testing TeleOpDecodeRobotBlueBioBuzzSeason V1 11182026")
//public class TeleOpDecodeRobotBlueBioBuzzSeason extends LinearOpMode {
//
//    // 调试/比赛摄像头预览开关（正式比赛建议改为 false 节省 CPU）
//    public static boolean ENABLE_CAMERA_LIVE_VIEW = true;
//
//    // 场地目标坐标（Pedro Pathing 坐标系，支持 Panels 实时微调）
//    public static double BLUE_GOAL_X = 83.5;
//    public static double BLUE_GOAL_Y = 79.5;
//    public static double RED_GOAL_X = 58.0;
//    public static double RED_GOAL_Y = 62.0;
//
//    // 1. 炮台物理安装偏心与行程/限速参数（借鉴 Out of the Box turret.java）
//    public static double TURRET_OFFSET_X = 0.0;             // 炮台旋转轴相对机器人中心的前后偏心（英寸，+向前，-向后）
//    public static double TURRET_OFFSET_Y = 0.0;             // 炮台旋转轴相对机器人中心的左右偏心（英寸，+向左，-向右）
//    public static double TURRET_MAX_ROTATION_DEG = 270.0;   // 最大允许旋转角度（±270°）
//    public static double TURRET_MAX_SPEED_DEG_PER_S = 540.0;// 目标角度最大变化速率（度/秒，0=不限速，防止猛甩打齿）
//    public static double TURRET_DISABLE_Y_MIN = 62.0;       // 禁区 Y 下限
//    public static double TURRET_DISABLE_Y_MAX = 79.5;       // 禁区 Y 上限
//    public static double TURRET_DEADZONE_RADIUS_IN = 12.0;  // 距离目标小于 12 英寸时防抖锁定
//
//    // 2. 全向底盘与航向锁定参数（借鉴 Out of the Box MainTeleOp）
//    public static double HEADING_KP = 0.95;                 // 航向锁定比例增益（若自转方向相反改为负数）
//    public static double HEADING_DEADBAND_DEG = 1.0;        // 航向死区（1° 以内不修正）
//    public static double SETTLE_SPEED_DEG_PER_S = 30.0;     // 松开摇杆后等待角速度降到此值以下再锁定航向
//    public static double STICK_DEADBAND = 0.01;             // 旋转摇杆死区
////    private double holdHeading = 0.0;                   // 当前锁定的目标航向（弧度）
////    private boolean holdHeadingSet = false;             // 是否已锁定有效航向
////    private boolean turnedSinceHold = false;            // 标记操作手刚刚转动过底盘
//    public float DriveTrains_ReducePOWER = 1.0f;
//    public float DriveTrains_smoothTurn = 1.0f;
//    public float intakePowerShoot = 0.90f;
//    public float intakePowerIntake = 0.95f;
//    public float intakePowerDump = -0.65f;
//    public float intakePowerOff = 0.0f;
//
//    public static final double blockageblockposition = 0.10;
//    public static final double blockagereleaseposition = 0.18;
//    public static final double HoodArmfarposition = 0.30;
//    public static final double HoodArmcloseposition = 0.35;
//
//    private static final double Far_SHOOTER_TARGET_Velocity = 1400;
//    private static double Med_SHOOTER_TARGET_SPEED = 1050;
//    public static double toleranceforShoot = 100;
//
//    @Configurable
//    public static class ShooterPIDFConfig {
//        public static double kP = 300.0;
//        public static double kI = 0.0;
//        public static double kD = 0.0;
//        public static double kF = 15.0;
//        public static double targetSPEED = Med_SHOOTER_TARGET_SPEED;
//        public static double flyWheelIdleSpeed = Med_SHOOTER_TARGET_SPEED * 0.6;
//        public static double targetSPEEDfar = Far_SHOOTER_TARGET_Velocity;
//        public static double flyWheelIdleSpeedfar = Far_SHOOTER_TARGET_Velocity * 0.6;
//    }
//
//    // =========================================================================
//// 基于距离的自动仰角 (Hood) 与飞轮转速线性插值表（单位：英寸）
//// 调试方法：在场地上选取 4-5 个不同距离点，试出最佳 (Hood位置, 飞轮速度) 后填入下方数组（必须按距离从小到大排列）
//// =========================================================================
//    public static boolean AUTO_RANGE_SHOOT_ENABLED = true; // true = 根据距离自动调节转速与仰角；false = 手动档位
//    public static double SHOOTER_VELO_TRIM = 0.0;          // 现场全局转速微调补偿（例如电池电压低时在 Panels 里填 +40）
//    public static double HOOD_POS_TRIM = 0.0;              // 现场全局仰角微调补偿（例如射偏高时填 -0.02）
//
//    // 标定距离点（英寸）：从近到远严格递增
//    private static final double[] DIST_TABLE_IN = { 35.0,  60.0,  85.0,  110.0, 140.0 };
//    /*
//    *
//三、 现场快速标定方法
//1. 把机器人停在距离目标 35"、60"、85"、110"、140" 的 5 个标记点上（看 Panels 上的 Goal Distance (in) 读数即可精确定位）。
//2. 在每个距离点上，先通过 Gamepad2 手动试出最准的 Hood 位置和飞轮转速。
//3. 把这 5 组实测数值填入 DIST_TABLE_IN、HOOD_TABLE 和 VELO_TABLE 数组中，之后在全场任何角度和距离移动，只要按住右扳机，机器人就会自动完成炮台水平追踪 + 仰角俯仰调节 + 飞轮无级变速。
//    *
//    * */
//    // 对应的 Hood 仰角舵机位置（0.0 ~ 1.0）
//    private static final double[] HOOD_TABLE    = { 0.35,  0.42,  0.48,  0.53,  0.58  };
//    // 对应的飞轮目标速度（Encoder Ticks/s）
//    private static final double[] VELO_TABLE    = { 950.0, 1050.0, 1180.0, 1320.0, 1450.0 };
//
//    // 实时计算出的目标距离、推荐转速与推荐仰角缓存
//    private double currentGoalDistanceIn = 0.0;
//    private double dynamicTargetVelo = 1050.0;
//    private double dynamicTargetHood = 0.50;
//
//    private final HardwareDecodeRobotBlueBioBuzzSeason robot = new HardwareDecodeRobotBlueBioBuzzSeason();
//    private final Gamepad currentGamepad1 = new Gamepad();
//    private final Gamepad currentGamepad2 = new Gamepad();
//    private final Gamepad previousGamepad1 = new Gamepad();
//    private final Gamepad previousGamepad2 = new Gamepad();
//
//    private Follower follower;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
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
//    // 炮台控制与超限位变量
//    private boolean turretAutoAim = true;
//    private boolean isInTurretDeadZone = false;
//    private boolean turretReachable = true;
//    private boolean wasTurretReachable = true;
//    private double turretOverLimitDeg = 0.0;
//    private double turretManualTrimDeg = 0.0;
//    private double turretTargetDeg = 0.0;
//    // 炮台目标角度变化率平滑限速参数（度/秒，支持在 Panels 网页端实时调整；设为 0 则关闭限速）
//    private double smoothedTurretTargetDeg = 0.0;
//
//
//
//    // 航向锁定状态变量
//    private double holdHeading = 0.0;
//    private boolean holdHeadingSet = false;
//    private boolean turnedSinceHold = false;
//
//    // 支持在 Panels 网页端实时修改的炮台 PID 参数
//    public static double TURRET_KP = 0.0069;
//    public static double TURRET_KI = 0.0000;
//    public static double TURRET_KD = 0.0002;
//    public static double TURRET_MAX_POWER = 0.5;
//
//    private double lastTurretKP = -1;
//    private double lastTurretKI = -1;
//    private double lastTurretKD = -1;
//    private double lastTurretMaxPower = -1;
//    public static double TURRET_LOOKAHEAD_S = 0.08;
//    public static double TURRET_TRIM_LIMIT_DEG = 15.0;
//    public static double SHOT_FLIGHT_TIME_S = 0.0;
//
//    // 缓存速度与 LED 防抖
//    private double cachedShooterVeloL = 0.0;
//    private double cachedShooterVeloR = 0.0;
//    private String lastLedColor = "";
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//        robot.alliance = Alliance.BLUE;
//        robot.initVision(ENABLE_CAMERA_LIVE_VIEW);
//        initShooterPIDF();
//
//        if (robot.axonTurretArmL != null) {
//            robot.axonTurretArmL.setTargetRotation(0.0);
//        }
//
//        follower = Constants.create(hardwareMap);
//        Pose startPose = poseFactory.of(83.5, 133.5, 0.0);
//        follower.setPose(startPose);
//        follower.update();
//
//        // 初始化航向锁定基准角
//        holdHeading = follower.pose().heading();
//        holdHeadingSet = true;
//
//        // 启用 JoinedTelemetry 实现手机端与 Panels 网页端双端自动同步
//        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
//        telemetry.addData("Status", "BioBuzz TeleOp Ready at (83.5, 133.5, 0°)");
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
//            updateIntake();
//            updateShooter();
//            checkShooterVelocity();
//            updateBlockage();
//            updateHood();
//            updateLEDs();
//
//            // 7. 炮台偏心解算、平滑限速、超限震动与闭环追踪
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
//    /**
//     * 整合：偏心运动学解算 + 禁区/近距离防抖 + 目标平滑限速 + 超限位震动提醒
//     */
//    private void updateTurretControl() {
//        // 1. 实时同步 Panels 网页端修改的炮台 PID 参数
//        if (TURRET_KP != lastTurretKP || TURRET_KI != lastTurretKI || TURRET_KD != lastTurretKD) {
//            robot.axonTurretArmL.setPidCoeffs(TURRET_KP, TURRET_KI, TURRET_KD);
//            lastTurretKP = TURRET_KP;
//            lastTurretKI = TURRET_KI;
//            lastTurretKD = TURRET_KD;
//        }
//        if (TURRET_MAX_POWER != lastTurretMaxPower) {
//            robot.axonTurretArmL.setMaxPower(TURRET_MAX_POWER);
//            lastTurretMaxPower = TURRET_MAX_POWER;
//        }
//
//        // 2. Gamepad2 Back 键切换自动跟踪 / 手动回中模式
//        if (currentGamepad2.back && !previousGamepad2.back) {
//            turretAutoAim = !turretAutoAim;
//        }
//
//        // 3. Gamepad2 左右方向键提供 ±2° 实时微调补偿
//        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
//            turretManualTrimDeg += 2.0;
//        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
//            turretManualTrimDeg -= 2.0;
//        }
//        turretManualTrimDeg = Range.clip(turretManualTrimDeg, -TURRET_TRIM_LIMIT_DEG, TURRET_TRIM_LIMIT_DEG);
//
//        Pose p = follower.pose();
//        Velocity v = follower.velocity();
//        double h = p.heading();
//        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
//
//        // 4. 计算炮台旋转轴在场地上的真实偏心坐标 (pivotX, pivotY)
//        double pivotX = p.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h);
//        double pivotY = p.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h);
//
//        double targetX = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
//        double targetY = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
//        if (SHOT_FLIGHT_TIME_S > 0) {
//            targetX -= v.vx * SHOT_FLIGHT_TIME_S;
//            targetY -= v.vy * SHOT_FLIGHT_TIME_S;
//        }
//        double dx = targetX - pivotX;
//        double dy = targetY - pivotY;
//        double rangeSq = dx * dx + dy * dy;
//
//        // 计算炮台转轴到目标的实时直线距离（英寸）
//        currentGoalDistanceIn = Math.sqrt(rangeSq);
//        // 根据距离查表计算推荐仰角与飞轮速度，并叠加上网页端微调补偿
//        dynamicTargetHood = interpolateTable(DIST_TABLE_IN, HOOD_TABLE, currentGoalDistanceIn) + HOOD_POS_TRIM;
//        dynamicTargetVelo = interpolateTable(DIST_TABLE_IN, VELO_TABLE, currentGoalDistanceIn) + SHOOTER_VELO_TRIM;
//
//        // 5. 有效射击扇形环带与禁区判断（基于 12.04 in. 真实纵深与 45.5 in. 高度差）
//        // 5.1 位于目标开口正面（蓝方上方目标开口朝 +Y，故机器人需在目标上方即 dy < 0；红方下方目标开口朝 -Y，故 dy > 0）
//        boolean inFrontOfOpening = (robot.alliance == Alliance.BLUE) ? (dy < 0) : (dy > 0);
//        // 5.2 水平距离不小于最小仰角盲区（35.0 英寸，防止仰角过陡撞顶棚或下沿）
//        boolean aboveMinRange = (rangeSq >= 35.0 * 35.0);
//        // 5.3 水平入射偏角在有效窗口内（纵深 12.04 in.：8.5 为物理不碰壁极限 ±35.2°；若想收紧到高命中区可改为 6.0 即 ±26.5°）
//        boolean withinEntryAngle = Math.abs(dx) <= Math.abs(dy) * (8.5 / 12.04);
//
//        boolean isValidShootingZone = inFrontOfOpening && aboveMinRange && withinEntryAngle;
//        isInTurretDeadZone = !isValidShootingZone;
//
//        double desiredTargetDeg;
//        if (isInTurretDeadZone || !turretAutoAim) {
//            // 处于背面、太近(<35")、偏角过大(>35.2°)或手动关闭自瞄时：炮台自动回正指向车头 (0°)
//            desiredTargetDeg = 0.0;
//            turretReachable = true;
//            turretOverLimitDeg = 0.0;
//        } else {
//            double relAngle = Math.atan2(dy, dx) - h;
//            while (relAngle >  Math.PI) relAngle -= 2 * Math.PI;
//            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
//
//            double bearingRate = (rangeSq > 1e-6) ? (dy * v.vx - dx * v.vy) / rangeSq : 0.0;
//            double turretRate  = bearingRate - v.omega;
//            double baseTargetDeg = Math.toDegrees(relAngle + turretRate * TURRET_LOOKAHEAD_S)
//                    + turretManualTrimDeg;
//
//            // 选取 ±270° 行程内距离当前角度最近的等效角度
//            double candidateDeg = baseTargetDeg;
//            if (candidateDeg - currentTurretDeg > 180.0
//                    && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
//                candidateDeg -= 360.0;
//            } else if (candidateDeg - currentTurretDeg < -180.0
//                    && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
//                candidateDeg += 360.0;
//            }
//
//            // 6. 超限位检测与单次手柄震动提醒
//            turretReachable = Math.abs(candidateDeg) <= TURRET_MAX_ROTATION_DEG;
//            if (!turretReachable) {
//                turretOverLimitDeg = (candidateDeg > TURRET_MAX_ROTATION_DEG)
//                        ? (candidateDeg - TURRET_MAX_ROTATION_DEG)
//                        : (candidateDeg + TURRET_MAX_ROTATION_DEG);
//            } else {
//                turretOverLimitDeg = 0.0;
//            }
//
//            if (wasTurretReachable && !turretReachable) {
//                gamepad1.rumble(300);
//            }
//            wasTurretReachable = turretReachable;
//
//            desiredTargetDeg = Range.clip(candidateDeg, -TURRET_MAX_ROTATION_DEG, TURRET_MAX_ROTATION_DEG);
//        }
//        turretTargetDeg = desiredTargetDeg;
//
//        // 7. 目标角度变化率平滑限速（防止大角度阶跃瞬间猛甩打齿）
//        double dt = turretRateTimer.seconds();
//        turretRateTimer.reset();
//
//        if (TURRET_MAX_SPEED_DEG_PER_S > 0 && dt > 0.0 && dt < 0.5) {
//            double maxStep = TURRET_MAX_SPEED_DEG_PER_S * dt;
//            double delta = Range.clip(turretTargetDeg - smoothedTurretTargetDeg, -maxStep, maxStep);
//            smoothedTurretTargetDeg += delta;
//        } else {
//            smoothedTurretTargetDeg = turretTargetDeg;
//        }
//
//        // 8. 下发平滑后的目标角度给主 Axon 连续旋转舵机
//        robot.axonTurretArmL.trackTargetRotation(smoothedTurretTargetDeg);
//
//        // 9. 纯数字遥测输出（Panels Graph 可直接画图）
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
//
////    private void updateTurretControl() {
////        // 1. 实时同步 Panels 网页端修改的炮台 PID 参数
////        if (TURRET_KP != lastTurretKP || TURRET_KI != lastTurretKI || TURRET_KD != lastTurretKD) {
////            robot.axonTurretArmL.setPidCoeffs(TURRET_KP, TURRET_KI, TURRET_KD);
////            lastTurretKP = TURRET_KP;
////            lastTurretKI = TURRET_KI;
////            lastTurretKD = TURRET_KD;
////        }
////        if (TURRET_MAX_POWER != lastTurretMaxPower) {
////            robot.axonTurretArmL.setMaxPower(TURRET_MAX_POWER);
////            lastTurretMaxPower = TURRET_MAX_POWER;
////        }
////
////        // 2. Gamepad2 Back 键切换自动跟踪 / 手动回中模式
////        if (currentGamepad2.back && !previousGamepad2.back) {
////            turretAutoAim = !turretAutoAim;
////        }
////
////        // 3. Gamepad2 左右方向键提供 ±2° 实时微调补偿
////        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
////            turretManualTrimDeg += 2.0;
////        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
////            turretManualTrimDeg -= 2.0;
////        }
////        turretManualTrimDeg = Range.clip(turretManualTrimDeg, -TURRET_TRIM_LIMIT_DEG, TURRET_TRIM_LIMIT_DEG);
////
////        Pose p = follower.pose();
////        Velocity v = follower.velocity();
////        double h = p.heading();
////        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
////
////        // 4. 计算炮台旋转轴在场地上的真实偏心坐标 (pivotX, pivotY)
////        double pivotX = p.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h);
////        double pivotY = p.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h);
////
////        double targetX = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
////        double targetY = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
////        if (SHOT_FLIGHT_TIME_S > 0) {
////            targetX -= v.vx * SHOT_FLIGHT_TIME_S;
////            targetY -= v.vy * SHOT_FLIGHT_TIME_S;
////        }
////        double dx = targetX - pivotX;
////        double dy = targetY - pivotY;
////        double rangeSq = dx * dx + dy * dy;
////        // 计算炮台转轴到目标的实时直线距离（英寸）
////        currentGoalDistanceIn = Math.sqrt(rangeSq);
////        // 根据距离查表计算推荐仰角与飞轮速度，并叠加上网页端微调补偿
////        dynamicTargetHood = interpolateTable(DIST_TABLE_IN, HOOD_TABLE, currentGoalDistanceIn) + HOOD_POS_TRIM;
////        dynamicTargetVelo = interpolateTable(DIST_TABLE_IN, VELO_TABLE, currentGoalDistanceIn) + SHOOTER_VELO_TRIM;
////
////// 5. 禁区判断（合并：Y 轴禁区带 OR 距离目标小于 12 英寸防抖死区）
////
////        boolean inYBand = (p.y() >= TURRET_DISABLE_Y_MIN && p.y() <= TURRET_DISABLE_Y_MAX);
////        boolean tooClose = (TURRET_DEADZONE_RADIUS_IN > 0)
////                && (rangeSq < TURRET_DEADZONE_RADIUS_IN * TURRET_DEADZONE_RADIUS_IN);
////        isInTurretDeadZone = inYBand || tooClose;
////        double desiredTargetDeg;
////        if (inYBand || !turretAutoAim) {
////            desiredTargetDeg = 0.0;
////            turretReachable = true;
////            turretOverLimitDeg = 0.0;
////        } else if (tooClose) {
////// 距离目标太近时，保持当前角度不动，防止角度剧烈跳变
////            desiredTargetDeg = turretTargetDeg;
////            turretReachable = true;
////            turretOverLimitDeg = 0.0;
////        } else {
////            double relAngle = Math.atan2(dy, dx) - h;
////            while (relAngle >  Math.PI) relAngle -= 2 * Math.PI;
////            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
////            double bearingRate = (rangeSq > 1e-6) ? (dy * v.vx - dx * v.vy) / rangeSq : 0.0;
////            double turretRate  = bearingRate - v.omega;
////            double baseTargetDeg = Math.toDegrees(relAngle + turretRate * TURRET_LOOKAHEAD_S)
////                    + turretManualTrimDeg;
////// 选取 ±270° 行程内距离当前角度最近的等效角度
////            double candidateDeg = baseTargetDeg;
////            if (candidateDeg - currentTurretDeg > 180.0
////                    && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
////                candidateDeg -= 360.0;
////            } else if (candidateDeg - currentTurretDeg < -180.0
////                    && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
////                candidateDeg += 360.0;
////            }
////// 6. 超限位检测与单次手柄震动提醒
////            turretReachable = Math.abs(candidateDeg) <= TURRET_MAX_ROTATION_DEG;
////            if (!turretReachable) {
////                turretOverLimitDeg = (candidateDeg > TURRET_MAX_ROTATION_DEG)
////                        ? (candidateDeg - TURRET_MAX_ROTATION_DEG)
////                        : (candidateDeg + TURRET_MAX_ROTATION_DEG);
////            } else {
////                turretOverLimitDeg = 0.0;
////            }
////            if (wasTurretReachable && !turretReachable) {
////                gamepad1.rumble(300);
////            }
////            wasTurretReachable = turretReachable;
////            desiredTargetDeg = Range.clip(candidateDeg, -TURRET_MAX_ROTATION_DEG, TURRET_MAX_ROTATION_DEG);
////        }
////        turretTargetDeg = desiredTargetDeg;
////// 7. 目标角度变化率平滑限速（防止大角度阶跃瞬间猛甩打齿）
////        double dt = turretRateTimer.seconds();
////        turretRateTimer.reset();
////
////        // 目标角度变化率平滑限速：限制每帧目标角度的最大步进量
////        if (TURRET_MAX_SPEED_DEG_PER_S > 0 && dt > 0.0 && dt < 0.5) {
////            double maxStep = TURRET_MAX_SPEED_DEG_PER_S * dt;
////            double delta = Range.clip(turretTargetDeg - smoothedTurretTargetDeg, -maxStep, maxStep);
////            smoothedTurretTargetDeg += delta;
////        } else {
////            smoothedTurretTargetDeg = turretTargetDeg;
////        }
////
////
////// 8. 下发平滑后的目标角度给主 Axon 连续旋转舵机
////        robot.axonTurretArmL.trackTargetRotation(smoothedTurretTargetDeg);
////// 9. 纯数字遥测输出（Panels Graph 可直接画图）
////        telemetry.addData("Turret Target (deg)", smoothedTurretTargetDeg);
////        telemetry.addData("Turret Actual (deg)", currentTurretDeg);
////        telemetry.addData("Turret Error (deg)", smoothedTurretTargetDeg - currentTurretDeg);
////        telemetry.addData("Turret Servo Power", robot.axonTurretArmL.getPower());
////        telemetry.addData("Turret Reachable", turretReachable ? 1 : 0);
////        if (!turretReachable) {
////            telemetry.addData("TURN ROBOT WARNING", "Turn %.0f deg %s!",
////                    Math.abs(turretOverLimitDeg), turretOverLimitDeg > 0 ? "LEFT" : "RIGHT");
////        }
////    }
//    /**
//     * 整合全向底盘场地坐标系驱动 + 双向抗饱和航向锁定（Heading Hold）
//     */
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
//// 1. 航向锁定状态机：操作手转动时解除锁定，松开摇杆且车体停稳后锁定当前朝向
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
//// 2. 横移走位抗偏转闭环修正（注意：Pedro 坐标系左转为正，若发现反向自转把 HEADING_KP 改为负数）
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
//    /**
//     * 修复按键冲突：分开航向清零与坐标重置（同步更新 holdHeading 防止重置后回弹）
//     */
//    private void handleHeadingAndPositionReset() {
//        if (imuResetInCooldown && imuResetTimer.milliseconds() > IMU_RESET_COOLDOWN_MS) {
//            imuResetInCooldown = false;
//        }
//// 1. Back + Start 同时按下：重置回预设起始坐标 (83.5, 133.5, 0°)
//        if (currentGamepad1.back && currentGamepad1.start) {
//            follower.setPose(poseFactory.of(83.5, 133.5, 0.0));
//            holdHeading = 0.0;
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
//// 3. 单独按 Start（无 Back）：仅重置当前车头朝向为 0°（不影响 X/Y 坐标）
//        else if (currentGamepad1.start && !previousGamepad1.start && !imuResetInCooldown) {
//            follower.setHeading(0.0);
//            holdHeading = 0.0;
//            holdHeadingSet = true;
//            imuResetInCooldown = true;
//            imuResetTimer.reset();
//            gamepad1.rumble(200);
//        }
//    }
//    public void initShooterPIDF() {
//        robot.shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        PIDFCoefficients pidf = new PIDFCoefficients(
//                ShooterPIDFConfig.kP,
//                ShooterPIDFConfig.kI,
//                ShooterPIDFConfig.kD,
//                ShooterPIDFConfig.kF
//        );
//        robot.shooterL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//        robot.shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//        robot.shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
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
//
//    private void startShooter() {
//        robot.IntakeMotorL.setPower(0);
//        robot.IntakeMotorR.setPower(0);
//
//        if (AUTO_RANGE_SHOOT_ENABLED) {
//            // 全自动距离变焦模式：同时设置对应的 Hood 仰角和飞轮目标转速
//            robot.setHoodPosition(dynamicTargetHood);
//            robot.shooterL.setVelocity(Math.abs(dynamicTargetVelo));
//            robot.shooterR.setVelocity(Math.abs(dynamicTargetVelo));
//        } else {
//            // 手动近射固定模式
//            robot.setHoodPosition(0.50);
//            robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
//            robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
//        }
//        shooterIsOn = true;
//    }
//
//
//
//    //    private void startShooter() {
////        robot.IntakeMotorL.setPower(0);
////        robot.IntakeMotorR.setPower(0);
////        robot.setHoodPosition(0.5);
////        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
////        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
////        shooterIsOn = true;
////    }
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
////    private void checkShooterVelocity() {
////        double activeTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
////        if (Math.abs(cachedShooterVeloL - activeTarget) <= toleranceforShoot) {
////            isShooterAtSpeed = true;
////        } else {
////            isShooterAtSpeed = false;
////            fireRequested = false;
////        }
////    }
//
//    private void checkShooterVelocity() {
//        double activeTarget = AUTO_RANGE_SHOOT_ENABLED
//                ? dynamicTargetVelo
//                : (isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED);
//
//        if (Math.abs(cachedShooterVeloL - activeTarget) <= toleranceforShoot) {
//            isShooterAtSpeed = true;
//        } else {
//            isShooterAtSpeed = false;
//            fireRequested = false;
//        }
//    }
//
//
//    private void updateHood() {
//        // 1. 按 Gamepad2 Start 键一键切换“全自动距离变焦”与“手动调节”模式
//        if (currentGamepad2.start && !previousGamepad2.start) {
//            AUTO_RANGE_SHOOT_ENABLED = !AUTO_RANGE_SHOOT_ENABLED;
//        }
//
//        // 2. 开启自动距离模式且飞轮正在运转时，实时跟随距离连续微调仰角舵机与飞轮转速
//        if (AUTO_RANGE_SHOOT_ENABLED && shooterIsOn) {
//            robot.setHoodPosition(dynamicTargetHood);
//            double targetV = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : dynamicTargetVelo;
//            robot.shooterL.setVelocity(Math.abs(targetV));
//            robot.shooterR.setVelocity(Math.abs(targetV));
//        } else if (!AUTO_RANGE_SHOOT_ENABLED) {
//            // 手动应急调节
//            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
//                robot.setHoodPosition(HoodArmfarposition);
//            } else if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
//                robot.setHoodPosition(HoodArmcloseposition);
//            } else if (currentGamepad2.y && !previousGamepad2.y) {
//                robot.setHoodPosition(0.50);
//            }
//        }
//    }
//
//
////    private void updateHood() {
////        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
////            robot.setHoodPosition(HoodArmfarposition);
////        } else if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
////            robot.setHoodPosition(HoodArmcloseposition);
////        } else if (currentGamepad2.y && !previousGamepad2.y) {
////            robot.setHoodPosition(0.50);
////        }
////    }
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
//                boolean allianceMatch = name.startsWith(robot.alliance == Alliance.RED ? "R" : "B");
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
//    }    /**
//     * 安全一维线性插值函数（自带首尾边界限幅，永不越界报错）
//     */
//    private double interpolateTable(double[] xTable, double[] yTable, double x) {
//        int n = xTable.length;
//        if (x <= xTable[0]) return yTable[0];
//        if (x >= xTable[n - 1]) return yTable[n - 1];
//
//        for (int i = 0; i < n - 1; i++) {
//            if (x >= xTable[i] && x <= xTable[i + 1]) {
//                double ratio = (x - xTable[i]) / (xTable[i + 1] - xTable[i]);
//                return yTable[i] + ratio * (yTable[i + 1] - yTable[i]);
//            }
//        }
//        return yTable[n - 1];
//    }
//
//
//    private void updateAllTelemetry(double loopTimeMs) {
//        Pose pose = follower.pose();
//        double activeTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
//
//        // 纯数字序列（可直接用于 Panels Graph 折线图）
//        telemetry.addData("Loop Time (ms)", loopTimeMs);
//        telemetry.addData("Shooter Target", activeTarget);
//        telemetry.addData("Shooter Left Velo", cachedShooterVeloL);
//        telemetry.addData("Shooter Right Velo", cachedShooterVeloR);
//        telemetry.addData("Hive Scorable", robot.hiveScorable ? 1 : 0);
//        telemetry.addData("Goal Distance (in)", currentGoalDistanceIn);
//        telemetry.addData("Dynamic Velo Target", dynamicTargetVelo);
//        telemetry.addData("Dynamic Hood Target", dynamicTargetHood);
//
//// 状态摘要文字
//        telemetry.addData("Shoot Mode", isFarMode ? "FAR" : "NEAR");
//        telemetry.addData("Pose (X/Y/H)", "%.1f / %.1f / %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
//        telemetry.addData("Hold Heading", "%.1f° (Active:%b)", Math.toDegrees(holdHeading), holdHeadingSet);
//        telemetry.addData("Hive Cluster", robot.hiveClusterName);
//    }
//}
//
//
//
//
////package org.firstinspires.ftc.teamcode.pedro;
////
////import com.bylazar.configurables.annotations.Configurable;
////import com.bylazar.telemetry.PanelsTelemetry;
////import com.bylazar.telemetry.TelemetryManager;
////import com.pedropathing.follower.Follower;
////import com.pedropathing.math.Pose;
////import com.pedropathing.api.PoseFactory;
////import com.pedropathing.math.Velocity;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.Gamepad;
////import com.qualcomm.robotcore.hardware.PIDFCoefficients;
////import com.qualcomm.robotcore.util.ElapsedTime;
////import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
////import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
////import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
////import com.bylazar.telemetry.JoinedTelemetry;
////
////import com.bylazar.panels.Panels;
////import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//////import com.bylazar.graph.GraphManager;
//////import com.bylazar.graph.PanelsGraph;
////
////@Configurable
////@TeleOp(name = "AAAAAA turret testing TeleOpDecodeRobotBlueBioBuzzSeason V1 11182026")
////public class TeleOpDecodeRobotBlueBioBuzzSeason extends LinearOpMode {
////
////    // 调试/比赛摄像头预览开关（正式比赛建议改为 false 节省 CPU）
////    public static boolean ENABLE_CAMERA_LIVE_VIEW = true;
////
////    // 场地目标坐标（Pedro Pathing 坐标系，支持 Panels 实时微调）
////    public static double BLUE_GOAL_X = 83.5;
////    public static double BLUE_GOAL_Y = 79.5;
////    public static double RED_GOAL_X = 58.0;
////    public static double RED_GOAL_Y = 62.0;
////
////    // Turret 禁区 Y 坐标范围与最大旋转角度限制
////    public static double TURRET_DISABLE_Y_MIN = 62.0;
////    public static double TURRET_DISABLE_Y_MAX = 79.5;
////    public static double TURRET_DISABLE_X_MIN = 46.0;
////    public static double TURRET_DISABLE_X_MAX = 95.5;
////    public static double TURRET_MAX_ROTATION_DEG = 270.0;
////
////    public float DriveTrains_ReducePOWER = 1.0f;
////    public float DriveTrains_smoothTurn = 1.0f;
////    public float intakePowerShoot = 0.90f;
////    public float intakePowerIntake = 0.95f;
////    public float intakePowerDump = -0.65f;
////    public float intakePowerOff = 0.0f;
////
////    public static final double blockageblockposition = 0.10;
////    public static final double blockagereleaseposition = 0.18;
////    public static final double HoodArmfarposition = 0.30;
////    public static final double HoodArmcloseposition = 0.35;
////
////    private static final double Far_SHOOTER_TARGET_Velocity = 1400;
////    private static double Med_SHOOTER_TARGET_SPEED = 1050;
////    public static double toleranceforShoot = 100;
////
////    @Configurable
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
////
////    private final HardwareDecodeRobotBlueBioBuzzSeason robot = new HardwareDecodeRobotBlueBioBuzzSeason();
////    private final Gamepad currentGamepad1 = new Gamepad();
////    private final Gamepad currentGamepad2 = new Gamepad();
////    private final Gamepad previousGamepad1 = new Gamepad();
////    private final Gamepad previousGamepad2 = new Gamepad();
////
////    private Follower follower;
////    private TelemetryManager panelsTelemetry;
//////    private final TelemetryManager panelsTelemetry = Panels.getTelemetry();
////    private final PoseFactory poseFactory = PoseFactory.degrees();
////    private final ElapsedTime imuResetTimer = new ElapsedTime();
////    private final ElapsedTime loopTimer = new ElapsedTime();
////
////    // 状态标志与每帧缓存
////    private boolean isFarMode = false;
////    private boolean isShooterAtSpeed = false;
////    private boolean shooterIsOn = false;
////    private boolean intakeIsOn = false;
////    private boolean fireRequested = false;
////    private boolean imuResetInCooldown = false;
////    private static final long IMU_RESET_COOLDOWN_MS = 300;
////
////    // 炮台控制变量
////    private boolean turretAutoAim = true;
////    private boolean isInTurretDeadZone = false;
////    private double turretManualTrimDeg = 0.0;
////    private double turretTargetDeg = 0.0;
////    // 支持在 Panels 网页端 Configurables 面板实时修改的炮台 PID 参数
////    public static double TURRET_KP = 0.069;
////    public static double TURRET_KI = 0.0000;
////    public static double TURRET_KD = 0.0002;
////    public static double TURRET_MAX_POWER = 0.5;
////
////    // 缓存上一帧的 PID 参数，仅在网页端数值变化时才触发更新（避免每帧无效重置积分）
////    private double lastTurretKP = -1;
////    private double lastTurretKI = -1;
////    private double lastTurretKD = -1;
////    private double lastTurretMaxPower = -1;
////    public static double TURRET_LOOKAHEAD_S       = 0.08;  // effective turret lag; tune 0.04–0.15
////    public static double TURRET_TRIM_LIMIT_DEG    = 15.0;  // cap on manual trim
////    public static double TURRET_DEADZONE_RADIUS_IN = 0.0;  // 0 = no dead zone
////    public static double SHOT_FLIGHT_TIME_S       = 0.0;   // 0 = aim at goal; >0 = lead for shot-on-move
////    // 炮台超限位跟踪与状态提醒变量
////    private boolean turretReachable = true;       // 当前目标是否在 ±270° 可达范围内
////    private boolean wasTurretReachable = true;    // 上一帧是否可达（用于捕捉刚超限的瞬间触发一次震动）
////    private double turretOverLimitDeg = 0.0;      // 超出限位的角度差值（正数=需左转底盘，负数=需右转底盘）
////
////
////    // 缓存速度与 LED 防抖
////    private double cachedShooterVeloL = 0.0;
////    private double cachedShooterVeloR = 0.0;
////    private String lastLedColor = "";
////
////    @Override
////    public void runOpMode() {
////        robot.init(hardwareMap);
////        robot.alliance = Alliance.BLUE;
////        robot.initVision(ENABLE_CAMERA_LIVE_VIEW);
////        initShooterPIDF();
////
////        // 确保 Turret 初始指向 0 度方向（机器人正前方）
////        if (robot.axonTurretArmL != null) {
////            robot.axonTurretArmL.setTargetRotation(0.0);
////        }
////
////        follower = Constants.create(hardwareMap);
////        // 机器人初始位置设为 (83.5, 133.5)，初始朝向 0 度
////        Pose startPose = poseFactory.of(83.5, 133.5, 0.0);
////        follower.setPose(startPose);
////        follower.update();
////
//////        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//////        panelsTelemetry.debug("Status", "BioBuzz TeleOp Ready at (83.5, 133.5, 0°)");
//////        panelsTelemetry.setUpdateInterval(20L);
//////        panelsTelemetry.update(telemetry);
////// 将系统自带的 telemetry 包装为双端同步遥测（手机端 + Panels 网页端）
////        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
////        telemetry.addData("Status", "BioBuzz TeleOp Ready at (83.5, 133.5, 0°)");
////        telemetry.update();
////
////        waitForStart();
////        loopTimer.reset();
////
////        while (opModeIsActive()) {
////            // 1. 计算单帧 Loop Time 并刷新 REV Hub 批量缓存
////            double loopTimeMs = loopTimer.milliseconds();
////            loopTimer.reset();
////            robot.clearBulkCache();
////
////            // 2. 本轮主循环单次读取左右飞轮速度并缓存
////            cachedShooterVeloL = Math.abs(robot.MasterShooterMotorL.getVelocity());
////            cachedShooterVeloR = Math.abs(robot.SlaveShooterMotorR.getVelocity());
////
////            // 3. 更新手柄边沿状态
////            previousGamepad1.copy(currentGamepad1);
////            previousGamepad2.copy(currentGamepad2);
////            currentGamepad1.copy(gamepad1);
////            currentGamepad2.copy(gamepad2);
////
////            // 4. 更新机器人定位与底盘控制
////            follower.update();
////            updateDrivetrain_FieldCentric();
////            handleIMUReset();
////            handlePositionReset();
////
////            // 5. 远近射模式切换（Gamepad2 X 切远射 / B 切近射）
////            if (currentGamepad2.x && !previousGamepad2.x) {
////                isFarMode = true;
////            } else if (currentGamepad2.b && !previousGamepad2.b) {
////                isFarMode = false;
////            }
////
////            // 6. 机构控制（全非阻塞边沿触发）
////            updateIntake();
////            updateShooter();
////            checkShooterVelocity();
////            updateBlockage();
////            updateHood();
////            updateLEDs();
////
////            // 7. 炮台闭环追踪与视觉状态刷新
////            updateTurretControl();
////            robot.updateTurretAxons();
////            updateHiveStatus();
////
////            // 8. 统一双端刷新遥测
////            updateAllTelemetry(loopTimeMs);
//////            panelsTelemetry.update(telemetry);
////            telemetry.update();
////        }
////    }
////
////    /**
////     * 炮台自动追踪控制：
////     * 1. 当机器人中心 Y 位于 [62.0, 79.5] 之间时，关闭追踪并回正指向机器人前进方向 (0°)
////     * 2. 其他区域始终对准目标 (83.5, 79.5)，且顺时针/逆时针旋转角度严格限制在 ±270° 以内
////     */
////
////    private void updateTurretControl() {
////        // 1. 实时同步 Panels 网页端修改的炮台 PID 参数
////        if (TURRET_KP != lastTurretKP || TURRET_KI != lastTurretKI || TURRET_KD != lastTurretKD) {
////            robot.axonTurretArmL.setPidCoeffs(TURRET_KP, TURRET_KI, TURRET_KD);
////            lastTurretKP = TURRET_KP;
////            lastTurretKI = TURRET_KI;
////            lastTurretKD = TURRET_KD;
////        }
////        if (TURRET_MAX_POWER != lastTurretMaxPower) {
////            robot.axonTurretArmL.setMaxPower(TURRET_MAX_POWER);
////            lastTurretMaxPower = TURRET_MAX_POWER;
////        }
////
////        // 2. Gamepad2 Back 键切换自动跟踪 / 手动回中模式
////        if (currentGamepad2.back && !previousGamepad2.back) {
////            turretAutoAim = !turretAutoAim;
////        }
////
////        // 3. Gamepad2 左右方向键提供 ±2° 实时微调补偿（限制在 ±15° 内）
////        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
////            turretManualTrimDeg += 2.0;
////        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
////            turretManualTrimDeg -= 2.0;
////        }
////        turretManualTrimDeg = Math.max(-TURRET_TRIM_LIMIT_DEG,
////                Math.min(TURRET_TRIM_LIMIT_DEG, turretManualTrimDeg));
////
////        Pose p = follower.pose();
////        Velocity v = follower.velocity();
////        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
////
////        double targetX = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
////        double targetY = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
////
////        // 4. 移动射击飞行时间提前量补偿
////        if (SHOT_FLIGHT_TIME_S > 0) {
////            targetX -= v.vx * SHOT_FLIGHT_TIME_S;
////            targetY -= v.vy * SHOT_FLIGHT_TIME_S;
////        }
////
////        double dx = targetX - p.x();
////        double dy = targetY - p.y();
////        double rangeSq = dx * dx + dy * dy;
////
////        isInTurretDeadZone = (TURRET_DEADZONE_RADIUS_IN > 0)
////                && (rangeSq < TURRET_DEADZONE_RADIUS_IN * TURRET_DEADZONE_RADIUS_IN);
////
////        if (isInTurretDeadZone || !turretAutoAim) {
////            turretTargetDeg = 0.0;
////            turretReachable = true;
////            turretOverLimitDeg = 0.0;
////        } else {
////            double relAngle = Math.atan2(dy, dx) - p.heading();
////            while (relAngle >  Math.PI) relAngle -= 2 * Math.PI;
////            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
////
////            // 前馈角速度补偿
////            double bearingRate = (rangeSq > 1e-6) ? (dy * v.vx - dx * v.vy) / rangeSq : 0.0;
////            double turretRate  = bearingRate - v.omega;
////
////            double baseTargetDeg = Math.toDegrees(relAngle + turretRate * TURRET_LOOKAHEAD_S)
////                    + turretManualTrimDeg;
////
////            // 选择距离当前角度最近的合法等效角度
////            double candidateDeg = baseTargetDeg;
////            if (candidateDeg - currentTurretDeg > 180.0
////                    && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
////                candidateDeg -= 360.0;
////            } else if (candidateDeg - currentTurretDeg < -180.0
////                    && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
////                candidateDeg += 360.0;
////            }
////
////            // 5. 超限位判断与底盘转向指引计算（借鉴 Out of the Box turret.java）
////            turretReachable = Math.abs(candidateDeg) <= TURRET_MAX_ROTATION_DEG;
////            if (!turretReachable) {
////                // 计算超出限位多少度：正数表示目标在左侧超限，负数表示在右侧超限
////                turretOverLimitDeg = (candidateDeg > TURRET_MAX_ROTATION_DEG)
////                        ? (candidateDeg - TURRET_MAX_ROTATION_DEG)
////                        : (candidateDeg + TURRET_MAX_ROTATION_DEG);
////            } else {
////                turretOverLimitDeg = 0.0;
////            }
////
////            // 当目标刚刚超出 ±270° 限位的瞬间，触发手柄震动 300ms 提醒操作手
////            if (wasTurretReachable && !turretReachable) {
////                gamepad1.rumble(300);
////            }
////            wasTurretReachable = turretReachable;
////
////            // 严格限幅在 [-270°, +270°]
////            turretTargetDeg = Math.max(-TURRET_MAX_ROTATION_DEG,
////                    Math.min(TURRET_MAX_ROTATION_DEG, candidateDeg));
////        }
////
////        // 6. 下发目标角度给主 Axon 连续旋转舵机
////        robot.axonTurretArmL.trackTargetRotation(turretTargetDeg);
////
////        // 7. 遥测输出（纯数字供 Graph 绘图，文字提示操作手转向）
////        telemetry.addData("Turret Target (deg)", turretTargetDeg);
////        telemetry.addData("Turret Actual (deg)", currentTurretDeg);
////        telemetry.addData("Turret Error (deg)", turretTargetDeg - currentTurretDeg);
////        telemetry.addData("Turret Servo Power", robot.axonTurretArmL.getPower());
////        telemetry.addData("Turret Reachable", turretReachable ? 1 : 0);
////        if (!turretReachable) {
////            telemetry.addData("TURN ROBOT WARNING", "Turn %.0f deg %s!",
////                    Math.abs(turretOverLimitDeg), turretOverLimitDeg > 0 ? "LEFT" : "RIGHT");
////        }
////    }
////
////
//////    private void updateTurretControl() {
//////        // 1. sync PID gains from the web UI (unchanged)
//////        if (TURRET_KP != lastTurretKP || TURRET_KI != lastTurretKI || TURRET_KD != lastTurretKD) {
//////            robot.axonTurretArmL.setPidCoeffs(TURRET_KP, TURRET_KI, TURRET_KD);
//////            lastTurretKP = TURRET_KP; lastTurretKI = TURRET_KI; lastTurretKD = TURRET_KD;
//////        }
//////        if (TURRET_MAX_POWER != lastTurretMaxPower) {
//////            robot.axonTurretArmL.setMaxPower(TURRET_MAX_POWER);
//////            lastTurretMaxPower = TURRET_MAX_POWER;
//////        }
//////
//////        // 2. auto-aim toggle
//////        if (currentGamepad2.back && !previousGamepad2.back) turretAutoAim = !turretAutoAim;
//////
//////        // 3. manual trim, now bounded
//////        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)        turretManualTrimDeg += 2.0;
//////        else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) turretManualTrimDeg -= 2.0;
//////        turretManualTrimDeg = Math.max(-TURRET_TRIM_LIMIT_DEG,
//////                Math.min(TURRET_TRIM_LIMIT_DEG, turretManualTrimDeg));
//////
//////        Pose p = follower.pose();
//////        Velocity v = follower.velocity();                 // com.pedropathing.math.Velocity
//////        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
//////
//////        double targetX = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
//////        double targetY = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
//////
//////        // 4. shoot-on-the-move: the ball inherits the robot's velocity, so aim at a
//////        //    virtual target displaced backwards along that velocity.
//////        if (SHOT_FLIGHT_TIME_S > 0) {
//////            targetX -= v.vx * SHOT_FLIGHT_TIME_S;
//////            targetY -= v.vy * SHOT_FLIGHT_TIME_S;
//////        }
//////
//////        double dx = targetX - p.x();
//////        double dy = targetY - p.y();
//////        double rangeSq = dx * dx + dy * dy;
//////
//////        isInTurretDeadZone = (TURRET_DEADZONE_RADIUS_IN > 0)
//////                && (rangeSq < TURRET_DEADZONE_RADIUS_IN * TURRET_DEADZONE_RADIUS_IN);
//////
//////        if (isInTurretDeadZone || !turretAutoAim) {
//////            turretTargetDeg = 0.0;
//////        } else {
//////            double relAngle = Math.atan2(dy, dx) - p.heading();
//////            while (relAngle >  Math.PI) relAngle -= 2 * Math.PI;
//////            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
//////
//////            // 5. FEEDFORWARD — the rate the turret must turn just to hold aim
//////            double bearingRate = (rangeSq > 1e-6) ? (dy * v.vx - dx * v.vy) / rangeSq : 0.0;
//////            double turretRate  = bearingRate - v.omega;        // rad/s, turret frame
//////
//////            double baseTargetDeg = Math.toDegrees(relAngle + turretRate * TURRET_LOOKAHEAD_S)
//////                    + turretManualTrimDeg;
//////
//////            // 6. pick the nearest legal equivalent within ±270° (unchanged, this part was right)
//////            double candidateDeg = baseTargetDeg;
//////            if (candidateDeg - currentTurretDeg > 180.0
//////                    && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
//////                candidateDeg -= 360.0;
//////            } else if (candidateDeg - currentTurretDeg < -180.0
//////                    && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
//////                candidateDeg += 360.0;
//////            }
//////            turretTargetDeg = Math.max(-TURRET_MAX_ROTATION_DEG,
//////                    Math.min(TURRET_MAX_ROTATION_DEG, candidateDeg));
//////        }
//////
//////        robot.axonTurretArmL.trackTargetRotation(turretTargetDeg);
//////
////////        if (panelsTelemetry != null) {
////////            panelsTelemetry.addData("Turret Target (deg)", turretTargetDeg);
////////            panelsTelemetry.addData("Turret Actual (deg)", currentTurretDeg);
////////            panelsTelemetry.addData("Turret Error (deg)", turretTargetDeg - currentTurretDeg);
////////            panelsTelemetry.addData("Turret Servo Power", robot.axonTurretArmL.getPower());
////////            panelsTelemetry.addData("Turret FF (deg/s)",
////////                    Math.toDegrees((rangeSq > 1e-6 ? (dy * v.vx - dx * v.vy) / rangeSq : 0) - v.omega));
////////            panelsTelemetry.addData("Range (in)", Math.sqrt(rangeSq));
////////        }
//////
//////        // 只需写一次 telemetry.addData，双端自动同步，且纯数字可直接被 Panels Graph 识别画图！
//////        telemetry.addData("Turret Target (deg)", turretTargetDeg);
//////        telemetry.addData("Turret Actual (deg)", currentTurretDeg);
//////        telemetry.addData("Turret Error (deg)", turretTargetDeg - currentTurretDeg);
//////        telemetry.addData("Turret Servo Power", robot.axonTurretArmL.getPower());
//////        telemetry.addData("Turret FF (deg/s)",
//////                Math.toDegrees((rangeSq > 1e-6 ? (dy * v.vx - dx * v.vy) / rangeSq : 0) - v.omega));
//////        telemetry.addData("Range (in)", Math.sqrt(rangeSq));
//////
//////
//////    }
////
////
//////    private void updateTurretControl() {
//////        // 1. 实时同步 Panels 网页端修改的炮台 PID 参数
//////        if (TURRET_KP != lastTurretKP || TURRET_KI != lastTurretKI || TURRET_KD != lastTurretKD) {
//////            robot.axonTurretArmL.setPidCoeffs(TURRET_KP, TURRET_KI, TURRET_KD);
//////            lastTurretKP = TURRET_KP;
//////            lastTurretKI = TURRET_KI;
//////            lastTurretKD = TURRET_KD;
//////        }
//////        if (TURRET_MAX_POWER != lastTurretMaxPower) {
//////            robot.axonTurretArmL.setMaxPower(TURRET_MAX_POWER);
//////            lastTurretMaxPower = TURRET_MAX_POWER;
//////        }
//////
//////        // 2. Gamepad2 Back 键切换自动跟踪 / 手动回中模式
//////        if (currentGamepad2.back && !previousGamepad2.back) {
//////            turretAutoAim = !turretAutoAim;
//////        }
//////
//////        // 3. Gamepad2 左右方向键提供 ±2° 实时微调补偿
//////        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
//////            turretManualTrimDeg += 2.0;
//////        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
//////            turretManualTrimDeg -= 2.0;
//////        }
//////
//////        Pose currentPose = follower.pose();
//////        double robotY = currentPose.y();
//////        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
//////
//////        // 4. 检查机器人中心 Y 坐标是否位于 [62.0, 79.5] 禁区内
//////        isInTurretDeadZone = (robotY >= TURRET_DISABLE_Y_MIN && robotY <= TURRET_DISABLE_Y_MAX);
//////
//////        if (isInTurretDeadZone || !turretAutoAim) {
//////            // 处于禁区或手动关闭自瞄时：关闭追踪，Turret 回到初始位置指向机器人前进方向 (0°)
//////            turretTargetDeg = 0.0;
//////        } else {
//////            double targetX = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
//////            double targetY = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
//////
//////            double dx = targetX - currentPose.x();
//////            double dy = targetY - currentPose.y();
//////            double overallAngle = Math.atan2(dy, dx);
//////
//////            // 计算相对机器人车头方向的夹角，并归一化到 [-180°, +180°]
//////            double relAngle = overallAngle - currentPose.heading();
//////            while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
//////            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
//////
//////            double baseTargetDeg = Math.toDegrees(relAngle) + turretManualTrimDeg;
//////
//////            // 利用 ±270° 行程选择距离当前角度最近的合法等效角度（避免不必要的整圈反绕）
//////            double candidateDeg = baseTargetDeg;
//////            if (candidateDeg - currentTurretDeg > 180.0 && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
//////                candidateDeg -= 360.0;
//////            } else if (candidateDeg - currentTurretDeg < -180.0 && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
//////                candidateDeg += 360.0;
//////            }
//////
//////            // 严格限制顺时针和逆时针最大旋转角度不超过 270 度
//////            turretTargetDeg = Math.max(-TURRET_MAX_ROTATION_DEG, Math.min(TURRET_MAX_ROTATION_DEG, candidateDeg));
//////        }
//////
//////        // 5. 下发目标角度给主 Axon 连续旋转舵机
//////        robot.axonTurretArmL.trackTargetRotation(turretTargetDeg);
//////
//////        // 6. 推送炮台目标值、实际值、误差与输出功率到 Panels 网页端，供 Graph 实时画图调 PID
//////        if (panelsTelemetry != null) {
//////            panelsTelemetry.addData("Turret Target (deg)", turretTargetDeg);
//////            panelsTelemetry.addData("Turret Actual (deg)", currentTurretDeg);
//////            panelsTelemetry.addData("Turret Error (deg)", turretTargetDeg - currentTurretDeg);
//////            panelsTelemetry.addData("Turret Servo Power", robot.axonTurretArmL.getPower());
//////
////////            panelsTelemetry.debug("Turret Target (deg)", turretTargetDeg);
////////            panelsTelemetry.debug("Turret Actual (deg)", currentTurretDeg);
////////            panelsTelemetry.debug("Turret Error (deg)", turretTargetDeg - currentTurretDeg);
////////            panelsTelemetry.debug("Turret Servo Power", robot.axonTurretArmL.getPower());
//////        }
//////        // 6. 推送炮台数据到 Panels：使用 graph() 专供折线图选择，debug() 供文本列表查看
////////        if (panelsTelemetry != null) {
////////            panelsTelemetry.graph("Turret Target (deg)", turretTargetDeg);
////////            panelsTelemetry.graph("Turret Actual (deg)", currentTurretDeg);
////////            panelsTelemetry.graph("Turret Error (deg)", turretTargetDeg - currentTurretDeg);
////////            panelsTelemetry.graph("Turret Servo Power", robot.axonTurretArmL.getPower());
////////        }
//////
//////
//////    }
////
////    //    private void updateTurretControl() {
//////        // Gamepad2 Back 键切换自动跟踪 / 手动回中模式
//////        if (currentGamepad2.back && !previousGamepad2.back) {
//////            turretAutoAim = !turretAutoAim;
//////        }
//////
//////        // Gamepad2 左右方向键提供 ±2° 实时微调补偿
//////        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
//////            turretManualTrimDeg += 2.0;
//////        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
//////            turretManualTrimDeg -= 2.0;
//////        }
//////
//////        Pose currentPose = follower.pose();
//////        double robotY = currentPose.y();
//////        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
//////
//////        // 检查机器人中心 Y 坐标是否位于 [62.0, 79.5] 禁区内
//////        isInTurretDeadZone = (robotY >= TURRET_DISABLE_Y_MIN && robotY <= TURRET_DISABLE_Y_MAX);
//////
//////        if (isInTurretDeadZone || !turretAutoAim) {
//////            // 处于禁区或手动关闭自瞄时：关闭追踪，Turret 回到初始位置指向机器人前进方向 (0°)
//////            turretTargetDeg = 0.0;
//////        } else {
//////            double targetX = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
//////            double targetY = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
//////
//////            double dx = targetX - currentPose.x();
//////            double dy = targetY - currentPose.y();
//////            double overallAngle = Math.atan2(dy, dx);
//////
//////            // 计算相对机器人车头方向的夹角，并归一化到 [-180°, +180°]
//////            double relAngle = overallAngle - currentPose.heading();
//////            while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
//////            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
//////
//////            double baseTargetDeg = Math.toDegrees(relAngle) + turretManualTrimDeg;
//////
//////            // 利用 ±270° 行程选择距离当前角度最近的合法等效角度（避免不必要的整圈反绕）
//////            double candidateDeg = baseTargetDeg;
//////            if (candidateDeg - currentTurretDeg > 180.0 && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
//////                candidateDeg -= 360.0;
//////            } else if (candidateDeg - currentTurretDeg < -180.0 && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
//////                candidateDeg += 360.0;
//////            }
//////
//////            // 严格限制顺时针和逆时针最大旋转角度不超过 270 度
//////            turretTargetDeg = Math.max(-TURRET_MAX_ROTATION_DEG, Math.min(TURRET_MAX_ROTATION_DEG, candidateDeg));
//////        }
//////
//////        // 下发目标角度给主 Axon 连续旋转舵机
//////        robot.axonTurretArmL.trackTargetRotation(turretTargetDeg);
//////
//////        // ✅ 新增：推送炮台目标值、实际值、误差与输出功率到 Panels 网页端，供 Graph 实时画图调 PID
//////        if (panelsTelemetry != null) {
//////            panelsTelemetry.debug("Turret Target (deg)", turretTargetDeg);
//////            panelsTelemetry.debug("Turret Actual (deg)", currentTurretDeg);
//////            panelsTelemetry.debug("Turret Error (deg)", turretTargetDeg - currentTurretDeg);
//////            panelsTelemetry.debug("Turret Servo Power", robot.axonTurretArmL.getPower());
//////        }
//////    }
////
//////    private void updateTurretControl() {
//////        // Gamepad2 Back 键切换自动跟踪 / 手动回中模式
//////        if (currentGamepad2.back && !previousGamepad2.back) {
//////            turretAutoAim = !turretAutoAim;
//////        }
//////
//////        // Gamepad2 左右方向键提供 ±2° 实时微调补偿
//////        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
//////            turretManualTrimDeg += 2.0;
//////        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
//////            turretManualTrimDeg -= 2.0;
//////        }
//////
//////        Pose currentPose = follower.pose();
//////        double robotY = currentPose.y();
//////
//////        // 检查机器人中心 Y 坐标是否位于 [62.0, 79.5] 禁区内
//////        isInTurretDeadZone = (robotY >= TURRET_DISABLE_Y_MIN && robotY <= TURRET_DISABLE_Y_MAX);
//////
//////        if (isInTurretDeadZone || !turretAutoAim) {
//////            // 处于禁区或手动关闭自瞄时：关闭追踪，Turret 回到初始位置指向机器人前进方向 (0°)
//////            turretTargetDeg = 0.0;
//////        } else {
//////            double targetX = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
//////            double targetY = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
//////
//////            double dx = targetX - currentPose.x();
//////            double dy = targetY - currentPose.y();
//////            double overallAngle = Math.atan2(dy, dx);
//////
//////            // 计算相对机器人车头方向的夹角，并归一化到 [-180°, +180°]
//////            double relAngle = overallAngle - currentPose.heading();
//////            while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
//////            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
//////
//////            double baseTargetDeg = Math.toDegrees(relAngle) + turretManualTrimDeg;
//////
//////            // 利用 ±270° 行程选择距离当前角度最近的合法等效角度（避免不必要的整圈反绕）
//////            double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
//////            double candidateDeg = baseTargetDeg;
//////            if (candidateDeg - currentTurretDeg > 180.0 && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
//////                candidateDeg -= 360.0;
//////            } else if (candidateDeg - currentTurretDeg < -180.0 && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
//////                candidateDeg += 360.0;
//////            }
//////
//////            // 严格限制顺时针和逆时针最大旋转角度不超过 270 度
//////            turretTargetDeg = Math.max(-TURRET_MAX_ROTATION_DEG, Math.min(TURRET_MAX_ROTATION_DEG, candidateDeg));
//////        }
//////
//////        // 下发目标角度给主 Axon 连续旋转舵机
//////        robot.axonTurretArmL.trackTargetRotation(turretTargetDeg);
//////    }
////
////    private void handlePositionReset() {
////        if (currentGamepad1.back && currentGamepad1.start) {
////            // 重置回本赛季设定的初始位姿 (83.5, 133.5, 0°)
////            follower.setPose(poseFactory.of(83.5, 133.5, 0.0));
////            gamepad1.rumble(300);
////        } else if (currentGamepad1.back && !previousGamepad1.back) {
////            if (robot.finalAutoX != 0 || robot.finalAutoY != 0) {
////                follower.setPose(new Pose(robot.finalAutoX, robot.finalAutoY, robot.finalAutoHeading));
////            }
////        }
////    }
////
////    public void initShooterPIDF() {
////        robot.shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////
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
////    public void updateIntake() {
////        if (currentGamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger <= 0.1) {
////            robot.BlockageArmL.setPosition(blockageblockposition);
////            robot.BlockageArmR.setPosition(blockageblockposition);
////            robot.IntakeMotorL.setPower(intakePowerIntake);
////            robot.IntakeMotorR.setPower(intakePowerIntake);
////            intakeIsOn = true;
////        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
////            robot.IntakeMotorL.setPower(intakePowerDump);
////            robot.IntakeMotorR.setPower(intakePowerDump);
////            intakeIsOn = true;
////        } else if (currentGamepad1.y && !previousGamepad1.y) {
////            stopIntake();
////        }
////    }
////    private void stopIntake() {
////        robot.IntakeMotorL.setPower(intakePowerOff);
////        robot.IntakeMotorR.setPower(intakePowerOff);
////        intakeIsOn = false;
////        isShooterAtSpeed = false;
////        fireRequested = false;
////        if (isFarMode) {
////            startShooterIdleFar();
////        } else {
////            startShooterIdle();
////        }
////    }
////    public void updateShooter() {
////        if (currentGamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger <= 0.1) {
////            if (isFarMode) {
////                startShooterFar();
////            } else {
////                startShooter();
////            }
////        }
////        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
////            fireRequested = true;
////            robot.BlockageArmL.setPosition(blockagereleaseposition);
////            robot.BlockageArmR.setPosition(blockagereleaseposition);
////            robot.IntakeMotorL.setPower(intakePowerShoot);
////            robot.IntakeMotorR.setPower(intakePowerShoot);
////            intakeIsOn = true;
////        }
////        if (currentGamepad1.b && !previousGamepad1.b) {
////            stopShooter();
////            robot.BlockageArmL.setPosition(blockageblockposition);
////            robot.BlockageArmR.setPosition(blockageblockposition);
////        }
////        if (currentGamepad1.a && !previousGamepad1.a) {
////            stopIntakeAndShooter();
////        }
////    }
////    private void startShooterIdle() {
////        robot.IntakeMotorL.setPower(0);
////        robot.IntakeMotorR.setPower(0);
////        robot.setHoodPosition(0.5);
////        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeed));
////        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeed));
////    }
////    private void startShooterIdleFar() {
////        robot.IntakeMotorL.setPower(0);
////        robot.IntakeMotorR.setPower(0);
////        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeedfar));
////        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeedfar));
////    }
////    private void startShooter() {
////        robot.IntakeMotorL.setPower(0);
////        robot.IntakeMotorR.setPower(0);
////        robot.setHoodPosition(0.5);
////        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
////        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
////        shooterIsOn = true;
////    }
////    private void startShooterFar() {
////        robot.IntakeMotorL.setPower(0);
////        robot.IntakeMotorR.setPower(0);
////        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEEDfar));
////        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEEDfar));
////        shooterIsOn = true;
////    }
////    private void stopShooter() {
////        if (isFarMode) {
////            startShooterIdleFar();
////        } else {
////            startShooterIdle();
////        }
////        intakeIsOn = false;
////        isShooterAtSpeed = false;
////        fireRequested = false;
////        shooterIsOn = false;
////    }
////    private void stopIntakeAndShooter() {
////        robot.MasterShooterMotorL.setVelocity(0);
////        robot.SlaveShooterMotorR.setVelocity(0);
////        robot.IntakeMotorL.setPower(0);
////        robot.IntakeMotorR.setPower(0);
////        intakeIsOn = false;
////        isShooterAtSpeed = false;
////        fireRequested = false;
////        shooterIsOn = false;
////    }
////    private void checkShooterVelocity() {
////        double activeTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
////        if (Math.abs(cachedShooterVeloL - activeTarget) <= toleranceforShoot) {
////            isShooterAtSpeed = true;
////        } else {
////            isShooterAtSpeed = false;
////            fireRequested = false;
////        }
////    }
////    private void updateHood() {
////        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
////            robot.setHoodPosition(HoodArmfarposition);
////        } else if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
////            robot.setHoodPosition(HoodArmcloseposition);
////        } else if (currentGamepad2.y && !previousGamepad2.y) {
////            robot.setHoodPosition(0.50);
////        }
////    }
////    private void updateBlockage() {
////        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
////            robot.BlockageArmL.setPosition(blockageblockposition);
////            robot.BlockageArmR.setPosition(blockageblockposition);
////        } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
////            robot.BlockageArmL.setPosition(blockagereleaseposition);
////            robot.BlockageArmR.setPosition(blockagereleaseposition);
////        }
////    }
//////    private void updateLEDs() {
//////        if (isShooterAtSpeed && !isInTurretDeadZone && robot.axonTurretArmL.isAtTarget(4.0)) {
//////            setLEDColor("GREEN");
//////        } else {
//////            setLEDColor("RED");
//////        }
//////    }
////
////    private void updateLEDs() {
////        if (isShooterAtSpeed && !isInTurretDeadZone && turretReachable && robot.axonTurretArmL.isAtTarget(4.0)) {
////            setLEDColor("GREEN");
////        } else {
////            setLEDColor("RED");
////        }
////    }
////
////    private void setLEDColor(String color) {
////        if (color.equals(lastLedColor)) return;
////        try {
////            if ("GREEN".equals(color)) {
////                if (robot.greenLED != null) robot.greenLED.setState(false);
////                if (robot.redLED != null) robot.redLED.setState(true);
////            } else if ("RED".equals(color)) {
////                if (robot.greenLED != null) robot.greenLED.setState(true);
////                if (robot.redLED != null) robot.redLED.setState(false);
////            }
////            lastLedColor = color;
////        } catch (Exception e) {
////            telemetry.addData("LED Error", e.getMessage());
////        }
////    }
////    private void updateHiveStatus() {
////        if (robot.myAprilTagProcessor == null) return;
////        robot.hiveScorable = false;
////        robot.hiveClusterName = "";
////        robot.hiveRoll = 0.0;
////        robot.hivePercentFound = 0.0;
////        for (AprilTagDetection d : robot.myAprilTagProcessor.getDetections()) {
////            double rawRoll = (d.ftcPose != null) ? d.ftcPose.roll : 0.0;
////            if (d instanceof AprilTagClusterDetection) {
////                AprilTagClusterDetection cluster = (AprilTagClusterDetection) d;
////                String name = (cluster.metadata != null && cluster.metadata.name != null)
////                        ? cluster.metadata.name : "Unknown_Cluster";
////                boolean allianceMatch = name.startsWith(robot.alliance == Alliance.RED ? "R" : "B");
////                if (!allianceMatch || Math.abs(rawRoll) >= 90.0 || cluster.percentClusterFound < 0.5) continue;
////                robot.hiveScorable = true;
////                robot.hiveClusterName = name;
////                robot.hiveRoll = rawRoll;
////                robot.hivePercentFound = cluster.percentClusterFound;
////                break;
////            } else if (d instanceof AprilTagSingleDetection) {
////                AprilTagSingleDetection single = (AprilTagSingleDetection) d;
////                String name = (single.metadata != null && single.metadata.name != null)
////                        ? single.metadata.name : ("ID_" + single.id);
////                robot.hiveScorable = true;
////                robot.hiveClusterName = name;
////                robot.hiveRoll = rawRoll;
////                robot.hivePercentFound = 1.0;
////                break;
////            }
////        }
////    }
////    public void updateDrivetrain_FieldCentric() {
////        double y = -gamepad1.left_stick_y;
////        double x = gamepad1.left_stick_x;
////        double rx = gamepad1.right_stick_x * DriveTrains_smoothTurn;
//////        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
////        double botHeading = follower.pose().heading();
////        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
////        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
////        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
////        double frontLeftPower  = (rotY + rotX + rx) / denominator;
////        double backLeftPower   = (rotY - rotX + rx) / denominator;
////        double frontRightPower = (rotY - rotX - rx) / denominator;
////        double backRightPower  = (rotY + rotX - rx) / denominator;
////        robot.leftFrontMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
////        robot.leftRearMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
////        robot.rightFrontMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
////        robot.rightRearMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
////    }
////    private void handleIMUReset() {
////        if (imuResetInCooldown) {
////            if (imuResetTimer.milliseconds() > IMU_RESET_COOLDOWN_MS) {
////                imuResetInCooldown = false;
////            } else {
////                return;
////            }
////        }
////        if (currentGamepad1.dpad_up && !previousGamepad1.back) {
//////            robot.imu.resetYaw();
////            follower.setHeading(0);
////            imuResetInCooldown = true;
////            imuResetTimer.reset();
////            gamepad1.rumble(300);
////        }
////    }
////    private void updateAllTelemetry(double loopTimeMs) {
////        Pose pose = follower.pose();
////        double activeTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
//////        if (panelsTelemetry != null) {
//////            panelsTelemetry.addData("Loop Time (ms)", loopTimeMs);
//////            panelsTelemetry.addData("Shoot Mode", isFarMode ? "FAR" : "NEAR");
//////            panelsTelemetry.addData("Shooter Target", activeTarget);
//////            panelsTelemetry.addData("Shooter Left Velo", cachedShooterVeloL);
//////            panelsTelemetry.addData("Shooter Right Velo", cachedShooterVeloR);
//////            panelsTelemetry.addData("Turret AutoAim", turretAutoAim);
//////            panelsTelemetry.addData("Turret DeadZone (Y 62-79.5)", isInTurretDeadZone);
//////            panelsTelemetry.addData("Turret Target (deg)", turretTargetDeg);
//////            panelsTelemetry.addData("Turret Current (deg)", robot.axonTurretArmL.getTotalRotation());
//////            panelsTelemetry.addData("Hive Scorable", robot.hiveScorable);
//////            panelsTelemetry.addData("Hive Cluster", robot.hiveClusterName);
//////        }
////        // 纯数字独立输出，方便 Panels Graph 直接拖入画曲线
////        telemetry.addData("Loop Time (ms)", loopTimeMs);
////        telemetry.addData("Shooter Target", activeTarget);
////        telemetry.addData("Shooter Left Velo", cachedShooterVeloL);
////        telemetry.addData("Shooter Right Velo", cachedShooterVeloR);
////
////        // 状态文字输出
////        telemetry.addData("Shoot Mode", isFarMode ? "FAR" : "NEAR");
////        telemetry.addData("Pose (X/Y/H)", "%.1f / %.1f / %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
////        telemetry.addData("Turret AutoAim", turretAutoAim);
////        telemetry.addData("Turret DeadZone", isInTurretDeadZone);
////        telemetry.addData("Hive Scorable", robot.hiveScorable ? 1 : 0);
////        telemetry.addData("Hive Cluster", robot.hiveClusterName);
////
////        telemetry.addData("Loop Time", "%.1f ms | Mode: %s", loopTimeMs, isFarMode ? "FAR" : "NEAR");
////        telemetry.addData("Pose (X/Y/H)", "%.1f / %.1f / %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
////        telemetry.addData("Shooter (T/L/R)", "%.0f / %.0f / %.0f", activeTarget, cachedShooterVeloL, cachedShooterVeloR);
////        telemetry.addData("Turret", "Auto:%b | DeadZone:%b | Set:%.1f° | Cur:%.1f°",
////                turretAutoAim, isInTurretDeadZone, turretTargetDeg, robot.axonTurretArmL.getTotalRotation());
////        telemetry.addData("Hive", "%b (%s)", robot.hiveScorable, robot.hiveClusterName);
////    }
////}
////
////
////
////
////
////////////////////////////////////////11192026 調整目標和初始位置之前代碼
//////package org.firstinspires.ftc.teamcode.pedro;
//////
//////import com.bylazar.configurables.annotations.Configurable;
//////import com.bylazar.telemetry.PanelsTelemetry;
//////import com.bylazar.telemetry.TelemetryManager;
//////import com.pedropathing.follower.Follower;
//////import com.pedropathing.math.Pose;
//////import com.pedropathing.api.PoseFactory;
//////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//////import com.qualcomm.robotcore.hardware.DcMotor;
//////import com.qualcomm.robotcore.hardware.Gamepad;
//////import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//////import com.qualcomm.robotcore.util.ElapsedTime;
//////import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//////import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
//////import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//////import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
//////@Configurable
//////@TeleOp(name = "AAAAAA turret testing TeleOpDecodeRobotBlueBioBuzzSeason V1 11182026")
//////public class TeleOpDecodeRobotBlueBioBuzzSeason extends LinearOpMode {
//////
//////    // 调试/比赛摄像头预览开关（正式比赛建议改为 false 节省 CPU）
//////    public static boolean ENABLE_CAMERA_LIVE_VIEW = true;
//////    // 场地目标坐标（Pedro Pathing 0-144 坐标系，支持 Panels 实时微调）
//////    public static double BLUE_GOAL_X = 0.0;
//////    public static double BLUE_GOAL_Y = 144.0;
//////    public static double RED_GOAL_X = 144.0;
//////    public static double RED_GOAL_Y = 144.0;
//////    public float DriveTrains_ReducePOWER = 1.0f;
//////    public float DriveTrains_smoothTurn = 1.0f;
//////    public float intakePowerShoot = 0.90f;
//////    public float intakePowerIntake = 0.95f;
//////    public float intakePowerDump = -0.65f;
//////    public float intakePowerOff = 0.0f;
//////    public static final double blockageblockposition = 0.10;
//////    public static final double blockagereleaseposition = 0.18;
//////    public static final double HoodArmfarposition = 0.30;
//////    public static final double HoodArmcloseposition = 0.35;
//////    private static final double Far_SHOOTER_TARGET_Velocity = 1400;
//////    private static double Med_SHOOTER_TARGET_SPEED = 1050;
//////    public static double toleranceforShoot = 100;
//////
//////    @Configurable
//////    public static class ShooterPIDFConfig {
//////        public static double kP = 300.0;
//////        public static double kI = 0.0;
//////        public static double kD = 0.0;
//////        public static double kF = 15.0;
//////        public static double targetSPEED = Med_SHOOTER_TARGET_SPEED;
//////        public static double flyWheelIdleSpeed = Med_SHOOTER_TARGET_SPEED * 0.6;
//////        public static double targetSPEEDfar = Far_SHOOTER_TARGET_Velocity;
//////        public static double flyWheelIdleSpeedfar = Far_SHOOTER_TARGET_Velocity * 0.6;
//////    }
//////
//////    private final HardwareDecodeRobotBlueBioBuzzSeason robot = new HardwareDecodeRobotBlueBioBuzzSeason();
//////    private final Gamepad currentGamepad1 = new Gamepad();
//////    private final Gamepad currentGamepad2 = new Gamepad();
//////    private final Gamepad previousGamepad1 = new Gamepad();
//////    private final Gamepad previousGamepad2 = new Gamepad();
//////    private Follower follower;
//////    private TelemetryManager panelsTelemetry;
//////    private final PoseFactory poseFactory = PoseFactory.degrees();
//////    private final ElapsedTime imuResetTimer = new ElapsedTime();
//////    private final ElapsedTime loopTimer = new ElapsedTime();
//////    // 状态标志与每帧缓存
//////    private boolean isFarMode = false;
//////    private boolean isShooterAtSpeed = false;
//////    private boolean shooterIsOn = false;
//////    private boolean intakeIsOn = false;
//////    private boolean fireRequested = false;
//////    private boolean imuResetInCooldown = false;
//////    private static final long IMU_RESET_COOLDOWN_MS = 300;
//////    // 炮台控制变量
//////    private boolean turretAutoAim = true;
//////    private double turretManualTrimDeg = 0.0;
//////    private double turretTargetDeg = 0.0;
//////    // 缓存速度与 LED 防抖
//////    private double cachedShooterVeloL = 0.0;
//////    private double cachedShooterVeloR = 0.0;
//////    private String lastLedColor = "";
//////
//////    @Override
//////    public void runOpMode() {
//////        robot.init(hardwareMap);
//////        robot.alliance = Alliance.BLUE;
//////        robot.initVision(ENABLE_CAMERA_LIVE_VIEW);
//////        initShooterPIDF();
//////
//////        follower = Constants.create(hardwareMap);
//////        Pose startPose = new Pose(32, 92.25, 0);
//////        Pose fallbackPose = poseFactory.of(0, 0, 0);
//////        follower.setPose(startPose == null ? fallbackPose : startPose);
//////        follower.update();
//////
//////        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//////        panelsTelemetry.debug("Status", "Championship TeleOp Ready");
//////        panelsTelemetry.update(telemetry);
//////
//////        waitForStart();
//////        loopTimer.reset();
//////
//////        while (opModeIsActive()) {
//////            // 1. 计算单帧 Loop Time 并刷新 REV Hub 批量缓存
//////            double loopTimeMs = loopTimer.milliseconds();
//////            loopTimer.reset();
//////            robot.clearBulkCache();
//////
//////            // 2. 本轮主循环单次读取左右飞轮速度并缓存
//////            cachedShooterVeloL = Math.abs(robot.MasterShooterMotorL.getVelocity());
//////            cachedShooterVeloR = Math.abs(robot.SlaveShooterMotorR.getVelocity());
//////
//////            // 3. 更新手柄边沿状态
//////            previousGamepad1.copy(currentGamepad1);
//////            previousGamepad2.copy(currentGamepad2);
//////            currentGamepad1.copy(gamepad1);
//////            currentGamepad2.copy(gamepad2);
//////
//////            // 4. 更新机器人定位与底盘控制
//////            follower.update();
//////            updateDrivetrain_FieldCentric();
//////            handleIMUReset();
//////            handlePositionReset();
//////
//////            // 5. 远近射模式切换（Gamepad2 X 切远射 / B 切近射）
//////            if (currentGamepad2.x && !previousGamepad2.x) {
//////                isFarMode = true;
//////            } else if (currentGamepad2.b && !previousGamepad2.b) {
//////                isFarMode = false;
//////            }
//////
//////            // 6. 机构控制（全非阻塞边沿触发）
//////            updateIntake();
//////            updateShooter();
//////            checkShooterVelocity();
//////            updateBlockage();
//////            updateHood();
//////            updateLEDs();
//////
//////            // 7. 炮台闭环追踪与视觉状态刷新
//////            updateTurretControl();
//////            robot.updateTurretAxons();
//////            updateHiveStatus();
//////
//////            // 8. 统一双端刷新遥测
//////            updateAllTelemetry(loopTimeMs);
//////            panelsTelemetry.update(telemetry);
//////        }
//////    }
//////
//////    /**
//////     * 炮台自动追踪与手动微调控制（取代原有的阻塞式 turretupdate）
//////     */
//////    private void updateTurretControl() {
//////        // Gamepad2 Back 键切换自动跟踪 / 手动回中模式
//////        if (currentGamepad2.back && !previousGamepad2.back) {
//////            turretAutoAim = !turretAutoAim;
//////        }
//////
//////        // Gamepad2 左右方向键提供 ±2° 实时微调补偿
//////        if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
//////            turretManualTrimDeg += 2.0;
//////        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
//////            turretManualTrimDeg -= 2.0;
//////        }
//////
//////        if (turretAutoAim) {
//////            Pose currentPose = follower.pose();
//////            double targetX = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
//////            double targetY = (robot.alliance == Alliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
//////
//////            double dx = targetX - currentPose.x();
//////            double dy = targetY - currentPose.y();
//////            double overallAngle = Math.atan2(dy, dx);
//////
//////            double relAngle = overallAngle - currentPose.heading();
//////            while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
//////            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
//////
//////            double rawDeg = Math.toDegrees(relAngle) + turretManualTrimDeg;
//////            // 安全限位 [-90°, +90°]
//////            turretTargetDeg = Math.max(-90.0, Math.min(90.0, rawDeg));
//////        } else {
//////            // 手动模式下归零 + 微调
//////            turretTargetDeg = Math.max(-90.0, Math.min(90.0, turretManualTrimDeg));
//////        }
//////
//////        // 下发目标角度给左右双 Axon 连续旋转舵机
//////        robot.axonTurretArmL.trackTargetRotation(turretTargetDeg);
////////        robot.axonTurretArmR.trackTargetRotation(turretTargetDeg);
//////    }
//////
//////    private void handlePositionReset() {
//////        if (currentGamepad1.back && currentGamepad1.start) {
//////            follower.setPose(new Pose(70, 70, 0));
//////            gamepad1.rumble(300);
//////        } else if (currentGamepad1.back && !previousGamepad1.back) {
//////            if (robot.finalAutoX != 0 || robot.finalAutoY != 0) {
//////                follower.setPose(new Pose(robot.finalAutoX, robot.finalAutoY, robot.finalAutoHeading));
//////            }
//////        }
//////    }
//////
//////    public void initShooterPIDF() {
//////        robot.shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//////        robot.shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//////
//////        PIDFCoefficients pidf = new PIDFCoefficients(
//////                ShooterPIDFConfig.kP,
//////                ShooterPIDFConfig.kI,
//////                ShooterPIDFConfig.kD,
//////                ShooterPIDFConfig.kF
//////        );
//////        robot.shooterL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//////        robot.shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//////
//////        robot.shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////        robot.shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////    }
//////
//////    public void updateIntake() {
//////        if (currentGamepad1.left_trigger > 0.1 && previousGamepad1.left_trigger <= 0.1) {
//////            robot.BlockageArmL.setPosition(blockageblockposition);
//////            robot.BlockageArmR.setPosition(blockageblockposition);
//////            robot.IntakeMotorL.setPower(intakePowerIntake);
//////            robot.IntakeMotorR.setPower(intakePowerIntake);
//////            intakeIsOn = true;
//////        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
//////            robot.IntakeMotorL.setPower(intakePowerDump);
//////            robot.IntakeMotorR.setPower(intakePowerDump);
//////            intakeIsOn = true;
//////        } else if (currentGamepad1.y && !previousGamepad1.y) {
//////            stopIntake();
//////        }
//////    }
//////    private void stopIntake() {
//////        robot.IntakeMotorL.setPower(intakePowerOff);
//////        robot.IntakeMotorR.setPower(intakePowerOff);
//////        intakeIsOn = false;
//////        isShooterAtSpeed = false;
//////        fireRequested = false;
//////// 切回对应模式的怠速，不重置编码器
//////        if (isFarMode) {
//////            startShooterIdleFar();
//////        } else {
//////            startShooterIdle();
//////        }
//////    }
//////    public void updateShooter() {
//////        if (currentGamepad1.right_trigger > 0.1 && previousGamepad1.right_trigger <= 0.1) {
//////            if (isFarMode) {
//////                startShooterFar();
//////            } else {
//////                startShooter();
//////            }
//////        }
//////        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
//////            fireRequested = true;
//////            robot.BlockageArmL.setPosition(blockagereleaseposition);
//////            robot.BlockageArmR.setPosition(blockagereleaseposition);
//////            robot.IntakeMotorL.setPower(intakePowerShoot);
//////            robot.IntakeMotorR.setPower(intakePowerShoot);
//////            intakeIsOn = true;
//////        }
//////        if (currentGamepad1.b && !previousGamepad1.b) {
//////            stopShooter();
//////            robot.BlockageArmL.setPosition(blockageblockposition);
//////            robot.BlockageArmR.setPosition(blockageblockposition);
//////        }
//////        if (currentGamepad1.a && !previousGamepad1.a) {
//////            stopIntakeAndShooter();
//////        }
//////    }
//////    private void startShooterIdle() {
//////        robot.IntakeMotorL.setPower(0);
//////        robot.IntakeMotorR.setPower(0);
//////        robot.setHoodPosition(0.5);
//////        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeed));
//////        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeed));
//////    }
//////    private void startShooterIdleFar() {
//////        robot.IntakeMotorL.setPower(0);
//////        robot.IntakeMotorR.setPower(0);
//////        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeedfar));
//////        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.flyWheelIdleSpeedfar));
//////    }
//////    private void startShooter() {
//////        robot.IntakeMotorL.setPower(0);
//////        robot.IntakeMotorR.setPower(0);
//////        robot.setHoodPosition(0.5);
//////        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
//////        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEED));
//////        shooterIsOn = true;
//////    }
//////    private void startShooterFar() {
//////        robot.IntakeMotorL.setPower(0);
//////        robot.IntakeMotorR.setPower(0);
//////        robot.shooterL.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEEDfar));
//////        robot.shooterR.setVelocity(Math.abs(ShooterPIDFConfig.targetSPEEDfar));
//////        shooterIsOn = true;
//////    }
//////    private void stopShooter() {
//////        if (isFarMode) {
//////            startShooterIdleFar();
//////        } else {
//////            startShooterIdle();
//////        }
//////        intakeIsOn = false;
//////        isShooterAtSpeed = false;
//////        fireRequested = false;
//////        shooterIsOn = false;
//////    }
//////    private void stopIntakeAndShooter() {
//////        robot.MasterShooterMotorL.setVelocity(0);
//////        robot.SlaveShooterMotorR.setVelocity(0);
//////        robot.IntakeMotorL.setPower(0);
//////        robot.IntakeMotorR.setPower(0);
//////        intakeIsOn = false;
//////        isShooterAtSpeed = false;
//////        fireRequested = false;
//////        shooterIsOn = false;
//////    }
//////    private void checkShooterVelocity() {
//////        double activeTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
//////        if (Math.abs(cachedShooterVeloL - activeTarget) <= toleranceforShoot) {
//////            isShooterAtSpeed = true;
//////        } else {
//////            isShooterAtSpeed = false;
//////            fireRequested = false;
//////        }
//////    }
//////    private void updateHood() {
//////        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
//////            robot.setHoodPosition(HoodArmfarposition);
//////        } else if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
//////            robot.setHoodPosition(HoodArmcloseposition);
//////        } else if (currentGamepad2.y && !previousGamepad2.y) {
//////            robot.setHoodPosition(0.50);
//////        }
//////    }
//////    private void updateBlockage() {
//////        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
//////            robot.BlockageArmL.setPosition(blockageblockposition);
//////            robot.BlockageArmR.setPosition(blockageblockposition);
//////        } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
//////            robot.BlockageArmL.setPosition(blockagereleaseposition);
//////            robot.BlockageArmR.setPosition(blockagereleaseposition);
//////        }
//////    }
//////    private void updateLEDs() {
//////        if (isShooterAtSpeed && robot.axonTurretArmL.isAtTarget(4.0)) {
//////            setLEDColor("GREEN");
//////        } else {
//////            setLEDColor("RED");
//////        }
//////    }
//////    private void setLEDColor(String color) {
//////        if (color.equals(lastLedColor)) return;
//////        try {
//////            if ("GREEN".equals(color)) {
//////                if (robot.greenLED != null) robot.greenLED.setState(false);
//////                if (robot.redLED != null) robot.redLED.setState(true);
//////            } else if ("RED".equals(color)) {
//////                if (robot.greenLED != null) robot.greenLED.setState(true);
//////                if (robot.redLED != null) robot.redLED.setState(false);
//////            }
//////            lastLedColor = color;
//////        } catch (Exception e) {
//////            telemetry.addData("LED Error", e.getMessage());
//////        }
//////    }
//////    private void updateHiveStatus() {
//////        if (robot.myAprilTagProcessor == null) return;
//////        robot.hiveScorable = false;
//////        robot.hiveClusterName = "";
//////        robot.hiveRoll = 0.0;
//////        robot.hivePercentFound = 0.0;
//////        for (AprilTagDetection d : robot.myAprilTagProcessor.getDetections()) {
//////            double rawRoll = (d.ftcPose != null) ? d.ftcPose.roll : 0.0;
//////            if (d instanceof AprilTagClusterDetection) {
//////                AprilTagClusterDetection cluster = (AprilTagClusterDetection) d;
//////                String name = (cluster.metadata != null && cluster.metadata.name != null)
//////                        ? cluster.metadata.name : "Unknown_Cluster";
//////                boolean allianceMatch = name.startsWith(robot.alliance == Alliance.RED ? "R" : "B");
//////                if (!allianceMatch || Math.abs(rawRoll) >= 90.0 || cluster.percentClusterFound < 0.5) continue;
//////                robot.hiveScorable = true;
//////                robot.hiveClusterName = name;
//////                robot.hiveRoll = rawRoll;
//////                robot.hivePercentFound = cluster.percentClusterFound;
//////                break;
//////            } else if (d instanceof AprilTagSingleDetection) {
//////                AprilTagSingleDetection single = (AprilTagSingleDetection) d;
//////                String name = (single.metadata != null && single.metadata.name != null)
//////                        ? single.metadata.name : ("ID_" + single.id);
//////                robot.hiveScorable = true;
//////                robot.hiveClusterName = name;
//////                robot.hiveRoll = rawRoll;
//////                robot.hivePercentFound = 1.0;
//////                break;
//////            }
//////        }
//////    }
//////    public void updateDrivetrain_FieldCentric() {
//////        double y = -gamepad1.left_stick_y;
//////        double x = gamepad1.left_stick_x;
//////        double rx = gamepad1.right_stick_x * DriveTrains_smoothTurn;
//////        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//////        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
//////        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//////        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
//////        double frontLeftPower  = (rotY + rotX + rx) / denominator;
//////        double backLeftPower   = (rotY - rotX + rx) / denominator;
//////        double frontRightPower = (rotY - rotX - rx) / denominator;
//////        double backRightPower  = (rotY + rotX - rx) / denominator;
//////        robot.leftFrontMotor.setPower(frontLeftPower * DriveTrains_ReducePOWER);
//////        robot.leftRearMotor.setPower(backLeftPower * DriveTrains_ReducePOWER);
//////        robot.rightFrontMotor.setPower(frontRightPower * DriveTrains_ReducePOWER);
//////        robot.rightRearMotor.setPower(backRightPower * DriveTrains_ReducePOWER);
//////    }
//////    private void handleIMUReset() {
//////        if (imuResetInCooldown) {
//////            if (imuResetTimer.milliseconds() > IMU_RESET_COOLDOWN_MS) {
//////                imuResetInCooldown = false;
//////            } else {
//////                return;
//////            }
//////        }
//////        if (currentGamepad1.back && !previousGamepad1.back) {
//////            robot.imu.resetYaw();
//////            imuResetInCooldown = true;
//////            imuResetTimer.reset();
//////            gamepad1.rumble(300);
//////        }
//////    }
//////    private void updateAllTelemetry(double loopTimeMs) {
//////        Pose pose = follower.pose();
//////        double activeTarget = isFarMode ? ShooterPIDFConfig.targetSPEEDfar : ShooterPIDFConfig.targetSPEED;
//////        if (panelsTelemetry != null) {
//////            panelsTelemetry.debug("Loop Time (ms)", loopTimeMs);
//////            panelsTelemetry.debug("Shoot Mode", isFarMode ? "FAR" : "NEAR");
//////            panelsTelemetry.debug("Shooter Target", activeTarget);
//////            panelsTelemetry.debug("Shooter Left Velo", cachedShooterVeloL);
//////            panelsTelemetry.debug("Shooter Right Velo", cachedShooterVeloR);
//////            panelsTelemetry.debug("Turret AutoAim", turretAutoAim);
//////            panelsTelemetry.debug("Turret Target (deg)", turretTargetDeg);
//////            panelsTelemetry.debug("Turret Current (deg)", robot.axonTurretArmL.getTotalRotation());
//////            panelsTelemetry.debug("Hive Scorable", robot.hiveScorable);
//////            panelsTelemetry.debug("Hive Cluster", robot.hiveClusterName);
//////        }
//////        telemetry.addData("Loop Time", "%.1f ms | Mode: %s", loopTimeMs, isFarMode ? "FAR" : "NEAR");
//////        telemetry.addData("Pose (X/Y/H)", "%.1f / %.1f / %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
//////        telemetry.addData("Shooter (T/L/R)", "%.0f / %.0f / %.0f", activeTarget, cachedShooterVeloL, cachedShooterVeloR);
////////        telemetry.addData("Turret (Auto:%b)", "Set:%.1f° | Cur:%.1f° | Trim:%.1f°",
////////                turretAutoAim, turretTargetDeg, robot.axonTurretArmL.getTotalRotation(), turretManualTrimDeg);
//////
//////        telemetry.addData("Turret", "Auto:%b | Set:%.1f° | Cur:%.1f° | Trim:%.1f°",
//////                turretAutoAim, turretTargetDeg, robot.axonTurretArmL.getTotalRotation(), turretManualTrimDeg);
//////
//////        telemetry.addData("Hive", "%b (%s)", robot.hiveScorable, robot.hiveClusterName);
//////    }
//////}