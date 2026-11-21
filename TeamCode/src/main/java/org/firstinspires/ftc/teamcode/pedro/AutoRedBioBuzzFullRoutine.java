package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedro.HardwareDecodeRobotBioBuzzSeason.ShooterPIDFConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Configurable
@Autonomous(name = "AAA BioBuzz Auto Red Full Routine (Unified Hardware + Vision Gate)")
public class AutoRedBioBuzzFullRoutine extends LinearOpMode {

    // 红方半场实测基准目标坐标（蓝方自动由 robot.toAllianceX/Y 镜像）
    public static double RED_LOWER_GOAL_X = 58.0;
    public static double RED_LOWER_GOAL_Y = 62.0;
    public static double RED_UPPER_GOAL_X = 58.0;
    public static double RED_UPPER_GOAL_Y = 79.5;

    // 炮台物理安装偏心与行程/限速参数（与 TeleOp 保持一致）
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;
    public static double TURRET_MAX_ROTATION_DEG = 270.0;
    public static double TURRET_MAX_SPEED_DEG_PER_S = 540.0;
    public static double MIN_SHOOT_RANGE_IN = 35.0;

    // 机构功率与舵机位置
    public float intakePowerIntake = 0.95f;
    public float intakePowerShoot = 0.90f;
    public float intakePowerOff = 0.0f;

    public static final double blockageblockposition = 0.10;
    public static final double blockagereleaseposition = 0.18;
    public static double toleranceforShoot = 80.0;

    private final HardwareDecodeRobotBioBuzzSeason robot = new HardwareDecodeRobotBioBuzzSeason();
    private Follower follower;

    private final ElapsedTime pathTimer = new ElapsedTime();
    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime turretRateTimer = new ElapsedTime();

    // 自动主状态机与发射子状态机
    private AutoState autoState = AutoState.PRELOAD_HIVE_BASELINE_CHECK;
    private ShootSubState shootSubState = ShootSubState.IDLE;

    // 视觉与 Hive 倾倒基准记录
    private boolean isCameraStreaming = false;
    private double baselineHiveRoll = 0.0;

    // 炮台与发射实时变量
    private boolean turretAutoAim = true;
    private double activeGoalX = RED_LOWER_GOAL_X;
    private double activeGoalY = RED_LOWER_GOAL_Y;
    private double turretTargetDeg = 0.0;
    private double smoothedTurretTargetDeg = 0.0;

    private double currentGoalDistanceIn = 0.0;
    private double dynamicTargetVelo = 1050.0;
    private double dynamicTargetHood = 0.50;
    private double cachedShooterVeloL = 0.0;
    private double cachedShooterVeloR = 0.0;

    // 连续路径数组队列调度状态
    private Path[] activePathChain = null;
    private int activeChainIndex = 0;

    // Pedro Pathing v3.0+ 单段路径与连续直线数组 (Path[])
    private Path path1_ToPreloadShoot;
    private Path[] chain2_3_GardenIntakeWall;
    private Path[] chain4_5_ToHiveCheck;

    // 分支 A（未倾倒）：回 (58,8) 补射 -> 再回 (40,12) 二次检测
    private Path path6b_ReturnToSecondShoot;
    private Path[] chain4_5_SecondHiveCheck;

    // 分支 A2（二次检测仍未倾倒）：去另一侧近区 Flower 吸球补射 -> 停靠
    private Path[] chainFallback_LowerFlowerIntake;
    private Path pathFallback_ReturnToShoot;
    private Path pathFallback_ToLowerPark;

    // 分支 B（已倾倒）：直接前往上半场 (40,130) -> 完成上半场与 Flower 4球
    private Path path6_DirectToUpperMid;
    private Path path7_TurnAtUpperMid;
    private Path[] chain8_9_UpperShoot;
    private Path[] chain10_11_ToFlowerPrep;
    private Path[] chain12_13_FlowerIntakeWall;
    private Path path14_BackToFlowerShoot;
    private Path path15_ToUpperPark;

    public enum AutoState {
        PRELOAD_HIVE_BASELINE_CHECK,    // 0. 发射预装球前采集一次 Hive 基准状态，随后关摄像头
        DRIVE_TO_PRELOAD_SHOOT,         // 1. Path1: (56,8) -> (58,8)
        SHOOT_PRELOAD,                  // 2. 射出预装球
        DRIVE_TO_GARDEN_WALL,           // 3. Path2->3: 连续直线经过 (32.5,8) 顶墙至 (8.5,8) 吸取 Garden 球
        DRIVE_AND_TURN_TO_HIVE_CHECK,   // 4. Path4->5: 退回 (40,12) 并转向 90° 对准 Hive
        CHECK_HIVE_TIP_1,               // 5. 开摄像头进行第一次 Hive 倾倒检测 (0.5s)

        // --- 分支 A：第一次检测未倾倒 -> 补射并二次检测 ---
        DRIVE_TO_SECOND_SHOOT,          // 6A. (40,12) -> (58,8)
        SHOOT_SECOND_BATCH,             // 7A. 射出 Garden 吸到的球
        RETURN_FOR_SECOND_HIVE_CHECK,   // 8A. 退回 (40,12) 并转向 90°
        CHECK_HIVE_TIP_2,               // 9A. 开摄像头进行第二次 Hive 倾倒检测 (0.5s)

        // --- 分支 A2：二次检测仍未倾倒 -> 另一侧近区 Flower 兜底补射 ---
        DRIVE_FALLBACK_LOWER_FLOWER,    // 10A2. 驶向近区 Flower (48,22) -> 顶墙 (48,8.5) 吸球
        DRIVE_FALLBACK_SHOOT_POS,       // 11A2. 返回 (58,8) 射击位
        SHOOT_FALLBACK_BATCH,           // 12A2. 射出兜底吸到的球
        DRIVE_TO_LOWER_PARK,            // 13A2. 驶入近区停靠位

        // --- 分支 B：已倾倒 -> 冲刺上半场 + 上半场 Flower 4球 ---
        DRIVE_DIRECT_TO_UPPER_MID,      // 6B. Path6: (40,12) -> (40,130)
        TURN_AT_UPPER_MID,              // 7B. Path7: 在 (40,130) 转到 180°
        DRIVE_TO_UPPER_SHOOT,           // 8B. Path8->9: 连续直线经过 (58,130) 抵达 (59,133)
        SHOOT_UPPER_BATCH,              // 9B. 在 (59,133) 发射
        DRIVE_AND_TURN_TO_FLOWER,       // 10B. Path10->11: 退回 (59,120) 并转向 90°
        DRIVE_TO_FLOWER_WALL,           // 11B. Path12->13: 对齐 (48,120) 后不停顿顶墙至 (48,133) 吸 4 球
        DRIVE_TO_FLOWER_SHOOT,          // 12B. Path14: 退回 (48,110) 射击位
        SHOOT_FLOWER_BATCH,             // 13B. 旋转炮台瞄准并射出 Flower 4 球
        DRIVE_TO_UPPER_PARK,            // 14B. Path15: (48,110) -> (10,110) 停靠

        SHUTDOWN_AND_END                // 结束自动：全部断电并保存位姿给 TeleOp
    }

    private enum ShootSubState {
        IDLE,
        SPINNING_UP,
        FEEDING,
        DONE
    }

    @Override
    public void runOpMode() {
        // 1. 初始化统一硬件类（自动加载唯一的炮台 PID 与飞轮 PIDF 参数）
        robot.init(hardwareMap);
        robot.alliance = Alliance.RED; // 红方基准（切蓝方只需改为 Alliance.BLUE，全路径自动镜像）

        // 2. 初始化视觉处理器（默认关闭预览画面节省算力，先开启流采集基准）
        robot.initVision(false);
        enableVisionCamera();

        robot.BlockageArmL.setPosition(blockageblockposition);
        robot.BlockageArmR.setPosition(blockageblockposition);
        if (robot.axonTurretArmL != null) {
            robot.axonTurretArmL.setTargetRotation(0.0);
        }

        // 3. 创建 Pedro Pathing Follower 并规范初始化起始位姿
        follower = Constants.create(hardwareMap);
        robot.clearBulkCache();
        Pose startPose = robot.toAlliancePose(56.0, 8.0, 180.0);
        follower.setPose(startPose);
        follower.update();

        buildAllPaths();

        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        telemetry.addData("Status", "Auto Unified Ready | Alliance: " + robot.alliance.name());
        telemetry.addData("Start Pose", "(%.1f, %.1f, %.0f°)",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.update();

        waitForStart();
        loopTimer.reset();
        turretRateTimer.reset();
        enterState(AutoState.PRELOAD_HIVE_BASELINE_CHECK);

        while (opModeIsActive() && autoState != AutoState.SHUTDOWN_AND_END) {
            double loopTimeMs = loopTimer.milliseconds();
            loopTimer.reset();

            // 1. 刷新 REV Hub 批量缓存并单次读取飞轮速度
            robot.clearBulkCache();
            cachedShooterVeloL = Math.abs(robot.MasterShooterMotorL.getVelocity());
            cachedShooterVeloR = Math.abs(robot.SlaveShooterMotorR.getVelocity());

            // 2. 路径跟踪、连续队列推进与炮台闭环同步更新
            follower.update();
            updateActiveChain();
            updateAutoTurretControl();
            robot.updateTurretAxons();

            // 3. 仅在摄像头开启时才解析 AprilTag，节省 CPU 与 Loop Time
            if (isCameraStreaming) {
                updateHiveStatus();
            }

            // 4. 推进主状态机
            updateAutoStateMachine();

            // 5. 双端刷新遥测
            updateAutoTelemetry(loopTimeMs);
            telemetry.update();
        }

        // 自动结束：安全关闭所有电机与摄像头，并保存最终位置供 TeleOp 继承
        disableVisionCamera();
        stopAllMechanisms();
        saveFinalPoseForTeleOp();
    }

    // =========================================================================
    // 连续直线数组 Path[] 队列调度器（适配 follower.follow(Path)）
    // =========================================================================
    private void followChain(Path[] chain) {
        activePathChain = chain;
        activeChainIndex = 0;
        if (chain != null && chain.length > 0) {
            follower.follow(chain[0]);
        }
    }



    private void updateActiveChain() {
        if (activePathChain != null && !follower.isBusy()) {
            activeChainIndex++;
            if (activeChainIndex < activePathChain.length) {
                follower.follow(activePathChain[activeChainIndex]);
            } else {
                activePathChain = null;
            }
        }
    }

    private boolean isRobotMoving() {
        return follower.isBusy() || (activePathChain != null && activeChainIndex < activePathChain.length);
    }

    // =========================================================================
    // 按需启停摄像头控制
    // =========================================================================
    private void enableVisionCamera() {
        if (robot.myVisionPortal != null && !isCameraStreaming) {
            if (robot.myVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING
                    || robot.myVisionPortal.getCameraState() == VisionPortal.CameraState.STOPPING_STREAM) {
                isCameraStreaming = true;
            } else {
                robot.myVisionPortal.resumeStreaming();
                isCameraStreaming = true;
            }
        }
    }
    private void disableVisionCamera() {
        if (robot.myVisionPortal != null && isCameraStreaming) {
            robot.myVisionPortal.stopStreaming();
            isCameraStreaming = false;
        }
    }
    private double getRedEquivalentX() {
        return (robot.alliance == Alliance.BLUE)
                ? (HardwareDecodeRobotBioBuzzSeason.FIELD_SIZE_IN - follower.pose().x())
                : follower.pose().x();
    }
    // =========================================================================
// 构建全部 Pedro Pathing v3.0+ Path 与 Path[] 连续数组（直接调用硬件对称引擎）
// =========================================================================
    private void buildAllPaths() {
        Pose p56_8_180   = robot.toAlliancePose(56.0, 8.0, 180.0);
        Pose p58_8_180   = robot.toAlliancePose(58.0, 8.0, 180.0);
        Pose p32_8_180   = robot.toAlliancePose(32.5, 8.0, 180.0);
        Pose p8_8_180    = robot.toAlliancePose(8.5,  8.0, 180.0);
        Pose p40_12_180  = robot.toAlliancePose(40.0, 12.0, 180.0);
        Pose p40_12_90   = robot.toAlliancePose(40.0, 12.1, 90.0);   // 连续转向 90° 对准 Hive
        Pose p40_130_90  = robot.toAlliancePose(40.0, 130.0, 90.0);
        Pose p40_130_180 = robot.toAlliancePose(40.1, 130.0, 180.0); // 连续转向 180°
        Pose p58_130_270 = robot.toAlliancePose(58.0, 130.0, 270.0);
        Pose p59_133_270 = robot.toAlliancePose(59.0, 133.0, 270.0);
        Pose p59_120_270 = robot.toAlliancePose(59.0, 120.0, 270.0);
        Pose p59_120_90  = robot.toAlliancePose(59.0, 120.1, 90.0);  // 连续转向 90° 面向 Flower
        Pose p48_120_90  = robot.toAlliancePose(48.0, 120.0, 90.0);
        Pose p48_133_90  = robot.toAlliancePose(48.0, 133.0, 90.0);
        Pose p48_110_90  = robot.toAlliancePose(48.0, 110.0, 90.0);
        Pose p10_110_180 = robot.toAlliancePose(10.0, 110.0, 180.0);
// 近区另一侧 Flower 兜底路径点
        Pose p48_22_270  = robot.toAlliancePose(48.0, 22.0, 270.0);
        Pose p48_8_270   = robot.toAlliancePose(48.0, 8.5,  270.0);
        Pose p10_30_180  = robot.toAlliancePose(10.0, 30.0, 180.0);
// 1. Preload 准备微移
        path1_ToPreloadShoot = line(p56_8_180, p58_8_180).linear(p56_8_180.heading(), p58_8_180.heading());
// 2. Path2 -> Path3: 连续直线经过 (32.5, 8.0) 不停顿直接顶向墙边 (8.5, 8.0) 吸球
        chain2_3_GardenIntakeWall = new Path[] {
                line(p58_8_180, p32_8_180).linear(p58_8_180.heading(), p32_8_180.heading()),
                line(p32_8_180, p8_8_180).linear(p32_8_180.heading(), p8_8_180.heading())
        };
// 3. Path4 -> Path5: 连续退回 (40.0, 12.0) 并无缝转向 90° 检查 Hive
        chain4_5_ToHiveCheck = new Path[] {
                line(p8_8_180, p40_12_180).linear(p8_8_180.heading(), p40_12_180.heading()),
                line(p40_12_180, p40_12_90).linear(p40_12_180.heading(), p40_12_90.heading())
        };
// 4. 分支 A（未倾倒）：退回 (58.0, 8.0) 补射 -> 再回 (40.0, 12.0) 二次检测
        path6b_ReturnToSecondShoot = line(p40_12_90, p58_8_180).linear(p40_12_90.heading(), p58_8_180.heading());
        chain4_5_SecondHiveCheck = new Path[] {
                line(p58_8_180, p40_12_180).linear(p58_8_180.heading(), p40_12_180.heading()),
                line(p40_12_180, p40_12_90).linear(p40_12_180.heading(), p40_12_90.heading())
        };
// 5. 分支 A2（二次检测仍未倾倒）：去近区另一侧 Flower 吸球 -> 回来补射 -> 近区停靠
        chainFallback_LowerFlowerIntake = new Path[] {
                line(p40_12_90, p48_22_270).linear(p40_12_90.heading(), p48_22_270.heading()),
                line(p48_22_270, p48_8_270).linear(p48_22_270.heading(), p48_8_270.heading())
        };
        pathFallback_ReturnToShoot = line(p48_8_270, p58_8_180).linear(p48_8_270.heading(), p58_8_180.heading());
        pathFallback_ToLowerPark   = line(p58_8_180, p10_30_180).linear(p58_8_180.heading(), p10_30_180.heading());
// 6. 分支 B（已倾倒）：Path6 -> Path7 前往上半场
        path6_DirectToUpperMid = line(p40_12_90, p40_130_90).linear(p40_12_90.heading(), p40_130_90.heading());
        path7_TurnAtUpperMid   = line(p40_130_90, p40_130_180).linear(p40_130_90.heading(), p40_130_180.heading());
// 7. Path8 -> Path9: 连续切入上半场射击位 (40,130) -> (58,130) -> (59,133)
        chain8_9_UpperShoot = new Path[] {
                line(p40_130_180, p58_130_270).linear(p40_130_180.heading(), p58_130_270.heading()),
                line(p58_130_270, p59_133_270).linear(p58_130_270.heading(), p59_133_270.heading())
        };
// 8. Path10 -> Path11: 连续退离上墙并转向 90° 面向 Flower
        chain10_11_ToFlowerPrep = new Path[] {
                line(p59_133_270, p59_120_270).linear(p59_133_270.heading(), p59_120_270.heading()),
                line(p59_120_270, p59_120_90).linear(p59_120_270.heading(), p59_120_90.heading())
        };
// 9. Path12 -> Path13: 对齐 Flower (48,120) 后不停顿顶墙至 (48,133) 吸 4 球
        chain12_13_FlowerIntakeWall = new Path[] {
                line(p59_120_90, p48_120_90).linear(p59_120_90.heading(), p48_120_90.heading()),
                line(p48_120_90, p48_133_90).linear(p48_120_90.heading(), p48_133_90.heading())
        };
// 10. Path14 -> Path15: 退回 (48,110) 射击后驶入停靠位 (10,110)
        path14_BackToFlowerShoot = line(p48_133_90, p48_110_90).linear(p48_133_90.heading(), p48_110_90.heading());
        path15_ToUpperPark       = line(p48_110_90, p10_110_180).linear(p48_110_90.heading(), p10_110_180.heading());
    }
    private void enterState(AutoState nextState) {
        autoState = nextState;
        pathTimer.reset();
    }
    private void startNewShootSequence() {
        shootSubState = ShootSubState.IDLE;
        shootTimer.reset();
    }
    private boolean isHiveTippedDetected() {
        return robot.hiveScorable || Math.abs(robot.hiveRoll - baselineHiveRoll) > 15.0;
    }
    // =========================================================================
// 主状态机逻辑
// =========================================================================
    private void updateAutoStateMachine() {
        switch (autoState) {
            case PRELOAD_HIVE_BASELINE_CHECK:
                activeGoalX = RED_LOWER_GOAL_X;
                activeGoalY = RED_LOWER_GOAL_Y;
                turretAutoAim = true;
                enableVisionCamera();
                spinUpShooterForCurrentDistance();
                if (pathTimer.seconds() >= 0.25) {
                    baselineHiveRoll = robot.hiveRoll;
                    disableVisionCamera();
                    follower.follow(path1_ToPreloadShoot);
                    enterState(AutoState.DRIVE_TO_PRELOAD_SHOOT);
                }
                break;
            case DRIVE_TO_PRELOAD_SHOOT:
                spinUpShooterForCurrentDistance();
                if (!follower.isBusy()) {
                    startNewShootSequence();
                    enterState(AutoState.SHOOT_PRELOAD);
                }
                break;
            case SHOOT_PRELOAD:
                runNonBlockingShootSequence(1.1);
                if (shootSubState == ShootSubState.DONE) {
                    followChain(chain2_3_GardenIntakeWall);
                    enterState(AutoState.DRIVE_TO_GARDEN_WALL);
                }
                break;
            case DRIVE_TO_GARDEN_WALL:
                if (getRedEquivalentX() <= 35.0) {
                    startIntakeAction();
                }
                if (!isRobotMoving() && pathTimer.seconds() > 1.8) {
                    stopIntakeAction();
                    enableVisionCamera();
                    followChain(chain4_5_ToHiveCheck);
                    enterState(AutoState.DRIVE_AND_TURN_TO_HIVE_CHECK);
                }
                break;
            case DRIVE_AND_TURN_TO_HIVE_CHECK:
                activeGoalX = RED_LOWER_GOAL_X;
                activeGoalY = RED_LOWER_GOAL_Y;
                turretAutoAim = true;
                if (!isRobotMoving() && robot.axonTurretArmL.isAtTarget(5.0)) {
                    enterState(AutoState.CHECK_HIVE_TIP_1);
                }
                break;
            case CHECK_HIVE_TIP_1:
                if (pathTimer.seconds() >= 0.5) {
                    boolean tipped = isHiveTippedDetected();
                    disableVisionCamera();
                    if (tipped) {
                        follower.follow(path6_DirectToUpperMid);
                        enterState(AutoState.DRIVE_DIRECT_TO_UPPER_MID);
                    } else {
                        spinUpShooterForCurrentDistance();
                        follower.follow(path6b_ReturnToSecondShoot);
                        enterState(AutoState.DRIVE_TO_SECOND_SHOOT);
                    }
                }
                break;
// ---------------- 分支 A：未倾倒补射与二次检测 ----------------
            case DRIVE_TO_SECOND_SHOOT:
                spinUpShooterForCurrentDistance();
                if (!follower.isBusy()) {
                    startNewShootSequence();
                    enterState(AutoState.SHOOT_SECOND_BATCH);
                }
                break;
            case SHOOT_SECOND_BATCH:
                runNonBlockingShootSequence(1.1);
                if (shootSubState == ShootSubState.DONE) {
                    enableVisionCamera();
                    followChain(chain4_5_SecondHiveCheck);
                    enterState(AutoState.RETURN_FOR_SECOND_HIVE_CHECK);
                }
                break;
            case RETURN_FOR_SECOND_HIVE_CHECK:
                if (!isRobotMoving() && robot.axonTurretArmL.isAtTarget(5.0)) {
                    enterState(AutoState.CHECK_HIVE_TIP_2);
                }
                break;
            case CHECK_HIVE_TIP_2:
                if (pathTimer.seconds() >= 0.5) {
                    boolean tipped = isHiveTippedDetected();
                    disableVisionCamera();
                    if (tipped) {
                        follower.follow(path6_DirectToUpperMid);
                        enterState(AutoState.DRIVE_DIRECT_TO_UPPER_MID);
                    } else {
                        startIntakeAction();
                        followChain(chainFallback_LowerFlowerIntake);
                        enterState(AutoState.DRIVE_FALLBACK_LOWER_FLOWER);
                    }
                }
                break;
// ---------------- 分支 A2：另一侧近区 Flower 兜底补射 ----------------
            case DRIVE_FALLBACK_LOWER_FLOWER:
                startIntakeAction();
                if (!isRobotMoving() && pathTimer.seconds() > 1.8) {
                    stopIntakeAction();
                    spinUpShooterForCurrentDistance();
                    follower.follow(pathFallback_ReturnToShoot);
                    enterState(AutoState.DRIVE_FALLBACK_SHOOT_POS);
                }
                break;
            case DRIVE_FALLBACK_SHOOT_POS:
                spinUpShooterForCurrentDistance();
                if (!follower.isBusy()) {
                    startNewShootSequence();
                    enterState(AutoState.SHOOT_FALLBACK_BATCH);
                }
                break;
            case SHOOT_FALLBACK_BATCH:
                runNonBlockingShootSequence(1.2);
                if (shootSubState == ShootSubState.DONE) {
                    turretAutoAim = false;
                    robot.setHoodPosition(0.35);
                    follower.follow(pathFallback_ToLowerPark);
                    enterState(AutoState.DRIVE_TO_LOWER_PARK);
                }
                break;
            case DRIVE_TO_LOWER_PARK:
                if (!follower.isBusy()) {
                    enterState(AutoState.SHUTDOWN_AND_END);
                }
                break;
// ---------------- 分支 B：上半场与 Flower 4 球 ----------------
            case DRIVE_DIRECT_TO_UPPER_MID:
                if (!follower.isBusy()) {
                    follower.follow(path7_TurnAtUpperMid);
                    enterState(AutoState.TURN_AT_UPPER_MID);
                }
                break;
            case TURN_AT_UPPER_MID:
                activeGoalX = RED_UPPER_GOAL_X;
                activeGoalY = RED_UPPER_GOAL_Y;
                turretAutoAim = true;
                if (!follower.isBusy()) {
                    followChain(chain8_9_UpperShoot);
                    enterState(AutoState.DRIVE_TO_UPPER_SHOOT);
                }
                break;
            case DRIVE_TO_UPPER_SHOOT:
                spinUpShooterForCurrentDistance();
                if (!isRobotMoving()) {
                    startNewShootSequence();
                    enterState(AutoState.SHOOT_UPPER_BATCH);
                }
                break;
            case SHOOT_UPPER_BATCH:
                runNonBlockingShootSequence(1.1);
                if (shootSubState == ShootSubState.DONE) {
                    followChain(chain10_11_ToFlowerPrep);
                    enterState(AutoState.DRIVE_AND_TURN_TO_FLOWER);
                }
                break;
            case DRIVE_AND_TURN_TO_FLOWER:
                if (!isRobotMoving()) {
                    followChain(chain12_13_FlowerIntakeWall);
                    enterState(AutoState.DRIVE_TO_FLOWER_WALL);
                }
                break;
            case DRIVE_TO_FLOWER_WALL:
                if (getRedEquivalentX() <= 50.0) {
                    startIntakeAction();
                }
                if (!isRobotMoving() && pathTimer.seconds() > 1.8) {
                    spinUpShooterForCurrentDistance();
                    follower.follow(path14_BackToFlowerShoot);
                    enterState(AutoState.DRIVE_TO_FLOWER_SHOOT);
                }
                break;
            case DRIVE_TO_FLOWER_SHOOT:
                activeGoalX = RED_UPPER_GOAL_X;
                activeGoalY = RED_UPPER_GOAL_Y;
                turretAutoAim = true;
                spinUpShooterForCurrentDistance();
                if (!follower.isBusy()) {
                    startNewShootSequence();
                    enterState(AutoState.SHOOT_FLOWER_BATCH);
                }
                break;
            case SHOOT_FLOWER_BATCH:
                runNonBlockingShootSequence(1.4);
                if (shootSubState == ShootSubState.DONE) {
                    stopIntakeAction();
                    turretAutoAim = false;
                    robot.setHoodPosition(0.35);
                    follower.follow(path15_ToUpperPark);
                    enterState(AutoState.DRIVE_TO_UPPER_PARK);
                }
                break;
            case DRIVE_TO_UPPER_PARK:
                if (!follower.isBusy()) {
                    enterState(AutoState.SHUTDOWN_AND_END);
                }
                break;
            case SHUTDOWN_AND_END:
                break;
        }
    }
    private void runNonBlockingShootSequence(double feedDurationSec) {
        switch (shootSubState) {
            case IDLE:
                robot.BlockageArmL.setPosition(blockageblockposition);
                robot.BlockageArmR.setPosition(blockageblockposition);
                spinUpShooterForCurrentDistance();
                shootTimer.reset();
                shootSubState = ShootSubState.SPINNING_UP;
                break;
            case SPINNING_UP:
                spinUpShooterForCurrentDistance();
                boolean speedReady = Math.abs(cachedShooterVeloL - dynamicTargetVelo) <= toleranceforShoot;
                boolean turretReady = robot.axonTurretArmL.isAtTarget(4.0);
                if ((speedReady && turretReady) || shootTimer.seconds() > 1.5) {
                    robot.BlockageArmL.setPosition(blockagereleaseposition);
                    robot.BlockageArmR.setPosition(blockagereleaseposition);
                    robot.IntakeMotorL.setPower(intakePowerShoot);
                    robot.IntakeMotorR.setPower(intakePowerShoot);
                    shootTimer.reset();
                    shootSubState = ShootSubState.FEEDING;
                }
                break;
            case FEEDING:
                spinUpShooterForCurrentDistance();
                if (shootTimer.seconds() >= feedDurationSec) {
                    robot.BlockageArmL.setPosition(blockageblockposition);
                    robot.BlockageArmR.setPosition(blockageblockposition);
                    robot.IntakeMotorL.setPower(intakePowerOff);
                    robot.IntakeMotorR.setPower(intakePowerOff);
                    robot.shooterL.setVelocity(ShooterPIDFConfig.flyWheelIdleSpeed);
                    robot.shooterR.setVelocity(ShooterPIDFConfig.flyWheelIdleSpeed);
                    shootSubState = ShootSubState.DONE;
                }
                break;
            case DONE:
                break;
        }
    }
    private void spinUpShooterForCurrentDistance() {
        robot.setHoodPosition(dynamicTargetHood);
        robot.shooterL.setVelocity(Math.abs(dynamicTargetVelo));
        robot.shooterR.setVelocity(Math.abs(dynamicTargetVelo));
    }
    private void startIntakeAction() {
        robot.BlockageArmL.setPosition(blockageblockposition);
        robot.BlockageArmR.setPosition(blockageblockposition);
        robot.IntakeMotorL.setPower(intakePowerIntake);
        robot.IntakeMotorR.setPower(intakePowerIntake);
    }
    private void stopIntakeAction() {
        robot.IntakeMotorL.setPower(intakePowerOff);
        robot.IntakeMotorR.setPower(intakePowerOff);
    }
    private void stopAllMechanisms() {
        robot.setAllPower(0);
        robot.MasterShooterMotorL.setVelocity(0);
        robot.SlaveShooterMotorR.setVelocity(0);
        robot.IntakeMotorL.setPower(0);
        robot.IntakeMotorR.setPower(0);
        robot.BlockageArmL.setPosition(blockageblockposition);
        robot.BlockageArmR.setPosition(blockageblockposition);
        if (robot.servoTurretArmL != null) robot.servoTurretArmL.setPower(0);
        if (robot.servoTurretArmR != null) robot.servoTurretArmR.setPower(0);
    }
    private void saveFinalPoseForTeleOp() {
        if (follower != null) {
            Pose endPose = follower.pose();
            robot.finalAutoX = endPose.x();
            robot.finalAutoY = endPose.y();
            robot.finalAutoHeading = endPose.heading();
        }
    }
    /**
     * 自动阶段炮台自瞄与距离查表控制（直接调用硬件统一对称引擎与插值计算）
     */
    private void updateAutoTurretControl() {
        Pose p = follower.pose();
        double h = p.heading();
        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
        double pivotX = p.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h);
        double pivotY = p.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h);
        double targetX = robot.toAllianceX(activeGoalX);
        double targetY = robot.toAllianceY(activeGoalY);
        double dx = targetX - pivotX;
        double dy = targetY - pivotY;
        double rangeSq = dx * dx + dy * dy;
        currentGoalDistanceIn = Math.sqrt(rangeSq);
        dynamicTargetHood = robot.getDynamicHoodPosition(currentGoalDistanceIn);
        dynamicTargetVelo = robot.getDynamicShooterVelocity(currentGoalDistanceIn);
        if (!turretAutoAim || rangeSq < MIN_SHOOT_RANGE_IN * MIN_SHOOT_RANGE_IN) {
            turretTargetDeg = 0.0;
        } else {
            double relAngle = Math.atan2(dy, dx) - h;
            while (relAngle >  Math.PI) relAngle -= 2 * Math.PI;
            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
            double candidateDeg = Math.toDegrees(relAngle);
            if (candidateDeg - currentTurretDeg > 180.0
                    && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
                candidateDeg -= 360.0;
            } else if (candidateDeg - currentTurretDeg < -180.0
                    && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
                candidateDeg += 360.0;
            }
            turretTargetDeg = Range.clip(candidateDeg, -TURRET_MAX_ROTATION_DEG, TURRET_MAX_ROTATION_DEG);
        }
        double dt = turretRateTimer.seconds();
        turretRateTimer.reset();
        if (TURRET_MAX_SPEED_DEG_PER_S > 0 && dt > 0.0 && dt < 0.5) {
            double maxStep = TURRET_MAX_SPEED_DEG_PER_S * dt;
            smoothedTurretTargetDeg += Range.clip(turretTargetDeg - smoothedTurretTargetDeg, -maxStep, maxStep);
        } else {
            smoothedTurretTargetDeg = turretTargetDeg;
        }
        robot.axonTurretArmL.trackTargetRotation(smoothedTurretTargetDeg);
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
    private void updateAutoTelemetry(double loopTimeMs) {
        Pose pose = follower.pose();
        telemetry.addData("Loop Time (ms)", loopTimeMs);
        telemetry.addData("Camera Streaming", isCameraStreaming ? 1 : 0);
        telemetry.addData("Auto State", autoState.name());
        telemetry.addData("Shoot SubState", shootSubState.name());
        telemetry.addData("Pose (X/Y/H)", "%.1f / %.1f / %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Goal Dist (in)", currentGoalDistanceIn);
        telemetry.addData("Turret Target (deg)", smoothedTurretTargetDeg);
        telemetry.addData("Turret Actual (deg)", robot.axonTurretArmL.getTotalRotation());
        telemetry.addData("Hive Scorable", robot.hiveScorable ? 1 : 0);
        telemetry.addData("Hive Roll (deg)", robot.hiveRoll);
    }
}


//package org.firstinspires.ftc.teamcode.pedro;
//
//import static com.pedropathing.api.Paths.*;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.JoinedTelemetry;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
//
//@Configurable
//@Autonomous(name = "AAA BioBuzz Auto Red Full Routine V3 (Path[] + Vision Gate)")
//public class AutoRedBioBuzzFullRoutine extends LinearOpMode {
//
//    // 场地目标基准坐标（红方下半场目标与上半场目标，蓝方自动中心对称镜像）
//    public static double RED_LOWER_GOAL_X = 58.0;
//    public static double RED_LOWER_GOAL_Y = 62.0;
//    public static double RED_UPPER_GOAL_X = 58.0;
//    public static double RED_UPPER_GOAL_Y = 79.5;
//
//    // 炮台物理安装偏心与行程/限速参数（与 TeleOp 保持完全一致）
//    public static double TURRET_OFFSET_X = 0.0;
//    public static double TURRET_OFFSET_Y = 0.0;
//    public static double TURRET_MAX_ROTATION_DEG = 270.0;
//    public static double TURRET_MAX_SPEED_DEG_PER_S = 540.0;
//    public static double MIN_SHOOT_RANGE_IN = 35.0;
//
//    // 机构功率与舵机位置
//    public float intakePowerIntake = 0.95f;
//    public float intakePowerShoot = 0.90f;
//    public float intakePowerOff = 0.0f;
//
//    public static final double blockageblockposition = 0.10;
//    public static final double blockagereleaseposition = 0.18;
//
//    // 基于距离的自动仰角 (Hood) 与飞轮转速线性插值表（与 TeleOp 保持完全一致）
//    public static double SHOOTER_VELO_TRIM = 0.0;
//    public static double HOOD_POS_TRIM = 0.0;
//    public static double toleranceforShoot = 80.0;
//
//    private static final double[] DIST_TABLE_IN = { 35.0,  60.0,  85.0,  110.0, 140.0 };
//    private static final double[] HOOD_TABLE    = { 0.35,  0.42,  0.48,  0.53,  0.58  };
//    private static final double[] VELO_TABLE    = { 950.0, 1050.0, 1180.0, 1320.0, 1450.0 };
//
//    @Configurable
//    public static class ShooterPIDFConfig {
//        public static double kP = 300.0;
//        public static double kI = 0.0;
//        public static double kD = 0.0;
//        public static double kF = 15.0;
//        public static double flyWheelIdleSpeed = 650.0;
//    }
//
//    private final HardwareDecodeRobotBioBuzzSeason robot = new HardwareDecodeRobotBioBuzzSeason();
//    private Follower follower;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    private final ElapsedTime pathTimer = new ElapsedTime();
//    private final ElapsedTime shootTimer = new ElapsedTime();
//    private final ElapsedTime loopTimer = new ElapsedTime();
//    private final ElapsedTime turretRateTimer = new ElapsedTime();
//
//    // 自动主状态机与发射子状态机
//    private AutoState autoState = AutoState.PRELOAD_HIVE_BASELINE_CHECK;
//    private ShootSubState shootSubState = ShootSubState.IDLE;
//
//    // 视觉与 Hive 倾倒基准记录
//    private boolean isCameraStreaming = false;
//    private double baselineHiveRoll = 0.0;
//
//    // 炮台与发射实时变量
//    private boolean turretAutoAim = true;
//    private double activeGoalX = RED_LOWER_GOAL_X;
//    private double activeGoalY = RED_LOWER_GOAL_Y;
//    private double turretTargetDeg = 0.0;
//    private double smoothedTurretTargetDeg = 0.0;
//
//    private double currentGoalDistanceIn = 0.0;
//    private double dynamicTargetVelo = 1050.0;
//    private double dynamicTargetHood = 0.50;
//    private double cachedShooterVeloL = 0.0;
//    private double cachedShooterVeloR = 0.0;
//
//    // =========================================================================
//    // Pedro Pathing v3.0+ 路径与连续直线数组 (Path[])
//    // =========================================================================
//    private Path[] activePathChain = null;
//    private int activeChainIndex = 0;
//    private Path path1_ToPreloadShoot;
//    private Path[] chain2_3_GardenIntakeWall;
//    private Path[] chain4_5_ToHiveCheck;
//
//    // 分支 A（未倾倒）：回 (58,8) 补射 -> 再回 (40,12) 二次检测
//    private Path path6b_ReturnToSecondShoot;
//    private Path[] chain4_5_SecondHiveCheck;
//
//    // 分支 A2（二次检测仍未倾倒）：去另一侧近区 Flower 吸球补射 -> 停靠
//    private Path[] chainFallback_LowerFlowerIntake;
//    private Path pathFallback_ReturnToShoot;
//    private Path pathFallback_ToLowerPark;
//
//    // 分支 B（已倾倒）：直接前往上半场 (40,130) -> 完成上半场与 Flower 4球
//    private Path path6_DirectToUpperMid;
//    private Path path7_TurnAtUpperMid;
//    private Path[] chain8_9_UpperShoot;
//    private Path[] chain10_11_ToFlowerPrep;
//    private Path[] chain12_13_FlowerIntakeWall;
//    private Path path14_BackToFlowerShoot;
//    private Path path15_ToUpperPark;
//
//    public enum AutoState {
//        PRELOAD_HIVE_BASELINE_CHECK,    // 0. 发射预装球前采集一次 Hive 基准状态，随后关摄像头
//        DRIVE_TO_PRELOAD_SHOOT,         // 1. Path1: (56,8) -> (58,8)
//        SHOOT_PRELOAD,                  // 2. 射出预装球
//        DRIVE_TO_GARDEN_WALL,           // 3. Path2->3: 连续直线经过 (32.5,8) 顶墙至 (8.5,8) 吸取 Garden 球
//        DRIVE_AND_TURN_TO_HIVE_CHECK,   // 4. Path4->5: 退回 (40,12) 并转向 90° 对准 Hive
//        CHECK_HIVE_TIP_1,               // 5. 开摄像头进行第一次 Hive 倾倒检测 (0.5s)
//
//        // --- 分支 A：第一次检测未倾倒 -> 补射并二次检测 ---
//        DRIVE_TO_SECOND_SHOOT,          // 6A. (40,12) -> (58,8)
//        SHOOT_SECOND_BATCH,             // 7A. 射出 Garden 吸到的球
//        RETURN_FOR_SECOND_HIVE_CHECK,   // 8A. 退回 (40,12) 并转向 90°
//        CHECK_HIVE_TIP_2,               // 9A. 开摄像头进行第二次 Hive 倾倒检测 (0.5s)
//
//        // --- 分支 A2：二次检测仍未倾倒 -> 另一侧近区 Flower 兜底补射 ---
//        DRIVE_FALLBACK_LOWER_FLOWER,    // 10A2. 驶向近区 Flower (48,22) -> 顶墙 (48,8.5) 吸球
//        DRIVE_FALLBACK_SHOOT_POS,       // 11A2. 返回 (58,8) 射击位
//        SHOOT_FALLBACK_BATCH,           // 12A2. 射出兜底吸到的球
//        DRIVE_TO_LOWER_PARK,            // 13A2. 驶入近区停靠位
//
//        // --- 分支 B：已倾倒 -> 冲刺上半场 + 上半场 Flower 4球 ---
//        DRIVE_DIRECT_TO_UPPER_MID,      // 6B. Path6: (40,12) -> (40,130)
//        TURN_AT_UPPER_MID,              // 7B. Path7: 在 (40,130) 转到 180°
//        DRIVE_TO_UPPER_SHOOT,           // 8B. Path8->9: 连续直线经过 (58,130) 抵达 (59,133)
//        SHOOT_UPPER_BATCH,              // 9B. 在 (59,133) 发射
//        DRIVE_AND_TURN_TO_FLOWER,       // 10B. Path10->11: 退回 (59,120) 并转向 90°
//        DRIVE_TO_FLOWER_WALL,           // 11B. Path12->13: 对齐 (48,120) 后不停顿顶墙至 (48,133) 吸 4 球
//        DRIVE_TO_FLOWER_SHOOT,          // 12B. Path14: 退回 (48,110) 射击位
//        SHOOT_FLOWER_BATCH,             // 13B. 旋转炮台瞄准并射出 Flower 4 球
//        DRIVE_TO_UPPER_PARK,            // 14B. Path15: (48,110) -> (10,110) 停靠
//
//        SHUTDOWN_AND_END                // 结束自动：全部断电并保存位姿给 TeleOp
//    }
//
//    private enum ShootSubState {
//        IDLE,
//        SPINNING_UP,
//        FEEDING,
//        DONE
//    }
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//        robot.alliance = Alliance.RED; // 红方基准（切蓝方只需改为 Alliance.BLUE，全路径自动镜像）
//        robot.initVision(false);       // 初始化视觉处理器（默认关闭预览省算力）
//        enableVisionCamera();          // 先开启摄像头用于起步前基准采集
//        initShooterPIDF();
//
//        robot.BlockageArmL.setPosition(blockageblockposition);
//        robot.BlockageArmR.setPosition(blockageblockposition);
//        if (robot.axonTurretArmL != null) {
//            robot.axonTurretArmL.setTargetRotation(0.0);
//        }
//
//        follower = Constants.create(hardwareMap);
//        Pose startPose = toSymPose(56.0, 8.0, 180.0);
//        follower.setPose(startPose);
//        follower.update();
//
//        buildAllPaths();
//
//        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
//        telemetry.addData("Status", "Auto Red V3 Ready | Alliance: " + robot.alliance.name());
//        telemetry.update();
//
//        waitForStart();
//        loopTimer.reset();
//        turretRateTimer.reset();
//        enterState(AutoState.PRELOAD_HIVE_BASELINE_CHECK);
//
//        while (opModeIsActive() && autoState != AutoState.SHUTDOWN_AND_END) {
//            double loopTimeMs = loopTimer.milliseconds();
//            loopTimer.reset();
//
//            // 1. 刷新 REV Hub 批量缓存并单次读取飞轮速度
//            robot.clearBulkCache();
//            cachedShooterVeloL = Math.abs(robot.MasterShooterMotorL.getVelocity());
//            cachedShooterVeloR = Math.abs(robot.SlaveShooterMotorR.getVelocity());
//
//            // 2. 路径跟踪与炮台闭环同步更新
//            follower.update();
//            updateActiveChain(); // ✅ 自动无缝衔接 Path[] 中的下一段
//
//            updateAutoTurretControl();
//            robot.updateTurretAxons();
//
//            // 3. 仅在摄像头开启时才解析 AprilTag，节省 CPU 与 Loop Time
//            if (isCameraStreaming) {
//                updateHiveStatus();
//            }
//
//            // 4. 推进主状态机
//            updateAutoStateMachine();
//
//            // 5. 双端刷新遥测
//            updateAutoTelemetry(loopTimeMs);
//            telemetry.update();
//        }
//
//        // 自动结束：安全关闭所有电机与摄像头，并保存最终位置供 TeleOp 继承
//        disableVisionCamera();
//        stopAllMechanisms();
//        saveFinalPoseForTeleOp();
//    }
//
//    // =========================================================================
//    // 按需启停摄像头控制（大幅降低主循环耗时）
//    // =========================================================================
//    private void enableVisionCamera() {
//        if (robot.myVisionPortal != null && !isCameraStreaming) {
//            if (robot.myVisionPortal.getCameraState() == VisionPortal.CameraState.STREAMING
//                    || robot.myVisionPortal.getCameraState() == VisionPortal.CameraState.STOPPING_STREAM) {
//                isCameraStreaming = true;
//            } else {
//                robot.myVisionPortal.resumeStreaming();
//                isCameraStreaming = true;
//            }
//        }
//    }
//
//    private void disableVisionCamera() {
//        if (robot.myVisionPortal != null && isCameraStreaming) {
//            robot.myVisionPortal.stopStreaming();
//            isCameraStreaming = false;
//        }
//    }
//    // =========================================================================
//// 中心对称辅助包装（红方原样输出，蓝方自动 180° 镜像）
//// =========================================================================
//    private Pose toSymPose(double redX, double redY, double redHeadingDeg) {
//        if (robot.alliance == Alliance.BLUE) {
//            double bx = 141.5 - redX;
//            double by = 141.5 - redY;
//            double bh = (redHeadingDeg + 180.0) % 360.0;
//            if (bh > 180.0) bh -= 360.0;
//            return poseFactory.of(bx, by, bh);
//        }
//        return poseFactory.of(redX, redY, redHeadingDeg);
//    }
//    private double toSymX(double redX) {
//        return (robot.alliance == Alliance.BLUE) ? (141.5 - redX) : redX;
//    }
//    private double toSymY(double redY) {
//        return (robot.alliance == Alliance.BLUE) ? (141.5 - redY) : redY;
//    }
//    private double getRedEquivalentX() {
//        return (robot.alliance == Alliance.BLUE) ? (141.5 - follower.pose().x()) : follower.pose().x();
//    }
//    // =========================================================================
//// 构建全部 Pedro Pathing v3.0+ Path 与 Path[] 连续数组
//// =========================================================================
//    private void buildAllPaths() {
//        Pose p56_8_180   = toSymPose(56.0, 8.0, 180.0);
//        Pose p58_8_180   = toSymPose(58.0, 8.0, 180.0);
//        Pose p32_8_180   = toSymPose(32.5, 8.0, 180.0);
//        Pose p8_8_180    = toSymPose(8.5,  8.0, 180.0);
//        Pose p40_12_180  = toSymPose(40.0, 12.0, 180.0);
//        Pose p40_12_90   = toSymPose(40.0, 12.1, 90.0);   // 连续转向 90° 对准 Hive
//        Pose p40_130_90  = toSymPose(40.0, 130.0, 90.0);
//        Pose p40_130_180 = toSymPose(40.1, 130.0, 180.0); // 连续转向 180°
//        Pose p58_130_270 = toSymPose(58.0, 130.0, 270.0);
//        Pose p59_133_270 = toSymPose(59.0, 133.0, 270.0);
//        Pose p59_120_270 = toSymPose(59.0, 120.0, 270.0);
//        Pose p59_120_90  = toSymPose(59.0, 120.1, 90.0);  // 连续转向 90° 面向 Flower
//        Pose p48_120_90  = toSymPose(48.0, 120.0, 90.0);
//        Pose p48_133_90  = toSymPose(48.0, 133.0, 90.0);
//        Pose p48_110_90  = toSymPose(48.0, 110.0, 90.0);
//        Pose p10_110_180 = toSymPose(10.0, 110.0, 180.0);
//// 近区另一侧 Flower 兜底路径点
//        Pose p48_22_270  = toSymPose(48.0, 22.0, 270.0);
//        Pose p48_8_270   = toSymPose(48.0, 8.5,  270.0);
//        Pose p10_30_180  = toSymPose(10.0, 30.0, 180.0);
//// 1. Preload 准备微移
//        path1_ToPreloadShoot = line(p56_8_180, p58_8_180).linear(p56_8_180.heading(), p58_8_180.heading());
//// 2. Path2 -> Path3: 连续直线经过 (32.5, 8.0) 不停顿直接顶向墙边 (8.5, 8.0) 吸球
//        chain2_3_GardenIntakeWall = new Path[] {
//                line(p58_8_180, p32_8_180).linear(p58_8_180.heading(), p32_8_180.heading()),
//                line(p32_8_180, p8_8_180).linear(p32_8_180.heading(), p8_8_180.heading())
//        };
//// 3. Path4 -> Path5: 连续退回 (40.0, 12.0) 并无缝转向 90° 检查 Hive
//        chain4_5_ToHiveCheck = new Path[] {
//                line(p8_8_180, p40_12_180).linear(p8_8_180.heading(), p40_12_180.heading()),
//                line(p40_12_180, p40_12_90).linear(p40_12_180.heading(), p40_12_90.heading())
//        };
//// 4. 分支 A（未倾倒）：退回 (58.0, 8.0) 补射 -> 再回 (40.0, 12.0) 二次检测
//        path6b_ReturnToSecondShoot = line(p40_12_90, p58_8_180).linear(p40_12_90.heading(), p58_8_180.heading());
//        chain4_5_SecondHiveCheck = new Path[] {
//                line(p58_8_180, p40_12_180).linear(p58_8_180.heading(), p40_12_180.heading()),
//                line(p40_12_180, p40_12_90).linear(p40_12_180.heading(), p40_12_90.heading())
//        };
//// 5. 分支 A2（二次检测仍未倾倒）：去近区另一侧 Flower 吸球 -> 回来补射 -> 近区停靠
//        chainFallback_LowerFlowerIntake = new Path[] {
//                line(p40_12_90, p48_22_270).linear(p40_12_90.heading(), p48_22_270.heading()),
//                line(p48_22_270, p48_8_270).linear(p48_22_270.heading(), p48_8_270.heading())
//        };
//        pathFallback_ReturnToShoot = line(p48_8_270, p58_8_180).linear(p48_8_270.heading(), p58_8_180.heading());
//        pathFallback_ToLowerPark   = line(p58_8_180, p10_30_180).linear(p58_8_180.heading(), p10_30_180.heading());
//// 6. 分支 B（已倾倒）：Path6 -> Path7 前往上半场
//        path6_DirectToUpperMid = line(p40_12_90, p40_130_90).linear(p40_12_90.heading(), p40_130_90.heading());
//        path7_TurnAtUpperMid   = line(p40_130_90, p40_130_180).linear(p40_130_90.heading(), p40_130_180.heading());
//// 7. Path8 -> Path9: 连续切入上半场射击位 (40,130) -> (58,130) -> (59,133)
//        chain8_9_UpperShoot = new Path[] {
//                line(p40_130_180, p58_130_270).linear(p40_130_180.heading(), p58_130_270.heading()),
//                line(p58_130_270, p59_133_270).linear(p58_130_270.heading(), p59_133_270.heading())
//        };
//// 8. Path10 -> Path11: 连续退离上墙并转向 90° 面向 Flower
//        chain10_11_ToFlowerPrep = new Path[] {
//                line(p59_133_270, p59_120_270).linear(p59_133_270.heading(), p59_120_270.heading()),
//                line(p59_120_270, p59_120_90).linear(p59_120_270.heading(), p59_120_90.heading())
//        };
//// 9. Path12 -> Path13: 对齐 Flower (48,120) 后不停顿直接顶墙至 (48,133) 吸 4 球
//        chain12_13_FlowerIntakeWall = new Path[] {
//                line(p59_120_90, p48_120_90).linear(p59_120_90.heading(), p48_120_90.heading()),
//                line(p48_120_90, p48_133_90).linear(p48_120_90.heading(), p48_133_90.heading())
//        };
//// 10. Path14 -> Path15: 退回 (48,110) 射击后驶入停靠位 (10,110)
//        path14_BackToFlowerShoot = line(p48_133_90, p48_110_90).linear(p48_133_90.heading(), p48_110_90.heading());
//        path15_ToUpperPark       = line(p48_110_90, p10_110_180).linear(p48_110_90.heading(), p10_110_180.heading());
//    }
//    private void enterState(AutoState nextState) {
//        autoState = nextState;
//        pathTimer.reset();
//    }
//
//    /**
//     * 启动一段连续直线数组 Path[]（替代直接传数组给 follower.follow）
//     */
//    private void followChain(Path[] chain) {
//        activePathChain = chain;
//        activeChainIndex = 0;
//        if (chain != null && chain.length > 0) {
//            follower.follow(chain[0]);
//        }
//    }
//
//    /**
//     * 每帧自动衔接数组中的下一段 Path，实现不停顿连续行驶
//     */
//    private void updateActiveChain() {
//        if (activePathChain != null && !follower.isBusy()) {
//            activeChainIndex++;
//            if (activeChainIndex < activePathChain.length) {
//                follower.follow(activePathChain[activeChainIndex]);
//            } else {
//                activePathChain = null; // 整组连续路径执行完毕
//            }
//        }
//    }
//
//    /**
//     * 统一判断机器人是否仍在执行单段 Path 或多段 Path[] 队列
//     */
//    private boolean isRobotMoving() {
//        return follower.isBusy() || (activePathChain != null && activeChainIndex < activePathChain.length);
//    }
//
//
//    private void startNewShootSequence() {
//        shootSubState = ShootSubState.IDLE;
//        shootTimer.reset();
//    }
//    private boolean isHiveTippedDetected() {
//// 判据：检测到有效倾倒信号或相对初始基准的倾角变化超过 15°
//        return robot.hiveScorable || Math.abs(robot.hiveRoll - baselineHiveRoll) > 15.0;
//    }
//    // =========================================================================
//// 主状态机逻辑
//// =========================================================================
//    private void updateAutoStateMachine() {
//        switch (autoState) {
//            case PRELOAD_HIVE_BASELINE_CHECK:
//// 发射 Preload 前先让摄像头采樣 0.25s 记录初始 Hive 基准，随后关闭摄像头节省算力
//                activeGoalX = RED_LOWER_GOAL_X;
//                activeGoalY = RED_LOWER_GOAL_Y;
//                turretAutoAim = true;
//                enableVisionCamera();
//                spinUpShooterForCurrentDistance();
//                if (pathTimer.seconds() >= 0.25) {
//                    baselineHiveRoll = robot.hiveRoll;
//                    disableVisionCamera();
//                    follower.follow(path1_ToPreloadShoot);
//                    enterState(AutoState.DRIVE_TO_PRELOAD_SHOOT);
//                }
//                break;
//            case DRIVE_TO_PRELOAD_SHOOT:
//                spinUpShooterForCurrentDistance();
//                if (!follower.isBusy()) {
//                    startNewShootSequence();
//                    enterState(AutoState.SHOOT_PRELOAD);
//                }
//                break;
//            case SHOOT_PRELOAD:
//                runNonBlockingShootSequence(1.1);
//                if (shootSubState == ShootSubState.DONE) {
//// 下发 Path2->3 连续直线数组，不停顿直奔 Garden 墙边
//                    followChain(chain2_3_GardenIntakeWall);
////                    follower.follow(chain2_3_GardenIntakeWall);
//                    enterState(AutoState.DRIVE_TO_GARDEN_WALL);
//                }
//                break;
//            case DRIVE_TO_GARDEN_WALL:
//// 行进中一经过 (32.5, 8.0) 附近即无缝放下吸球并启动吸球电机
//                if (getRedEquivalentX() <= 35.0) {
//                    startIntakeAction();
//                }
//// 到达墙边 (8.5, 8.0) 后停留 0.4s 确保吞稳球
//                if (!follower.isBusy() && pathTimer.seconds() > 1.8) {
//                    stopIntakeAction();
//// 提前唤醒摄像头准备在退回 (40,12) 时观察 Hive
//                    enableVisionCamera();
//                    followChain(chain4_5_ToHiveCheck);
//                    enterState(AutoState.DRIVE_AND_TURN_TO_HIVE_CHECK);
//                }
//                break;
//            case DRIVE_AND_TURN_TO_HIVE_CHECK:
//                activeGoalX = RED_LOWER_GOAL_X;
//                activeGoalY = RED_LOWER_GOAL_Y;
//                turretAutoAim = true;
//                if (!follower.isBusy() && robot.axonTurretArmL.isAtTarget(5.0)) {
//                    enterState(AutoState.CHECK_HIVE_TIP_1);
//                }
//                break;
//            case CHECK_HIVE_TIP_1:
//// 第一次检测 Hive 是否倾倒 (观察 0.5s)
//                if (pathTimer.seconds() >= 0.5) {
//                    boolean tipped = isHiveTippedDetected();
//// 得出结论后立刻关闭摄像头
//                    disableVisionCamera();
//                    if (tipped) {
//// 分支 B：已倾倒 -> 直接经 Path6 前往上半场 (40, 130)
//                        follower.follow(path6_DirectToUpperMid);
//                        enterState(AutoState.DRIVE_DIRECT_TO_UPPER_MID);
//                    } else {
//// 分支 A：未倾倒 -> 返回 (58, 8) 补射
//                        spinUpShooterForCurrentDistance();
//                        follower.follow(path6b_ReturnToSecondShoot);
//                        enterState(AutoState.DRIVE_TO_SECOND_SHOOT);
//                    }
//                }
//                break;
//// ---------------- 分支 A：未倾倒补射与二次检测 ----------------
//            case DRIVE_TO_SECOND_SHOOT:
//                spinUpShooterForCurrentDistance();
//                if (!follower.isBusy()) {
//                    startNewShootSequence();
//                    enterState(AutoState.SHOOT_SECOND_BATCH);
//                }
//                break;
//            case SHOOT_SECOND_BATCH:
//                runNonBlockingShootSequence(1.1);
//                if (shootSubState == ShootSubState.DONE) {
//// 补射完成后再次唤醒摄像头并退回 (40, 12) 进行二次检测
//                    enableVisionCamera();
//                    followChain(chain4_5_SecondHiveCheck);
//                    enterState(AutoState.RETURN_FOR_SECOND_HIVE_CHECK);
//                }
//                break;
//            case RETURN_FOR_SECOND_HIVE_CHECK:
//                if (!follower.isBusy() && robot.axonTurretArmL.isAtTarget(5.0)) {
//                    enterState(AutoState.CHECK_HIVE_TIP_2);
//                }
//                break;
//            case CHECK_HIVE_TIP_2:
//                if (pathTimer.seconds() >= 0.5) {
//                    boolean tipped = isHiveTippedDetected();
//// 无论是否倾倒，立刻关闭摄像头进入下一步
//                    disableVisionCamera();
//                    if (tipped) {
//// 二次检测确认已倾倒 -> 继续前往上半场 (40, 130)
//                        follower.follow(path6_DirectToUpperMid);
//                        enterState(AutoState.DRIVE_DIRECT_TO_UPPER_MID);
//                    } else {
//// 仍未倾倒 -> 执行兜底战术：去近区另一侧 Flower 吸球再回来射击
//                        startIntakeAction();
//                         followChain(chainFallback_LowerFlowerIntake);
//                        enterState(AutoState.DRIVE_FALLBACK_LOWER_FLOWER);
//                    }
//                }
//                break;
//// ---------------- 分支 A2：另一侧近区 Flower 兜底补射 ----------------
//            case DRIVE_FALLBACK_LOWER_FLOWER:
//                startIntakeAction();
//                if (!follower.isBusy() && pathTimer.seconds() > 1.8) {
//                    stopIntakeAction();
//                    spinUpShooterForCurrentDistance();
//                    follower.follow(pathFallback_ReturnToShoot);
//                    enterState(AutoState.DRIVE_FALLBACK_SHOOT_POS);
//                }
//                break;
//            case DRIVE_FALLBACK_SHOOT_POS:
//                spinUpShooterForCurrentDistance();
//                if (!follower.isBusy()) {
//                    startNewShootSequence();
//                    enterState(AutoState.SHOOT_FALLBACK_BATCH);
//                }
//                break;
//            case SHOOT_FALLBACK_BATCH:
//                runNonBlockingShootSequence(1.2);
//                if (shootSubState == ShootSubState.DONE) {
//                    turretAutoAim = false;
//                    robot.setHoodPosition(0.35);
//                    follower.follow(pathFallback_ToLowerPark);
//                    enterState(AutoState.DRIVE_TO_LOWER_PARK);
//                }
//                break;
//            case DRIVE_TO_LOWER_PARK:
//                if (!follower.isBusy()) {
//                    enterState(AutoState.SHUTDOWN_AND_END);
//                }
//                break;
//// ---------------- 分支 B：上半场与 Flower 4 球 ----------------
//            case DRIVE_DIRECT_TO_UPPER_MID:
//                if (!follower.isBusy()) {
//                    follower.follow(path7_TurnAtUpperMid);
//                    enterState(AutoState.TURN_AT_UPPER_MID);
//                }
//                break;
//            case TURN_AT_UPPER_MID:
//// 切换炮台目标为红方上半场目标 (58.0, 79.5)
//                activeGoalX = RED_UPPER_GOAL_X;
//                activeGoalY = RED_UPPER_GOAL_Y;
//                turretAutoAim = true;
//                if (!follower.isBusy()) {
//// 下发 Path8->9 连续直线数组切入上半场射击位 (59, 133)
//                     followChain(chain8_9_UpperShoot);
//                    enterState(AutoState.DRIVE_TO_UPPER_SHOOT);
//                }
//                break;
//            case DRIVE_TO_UPPER_SHOOT:
//                spinUpShooterForCurrentDistance();
//                if (!follower.isBusy()) {
//                    startNewShootSequence();
//                    enterState(AutoState.SHOOT_UPPER_BATCH);
//                }
//                break;
//            case SHOOT_UPPER_BATCH:
//                runNonBlockingShootSequence(1.1);
//                if (shootSubState == ShootSubState.DONE) {
//// 下发 Path10->11 连续数组退回 (59, 120) 并转向 90°
//                     followChain(chain10_11_ToFlowerPrep);
//                    enterState(AutoState.DRIVE_AND_TURN_TO_FLOWER);
//                }
//                break;
//            case DRIVE_AND_TURN_TO_FLOWER:
//                if (!follower.isBusy()) {
//// 下发 Path12->13 连续数组：对齐 (48, 120) 后不停顿顶墙至 (48, 133) 吸 4 球
//                     followChain(chain12_13_FlowerIntakeWall);
//                    enterState(AutoState.DRIVE_TO_FLOWER_WALL);
//                }
//                break;
//            case DRIVE_TO_FLOWER_WALL:
//// 一进入 Flower 对齐区 (X <= 50.0) 立刻放下吸球机构并启动电机
//                if (getRedEquivalentX() <= 50.0) {
//                    startIntakeAction();
//                }
//// 顶墙 (48, 133) 停留 0.6s 确保吃进全部 4 颗球
//                if (!follower.isBusy() && pathTimer.seconds() > 1.8) {
//                    spinUpShooterForCurrentDistance();
//                    follower.follow(path14_BackToFlowerShoot);
//                    enterState(AutoState.DRIVE_TO_FLOWER_SHOOT);
//                }
//                break;
//            case DRIVE_TO_FLOWER_SHOOT:
//                activeGoalX = RED_UPPER_GOAL_X;
//                activeGoalY = RED_UPPER_GOAL_Y;
//                turretAutoAim = true;
//                spinUpShooterForCurrentDistance();
//                if (!follower.isBusy()) {
//                    startNewShootSequence();
//                    enterState(AutoState.SHOOT_FLOWER_BATCH);
//                }
//                break;
//            case SHOOT_FLOWER_BATCH:
//                runNonBlockingShootSequence(1.4);
//                if (shootSubState == ShootSubState.DONE) {
//                    stopIntakeAction();
//                    turretAutoAim = false;
//                    robot.setHoodPosition(0.35);
//                    follower.follow(path15_ToUpperPark);
//                    enterState(AutoState.DRIVE_TO_UPPER_PARK);
//                }
//                break;
//            case DRIVE_TO_UPPER_PARK:
//                if (!follower.isBusy()) {
//                    enterState(AutoState.SHUTDOWN_AND_END);
//                }
//                break;
//            case SHUTDOWN_AND_END:
//                break;
//        }
//    }
//    private void runNonBlockingShootSequence(double feedDurationSec) {
//        switch (shootSubState) {
//            case IDLE:
//                robot.BlockageArmL.setPosition(blockageblockposition);
//                robot.BlockageArmR.setPosition(blockageblockposition);
//                spinUpShooterForCurrentDistance();
//                shootTimer.reset();
//                shootSubState = ShootSubState.SPINNING_UP;
//                break;
//            case SPINNING_UP:
//                spinUpShooterForCurrentDistance();
//                boolean speedReady = Math.abs(cachedShooterVeloL - dynamicTargetVelo) <= toleranceforShoot;
//                boolean turretReady = robot.axonTurretArmL.isAtTarget(4.0);
//                if ((speedReady && turretReady) || shootTimer.seconds() > 1.5) {
//                    robot.BlockageArmL.setPosition(blockagereleaseposition);
//                    robot.BlockageArmR.setPosition(blockagereleaseposition);
//                    robot.IntakeMotorL.setPower(intakePowerShoot);
//                    robot.IntakeMotorR.setPower(intakePowerShoot);
//                    shootTimer.reset();
//                    shootSubState = ShootSubState.FEEDING;
//                }
//                break;
//            case FEEDING:
//                spinUpShooterForCurrentDistance();
//                if (shootTimer.seconds() >= feedDurationSec) {
//                    robot.BlockageArmL.setPosition(blockageblockposition);
//                    robot.BlockageArmR.setPosition(blockageblockposition);
//                    robot.IntakeMotorL.setPower(intakePowerOff);
//                    robot.IntakeMotorR.setPower(intakePowerOff);
//                    robot.shooterL.setVelocity(ShooterPIDFConfig.flyWheelIdleSpeed);
//                    robot.shooterR.setVelocity(ShooterPIDFConfig.flyWheelIdleSpeed);
//                    shootSubState = ShootSubState.DONE;
//                }
//                break;
//            case DONE:
//                break;
//        }
//    }
//    private void spinUpShooterForCurrentDistance() {
//        robot.setHoodPosition(dynamicTargetHood);
//        robot.shooterL.setVelocity(Math.abs(dynamicTargetVelo));
//        robot.shooterR.setVelocity(Math.abs(dynamicTargetVelo));
//    }
//    private void startIntakeAction() {
//        robot.BlockageArmL.setPosition(blockageblockposition);
//        robot.BlockageArmR.setPosition(blockageblockposition);
//        robot.IntakeMotorL.setPower(intakePowerIntake);
//        robot.IntakeMotorR.setPower(intakePowerIntake);
//    }
//    private void stopIntakeAction() {
//        robot.IntakeMotorL.setPower(intakePowerOff);
//        robot.IntakeMotorR.setPower(intakePowerOff);
//    }
//    private void stopAllMechanisms() {
//        robot.setAllPower(0);
//        robot.MasterShooterMotorL.setVelocity(0);
//        robot.SlaveShooterMotorR.setVelocity(0);
//        robot.IntakeMotorL.setPower(0);
//        robot.IntakeMotorR.setPower(0);
//        robot.BlockageArmL.setPosition(blockageblockposition);
//        robot.BlockageArmR.setPosition(blockageblockposition);
//        if (robot.servoTurretArmL != null) robot.servoTurretArmL.setPower(0);
//        if (robot.servoTurretArmR != null) robot.servoTurretArmR.setPower(0);
//    }
//    private void saveFinalPoseForTeleOp() {
//        if (follower != null) {
//            Pose endPose = follower.pose();
//            robot.finalAutoX = endPose.x();
//            robot.finalAutoY = endPose.y();
//            robot.finalAutoHeading = endPose.heading();
//        }
//    }
//    private void updateAutoTurretControl() {
//        Pose p = follower.pose();
//        double h = p.heading();
//        double currentTurretDeg = robot.axonTurretArmL.getTotalRotation();
//        double pivotX = p.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h);
//        double pivotY = p.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h);
//        double targetX = toSymX(activeGoalX);
//        double targetY = toSymY(activeGoalY);
//        double dx = targetX - pivotX;
//        double dy = targetY - pivotY;
//        double rangeSq = dx * dx + dy * dy;
//        currentGoalDistanceIn = Math.sqrt(rangeSq);
//        dynamicTargetHood = interpolateTable(DIST_TABLE_IN, HOOD_TABLE, currentGoalDistanceIn) + HOOD_POS_TRIM;
//        dynamicTargetVelo = interpolateTable(DIST_TABLE_IN, VELO_TABLE, currentGoalDistanceIn) + SHOOTER_VELO_TRIM;
//        if (!turretAutoAim || rangeSq < MIN_SHOOT_RANGE_IN * MIN_SHOOT_RANGE_IN) {
//            turretTargetDeg = 0.0;
//        } else {
//            double relAngle = Math.atan2(dy, dx) - h;
//            while (relAngle >  Math.PI) relAngle -= 2 * Math.PI;
//            while (relAngle < -Math.PI) relAngle += 2 * Math.PI;
//            double candidateDeg = Math.toDegrees(relAngle);
//            if (candidateDeg - currentTurretDeg > 180.0
//                    && (candidateDeg - 360.0) >= -TURRET_MAX_ROTATION_DEG) {
//                candidateDeg -= 360.0;
//            } else if (candidateDeg - currentTurretDeg < -180.0
//                    && (candidateDeg + 360.0) <= TURRET_MAX_ROTATION_DEG) {
//                candidateDeg += 360.0;
//            }
//            turretTargetDeg = Range.clip(candidateDeg, -TURRET_MAX_ROTATION_DEG, TURRET_MAX_ROTATION_DEG);
//        }
//        double dt = turretRateTimer.seconds();
//        turretRateTimer.reset();
//        if (TURRET_MAX_SPEED_DEG_PER_S > 0 && dt > 0.0 && dt < 0.5) {
//            double maxStep = TURRET_MAX_SPEED_DEG_PER_S * dt;
//            smoothedTurretTargetDeg += Range.clip(turretTargetDeg - smoothedTurretTargetDeg, -maxStep, maxStep);
//        } else {
//            smoothedTurretTargetDeg = turretTargetDeg;
//        }
//        robot.axonTurretArmL.trackTargetRotation(smoothedTurretTargetDeg);
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
//    }
//    private void initShooterPIDF() {
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
//
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
//    //    private double interpolateTable(double[] xTable, double[] yTable, double x) {
////        int n = xTable.length;
////        if (x <= xTable) return yTable;
////        if (x >= xTable[n - 1]) return yTable[n - 1];
////        for (int i = 0; i < n - 1; i++) {
////            if (x >= xTable[i] && x <= xTable[i + 1]) {
////                double ratio = (x - xTable[i]) / (xTable[i + 1] - xTable[i]);
////                return yTable[i] + ratio * (yTable[i + 1] - yTable[i]);
////            }
////        }
////        return yTable[n - 1];
////    }
//    private void updateAutoTelemetry(double loopTimeMs) {
//        Pose pose = follower.pose();
//        telemetry.addData("Loop Time (ms)", loopTimeMs);
//        telemetry.addData("Camera Streaming", isCameraStreaming ? 1 : 0);
//        telemetry.addData("Auto State", autoState.name());
//        telemetry.addData("Shoot SubState", shootSubState.name());
//        telemetry.addData("Pose (X/Y/H)", "%.1f / %.1f / %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
//        telemetry.addData("Goal Dist (in)", currentGoalDistanceIn);
//        telemetry.addData("Turret Target (deg)", smoothedTurretTargetDeg);
//        telemetry.addData("Turret Actual (deg)", robot.axonTurretArmL.getTotalRotation());
//        telemetry.addData("Hive Scorable", robot.hiveScorable ? 1 : 0);
//        telemetry.addData("Hive Roll (deg)", robot.hiveRoll);
//    }
//}