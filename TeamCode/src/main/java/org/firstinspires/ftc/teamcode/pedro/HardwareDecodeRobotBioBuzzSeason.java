package org.firstinspires.ftc.teamcode.pedro;

import android.util.Size;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Configurable
public class HardwareDecodeRobotBioBuzzSeason {
    public HardwareMap hwMap = null;
    public List<LynxModule> allHubs;
    public Alliance alliance = Alliance.RED; // 預設紅方基準，藍方比賽時改為 Alliance.BLUE 即可

    // 場地總邊長常數（Pedro Pathing 141.5 x 141.5 中心對稱場地）
    public static final double FIELD_SIZE_IN = 141.5;

    // =========================================================================
    // 1. 【全工程唯一炮台 PID 基準參數】Auto / TeleOp / RTPAxon 全部直接引用此處
    // =========================================================================
    public static double TURRET_KP = 0.018;
    public static double TURRET_KI = 0.0000;
    public static double TURRET_KD = 0.0016;
    public static double TURRET_MAX_POWER = 0.5;

    // =========================================================================
    // 2. 【全工程唯一飛輪 PIDF 基準參數】Auto / TeleOp 全部直接引用此處
    // =========================================================================
    @Configurable
    public static class ShooterPIDFConfig {
        public static double kP = 300.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 15.0;
        public static double targetSPEED = 1050.0;
        public static double flyWheelIdleSpeed = 1050.0 * 0.6;
        public static double targetSPEEDfar = 1400.0;
        public static double flyWheelIdleSpeedfar = 1400.0 * 0.6;
    }

    // =========================================================================
    // 3. 【全工程唯一距離自適應仰角與轉速插值表】單位：英吋
    // =========================================================================
    public static double SHOOTER_VELO_TRIM = 0.0; // 現場全局轉速微調補償（例如電池電壓低時填 +40）
    public static double HOOD_POS_TRIM = 0.0;     // 現場全局仰角微調補償（例如射偏高時填 -0.02）

    public static final double[] DIST_TABLE_IN = { 35.0,  60.0,  85.0,  110.0, 140.0 };
    public static final double[] HOOD_TABLE    = { 0.35,  0.42,  0.48,  0.53,  0.58  };
    public static final double[] VELO_TABLE    = { 950.0, 1050.0, 1180.0, 1320.0, 1450.0 };

    // 底盤電機
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightRearMotor;
    public DcMotorEx leftRearMotor;

    // 發射與雙吸球電機
    public DcMotorEx MasterShooterMotorL;
    public DcMotorEx SlaveShooterMotorR;
    public DcMotorEx shooterL;
    public DcMotorEx shooterR;
    public DcMotorEx IntakeMotorL;
    public DcMotorEx IntakeMotorR;

    // 擋板與仰角舵機
    public ServoImplEx BlockageArmL;
    public ServoImplEx BlockageArmR;
    public ServoImplEx HoodArmL;
    public ServoImplEx HoodArmR;

    // 炮台連續旋轉舵機（單主 RTPAxon PID + 副舵機純功率跟隨 + 外置 REV 編碼器）
    public CRServo servoTurretArmL;
    public CRServo servoTurretArmR;
    public DcMotorEx revEncoder;
    public RTPAxon axonTurretArmL;

    // 傳感器與指示燈
    public DigitalChannel redLED;
    public DigitalChannel greenLED;
    public VoltageSensor voltageCHub;
    public VoltageSensor voltageExHub;
    public GoBildaPinpointDriver odo;
    public IMU imu;

    // 視覺識別組件
    public AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    public AprilTagProcessor myAprilTagProcessor;
    public AprilTagLibrary myAprilTagLibrary;
    public VisionPortal.Builder myVisionPortalBuilder;
    public VisionPortal myVisionPortal;

    // 視覺識別狀態變數
    public volatile boolean hiveScorable = false;
    public volatile String hiveClusterName = "";
    public volatile double hiveRoll = 0.0;
    public volatile double hivePercentFound = 0.0;

    // 通用參數與自動階段繼承位姿
    public static final double DriveTrains_POWER = 0.95;
    public double HoodArmsetPosition = 0.51;
    public boolean rtp = true;
    public double finalAutoX;
    public double finalAutoY;
    public double finalAutoHeading;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // 1. 開啟 REV Hub 批量緩存模式（大幅提升編碼器、Pinpoint 和飛輪速度讀取效率）
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. 初始化底盤電機
        leftFrontMotor  = hwMap.get(DcMotorEx.class, "LFMotor");
        leftRearMotor   = hwMap.get(DcMotorEx.class, "LBMotor");
        rightFrontMotor = hwMap.get(DcMotorEx.class, "RFMotor");
        rightRearMotor  = hwMap.get(DcMotorEx.class, "RBMotor");

        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 3. 初始化飛輪發射電機並下發全工程唯一 PIDF 參數
        MasterShooterMotorL = hwMap.get(DcMotorEx.class, "MasterShooterMotorL");
        SlaveShooterMotorR  = hwMap.get(DcMotorEx.class, "SlaveShooterMotorR");
        MasterShooterMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        MasterShooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SlaveShooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MasterShooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        applyShooterPIDF();

        MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL = MasterShooterMotorL;
        shooterR = SlaveShooterMotorR;

        // 4. 初始化雙吸球/傳送電機
        IntakeMotorL = hwMap.get(DcMotorEx.class, "IntakeMotorL");
        IntakeMotorR = hwMap.get(DcMotorEx.class, "IntakeMotorR");
        IntakeMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        IntakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setAllPower(0);

        // 5. 安全獲取電壓傳感器
        voltageCHub = hwMap.tryGet(VoltageSensor.class, "Control Hub");
        voltageExHub = hwMap.tryGet(VoltageSensor.class, "Expansion Hub 3");
        if (voltageExHub == null) {
            voltageExHub = hwMap.tryGet(VoltageSensor.class, "Expansion Hub 2");
        }

        // 6. 初始化雙連續旋轉舵機炮台（引用全工程唯一炮台 PID 基準參數）
        servoTurretArmL = hwMap.get(CRServo.class, "servoTurretArmL");
        servoTurretArmR = hwMap.get(CRServo.class, "servoTurretArmR");
        // 若兩隻舵機鏡像對裝導致轉動方向相反，取消下面這行註釋即可：
        // servoTurretArmR.setDirection(CRServo.Direction.REVERSE);
        revEncoder = hwMap.get(DcMotorEx.class, "RBMotor");

        axonTurretArmL = new RTPAxon(servoTurretArmL, revEncoder);
        axonTurretArmL.setMaxPower(TURRET_MAX_POWER);
        axonTurretArmL.setRtp(rtp);
        axonTurretArmL.setPidCoeffs(TURRET_KP, TURRET_KI, TURRET_KD);

        // 7. 初始化雙擋板舵機與雙仰角舵機
        BlockageArmL = hwMap.get(ServoImplEx.class, "BlockageArmL");
        BlockageArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        BlockageArmL.setDirection(ServoImplEx.Direction.REVERSE);
        BlockageArmL.setPosition(0.1);

        BlockageArmR = hwMap.get(ServoImplEx.class, "BlockageArmR");
        BlockageArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        BlockageArmR.setDirection(ServoImplEx.Direction.REVERSE);
        BlockageArmR.setPosition(0.1);



        //        // 7. 初始化双挡板舵机与双仰角舵机
//        BlockageArmL = hwMap.get(ServoImplEx.class, "BlockageArmL");// 11202026 Control  hub port 1
//        BlockageArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        BlockageArmL.setDirection(ServoImplEx.Direction.REVERSE);
//        BlockageArmL.setPosition(0.1);
//
//        BlockageArmR = hwMap.get(ServoImplEx.class, "BlockageArmR");// 11202026 Control  hub port 4
//        BlockageArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
////        BlockageArmR.setDirection(ServoImplEx.Direction.REVERSE);
//        BlockageArmR.setPosition(0.1);


        HoodArmL = hwMap.get(ServoImplEx.class, "HoodArmL");
        HoodArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        HoodArmL.setDirection(ServoImplEx.Direction.REVERSE);
        HoodArmL.setPosition(HoodArmsetPosition);

        HoodArmR = hwMap.get(ServoImplEx.class, "HoodArmR");
        HoodArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        HoodArmR.setPosition(HoodArmsetPosition);

        // 8. GoBilda Pinpoint 里程計交由 Pedro Pathing 的 Constants.create(hardwareMap) 統一管理，避免重複校準覆蓋

        // 9. 初始化內置 IMU
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
        // 10. 初始化狀態指示 LED
        redLED   = hwMap.get(DigitalChannel.class, "red");
        greenLED = hwMap.get(DigitalChannel.class, "green");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setState(true);
        greenLED.setState(true);
    }
    /**
     * 將 ShooterPIDFConfig 參數下發至左右飛輪（支援初始化與 Panels 網頁端即時熱更新）
     */
    public void applyShooterPIDF() {
        if (MasterShooterMotorL != null && SlaveShooterMotorR != null) {
            PIDFCoefficients pidf = new PIDFCoefficients(
                    ShooterPIDFConfig.kP,
                    ShooterPIDFConfig.kI,
                    ShooterPIDFConfig.kD,
                    ShooterPIDFConfig.kF
            );
            MasterShooterMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            SlaveShooterMotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }
    }
    public void initVision(boolean enableLiveView) {
        myAprilTagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
        myAprilTagProcessorBuilder.setDrawTagID(enableLiveView);
        myAprilTagProcessorBuilder.setDrawTagOutline(enableLiveView);
        myAprilTagProcessorBuilder.setDrawAxes(enableLiveView);
        myAprilTagProcessorBuilder.setDrawCubeProjection(enableLiveView);
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.enableLiveView(enableLiveView);
        myVisionPortalBuilder.setAutoStopLiveView(true);
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        myVisionPortal = myVisionPortalBuilder.build();
    }
    public void clearBulkCache() {
        if (allHubs != null) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
    public void setHoodPosition(double pos) {
        double safePos = Math.max(0.05, Math.min(0.95, pos));
        if (HoodArmL != null) HoodArmL.setPosition(safePos);
        if (HoodArmR != null) HoodArmR.setPosition(safePos);
    }
    public void updateTurretAxons() {
        if (axonTurretArmL != null) {
            axonTurretArmL.update();
            if (servoTurretArmR != null) {
                servoTurretArmR.setPower(axonTurretArmL.getPower());
            }
        }
    }
    public void setMotorPower(double lF, double rF, double lB, double rB) {
        leftFrontMotor.setPower(lF * DriveTrains_POWER);
        leftRearMotor.setPower(lB * DriveTrains_POWER);
        rightRearMotor.setPower(rB * DriveTrains_POWER);
        rightFrontMotor.setPower(rF * DriveTrains_POWER);
    }
    public void setAllPower(double p) {
        setMotorPower(p, p, p, p);
    }
    // =========================================================================
// 距離自適應仰角與飛輪轉速插值計算方法（Auto / TeleOp 共用）
// =========================================================================
    public double getDynamicHoodPosition(double distanceIn) {
        return interpolateTable(DIST_TABLE_IN, HOOD_TABLE, distanceIn) + HOOD_POS_TRIM;
    }
    public double getDynamicShooterVelocity(double distanceIn) {
        return interpolateTable(DIST_TABLE_IN, VELO_TABLE, distanceIn) + SHOOTER_VELO_TRIM;
    }
    public static double interpolateTable(double[] xTable, double[] yTable, double x) {
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
    // =========================================================================
// 紅藍聯盟全自動對稱轉換引擎（以紅方為基準，141.5 x 141.5 中心對稱場地）
// =========================================================================
    public double toAllianceX(double redX) {
        return (alliance == Alliance.BLUE) ? (FIELD_SIZE_IN - redX) : redX;
    }
    public double toAllianceY(double redY) {
        return (alliance == Alliance.BLUE) ? (FIELD_SIZE_IN - redY) : redY;
    }
    public double toAllianceHeadingDeg(double redHeadingDeg) {
        double deg = (alliance == Alliance.BLUE) ? (redHeadingDeg + 180.0) : redHeadingDeg;
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }
    public Pose toAlliancePose(double redX, double redY, double redHeadingDeg) {
        return PoseFactory.degrees().of(
                toAllianceX(redX),
                toAllianceY(redY),
                toAllianceHeadingDeg(redHeadingDeg)
        );
    }
    public boolean isInFrontOfGoal(double dyFromRobotToGoal) {
        return (alliance == Alliance.BLUE) ? (dyFromRobotToGoal < 0) : (dyFromRobotToGoal > 0);
    }
    public String getAllianceTagPrefix() {
        return (alliance == Alliance.RED) ? "R" : "B";
    }
}



//package org.firstinspires.ftc.teamcode.pedro;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
//import android.util.Size;
//
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.math.Pose;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.PwmControl;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import java.util.List;
//
//public class HardwareDecodeRobotBioBuzzSeason {
//    public HardwareMap hwMap = null;
//    public List<LynxModule> allHubs;
//    public Alliance alliance = Alliance.RED; // 默认红方基准，蓝方比赛时改为 Alliance.BLUE
////    public Alliance alliance = Alliance.BLUE; // 默认红方基准，蓝方比赛时改为 Alliance.BLUE
//
//    // 场地总边长常数（Pedro Pathing 141.5 x 141.5 中心对称场地）
//    public static final double FIELD_SIZE_IN = 141.5;
//
//    // 底盘电机
//    public DcMotorEx rightFrontMotor;
//    public DcMotorEx leftFrontMotor;
//    public DcMotorEx rightRearMotor;
//    public DcMotorEx leftRearMotor;
//
//    // 发射与双吸球电机
//    public DcMotorEx MasterShooterMotorL;
//    public DcMotorEx SlaveShooterMotorR;
//    public DcMotorEx shooterL;
//    public DcMotorEx shooterR;
//    public DcMotorEx IntakeMotorL;
//    public DcMotorEx IntakeMotorR;
//
//    // 挡板与仰角舵机
//    public ServoImplEx BlockageArmL;
//    public ServoImplEx BlockageArmR;
//    public ServoImplEx HoodArmL;
//    public ServoImplEx HoodArmR;
//
//    // 炮台连续旋转舵机（单主 RTPAxon PID + 副舵机纯功率跟随 + 外置 REV 编码器）
//    public CRServo servoTurretArmL;
//    public CRServo servoTurretArmR;
//    public DcMotorEx revEncoder;
//    public RTPAxon axonTurretArmL;
//
//    // 传感器与指示灯
//    public DigitalChannel redLED;
//    public DigitalChannel greenLED;
//    public VoltageSensor voltageCHub;
//    public VoltageSensor voltageExHub;
//    public GoBildaPinpointDriver odo;
//    public IMU imu;
//
//    // 视觉识别组件
//    public AprilTagProcessor.Builder myAprilTagProcessorBuilder;
//    public AprilTagProcessor myAprilTagProcessor;
//    public AprilTagLibrary myAprilTagLibrary;
//    public VisionPortal.Builder myVisionPortalBuilder;
//    public VisionPortal myVisionPortal;
//
//    // 视觉识别状态变量
//    public volatile boolean hiveScorable = false;
//    public volatile String hiveClusterName = "";
//    public volatile double hiveRoll = 0.0;
//    public volatile double hivePercentFound = 0.0;
//
//    // 通用参数与自动阶段继承位姿
//    public static final double DriveTrains_POWER = 0.95;
//    public double HoodArmsetPosition = 0.51;
//    public boolean rtp = true;
//    public double finalAutoX;
//    public double finalAutoY;
//    public double finalAutoHeading;
//
//    public static double TURRET_KP = 0.018;
//    public static double TURRET_KI = 0.0000;
//    public static double TURRET_KD = 0.0016;
//    public static double TURRET_MAX_POWER = 0.5;
//
//    // =========================================================================
//// 【全工程唯一飞轮 PIDF 基准参数】Auto / TeleOp 全部直接引用此处，支持 Panels 实时调节
//// =========================================================================
//    @com.bylazar.configurables.annotations.Configurable
//    public static class ShooterPIDFConfig {
//        public static double kP = 300.0;
//        public static double kI = 0.0;
//        public static double kD = 0.0;
//        public static double kF = 15.0;
//        public static double targetSPEED = 1050.0;
//        public static double flyWheelIdleSpeed = 1050.0 * 0.6;
//        public static double targetSPEEDfar = 1400.0;
//        public static double flyWheelIdleSpeedfar = 1400.0 * 0.6;
//    }
//
//
//    public void init(HardwareMap ahwMap) {
//        hwMap = ahwMap;
//
//        // 1. 开启 REV Hub 批量缓存模式
//        allHubs = hwMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//
//        // 2. 初始化底盘电机
//        leftFrontMotor  = hwMap.get(DcMotorEx.class, "LFMotor");//11202026 control hub port 0
//        leftRearMotor   = hwMap.get(DcMotorEx.class, "LBMotor");//11202026 control hub port 1
//        rightFrontMotor = hwMap.get(DcMotorEx.class, "RFMotor");//11202026 control hub port 2
//        rightRearMotor  = hwMap.get(DcMotorEx.class, "RBMotor");//11202026 control hub port 3
//
//        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // 3. 初始化飞轮发射电机
//        // 3. 初始化飞轮发射电机并下发全工程唯一 PIDF 基准参数
//        MasterShooterMotorL = hwMap.get(DcMotorEx.class, "MasterShooterMotorL");
//        SlaveShooterMotorR  = hwMap.get(DcMotorEx.class, "SlaveShooterMotorR");
//        MasterShooterMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        MasterShooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        SlaveShooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        MasterShooterMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlaveShooterMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        com.qualcomm.robotcore.hardware.PIDFCoefficients pidf = new com.qualcomm.robotcore.hardware.PIDFCoefficients(
//                ShooterPIDFConfig.kP, ShooterPIDFConfig.kI, ShooterPIDFConfig.kD, ShooterPIDFConfig.kF
//        );
//        MasterShooterMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//        SlaveShooterMotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
//
//        MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        shooterL = MasterShooterMotorL;
//        shooterR = SlaveShooterMotorR;
//
//
//        // 4. 初始化双吸球/传送电机
//        IntakeMotorL = hwMap.get(DcMotorEx.class, "IntakeMotorL");
//        IntakeMotorR = hwMap.get(DcMotorEx.class, "IntakeMotorR");
//        IntakeMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        IntakeMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        IntakeMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
//        IntakeMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        IntakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        setAllPower(0);
//
//        // 5. 安全获取电压传感器
//        voltageCHub = hwMap.tryGet(VoltageSensor.class, "Control Hub");
//        voltageExHub = hwMap.tryGet(VoltageSensor.class, "Expansion Hub 3");
//        if (voltageExHub == null) {
//            voltageExHub = hwMap.tryGet(VoltageSensor.class, "Expansion Hub 2");
//        }
//
//        // 6. 初始化双连续旋转舵机炮台（单主 PID + 副舵机纯功率跟随）
//        servoTurretArmL = hwMap.get(CRServo.class, "servoTurretArmL");// 11202026 Control  hub port 0
//        servoTurretArmR = hwMap.get(CRServo.class, "servoTurretArmR");// 11202026 Control  hub port 3
//        // 若两只舵机镜像对装导致转动方向相反，取消下面这行注释即可：
//        // servoTurretArmR.setDirection(CRServo.Direction.REVERSE);
//        revEncoder = hwMap.get(DcMotorEx.class, "RBMotor");
//
//        axonTurretArmL = new RTPAxon(servoTurretArmL, revEncoder);
//        axonTurretArmL.setMaxPower(TURRET_MAX_POWER);
//        axonTurretArmL.setRtp(rtp);
//        axonTurretArmL.setPidCoeffs(TURRET_KP, TURRET_KI, TURRET_KD);
//
//        // 7. 初始化双挡板舵机与双仰角舵机
//        BlockageArmL = hwMap.get(ServoImplEx.class, "BlockageArmL");// 11202026 Control  hub port 1
//        BlockageArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        BlockageArmL.setDirection(ServoImplEx.Direction.REVERSE);
//        BlockageArmL.setPosition(0.1);
//
//        BlockageArmR = hwMap.get(ServoImplEx.class, "BlockageArmR");// 11202026 Control  hub port 4
//        BlockageArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
////        BlockageArmR.setDirection(ServoImplEx.Direction.REVERSE);
//        BlockageArmR.setPosition(0.1);
//
//        HoodArmL = hwMap.get(ServoImplEx.class, "HoodArmL");// 11202026 expansion  hub port 0
//        HoodArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        HoodArmL.setDirection(ServoImplEx.Direction.REVERSE);
//        HoodArmL.setPosition(HoodArmsetPosition);
//
//        HoodArmR = hwMap.get(ServoImplEx.class, "HoodArmR");// 11202026 expansion  hub port 3
//        HoodArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        HoodArmR.setPosition(HoodArmsetPosition);
//
//        // 8. GoBilda Pinpoint 里程计交由 Pedro Pathing 的 Constants.create(hardwareMap) 统一管理，此处无需重复校准覆盖
//        /*
//        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
//        odo.setOffsets(28.5, 156.0, CM);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(
//                GoBildaPinpointDriver.EncoderDirection.REVERSED,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD
//        );
//        odo.recalibrateIMU();
//        odo.resetPosAndIMU();
//        */
//        // 8. 初始化 GoBilda Pinpoint 里程计电脑
////        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
////        odo.setOffsets(28.5, 156.0, CM);
////        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
////        odo.setEncoderDirections(
////                GoBildaPinpointDriver.EncoderDirection.REVERSED,
////                GoBildaPinpointDriver.EncoderDirection.FORWARD
////        );
////        odo.recalibrateIMU();
////        odo.resetPosAndIMU();
//
//        // 9. 初始化内置 IMU
//        imu = hwMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//        imu.resetYaw();
//
//        // 10. 初始化状态指示 LED
//        redLED   = hwMap.get(DigitalChannel.class, "red");
//        greenLED = hwMap.get(DigitalChannel.class, "green");
//        redLED.setMode(DigitalChannel.Mode.OUTPUT);
//        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
//        redLED.setState(true);
//        greenLED.setState(true);
//    }
//
//    public void initVision(boolean enableLiveView) {
//        myAprilTagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();
//
//        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
//        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
//        myAprilTagProcessorBuilder.setDrawTagID(enableLiveView);
//        myAprilTagProcessorBuilder.setDrawTagOutline(enableLiveView);
//        myAprilTagProcessorBuilder.setDrawAxes(enableLiveView);
//        myAprilTagProcessorBuilder.setDrawCubeProjection(enableLiveView);
//        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
//
//        myVisionPortalBuilder = new VisionPortal.Builder();
//        myVisionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
//        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
//        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//        myVisionPortalBuilder.enableLiveView(enableLiveView);
//        myVisionPortalBuilder.setAutoStopLiveView(true);
//        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
//        myVisionPortal = myVisionPortalBuilder.build();
//    }
//
//    public void clearBulkCache() {
//        if (allHubs != null) {
//            for (LynxModule hub : allHubs) {
//                hub.clearBulkCache();
//            }
//        }
//    }
//    public void setHoodPosition(double pos) {
//        double safePos = Math.max(0.05, Math.min(0.95, pos));
//        if (HoodArmL != null) HoodArmL.setPosition(safePos);
//        if (HoodArmR != null) HoodArmR.setPosition(safePos);
//    }
//    public void updateTurretAxons() {
//        if (axonTurretArmL != null) {
//            axonTurretArmL.update();
//            if (servoTurretArmR != null) {
//                servoTurretArmR.setPower(axonTurretArmL.getPower());
//            }
//        }
//    }
//    public void setMotorPower(double lF, double rF, double lB, double rB) {
//        leftFrontMotor.setPower(lF * DriveTrains_POWER);
//        leftRearMotor.setPower(lB * DriveTrains_POWER);
//        rightRearMotor.setPower(rB * DriveTrains_POWER);
//        rightFrontMotor.setPower(rF * DriveTrains_POWER);
//    }
//    public void setAllPower(double p) {
//        setMotorPower(p, p, p, p);
//    }
//// =========================================================================
//// 红蓝联盟全自动对称转换引擎（以红方为基准，141.5 x 141.5 中心对称场地）
//// =========================================================================
//    /**
//     * 1. 转换单个 X 坐标（红方基准 -> 蓝方自动中心对称镜像）
//     */
//    public double toAllianceX(double redX) {
//        return (alliance == Alliance.BLUE) ? (FIELD_SIZE_IN - redX) : redX;
//    }
//    /**
//     * 2. 转换单个 Y 坐标（红方基准 -> 蓝方自动中心对称镜像）
//     */
//    public double toAllianceY(double redY) {
//        return (alliance == Alliance.BLUE) ? (FIELD_SIZE_IN - redY) : redY;
//    }
//    /**
//     * 3. 转换场地朝向角度（单位：度，红方基准 -> 蓝方自动旋转 180° 并归一化到 -180° ~ +180°）
//     */
//    public double toAllianceHeadingDeg(double redHeadingDeg) {
//        double deg = (alliance == Alliance.BLUE) ? (redHeadingDeg + 180.0) : redHeadingDeg;
//        while (deg > 180.0) deg -= 360.0;
//        while (deg <= -180.0) deg += 360.0;
//        return deg;
//    }
//    /**
//     * 4. 一键生成对应联盟的 Pedro Pathing Pose（用于起始点、重置点、自动阶段所有路径点）
//     */
//    public Pose toAlliancePose(double redX, double redY, double redHeadingDeg) {
//        return PoseFactory.degrees().of(
//                toAllianceX(redX),
//                toAllianceY(redY),
//                toAllianceHeadingDeg(redHeadingDeg)
//        );
//    }
//    /**
//     * 5. 判断机器人是否处于当前联盟目标开口的正面（红方下方目标开口朝 -Y，蓝方上方目标开口朝 +Y）
//     */
//    public boolean isInFrontOfGoal(double dyFromRobotToGoal) {
//        return (alliance == Alliance.BLUE) ? (dyFromRobotToGoal < 0) : (dyFromRobotToGoal > 0);
//    }
//    /**
//     * 6. 获取当前联盟对应的 AprilTag 标签前缀（"R" 或 "B"）
//     */
//    public String getAllianceTagPrefix() {
//        return (alliance == Alliance.RED) ? "R" : "B";
//    }
//}
//
