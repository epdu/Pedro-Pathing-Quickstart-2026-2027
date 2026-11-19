package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import android.util.Size;
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
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class HardwareDecodeRobotBlueBioBuzzSeason {
    public HardwareMap hwMap = null;
    public List<LynxModule> allHubs;
    public Alliance alliance = Alliance.BLUE;

    // 底盘电机
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightRearMotor;
    public DcMotorEx leftRearMotor;

    // 发射与双吸球电机
    public DcMotorEx MasterShooterMotorL;
    public DcMotorEx SlaveShooterMotorR;
    public DcMotorEx shooterL;
    public DcMotorEx shooterR;
    public DcMotorEx IntakeMotorL;
    public DcMotorEx IntakeMotorR;

    // 挡板与仰角舵机
    public ServoImplEx BlockageArmL;
    public ServoImplEx BlockageArmR;
    public ServoImplEx HoodArmL;
    public ServoImplEx HoodArmR;

    // 炮台连续旋转舵机（RTPAxon + 外置 REV 编码器）
    public CRServo servoTurretArmL;
    public CRServo servoTurretArmR;
    public DcMotorEx revEncoder;
    public RTPAxon axonTurretArmL;
//    public RTPAxon axonTurretArmR;

    // 传感器与指示灯
    public DigitalChannel redLED;
    public DigitalChannel greenLED;
    public VoltageSensor voltageCHub;
    public VoltageSensor voltageExHub;
    public GoBildaPinpointDriver odo;
    public IMU imu;
    // 视觉识别组件
    public AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    public AprilTagProcessor myAprilTagProcessor;
    public AprilTagLibrary myAprilTagLibrary;
    public VisionPortal.Builder myVisionPortalBuilder;
    public VisionPortal myVisionPortal;
    // 视觉识别状态变量
    public volatile boolean hiveScorable = false;
    public volatile String hiveClusterName = "";
    public volatile double hiveRoll = 0.0;
    public volatile double hivePercentFound = 0.0;
    // 通用参数与自动阶段继承位姿
    public static final double DriveTrains_POWER = 0.95;
    public double HoodArmsetPosition = 0.51;
    public boolean rtp = true;
    public double finalAutoX;
    public double finalAutoY;
    public double finalAutoHeading;
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // 1. 开启 REV Hub 批量缓存模式（大幅提升编码器、Pinpoint 和飞轮速度读取效率）
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. 初始化底盘电机
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

        // 3. 初始化飞轮发射电机
        MasterShooterMotorL = hwMap.get(DcMotorEx.class, "MasterShooterMotorL");
        SlaveShooterMotorR  = hwMap.get(DcMotorEx.class, "SlaveShooterMotorR");
        MasterShooterMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        MasterShooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SlaveShooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterL = MasterShooterMotorL;
        shooterR = SlaveShooterMotorR;

        // 4. 初始化双吸球/传送电机
        IntakeMotorL = hwMap.get(DcMotorEx.class, "IntakeMotorL");
        IntakeMotorR = hwMap.get(DcMotorEx.class, "IntakeMotorR");
        IntakeMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        IntakeMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setAllPower(0);

        // 5. 安全获取电压传感器（兼容 Expansion Hub 3 或 2，防止名称变动导致闪退）
        voltageCHub = hwMap.tryGet(VoltageSensor.class, "Control Hub");
        voltageExHub = hwMap.tryGet(VoltageSensor.class, "Expansion Hub 3");
        if (voltageExHub == null) {
            voltageExHub = hwMap.tryGet(VoltageSensor.class, "Expansion Hub 2");
        }

        // 6. 初始化双连续旋转舵机炮台（RTPAxon + 外置 REV 编码器）
        servoTurretArmL = hwMap.get(CRServo.class, "servoTurretArmL");
        servoTurretArmR = hwMap.get(CRServo.class, "servoTurretArmR");
        // 若两只舵机镜像对装导致转动方向相反，取消下面这行注释即可：
        // servoTurretArmR.setDirection(CRServo.Direction.REVERSE);
        revEncoder = hwMap.get(DcMotorEx.class, "RBMotor");

        axonTurretArmL = new RTPAxon(servoTurretArmL, revEncoder);
//        axonTurretArmR = new RTPAxon(servoTurretArmR, revEncoder);

        axonTurretArmL.setMaxPower(0.5);
//        axonTurretArmR.setMaxPower(0.5);
        axonTurretArmL.setRtp(rtp);
//        axonTurretArmR.setRtp(rtp);
        axonTurretArmL.setPidCoeffs(0.0069, 0.000, 0.0002);
        //////////////////////tune PID here
//        axonTurretArmR.setPidCoeffs(0.01, 0.000, 0.00044);

        // 7. 初始化双挡板舵机与双仰角舵机
        BlockageArmL = hwMap.get(ServoImplEx.class, "BlockageArmL");
        BlockageArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        BlockageArmL.setPosition(0.1);

        BlockageArmR = hwMap.get(ServoImplEx.class, "BlockageArmR");
        BlockageArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        BlockageArmR.setDirection(ServoImplEx.Direction.REVERSE);
        BlockageArmR.setPosition(0.1);

        HoodArmL = hwMap.get(ServoImplEx.class, "HoodArmL");
        HoodArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        HoodArmL.setDirection(ServoImplEx.Direction.REVERSE);
        HoodArmL.setPosition(HoodArmsetPosition);

        HoodArmR = hwMap.get(ServoImplEx.class, "HoodArmR");
        HoodArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        HoodArmR.setPosition(HoodArmsetPosition);

        // 8. 初始化 GoBilda Pinpoint 里程计电脑
        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(28.5, 156.0, CM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        // 9. 初始化内置 IMU
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // 10. 初始化状态指示 LED
        redLED   = hwMap.get(DigitalChannel.class, "red");
        greenLED = hwMap.get(DigitalChannel.class, "green");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setState(true);
        greenLED.setState(true);
    }

    /**
     * 初始化视觉系统（单次调用，防止摄像头重复打开崩溃）
     * @param enableLiveView 调试阶段传 true 开启预览；正式比赛传 false 节省约 20% CPU 算力
     */
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

    /**
     * 每轮主循环开头调用一次，清空并拉取最新 REV Hub 批量缓存数据
     */
    public void clearBulkCache() {
        if (allHubs != null) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
    /**
     * 同步更新双侧仰角舵机位置（带安全范围限幅，防止超行程损坏舵机）
     */
    public void setHoodPosition(double pos) {
        double safePos = Math.max(0.05, Math.min(0.95, pos));
        if (HoodArmL != null) HoodArmL.setPosition(safePos);
        if (HoodArmR != null) HoodArmR.setPosition(safePos);
    }
    /**
     * 同步刷新双炮台 Axon 舵机闭环控制（每帧调用一次）
     */

    public void updateTurretAxons() {
        if (axonTurretArmL != null) {
            // 1. 主舵机执行唯一一次编码器读取与 PID 计算
            axonTurretArmL.update();

            // 2. 副舵机 100% 同步跟随主舵机算出的最终功率，零时间差、零内耗
            if (servoTurretArmR != null) {
                servoTurretArmR.setPower(axonTurretArmL.getPower());
            }
        }
    }

//    public void updateTurretAxons() {
//        if (axonTurretArmL != null) axonTurretArmL.update();
//        if (axonTurretArmR != null) axonTurretArmR.update();
//    }
    public void setMotorPower(double lF, double rF, double lB, double rB) {
        leftFrontMotor.setPower(lF * DriveTrains_POWER);
        leftRearMotor.setPower(lB * DriveTrains_POWER);
        rightRearMotor.setPower(rB * DriveTrains_POWER);
        rightFrontMotor.setPower(rF * DriveTrains_POWER);
    }
    public void setAllPower(double p) {
        setMotorPower(p, p, p, p);
    }
}