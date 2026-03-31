package org.firstinspires.ftc.teamcode.teleop.LinearOpMode.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;

@TeleOp(name = "JYF_RED_LIN", group = "LinearOp Red")
public class JYF_RED_LIN extends LinearOpMode {

    @Config
    public static class DriveTuning {
        // --- AprilTag / Husky ---
        public static int targetTagId = 5;          // -1 不限定
        public static int HUSKY_CENTER_X = 140;     // 画面中心点（调它修“对正偏左/偏右”）
        public static boolean strictIdFilter = true;
        public static boolean allowUnknownIdFallback = true; // strict 模式下可回退到未知 ID（-1）
        public static boolean invertAim = false;

        // --- AutoAim：像素误差 -> 目标角速度（deg/s） ---
        public static double aimPxToYawRateKp = 0.45;
        public static double maxTrackYawRate = 90.0;
        public static double trackYawRateKp = 0.0042; // 角速度闭环：deg/s 误差 -> rotCmd
        public static double aimMinCmd = 0.02;
        public static double aimMaxCmd = 0.34;
        public static int aimDeadbandPx = 10;         // 进入死区
        public static int aimReleasePx = 20;          // 退出死区（迟滞，防抖）
        public static double deadbandHoldSec = 0.14; // 死区内保持时间，抑制来回修正

        // --- 误差滤波 ---
        public static double errAlpha = 0.30;

        // --- 连续帧消抖 ---
        public static int aimAcquireFrames = 3;

        // --- 搜索参数（丢目标时） ---
        public static double scanPeriod = 1.2;
        public static double searchYawRateCmd = 42.0;
        public static double searchYawRateKp = 0.0045;
        public static double searchCmdMax = 0.16;

        // --- 输出平滑 ---
        public static double rotCmdAlpha = 0.42;
        public static double rotSlewPerSec = 1.25;

        // --- 对正判定稳定帧数（给 LED 用）---
        public static int alignedNeedFrames = 5;

        // --- LED 位置 ---
        public static double LED_RED_POS   = 0.28;
        public static double LED_GREEN_POS = 0.49;
        public static double LED_BLUE_POS  = 0.611;
        public static double LED_PURPLE_POS = 0.72;

        // --- Shooter 可调 ---
        public static double shooterTargetVel = 1000;
        public static double shooterReverseVel = 1000;
        public static double shooterReverseTime = 0.15;
        public static boolean shooterUseDistance = true;
        public static double shooterRefDistanceIn = 65.0;   // 参考距离：65 in
        public static double shooterRefVoltage = 12.05;     // 参考电压：12.05 V
        public static double shooterRefRpm = 1000.0;        // 参考转速：1000 rpm
        public static double shooterRpmPerIn = 3.0;         // 每英寸补偿 rpm（需场上调）
        public static double shooterMinVel = 900.0;
        public static double shooterMaxVel = 1400.0;
        public static boolean shooterUseVoltageComp = true;
        public static double shooterVoltageCompK = 0.35;    // 0~1，越大越补电压

        // --- 用 tag 面积估距 ---
        public static double distRefTagAreaPx = 784.0;     // 65in 时目标面积（像素），现场标定
        public static double distMinIn = 35.0;
        public static double distMaxIn = 140.0;
        public static double distAlpha = 0.30;              // 距离滤波
        public static double distHoldSec = 0.35;            // 丢目标保留距离

        // --- 红灯闪烁 ---
        public static double redBlinkPeriodSec = 0.5;

        // --- “看到目标 UI hold”（不参与控制）---
        public static double tagHoldSeconds = 0.25;

    }

    // ---------------- 硬件 ----------------
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    private DcMotorEx shootMotorL;
    private DcMotorEx shootMotorR;

    private DcMotor intake;

    private CRServo pullUp_back;
    private CRServo pullUp_mid;
    private CRServo pullUp_frontL;
    private CRServo pullUp_frontR;

    private Servo cam_rotate;
    private Servo statusLed;

    // --- HuskyLens ---
    private HuskyLens husky;
    private VoltageSensor batterySensor;

    // --- IMU ---
    private IMU imu;
    private boolean imuOk = false;

    // ---------------- 状态 ----------------
    private boolean shootMotorStatus = false;
    private boolean IntakeMotorStatus = false;

    private boolean lastIntakeButton = false;      // D-pad Up
    private boolean lastShooterToggleBtn = false;  // Y toggle
    private boolean lastDpadDown = false;
    private boolean lastInvertAimToggleBtn = false; // X toggle invertAim

    // --- 射球电机“预反转” ---
    private boolean shooterReversing = false;
    private final ElapsedTime shooterReverseTimer = new ElapsedTime();

    // --- Front PullUp “预反转”（按住 LB 时生效） ---
    private boolean pullupFrontReversing = false;
    private final ElapsedTime pullupFrontReverseTimer = new ElapsedTime();

    // --- D-pad Down 自动前后移动 ---
    private boolean autoMoveRunning = false;
    private int autoMoveStage = 0;              // 0=未运行, 1=前进, 2=后退
    private final ElapsedTime autoMoveTimer = new ElapsedTime();

    // ===== 自动瞄准滤波 + dt =====
    private double filtErr = 0.0;
    private final ElapsedTime aimDtTimer = new ElapsedTime();
    private final ElapsedTime deadbandHoldTimer = new ElapsedTime();
    private boolean inAimDeadband = false;

    // ===== 目标丢失保持（只给 UI 用）=====
    private boolean hasTargetTag = false;
    private boolean tagVisibleNow = false;
    private final ElapsedTime tagSeenTimer = new ElapsedTime();

    // ===== 对正帧数 =====
    private boolean alignedToTag = false;
    private int alignedFrames = 0;

    // ===== 扫描 timer =====
    private final ElapsedTime scanTimer = new ElapsedTime();

    // ===== slew 限速 + 输出记忆 =====
    private double lastRotCmd = 0.0;

    // ===== Husky 输出 =====
    private int targetErrorPx = 0;
    private int targetSeenId = -1;
    private int targetSeenX = -1;
    private int targetSeenAreaPx = 0;
    private double rawDistanceIn = -1.0;
    private double filteredDistanceIn = -1.0;
    private boolean hasDistanceEstimate = false;
    private double shooterCmdVel = 0.0;
    private final ElapsedTime distanceSeenTimer = new ElapsedTime();

    // ===== 红灯闪烁 timer（替代 sleep）=====
    private final ElapsedTime redBlinkTimer = new ElapsedTime();
    private boolean redBlinkOn = true;

    // ===== 新增：连续可见帧计数（用于消抖稳定可见）=====
    private int tagVisibleFrames = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ========== init 硬件 ==========
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "leftfront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightfront");
        leftBackMotor   = hardwareMap.get(DcMotor.class, "leftback");
        rightBackMotor  = hardwareMap.get(DcMotor.class, "rightback");

        shootMotorL = hardwareMap.get(DcMotorEx.class, "shootL");
        shootMotorR = hardwareMap.get(DcMotorEx.class, "shootR");

        intake = hardwareMap.get(DcMotor.class, "intake");

        pullUp_back   = hardwareMap.get(CRServo.class, "pullup_back");
        pullUp_frontL = hardwareMap.get(CRServo.class, "pullup_frontL");
        pullUp_frontR = hardwareMap.get(CRServo.class, "pullup_frontR");
        pullUp_mid    = hardwareMap.get(CRServo.class, "pullup_mid");
        cam_rotate    = hardwareMap.get(Servo.class, "cam_rotate");

        statusLed = hardwareMap.get(Servo.class, "statusLed");
        statusLed.setPosition(DriveTuning.LED_RED_POS);

        husky = hardwareMap.get(HuskyLens.class, "huskylens");
        safeKnock(husky);
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            batterySensor = vs;
            if (vs != null) break;
        }

        // Directions
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        shootMotorL.setDirection(DcMotor.Direction.REVERSE);
        shootMotorR.setDirection(DcMotor.Direction.FORWARD);

        pullUp_back.setDirection(CRServo.Direction.REVERSE);
        pullUp_frontR.setDirection(CRServo.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootMotorL.setVelocity(0);
        shootMotorR.setVelocity(0);
        intake.setPower(0);

        pullUp_back.setPower(0);
        pullUp_frontL.setPower(0);
        pullUp_frontR.setPower(0);
        pullUp_mid.setPower(0);

        cam_rotate.setPosition(0.5);

        // IMU 初始化（USB 在上方，Logo 朝左）
        initImu();

        // timers
        shooterReverseTimer.reset();
        pullupFrontReverseTimer.reset();
        autoMoveTimer.reset();
        aimDtTimer.reset();
        deadbandHoldTimer.reset();
        scanTimer.reset();
        tagSeenTimer.reset();
        distanceSeenTimer.reset();
        redBlinkTimer.reset();

        telemetry.addLine("Init done. Dashboard -> Config -> DriveTuning to live-tune.");
        telemetry.addData("IMU", imuOk ? "OK" : "NO");
        telemetry.update();

        waitForStart();

        aimDtTimer.reset();
        deadbandHoldTimer.reset();
        scanTimer.reset();
        tagSeenTimer.reset();
        distanceSeenTimer.reset();
        redBlinkTimer.reset();

        while (opModeIsActive()) {

            // =========================
            // 更新 HuskyLens 状态（含连续帧消抖）
            // =========================
            updateTagStatus();

            // =========================
            // LED：没目标=紫；Shooter 没开=红闪；开了且对正稳定=绿；否则蓝
            // =========================
            boolean alignedStable = alignedFrames >= DriveTuning.alignedNeedFrames;
            if (!hasTargetTag) statusLed.setPosition(DriveTuning.LED_PURPLE_POS);
            else if (!shootMotorStatus) ledRedBlinkNoSleep();
            else statusLed.setPosition(alignedStable ? DriveTuning.LED_GREEN_POS : DriveTuning.LED_BLUE_POS);

            // =========================
            // DRIVE 输入
            // =========================
            double forward = -gamepad1.right_stick_y;
            double pan     =  gamepad1.right_stick_x;
            double rotate  =  gamepad1.left_stick_x;

            double lf = forward + pan + rotate;
            double rf = forward - pan - rotate;
            double lb = forward - pan + rotate;
            double rb = forward + pan - rotate;

            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lb), Math.abs(rb)));
            if (max > 1.0) {
                lf /= max; rf /= max; lb /= max; rb /= max;
            }

            // =========================
            // D-pad Down 自动前后移动
            // =========================
            boolean dpadDownPressed = gamepad1.dpad_down && !lastDpadDown;
            lastDpadDown = gamepad1.dpad_down;

            if (dpadDownPressed && !autoMoveRunning) {
                autoMoveRunning = true;
                autoMoveStage = 1;
                autoMoveTimer.reset();
            }

            if (autoMoveRunning) {
                double p = 1.0;
                double cmd = 0.0;

                if (autoMoveStage == 1) {
                    if (autoMoveTimer.seconds() < 0.5) cmd = p;
                    else { autoMoveStage = 2; autoMoveTimer.reset(); cmd = 0.0; }
                } else if (autoMoveStage == 2) {
                    if (autoMoveTimer.seconds() < 0.5) cmd = -p;
                    else { autoMoveRunning = false; autoMoveStage = 0; cmd = 0.0; }
                }

                lf = cmd; rf = cmd; lb = cmd; rb = cmd;
            }

            // X：现场快速切换自动瞄准方向（防止方向配反）
            boolean invertAimTogglePressed = gamepad1.x && !lastInvertAimToggleBtn;
            lastInvertAimToggleBtn = gamepad1.x;
            if (invertAimTogglePressed) {
                DriveTuning.invertAim = !DriveTuning.invertAim;
            }

            // =========================
            // RB：按住自动对准（原地旋转） + IMU 阻尼
            // =========================
            boolean rbAimHeld = gamepad1.right_bumper;

            double yawDeg = 0.0;
            double yawRate = 0.0;
            if (imuOk) {
                AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                yawRate = av.zRotationRate;
                YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
                yawDeg = ypr.getYaw(AngleUnit.DEGREES);
            }

            if (rbAimHeld && !autoMoveRunning) {
                double dt = aimDtTimer.seconds();
                if (dt <= 0.0 || dt > 0.1) dt = 0.02;
                aimDtTimer.reset();

                boolean stableVisible = (tagVisibleFrames >= Math.max(1, DriveTuning.aimAcquireFrames));
                boolean justAcquired = (tagVisibleFrames == Math.max(1, DriveTuning.aimAcquireFrames));

                if (justAcquired) {
                    filtErr = targetErrorPx;
                    inAimDeadband = false;
                    deadbandHoldTimer.reset();
                }

                double rotCmd;
                double rawRotCmd;

                if (stableVisible) {
                    filtErr = DriveTuning.errAlpha * targetErrorPx + (1.0 - DriveTuning.errAlpha) * filtErr;
                    double absErr = Math.abs(filtErr);
                    int releasePx = Math.max(DriveTuning.aimDeadbandPx + 1, DriveTuning.aimReleasePx);

                    if (!inAimDeadband && absErr <= DriveTuning.aimDeadbandPx) {
                        inAimDeadband = true;
                        deadbandHoldTimer.reset();
                    } else if (inAimDeadband && absErr >= releasePx) {
                        inAimDeadband = false;
                    }

                    double targetYawRate = clamp(
                            DriveTuning.aimPxToYawRateKp * filtErr,
                            -DriveTuning.maxTrackYawRate,
                            DriveTuning.maxTrackYawRate
                    );

                    if (DriveTuning.invertAim) targetYawRate = -targetYawRate;

                    if (inAimDeadband) {
                        if (deadbandHoldTimer.seconds() >= DriveTuning.deadbandHoldSec) {
                            targetYawRate = 0.0;
                        } else {
                            targetYawRate *= 0.25;
                        }
                    } else if (absErr < releasePx) {
                        double span = Math.max(1.0, releasePx - DriveTuning.aimDeadbandPx);
                        double fade = clamp((absErr - DriveTuning.aimDeadbandPx) / span, 0.0, 1.0);
                        targetYawRate *= fade;
                    }

                    if (imuOk) {
                        double yawRateErr = targetYawRate - yawRate;
                        rawRotCmd = DriveTuning.trackYawRateKp * yawRateErr;
                    } else {
                        rawRotCmd = targetYawRate / Math.max(1.0, DriveTuning.maxTrackYawRate) * DriveTuning.aimMaxCmd;
                    }

                    rawRotCmd = clamp(rawRotCmd, -DriveTuning.aimMaxCmd, DriveTuning.aimMaxCmd);
                    if (!inAimDeadband && absErr > releasePx && Math.abs(rawRotCmd) < DriveTuning.aimMinCmd) {
                        rawRotCmd = Math.copySign(DriveTuning.aimMinCmd, rawRotCmd == 0.0 ? filtErr : rawRotCmd);
                    }

                } else {
                    if (tagVisibleNow) {
                        rawRotCmd = 0.0;
                    } else {
                        double dir = (Math.floor(scanTimer.seconds() / DriveTuning.scanPeriod) % 2 == 0) ? 1.0 : -1.0;
                        if (imuOk) {
                            double targetYawRate = dir * DriveTuning.searchYawRateCmd;
                            double errRate = targetYawRate - yawRate;
                            rawRotCmd = DriveTuning.searchYawRateKp * errRate;
                        } else {
                            rawRotCmd = dir * DriveTuning.searchCmdMax;
                        }
                        rawRotCmd = clamp(rawRotCmd, -DriveTuning.searchCmdMax, DriveTuning.searchCmdMax);
                    }
                }

                rotCmd = DriveTuning.rotCmdAlpha * rawRotCmd + (1.0 - DriveTuning.rotCmdAlpha) * lastRotCmd;

                // slew 限速
                double maxDelta = DriveTuning.rotSlewPerSec * dt;
                rotCmd = clamp(rotCmd, lastRotCmd - maxDelta, lastRotCmd + maxDelta);
                lastRotCmd = rotCmd;

                // 只旋转
                lf = rotCmd; rf = -rotCmd; lb = rotCmd; rb = -rotCmd;

            } else {
                inAimDeadband = false;
                deadbandHoldTimer.reset();
                filtErr *= 0.8;
                lastRotCmd *= 0.6;
            }

            // 写入底盘
            leftFrontMotor.setPower(lf);
            rightFrontMotor.setPower(rf);
            leftBackMotor.setPower(lb);
            rightBackMotor.setPower(rb);

            // =========================
            // SHOOTER：toggle = Y
            // =========================
            boolean shootPressed = gamepad1.y && !lastShooterToggleBtn;
            lastShooterToggleBtn = gamepad1.y;

            if (shootPressed) {
                boolean prev = shootMotorStatus;
                shootMotorStatus = !shootMotorStatus;

                if (!prev && shootMotorStatus) {
                    shooterReversing = true;
                    shooterReverseTimer.reset();
                } else if (!shootMotorStatus) {
                    shooterReversing = false;
                }
            }

            if (shootMotorStatus) {
                if (shooterReversing && shooterReverseTimer.seconds() < DriveTuning.shooterReverseTime) {
                    shootMotorL.setVelocity(-DriveTuning.shooterReverseVel);
                    shootMotorR.setVelocity(-DriveTuning.shooterReverseVel);
                } else {
                    shooterReversing = false;
                    shooterCmdVel = computeShooterVelocityTarget();
                    shootMotorL.setVelocity(shooterCmdVel);
                    shootMotorR.setVelocity(shooterCmdVel);
                }
            } else {
                shooterReversing = false;
                shooterCmdVel = 0.0;
                shootMotorL.setVelocity(0);
                shootMotorR.setVelocity(0);
            }

            // =========================
            // INTAKE（D-pad UP）
            // =========================
            boolean intakePressed = gamepad1.dpad_up && !lastIntakeButton;
            lastIntakeButton = gamepad1.dpad_up;

            if (intakePressed) IntakeMotorStatus = !IntakeMotorStatus;
            intake.setPower(IntakeMotorStatus ? 1.0 : 0.0);

            double pullPowerBackMid = 0.4;

            // =========================
            // FRONT PULLUP（LB）
            // =========================
            boolean lbHeld = gamepad1.left_bumper;

            double pullPowerFront = 0.7;
            double frontReverseTime = 0.15;

            if (lbHeld && !shootMotorStatus) {
                if (!pullupFrontReversing) {
                    pullupFrontReversing = true;
                    pullupFrontReverseTimer.reset();
                }
            } else {
                pullupFrontReversing = false;
            }

            double frontOutput;
            if (lbHeld) {
                if (pullupFrontReversing && pullupFrontReverseTimer.seconds() < frontReverseTime) {
                    frontOutput = -pullPowerFront;
                } else {
                    pullupFrontReversing = false;
                    frontOutput = pullPowerFront;
                }
            } else {
                frontOutput = 0.0;
            }

            pullUp_frontL.setPower(frontOutput);
            pullUp_frontR.setPower(frontOutput);

            if (shootMotorStatus && pullUp_frontL.getPower() > 0) {
                pullUp_mid.setPower(0.3);
                pullUp_back.setPower(0.3);
            } else {
                pullUp_back.setPower(IntakeMotorStatus ? pullPowerBackMid : 0.0);
                pullUp_mid.setPower(0.0);
            }

            // =========================
            // TELEMETRY
            // =========================
            telemetry.addData("Shoot Motor", shootMotorStatus ? "ON" : "OFF");
            telemetry.addData("ConfigClass", "DriveTuning");
            telemetry.addData("IMU", imuOk ? "OK" : "NO");
            telemetry.addData("Yaw(deg)", String.format("%.1f", yawDeg));
            telemetry.addData("YawRate(deg/s)", String.format("%.1f", yawRate));

            telemetry.addData("targetTagId", DriveTuning.targetTagId);
            telemetry.addData("HUSKY_CENTER_X", DriveTuning.HUSKY_CENTER_X);
            telemetry.addData("invertAim", DriveTuning.invertAim);
            telemetry.addData("strictIdFilter", DriveTuning.strictIdFilter);
            telemetry.addData("allowUnknownIdFallback", DriveTuning.allowUnknownIdFallback);

            telemetry.addData("tagVisibleNow", tagVisibleNow);
            telemetry.addData("tagVisibleFrames", tagVisibleFrames);
            telemetry.addData("stableVisibleNeed", DriveTuning.aimAcquireFrames);

            telemetry.addData("alignedFrames", alignedFrames);
            telemetry.addData("aimDeadbandPx", DriveTuning.aimDeadbandPx);

            telemetry.addData("TagSeenId", targetSeenId);
            telemetry.addData("TagX", targetSeenX);
            telemetry.addData("TagAreaPx", targetSeenAreaPx);
            telemetry.addData("ErrPx", targetErrorPx);
            telemetry.addData("FiltErr", String.format("%.1f", filtErr));
            telemetry.addData("InDeadband", inAimDeadband);
            telemetry.addData("rotCmd", String.format("%.2f", lastRotCmd));
            telemetry.addData("BattV", String.format("%.2f", getBatteryVoltage()));
            telemetry.addData("DistRaw(in)", String.format("%.1f", rawDistanceIn));
            telemetry.addData("DistFilt(in)", String.format("%.1f", filteredDistanceIn));
            telemetry.addData("ShooterCmd", String.format("%.0f", shooterCmdVel));

            telemetry.addData("Intake", IntakeMotorStatus ? "ON" : "OFF");
            telemetry.update();

            idle();
        }
    }

    // =========================
    // IMU 初始化（USB 上，Logo 左）
    // =========================
    private void initImu() {
        imuOk = false;
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imuOk = true;
        } catch (Exception e1) {
            try {
                imu = hardwareMap.get(IMU.class, "IMU");
                imuOk = true;
            } catch (Exception e2) {
                imuOk = false;
            }
        }
        if (!imuOk) return;

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
    }

    /**
     * 更新 HuskyLens 状态（含连续帧消抖 + 对正帧统计）
     */
    private void updateTagStatus() {
        tagVisibleNow = false;
        alignedToTag = false;
        targetSeenId = -1;
        targetSeenX = -1;
        targetSeenAreaPx = 0;
        targetErrorPx = 0;

        HuskyLens.Block[] blocks = getBlocksSafe();
        HuskyLens.Block best = null;
        int bestArea = 0;
        HuskyLens.Block fallbackUnknownIdBest = null;
        int fallbackUnknownIdBestArea = 0;

        if (blocks != null) {
            for (HuskyLens.Block b : blocks) {
                int id = getBlockIdSafe(b);
                int area = b.width * b.height;

                if (DriveTuning.targetTagId != -1) {
                    if (DriveTuning.strictIdFilter) {
                        if (id == DriveTuning.targetTagId) {
                            if (area > bestArea) {
                                bestArea = area;
                                best = b;
                            }
                        } else if (DriveTuning.allowUnknownIdFallback && id == -1) {
                            if (area > fallbackUnknownIdBestArea) {
                                fallbackUnknownIdBestArea = area;
                                fallbackUnknownIdBest = b;
                            }
                        }
                        continue;
                    } else {
                        if (id != -1 && id != DriveTuning.targetTagId) continue;
                    }
                }

                if (area > bestArea) {
                    bestArea = area;
                    best = b;
                }
            }
        }

        if (best == null && DriveTuning.targetTagId != -1 && DriveTuning.strictIdFilter
                && DriveTuning.allowUnknownIdFallback) {
            best = fallbackUnknownIdBest;
        }

        if (best != null) {
            tagVisibleNow = true;
            hasTargetTag = true;
            tagSeenTimer.reset();

            targetSeenId = getBlockIdSafe(best);
            targetSeenX = best.x;
            targetSeenAreaPx = bestArea;

            targetErrorPx = best.x - DriveTuning.HUSKY_CENTER_X;
            alignedToTag = Math.abs(targetErrorPx) <= DriveTuning.aimDeadbandPx;

            rawDistanceIn = estimateDistanceInFromArea(bestArea);
            if (rawDistanceIn > 0.0) {
                if (!hasDistanceEstimate || filteredDistanceIn <= 0.0) filteredDistanceIn = rawDistanceIn;
                else filteredDistanceIn = DriveTuning.distAlpha * rawDistanceIn + (1.0 - DriveTuning.distAlpha) * filteredDistanceIn;
                hasDistanceEstimate = true;
                distanceSeenTimer.reset();
            }
        } else {
            double hold = Math.max(0.0, DriveTuning.tagHoldSeconds);
            if (hasTargetTag && tagSeenTimer.seconds() <= hold) hasTargetTag = true;
            else hasTargetTag = false;

            if (hasDistanceEstimate && distanceSeenTimer.seconds() > Math.max(0.0, DriveTuning.distHoldSec)) {
                hasDistanceEstimate = false;
                rawDistanceIn = -1.0;
            }
        }

        // 连续可见帧数（控制用）
        if (tagVisibleNow) tagVisibleFrames++;
        else tagVisibleFrames = 0;

        // 对正帧数（LED 用）
        if (alignedToTag) alignedFrames++;
        else alignedFrames = 0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double estimateDistanceInFromArea(int areaPx) {
        double refArea = Math.max(1.0, DriveTuning.distRefTagAreaPx);
        if (areaPx <= 0) return -1.0;
        double d = DriveTuning.shooterRefDistanceIn * Math.sqrt(refArea / Math.max(1.0, areaPx));
        return clamp(d, DriveTuning.distMinIn, DriveTuning.distMaxIn);
    }

    private double computeShooterVelocityTarget() {
        double target = DriveTuning.shooterTargetVel;
        if (DriveTuning.shooterUseDistance && hasDistanceEstimate && filteredDistanceIn > 0.0) {
            target = DriveTuning.shooterRefRpm
                    + DriveTuning.shooterRpmPerIn * (filteredDistanceIn - DriveTuning.shooterRefDistanceIn);
        }

        if (DriveTuning.shooterUseVoltageComp) {
            double v = getBatteryVoltage();
            if (v > 1.0) {
                double ratio = DriveTuning.shooterRefVoltage / v;
                double comp = 1.0 + DriveTuning.shooterVoltageCompK * (ratio - 1.0);
                target *= clamp(comp, 0.85, 1.20);
            }
        }

        return clamp(target, DriveTuning.shooterMinVel, DriveTuning.shooterMaxVel);
    }

    private double getBatteryVoltage() {
        if (batterySensor != null) {
            double v = batterySensor.getVoltage();
            if (v > 0.0) return v;
        }
        double best = 0.0;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            if (vs == null) continue;
            double v = vs.getVoltage();
            if (v > best) best = v;
        }
        return best;
    }

    // =========================
    // HuskyLens 兼容层
    // =========================
    private HuskyLens.Block[] getBlocksSafe() {
        String[] methods = new String[] {
                "blocks",
                "getBlocks",
                "readBlocks",
                "getBlocksList",
                "getBlocksArray"
        };

        for (String m : methods) {
            try {
                Method method = husky.getClass().getMethod(m);
                Object r = method.invoke(husky);
                if (r == null) continue;

                if (r instanceof HuskyLens.Block[]) {
                    return (HuskyLens.Block[]) r;
                }

                if (r instanceof List) {
                    List<?> list = (List<?>) r;
                    HuskyLens.Block[] arr = new HuskyLens.Block[list.size()];
                    for (int i = 0; i < list.size(); i++) {
                        arr[i] = (HuskyLens.Block) list.get(i);
                    }
                    return arr;
                }
            } catch (Exception ignored) {}
        }
        return null;
    }

    private int getBlockIdSafe(HuskyLens.Block b) {
        if (b == null) return -1;
        String[] candidates = new String[] {"id", "ID", "label", "learnedID", "classId", "classID"};
        for (String name : candidates) {
            try {
                Field f;
                try {
                    f = b.getClass().getField(name);
                } catch (NoSuchFieldException e) {
                    f = b.getClass().getDeclaredField(name);
                    f.setAccessible(true);
                }
                Object v = f.get(b);
                if (v instanceof Integer) return (Integer) v;
                if (v instanceof Short) return ((Short) v).intValue();
                if (v instanceof Byte) return ((Byte) v).intValue();
            } catch (Exception ignored) {}
        }
        return -1;
    }

    private void safeKnock(HuskyLens h) {
        try {
            Method m = h.getClass().getMethod("knock");
            m.invoke(h);
        } catch (Exception ignored) {}
    }

    private void ledRedBlinkNoSleep() {
        double period = Math.max(0.1, DriveTuning.redBlinkPeriodSec);
        if (redBlinkTimer.seconds() >= period) {
            redBlinkTimer.reset();
            redBlinkOn = !redBlinkOn;
        }
        statusLed.setPosition(redBlinkOn ? DriveTuning.LED_RED_POS : 0.0);
    }
}
