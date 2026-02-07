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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;

@TeleOp(name = "LHX_RED_LIN", group = "LinearOp Red")
public class LHX_RED_LIN extends LinearOpMode {

    @Config
    public static class DriveTuning {
        // --- AprilTag / Husky ---
        public static int targetTagId = 5;          // -1 不限定
        public static int HUSKY_CENTER_X = 140;     // 画面中心点（调它修“对正偏左/偏右”）
        public static boolean strictIdFilter = true;
        public static boolean invertAim = false;

        // --- AutoAim PD ---
        public static double aimKp = 0.0048;
        public static double aimKd = 0.0017;
        public static double aimMin = 0.01;         // 建议 0.00~0.03 更不抽
        public static double aimMax = 0.42;
        public static int aimDeadbandPx = 10;       // 建议 10~12

        // --- 误差滤波 ---
        public static double errAlpha = 0.35;

        // --- 连续帧消抖：连续看到 N 帧才进入控制（解决“突然动两下”）---
        public static int aimAcquireFrames = 3;     // 2~4

        // --- D 项限幅（防 derr 爆炸导致抽一下）---
        public static double derrLimit = 900.0;     // 600~1500

        // --- rotCmd 低通滤波：越小越顺、越不抽（0~1）---
        public static double rotCmdAlpha = 0.28;    // 0.25~0.55

        // --- 对正判定稳定帧数（给 LED 用）---
        public static int alignedNeedFrames = 5;

        // --- 扫描参数 ---
        public static double scanPower = 0.14;
        public static double scanPeriod = 1.2;

        // --- 输出限速/刹车 ---
        public static double rotSlewPerSec = 1.6;
        public static double flipBrakeSeconds = 0.08;

        // --- IMU 阻尼（防过冲抽搐）---
        public static double yawRateDampK = 0.06;  // 0.03~0.08
        public static double yawRateLimit = 180.0;  // 120~220
        public static double scanYawRateCmd = 60.0; // 40~90

        // --- LED 位置 ---
        public static double LED_RED_POS   = 0.28;
        public static double LED_GREEN_POS = 0.49;
        public static double LED_BLUE_POS  = 0.611;

        // --- Shooter 可调 ---
        public static double shooterTargetVel = 1000;
        public static double shooterReverseVel = 1000;
        public static double shooterReverseTime = 0.15;

        // --- 红灯闪烁 ---
        public static double redBlinkPeriodSec = 0.5;

        // --- “看到目标 UI hold”（不参与控制）---
        public static double tagHoldSeconds = 0.25;

        // --- 非线性 soften 的分母（越小越敏感/越容易过冲；越大越柔）---
        public static double softenDiv = 120;      // 60~120
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

    // --- IMU ---
    private IMU imu;
    private boolean imuOk = false;

    // ---------------- 状态 ----------------
    private boolean shootMotorStatus = false;
    private boolean IntakeMotorStatus = false;

    private boolean lastIntakeButton = false;      // D-pad Up
    private boolean lastShooterToggleBtn = false;  // Y toggle
    private boolean lastDpadDown = false;

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

    // ===== 误差滤波 + dt =====
    private double filtErr = 0.0;
    private double lastFiltErr = 0.0;
    private final ElapsedTime aimDtTimer = new ElapsedTime();

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

    // ===== 过冲抑制 =====
    private int lastErrSign = 0;
    private final ElapsedTime signFlipTimer = new ElapsedTime();

    // ===== Husky 输出 =====
    private int targetErrorPx = 0;
    private int targetSeenId = -1;
    private int targetSeenX = -1;

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
        scanTimer.reset();
        tagSeenTimer.reset();
        signFlipTimer.reset();
        redBlinkTimer.reset();

        telemetry.addLine("Init done. Dashboard -> Config -> DriveTuning to live-tune.");
        telemetry.addData("IMU", imuOk ? "OK" : "NO");
        telemetry.update();

        waitForStart();

        aimDtTimer.reset();
        scanTimer.reset();
        tagSeenTimer.reset();
        signFlipTimer.reset();
        redBlinkTimer.reset();

        while (opModeIsActive()) {

            // =========================
            // 更新 HuskyLens 状态（含连续帧消抖）
            // =========================
            updateTagStatus();

            // =========================
            // LED：Shooter 没开=红闪；开了且对正稳定=绿；否则蓝
            // =========================
            boolean alignedStable = alignedFrames >= DriveTuning.alignedNeedFrames;
            if (!shootMotorStatus) ledRedBlinkNoSleep();
            else statusLed.setPosition(alignedStable ? DriveTuning.LED_GREEN_POS : DriveTuning.LED_BLUE_POS);

            // =========================
            // DRIVE 输入
            // =========================
            double forward = -gamepad1.right_stick_y;
            double pan     =  gamepad1.right_stick_x;
            double rotate  = -gamepad1.left_stick_x;

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

                // 连续看到 N 帧才允许进入控制（核心修复）
                boolean stableVisible = (tagVisibleFrames >= Math.max(1, DriveTuning.aimAcquireFrames));
                boolean justAcquired = stableVisible && (tagVisibleFrames == Math.max(1, DriveTuning.aimAcquireFrames));

                // 重新获得目标：清掉 D 项/输出记忆，避免“突然抽一下”
                if (justAcquired) {
                    filtErr = 0.0;
                    lastFiltErr = 0.0;
                    lastRotCmd = 0.0;
                    lastErrSign = 0;
                    signFlipTimer.reset();
                }

                double rotCmd;

                if (stableVisible) {
                    // 误差滤波
                    filtErr = DriveTuning.errAlpha * targetErrorPx + (1.0 - DriveTuning.errAlpha) * filtErr;

                    // D 项（限幅）
                    double derr = (filtErr - lastFiltErr) / dt;
                    lastFiltErr = filtErr;
                    derr = clamp(derr, -DriveTuning.derrLimit, DriveTuning.derrLimit);

                    // PD
                    rotCmd = DriveTuning.aimKp * filtErr + DriveTuning.aimKd * derr;
                    if (DriveTuning.invertAim) rotCmd = -rotCmd;

                    // 非线性 soften
                    double div = Math.max(20.0, DriveTuning.softenDiv);
                    double norm = Math.min(1.0, Math.abs(filtErr) / div);
                    double soften = norm * norm;
                    rotCmd *= soften;

                    // 过冲抑制：误差符号翻转短暂刹车
                    int sign = Integer.compare(targetErrorPx, 0);
                    if (sign != 0 && lastErrSign != 0 && sign != lastErrSign) signFlipTimer.reset();
                    lastErrSign = sign;
                    if (signFlipTimer.seconds() < DriveTuning.flipBrakeSeconds) rotCmd *= 0.2;

                    // deadband：进死区就 0，同时让记忆回落（更不抖）
                    if (Math.abs(targetErrorPx) <= DriveTuning.aimDeadbandPx) {
                        rotCmd = 0.0;
                        lastRotCmd *= 0.6;
                    } else {
                        // aimMin 只在大误差时用
                        if (Math.abs(targetErrorPx) > (DriveTuning.aimDeadbandPx + 6)) {
                            if (rotCmd > 0) rotCmd = Math.max(rotCmd, DriveTuning.aimMin);
                            else            rotCmd = Math.min(rotCmd, -DriveTuning.aimMin);
                        }
                    }

                    rotCmd = clamp(rotCmd, -DriveTuning.aimMax, DriveTuning.aimMax);

                    // IMU 阻尼：抑制过冲
                    if (imuOk) {
                        rotCmd -= DriveTuning.yawRateDampK * yawRate;
                        if (Math.abs(yawRate) > DriveTuning.yawRateLimit) rotCmd *= 0.35;
                    }

                    // rotCmd 低通（比单纯 slew 更能消“抽两下”）
                    rotCmd = DriveTuning.rotCmdAlpha * rotCmd + (1.0 - DriveTuning.rotCmdAlpha) * lastRotCmd;

                } else {
                    // 不稳定可见（看到了但没到 N 帧）：停住等待稳定，避免“抽一下”
                    if (tagVisibleNow) {
                        lastRotCmd *= 0.7;
                        rotCmd = 0.0;
                    } else {
                        // 真没看到：扫描
                        filtErr *= 0.85;
                        lastFiltErr = filtErr;
                        lastRotCmd *= 0.85;

                        double dir = (Math.floor(scanTimer.seconds() / DriveTuning.scanPeriod) % 2 == 0) ? 1.0 : -1.0;

                        if (imuOk) {
                            double desired = dir * DriveTuning.scanYawRateCmd;
                            double errRate = desired - yawRate;
                            rotCmd = clamp(0.0045 * errRate, -DriveTuning.scanPower, DriveTuning.scanPower);
                        } else {
                            rotCmd = dir * DriveTuning.scanPower;
                        }

                        // 扫描也做一次低通，避免扫的方向切换时“抽”
                        rotCmd = DriveTuning.rotCmdAlpha * rotCmd + (1.0 - DriveTuning.rotCmdAlpha) * lastRotCmd;
                    }
                }

                // slew 限速
                double maxDelta = DriveTuning.rotSlewPerSec * dt;
                rotCmd = clamp(rotCmd, lastRotCmd - maxDelta, lastRotCmd + maxDelta);
                lastRotCmd = rotCmd;

                // 只旋转
                lf = rotCmd; rf = -rotCmd; lb = rotCmd; rb = -rotCmd;

            } else {
                lastRotCmd *= 0.8;
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
                    shootMotorL.setVelocity(DriveTuning.shooterTargetVel);
                    shootMotorR.setVelocity(DriveTuning.shooterTargetVel);
                }
            } else {
                shooterReversing = false;
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

            telemetry.addData("tagVisibleNow", tagVisibleNow);
            telemetry.addData("tagVisibleFrames", tagVisibleFrames);
            telemetry.addData("stableVisibleNeed", DriveTuning.aimAcquireFrames);

            telemetry.addData("alignedFrames", alignedFrames);
            telemetry.addData("aimDeadbandPx", DriveTuning.aimDeadbandPx);

            telemetry.addData("TagSeenId", targetSeenId);
            telemetry.addData("TagX", targetSeenX);
            telemetry.addData("ErrPx", targetErrorPx);
            telemetry.addData("FiltErr", String.format("%.1f", filtErr));
            telemetry.addData("rotCmd", String.format("%.2f", lastRotCmd));

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
        targetErrorPx = 0;

        HuskyLens.Block[] blocks = getBlocksSafe();
        HuskyLens.Block best = null;
        int bestArea = 0;

        if (blocks != null) {
            for (HuskyLens.Block b : blocks) {
                int id = getBlockIdSafe(b);

                if (DriveTuning.targetTagId != -1) {
                    if (DriveTuning.strictIdFilter) {
                        if (id != DriveTuning.targetTagId) continue;
                    } else {
                        if (id != -1 && id != DriveTuning.targetTagId) continue;
                    }
                }

                int area = b.width * b.height;
                if (area > bestArea) {
                    bestArea = area;
                    best = b;
                }
            }
        }

        if (best != null) {
            tagVisibleNow = true;
            hasTargetTag = true;
            tagSeenTimer.reset();

            targetSeenId = getBlockIdSafe(best);
            targetSeenX = best.x;

            targetErrorPx = best.x - DriveTuning.HUSKY_CENTER_X;
            alignedToTag = Math.abs(targetErrorPx) <= DriveTuning.aimDeadbandPx;
        } else {
            double hold = Math.max(0.0, DriveTuning.tagHoldSeconds);
            if (hasTargetTag && tagSeenTimer.seconds() <= hold) hasTargetTag = true;
            else hasTargetTag = false;
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