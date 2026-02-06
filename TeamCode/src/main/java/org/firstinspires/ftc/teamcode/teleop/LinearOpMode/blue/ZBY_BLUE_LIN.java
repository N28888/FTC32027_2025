package org.firstinspires.ftc.teamcode.teleop.LinearOpMode.blue;

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

@TeleOp(name = "ZBY_BLUE_LIN", group = "TeleOp6")
public class ZBY_BLUE_LIN extends LinearOpMode {

    @Config
    public static class DriveTuning {
        // --- AprilTag / Husky ---
        public static int targetTagId = 5;          // -1 不限定
        public static int HUSKY_CENTER_X = 140;     // 画面中心点（你之前就是调这个让对准偏不偏）
        public static boolean strictIdFilter = true;
        public static boolean invertAim = false;

        // --- AutoAim PD ---
        public static double aimKp = 0.006;
        public static double aimKd = 0.0025;
        public static double aimMin = 0.05;
        public static double aimMax = 0.55;
        public static int aimDeadbandPx = 16;

        // --- 误差滤波 ---
        public static double errAlpha = 0.35;

        // --- 对正判定稳定帧数 ---
        public static int alignedNeedFrames = 5;

        // --- 扫描参数 ---
        public static double scanPower = 0.14;
        public static double scanPeriod = 1.2;

        // --- 输出限速/刹车 ---
        public static double rotSlewPerSec = 1.6;
        public static double flipBrakeSeconds = 0.08;

        // --- IMU 阻尼（防过冲抽搐）---
        public static double yawRateDampK = 0.045;  // 0.03~0.08
        public static double yawRateLimit = 180.0;  // 120~220
        public static double scanYawRateCmd = 60.0; // 40~90

        // --- LED 位置（你也可以在线微调到最准的红/绿/蓝）---
        public static double LED_RED_POS   = 0.28;
        public static double LED_GREEN_POS = 0.49;
        public static double LED_BLUE_POS  = 0.611;

        // --- Shooter 目标速度也可以在线调（可选）---
        public static double shooterTargetVel = 1000;
        public static double shooterReverseVel = 1000;
        public static double shooterReverseTime = 0.15;
        public static double redBlinkPeriodSec = 0.5; // 0.5=每半秒切一次
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
    private ElapsedTime shooterReverseTimer = new ElapsedTime();

    // --- Front PullUp “预反转”（按住 LB 时生效） ---
    private boolean pullupFrontReversing = false;
    private ElapsedTime pullupFrontReverseTimer = new ElapsedTime();

    // --- D-pad Down 自动前后移动 ---
    private boolean autoMoveRunning = false;
    private int autoMoveStage = 0;              // 0=未运行, 1=前进, 2=后退
    private ElapsedTime autoMoveTimer = new ElapsedTime();

    // ===== 误差滤波 + dt =====
    private double filtErr = 0.0;
    private double lastFiltErr = 0.0;
    private ElapsedTime aimDtTimer = new ElapsedTime();

    // ===== 目标丢失保持（只给 UI 用）=====
    private boolean hasTargetTag = false;
    private boolean tagVisibleNow = false;
    private ElapsedTime tagSeenTimer = new ElapsedTime();
    private double tagHoldSeconds = 0.25;

    // ===== 对正帧数 =====
    private boolean alignedToTag = false;
    private int alignedFrames = 0;

    // ===== 扫描 timer =====
    private ElapsedTime scanTimer = new ElapsedTime();

    // ===== slew 限速 =====
    private double lastRotCmd = 0.0;

    // ===== 过冲抑制 =====
    private int lastErrSign = 0;
    private ElapsedTime signFlipTimer = new ElapsedTime();

    // ===== Husky 输出 =====
    private int targetErrorPx = 0;
    private int targetSeenId = -1;
    private int targetSeenX = -1;

    // ===== 红灯闪烁 timer（替代 sleep）=====
    private ElapsedTime redBlinkTimer = new ElapsedTime();
    private boolean redBlinkOn = true;

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

        telemetry.addLine("Init done. Open Dashboard -> Config -> DriveTuning to live-tune.");
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
            // 更新 HuskyLens 状态（使用 Dashboard 可改参数）
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
            double forward = -gamepad1.left_stick_y;
            double pan     =  gamepad1.left_stick_x;
            double rotate  =  gamepad1.right_stick_x;

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
            // RB：按住自动对准（原地旋转） + IMU 阻尼（全部可实时改）
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

                double rotCmd;

                if (tagVisibleNow) {
                    // 误差滤波
                    filtErr = DriveTuning.errAlpha * targetErrorPx + (1.0 - DriveTuning.errAlpha) * filtErr;

                    // D 项
                    double derr = (filtErr - lastFiltErr) / dt;
                    lastFiltErr = filtErr;

                    // PD
                    rotCmd = DriveTuning.aimKp * filtErr + DriveTuning.aimKd * derr;
                    if (DriveTuning.invertAim) rotCmd = -rotCmd;

                    // 非线性缩放：接近中心更柔
                    double norm = Math.min(1.0, Math.abs(filtErr) / 80.0);
                    double soften = norm * norm;
                    rotCmd *= soften;

                    // 过冲抑制：误差符号翻转短暂刹车
                    int sign = Integer.compare(targetErrorPx, 0);
                    if (sign != 0 && lastErrSign != 0 && sign != lastErrSign) {
                        signFlipTimer.reset();
                    }
                    lastErrSign = sign;

                    if (signFlipTimer.seconds() < DriveTuning.flipBrakeSeconds) {
                        rotCmd *= 0.2;
                    }

                    // 接近中心不强制 aimMin
                    if (Math.abs(targetErrorPx) > (DriveTuning.aimDeadbandPx + 6)) {
                        if (rotCmd > 0) rotCmd = Math.max(rotCmd, DriveTuning.aimMin);
                        else            rotCmd = Math.min(rotCmd, -DriveTuning.aimMin);
                    } else {
                        if (Math.abs(targetErrorPx) <= DriveTuning.aimDeadbandPx) rotCmd = 0.0;
                    }

                    rotCmd = clamp(rotCmd, -DriveTuning.aimMax, DriveTuning.aimMax);

                    // IMU 阻尼：抑制过冲
                    if (imuOk) {
                        rotCmd -= DriveTuning.yawRateDampK * yawRate;
                        if (Math.abs(yawRate) > DriveTuning.yawRateLimit) rotCmd *= 0.35;
                    }

                } else {
                    // 没看到 tag：清状态 + 稳定扫描
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
            // SHOOTER：toggle = Y（速度也可实时改）
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
            // TELEMETRY（Dashboard + DS 都能看）
            // =========================
            telemetry.addData("Shoot Motor", shootMotorStatus ? "ON" : "OFF");
            telemetry.addData("ConfigClass", "DriveTuning (Dashboard->Config)");
            telemetry.addData("IMU", imuOk ? "OK" : "NO");
            telemetry.addData("Yaw(deg)", String.format("%.1f", yawDeg));
            telemetry.addData("YawRate(deg/s)", String.format("%.1f", yawRate));

            telemetry.addData("targetTagId", DriveTuning.targetTagId);
            telemetry.addData("HUSKY_CENTER_X", DriveTuning.HUSKY_CENTER_X);
            telemetry.addData("tagVisibleNow", tagVisibleNow);
            telemetry.addData("alignedFrames", alignedFrames);
            telemetry.addData("aimDeadbandPx", DriveTuning.aimDeadbandPx);

            telemetry.addData("TagSeenId", targetSeenId);
            telemetry.addData("TagX", targetSeenX);
            telemetry.addData("ErrPx", targetErrorPx);
            telemetry.addData("FiltErr", String.format("%.1f", filtErr));
            telemetry.addData("rotCmd", String.format("%.2f", lastRotCmd));

            telemetry.addData("Shooter", shootMotorStatus ? "ON" : "OFF");
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
     * 更新 HuskyLens 状态（使用 Dashboard 可改参数）
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
            if (hasTargetTag && tagSeenTimer.seconds() <= tagHoldSeconds) hasTargetTag = true;
            else hasTargetTag = false;
        }

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