package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

import java.util.List;

@TeleOp(name = "Blue", group = "LinearOp Blue")
public class Blue extends LinearOpMode {

    @Config
    public static class DriveTuning {
        // --- AprilTag / Limelight --
        public static int targetTagId = 1;          // -1 不限定
        public static int limelightPipeline = 0;
        public static double LIMELIGHT_CENTER_TX = 0.0; // deg，调它修“对正偏左/偏右”
        public static boolean strictIdFilter = true;
        public static boolean invertAim = false;

        // --- AutoAim：Limelight tx 生成目标 yaw，IMU yaw 闭环追踪 ---
        public static double aimKp = 0.028;
        public static double aimKd = 0.0025;        // IMU yaw-rate damping
        public static double aimMin = 0.06;         // 克服静摩擦；太冲就降到 0.04
        public static double aimMax = 0.35;
        public static double aimDeadbandDeg = 1.0;  // 建议 0.8~1.5
        public static double aimSlowdownDeg = 8.0;  // 进入此范围内逐渐减速
        public static double aimLostHoldSeconds = 0.25;

        // --- 误差滤波 ---
        public static double errAlpha = 0.35;

        // --- 连续帧消抖：连续看到 N 帧才进入控制（解决“突然动两下”）---
        public static int aimAcquireFrames = 3;     // 2~4

        // --- rotCmd 低通滤波：越小越顺、越不抽（0~1）---
        public static double rotCmdAlpha = 0.40;    // 0.25~0.55

        // --- 对正判定稳定帧数（给 LED 用）---
        public static int alignedNeedFrames = 5;

        // --- 扫描参数 ---
        public static double scanPower = 0.10;
        public static double scanPeriod = 1.0;

        // --- 输出限速 ---
        public static double rotSlewPerSec = 2.4;

        public static double scanYawRateCmd = 45.0; // 35~70

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

    // --- Limelight 3A ---
    private Limelight3A limelight;
    private int activeLimelightPipeline = -1;

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

    // ===== Limelight 输出 =====
    private boolean limelightResultValid = false;
    private double targetErrorDeg = 0.0;
    private int targetSeenId = -1;
    private double targetSeenTxDeg = 0.0;
    private double lastSeenErrorDeg = 0.0;
    private boolean aimLockValid = false;
    private double targetHeadingDeg = 0.0;
    private double headingErrorDeg = 0.0;

    // ===== 红灯闪烁 timer（替代 sleep）=====
    private final ElapsedTime redBlinkTimer = new ElapsedTime();
    private boolean redBlinkOn = true;

    // ===== 新增：连续可见帧计数（用于消抖稳定可见）=====
    private int tagVisibleFrames = 0;
    private boolean aimActiveLastLoop = false;
    private int searchDirection = 1;
    private String aimMode = "Idle";

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(20);
        applyLimelightPipeline();
        limelight.start();

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
        redBlinkTimer.reset();

        telemetry.addLine("Init done. Dashboard -> Config -> DriveTuning to live-tune.");
        telemetry.addData("IMU", imuOk ? "OK" : "NO");
        telemetry.update();

        waitForStart();

        aimDtTimer.reset();
        scanTimer.reset();
        tagSeenTimer.reset();
        redBlinkTimer.reset();

        while (opModeIsActive()) {

            // =========================
            // 更新 Limelight 状态（含连续帧消抖）
            // =========================
            applyLimelightPipeline();
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

            boolean aimActive = rbAimHeld && !autoMoveRunning;
            if (aimActive) {
                double dt = aimDtTimer.seconds();
                if (dt <= 0.0 || dt > 0.1) dt = 0.02;
                aimDtTimer.reset();

                if (!aimActiveLastLoop) {
                    resetAimController();
                    aimDtTimer.reset();
                    scanTimer.reset();
                    dt = 0.02;
                }

                double rotCmd;
                boolean stableVisible = tagVisibleFrames >= Math.max(1, DriveTuning.aimAcquireFrames);

                if (stableVisible) {
                    aimMode = "Track";
                    updateAimLock(yawDeg, targetErrorDeg);
                    rotCmd = calculateHeadingCommand(yawDeg, yawRate);
                } else {
                    filtErr = 0.0;
                    lastFiltErr = 0.0;

                    if (tagVisibleNow) {
                        aimMode = "Acquire";
                        lastRotCmd = 0.0;
                        rotCmd = 0.0;
                    } else if (aimLockValid && tagSeenTimer.seconds() <= DriveTuning.aimLostHoldSeconds) {
                        aimMode = "Hold";
                        rotCmd = calculateHeadingCommand(yawDeg, yawRate);
                    } else {
                        aimLockValid = false;
                        aimMode = "Search";
                        rotCmd = calculateSearchCommand(yawRate);
                    }
                }

                rotCmd = DriveTuning.rotCmdAlpha * rotCmd + (1.0 - DriveTuning.rotCmdAlpha) * lastRotCmd;
                rotCmd = slew(rotCmd, lastRotCmd, dt);
                lastRotCmd = rotCmd;

                // 只旋转
                lf = rotCmd; rf = -rotCmd; lb = rotCmd; rb = -rotCmd;

            } else {
                if (aimActiveLastLoop) resetAimController();
                aimMode = "Idle";
            }
            aimActiveLastLoop = aimActive;

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
            telemetry.addData("limelightPipeline", DriveTuning.limelightPipeline);
            telemetry.addData("limelightValid", limelightResultValid);
            telemetry.addData("LIMELIGHT_CENTER_TX", String.format("%.2f", DriveTuning.LIMELIGHT_CENTER_TX));
            telemetry.addData("AimMode", aimMode);
            telemetry.addData("SearchDir", searchDirection);

            telemetry.addData("tagVisibleNow", tagVisibleNow);
            telemetry.addData("tagVisibleFrames", tagVisibleFrames);
            telemetry.addData("stableVisibleNeed", DriveTuning.aimAcquireFrames);

            telemetry.addData("alignedFrames", alignedFrames);
            telemetry.addData("aimDeadbandDeg", String.format("%.2f", DriveTuning.aimDeadbandDeg));

            telemetry.addData("TagSeenId", targetSeenId);
            telemetry.addData("TagTX(deg)", String.format("%.2f", targetSeenTxDeg));
            telemetry.addData("ErrDeg", String.format("%.2f", targetErrorDeg));
            telemetry.addData("LastSeenErrDeg", String.format("%.2f", lastSeenErrorDeg));
            telemetry.addData("TargetHeading", String.format("%.2f", targetHeadingDeg));
            telemetry.addData("HeadingErr", String.format("%.2f", headingErrorDeg));
            telemetry.addData("FiltErrDeg", String.format("%.2f", filtErr));
            telemetry.addData("rotCmd", String.format("%.2f", lastRotCmd));

            telemetry.addData("Intake", IntakeMotorStatus ? "ON" : "OFF");
            telemetry.update();

            idle();
        }

        limelight.stop();
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

    private void resetAimController() {
        filtErr = 0.0;
        lastFiltErr = 0.0;
        lastRotCmd = 0.0;
        headingErrorDeg = 0.0;
        aimLockValid = false;
    }

    private void updateAimLock(double yawDeg, double txErrorDeg) {
        // Limelight tx 是目标相对镜头中心的角度；把它转成机器人应该面对的 yaw。
        double txToYaw = DriveTuning.invertAim ? -txErrorDeg : txErrorDeg;
        targetHeadingDeg = wrapDeg(yawDeg + txToYaw);
        aimLockValid = true;
    }

    private double calculateHeadingCommand(double yawDeg, double yawRate) {
        if (!aimLockValid) return 0.0;

        headingErrorDeg = wrapDeg(targetHeadingDeg - yawDeg);
        double alpha = clamp(DriveTuning.errAlpha, 0.0, 1.0);
        filtErr = alpha * headingErrorDeg + (1.0 - alpha) * filtErr;

        double deadband = Math.max(0.0, DriveTuning.aimDeadbandDeg);
        double absErr = Math.abs(filtErr);
        if (absErr <= deadband) {
            lastFiltErr = filtErr;
            return 0.0;
        }

        lastFiltErr = filtErr;

        double rotCmd = DriveTuning.aimKp * filtErr - DriveTuning.aimKd * yawRate;

        double max = Math.abs(DriveTuning.aimMax);
        double min = Math.min(Math.abs(DriveTuning.aimMin), max);
        double slowdown = Math.max(0.1, DriveTuning.aimSlowdownDeg);
        double ramp = clamp((absErr - deadband) / slowdown, 0.0, 1.0);

        double maxForError = Math.max(min * ramp, max * Math.max(0.25, ramp));
        rotCmd = clamp(rotCmd, -maxForError, maxForError);

        double minForError = min * ramp;
        if (Math.abs(rotCmd) < minForError) {
            rotCmd = Math.copySign(minForError, filtErr);
        }

        return clamp(rotCmd, -max, max);
    }

    private double calculateSearchCommand(double yawRate) {
        double period = Math.max(0.2, DriveTuning.scanPeriod);
        int targetSide = searchDirection;

        if (scanTimer.seconds() > period) {
            int phase = (int) Math.floor((scanTimer.seconds() - period) / period);
            targetSide = (phase % 2 == 0) ? -searchDirection : searchDirection;
        }

        double driveDirection = DriveTuning.invertAim ? -targetSide : targetSide;
        double maxScanPower = Math.abs(DriveTuning.scanPower);

        if (imuOk) {
            double desiredYawRate = driveDirection * DriveTuning.scanYawRateCmd;
            double rateCmd = 0.0035 * (desiredYawRate - yawRate);
            return clamp(rateCmd, -maxScanPower, maxScanPower);
        }

        return driveDirection * maxScanPower;
    }

    private double slew(double requested, double previous, double dt) {
        double maxDelta = Math.max(0.0, DriveTuning.rotSlewPerSec) * Math.max(0.001, dt);
        return clamp(requested, previous - maxDelta, previous + maxDelta);
    }

    /**
     * 更新 Limelight 状态（含连续帧消抖 + 对正帧统计）
     */
    private void updateTagStatus() {
        tagVisibleNow = false;
        alignedToTag = false;
        limelightResultValid = false;
        targetSeenId = -1;
        targetSeenTxDeg = 0.0;
        targetErrorDeg = 0.0;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            limelightResultValid = true;

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            LLResultTypes.FiducialResult best = chooseBestFiducial(fiducials);
            if (best != null) {
                setVisibleTarget(best.getFiducialId(), best.getTargetXDegrees());
            } else if (DriveTuning.targetTagId == -1
                    || (!DriveTuning.strictIdFilter && (fiducials == null || fiducials.isEmpty()))) {
                setVisibleTarget(-1, result.getTx());
            }
        }

        if (!tagVisibleNow) {
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

    private static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    // =========================
    // Limelight 兼容层
    // =========================
    private void applyLimelightPipeline() {
        if (DriveTuning.limelightPipeline == activeLimelightPipeline) return;
        limelight.pipelineSwitch(DriveTuning.limelightPipeline);
        activeLimelightPipeline = DriveTuning.limelightPipeline;
    }

    private LLResultTypes.FiducialResult chooseBestFiducial(List<LLResultTypes.FiducialResult> fiducials) {
        if (fiducials == null) return null;

        LLResultTypes.FiducialResult best = null;
        double bestCenterError = Double.POSITIVE_INFINITY;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();

            if (DriveTuning.targetTagId != -1) {
                if (DriveTuning.strictIdFilter) {
                    if (id != DriveTuning.targetTagId) continue;
                } else {
                    if (id != -1 && id != DriveTuning.targetTagId) continue;
                }
            }

            double centerError = Math.abs(fiducial.getTargetXDegrees() - DriveTuning.LIMELIGHT_CENTER_TX);
            if (centerError < bestCenterError) {
                bestCenterError = centerError;
                best = fiducial;
            }
        }

        return best;
    }

    private void setVisibleTarget(int id, double txDeg) {
        tagVisibleNow = true;
        hasTargetTag = true;
        tagSeenTimer.reset();

        targetSeenId = id;
        targetSeenTxDeg = txDeg;

        targetErrorDeg = txDeg - DriveTuning.LIMELIGHT_CENTER_TX;
        lastSeenErrorDeg = targetErrorDeg;
        if (Math.abs(targetErrorDeg) > Math.max(0.0, DriveTuning.aimDeadbandDeg)) {
            searchDirection = targetErrorDeg > 0.0 ? 1 : -1;
        }
        scanTimer.reset();

        alignedToTag = Math.abs(targetErrorDeg) <= DriveTuning.aimDeadbandDeg;
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
