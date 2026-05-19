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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "Blue", group = "LinearOp Blue")
public class Blue extends LinearOpMode {

    @Config
    public static class DriveTuning {
        // --- AprilTag / Limelight --
        public static int targetTagId = 20;          // -1 不限定
        public static int limelightPipeline = 0;
        public static double LIMELIGHT_CENTER_TX = 0.0; // deg，调它修“对正偏左/偏右”
        public static boolean strictIdFilter = true;

        // --- 3D AutoAim：Limelight botpose -> 场地目标点 -> 炮台角度/射角 ---
        public static double targetFieldX_M = 0.0;
        public static double targetFieldY_M = 0.0;
        public static double targetFieldZ_M = 0.90;
        public static double turretOffsetForward_M = 0.0;
        public static double turretOffsetLeft_M = 0.0;
        public static double shooterExitHeight_M = 0.35;
        public static double turretZeroOffsetDeg = 0.0; // encoder 0 时炮台相对车头的角度
        public static int poseMinTagCount = 1;
        public static double poseMaxStalenessMs = 250.0;

        // --- 炮台 CRServo 闭环（无编码器，用输出积分估算角度）---
        public static double turretDegPerSecAtFullPower = 180.0;
        public static boolean invertTurretPower = false;
        public static double turretKp = 0.014;
        public static double turretKi = 0.0004;
        public static double turretKd = 0.0012;
        public static double turretMinPower = 0.025;
        public static double turretMaxPower = 1;
        public static double turretDeadbandDeg = 1.5;
        public static double turretSlowdownDeg = 14.0;
        public static double turretStopLookaheadSec = 0.08;
        public static double turretIntegralZoneDeg = 10.0;
        public static double turretIntegralLimitDegSec = 35.0;
        public static double turretDerivativeAlpha = 0.45;
        public static double turretMinDeg = -120.0;
        public static double turretMaxDeg = 120.0;
        public static double turretManualPower = 1;
        public static double turretPowerSlewPerSec = 2.0;

        // --- 射角 servo 映射：servo = angleServoAt0Deg + angleDeg * angleServoDegToPos ---
        public static double angleServoAt0Deg = 0.50;
        public static double angleServoDegToPos = 0.006;
        public static double angleServoMinPos = 0.20;
        public static double angleServoMaxPos = 0.85;
        public static double angleServoDefaultPos = 0.50;
        public static double shotAngleOffsetDeg = 0.0;
        public static double shotAngleDistanceKDegPerM = 0.0;
        public static double angleManualStepPerSec = 0.25;

        // --- 误差滤波 ---
        public static double errAlpha = 0.55;

        // --- 连续帧消抖：连续看到 N 帧才进入控制（解决“突然动两下”）---
        public static int aimAcquireFrames = 3;     // 2~4

        // --- 对正判定稳定帧数（给 LED 用）---
        public static int alignedNeedFrames = 5;

        // --- LED 位置 ---
        public static double LED_RED_POS   = 0.28;
        public static double LED_GREEN_POS = 0.49;
        public static double LED_BLUE_POS  = 0.611;

        // --- Shooter 可调 ---
        public static double shooterTargetVel = 6000;

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

    private DcMotorEx shootMotor;
    private CRServo turretServo;

    private DcMotor intake_front;

    private CRServo intake_back;
    private CRServo intake_midL;
    private CRServo intake_midR;
    private CRServo vertical_left;
    private CRServo vertical_right;

    private Servo shootAngleServo;
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
    private boolean lastShooterToggleBtn = false;  // X toggle

    // ===== 炮台误差滤波 + dt =====
    private double filtErr = 0.0;
    private final ElapsedTime aimDtTimer = new ElapsedTime();
    private double turretIntegral = 0.0;
    private double lastTurretPidError = 0.0;
    private double turretErrorRateDegPerSec = 0.0;
    private boolean turretPidHasLastError = false;
    private double turretPidP = 0.0;
    private double turretPidI = 0.0;
    private double turretPidD = 0.0;

    // ===== 目标丢失保持（只给 UI 用）=====
    private boolean hasTargetTag = false;
    private boolean tagVisibleNow = false;
    private final ElapsedTime tagSeenTimer = new ElapsedTime();

    // ===== 对正帧数 =====
    private boolean alignedToTag = false;
    private int alignedFrames = 0;

    // ===== 炮台输出限速 + 输出记忆 =====
    private double lastTurretPower = 0.0;

    // ===== Limelight 输出 =====
    private boolean limelightResultValid = false;
    private boolean poseValidNow = false;
    private int poseVisibleFrames = 0;
    private int botposeTagCount = 0;
    private double botposeAvgDist = 0.0;
    private double robotFieldX = 0.0;
    private double robotFieldY = 0.0;
    private double robotFieldZ = 0.0;
    private double robotFieldYawDeg = 0.0;
    private double targetErrorDeg = 0.0;
    private int targetSeenId = -1;
    private double targetSeenTxDeg = 0.0;
    private double lastSeenErrorDeg = 0.0;
    private double turretTargetDeg = 0.0;
    private double turretMeasuredDeg = 0.0;
    private double turretErrorDeg = 0.0;
    private double turretPower = 0.0;
    private double lastTurretLogicalPower = 0.0;
    private double targetDistanceM = 0.0;
    private double targetHorizontalDistanceM = 0.0;
    private double shotAngleDeg = 0.0;
    private double shootAngleServoPos = 0.5;

    // ===== 红灯闪烁 timer（替代 sleep）=====
    private final ElapsedTime redBlinkTimer = new ElapsedTime();
    private boolean redBlinkOn = true;

    // ===== 新增：连续可见帧计数（用于消抖稳定可见）=====
    private int tagVisibleFrames = 0;
    private boolean autoAimLastLoop = false;
    private String aimMode = "Idle";

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ========== init 硬件 ==========
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "LF Drive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RF Drive");
        leftBackMotor   = hardwareMap.get(DcMotor.class, "LB Drive");
        rightBackMotor  = hardwareMap.get(DcMotor.class, "RB Drive");

        shootMotor = hardwareMap.get(DcMotorEx.class, "shoot");
        turretServo = hardwareMap.get(CRServo.class, "turret rotate");

        intake_front = hardwareMap.get(DcMotor.class, "intake front");

        intake_back = hardwareMap.get(CRServo.class, "intake back");
        vertical_left = hardwareMap.get(CRServo.class, "vertical left");
        vertical_right = hardwareMap.get(CRServo.class, "vertical right");
        intake_midL = hardwareMap.get(CRServo.class, "intake middle left");
        intake_midR = hardwareMap.get(CRServo.class, "intake middle right");
        shootAngleServo = hardwareMap.get(Servo.class, "shoot angle");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(20);
        applyLimelightPipeline();
        limelight.start();

        // Directions
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        intake_back.setDirection(CRServo.Direction.REVERSE);

        turretServo.setDirection(CRServo.Direction.FORWARD);

        vertical_right.setDirection(CRServo.Direction.REVERSE);
        intake_front.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootMotor.setVelocity(0);
        turretServo.setPower(0);
        intake_front.setPower(0);

        intake_back.setPower(0);
        vertical_left.setPower(0);
        vertical_right.setPower(0);
        intake_midL.setPower(0);
        intake_midR.setPower(0);
        shootAngleServoPos = clamp(DriveTuning.angleServoDefaultPos,
                DriveTuning.angleServoMinPos, DriveTuning.angleServoMaxPos);
        shootAngleServo.setPosition(shootAngleServoPos);

        // IMU 初始化（USB 在上方，Logo 朝左）
        initImu();

        // timers
        aimDtTimer.reset();
        tagSeenTimer.reset();
        redBlinkTimer.reset();

        telemetry.addLine("Init done. Dashboard -> Config -> DriveTuning to live-tune.");
        telemetry.addData("IMU", imuOk ? "OK" : "NO");
        telemetry.update();

        waitForStart();

        aimDtTimer.reset();
        tagSeenTimer.reset();
        redBlinkTimer.reset();

        while (opModeIsActive()) {

            // =========================
            // 更新 Limelight 状态（含连续帧消抖）
            // =========================
            applyLimelightPipeline();
            updateTagStatus();

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
            // RB：按住用 Limelight 3D botpose 自动转炮台 + 调射角
            // =========================
            boolean rbAimHeld = gamepad1.right_bumper || gamepad2.right_bumper;

            double yawDeg = 0.0;
            double yawRate = 0.0;
            if (imuOk) {
                AngularVelocity av = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                yawRate = av.zRotationRate;
                YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
                yawDeg = ypr.getYaw(AngleUnit.DEGREES);
            }

            double aimDt = aimDtTimer.seconds();
            if (aimDt <= 0.0 || aimDt > 0.1) aimDt = 0.02;
            aimDtTimer.reset();

            updateTurretAndShotAngle(rbAimHeld, aimDt);
            autoAimLastLoop = rbAimHeld;


            if (alignedToTag) alignedFrames++;
            else alignedFrames = 0;

            boolean alignedStable = alignedFrames >= DriveTuning.alignedNeedFrames;
            if (statusLed != null) {
                if (!shootMotorStatus) ledRedBlinkNoSleep();
                else statusLed.setPosition(alignedStable ? DriveTuning.LED_GREEN_POS : DriveTuning.LED_BLUE_POS);
            }

            // 写入底盘
            leftFrontMotor.setPower(lf);
            rightFrontMotor.setPower(rf);
            leftBackMotor.setPower(lb);
            rightBackMotor.setPower(rb);

            // =========================
            // SHOOTER：toggle = X
            // =========================
            boolean shootPressed = gamepad1.x && !lastShooterToggleBtn;
            lastShooterToggleBtn = gamepad1.x;

            if (shootPressed) {
                shootMotorStatus = !shootMotorStatus;
            }

            if (shootMotorStatus) {
                shootMotor.setVelocity(DriveTuning.shooterTargetVel);
            } else {
                shootMotor.setVelocity(0);
            }

            // =========================
            // INTAKE（D-pad UP）
            // =========================
            boolean intakePressed = gamepad1.dpad_up && !lastIntakeButton;
            lastIntakeButton = gamepad1.dpad_up;

            if (intakePressed) IntakeMotorStatus = !IntakeMotorStatus;
            intake_front.setPower(IntakeMotorStatus ? 1.0 : 0.0);

            double pullPowerBackMid = 1;

            // =========================
            // FRONT PULLUP（LB）
            // =========================
            boolean lbHeld = gamepad1.left_bumper;

            double pullPowerFront = 1;
            double frontOutput = lbHeld ? pullPowerFront : 0.0;

            vertical_left.setPower(frontOutput);
            vertical_right.setPower(frontOutput);

            if (shootMotorStatus && vertical_left.getPower() > 0) {
                intake_midL.setPower(1);
                intake_midR.setPower(1);
                intake_back.setPower(1);
            } else {
                double intakeServoPower = IntakeMotorStatus ? pullPowerBackMid : 0.0;
                intake_back.setPower(intakeServoPower);
                intake_midL.setPower(intakeServoPower);
                intake_midR.setPower(intakeServoPower);
            }

            double shootAngleInput = -gamepad2.right_stick_y;
            if (Math.abs(shootAngleInput) > 0.05) {
                shootAngleServoPos += shootAngleInput * DriveTuning.angleManualStepPerSec * aimDt;
                shootAngleServoPos = clamp(shootAngleServoPos,
                        DriveTuning.angleServoMinPos, DriveTuning.angleServoMaxPos);
                shootAngleServo.setPosition(shootAngleServoPos);
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

            telemetry.addData("poseValidNow", poseValidNow);
            telemetry.addData("poseVisibleFrames", poseVisibleFrames);
            telemetry.addData("botposeTagCount", botposeTagCount);
            telemetry.addData("botposeAvgDist", String.format("%.2f", botposeAvgDist));
            telemetry.addData("robotPose", String.format("x=%.2f y=%.2f z=%.2f yaw=%.1f",
                    robotFieldX, robotFieldY, robotFieldZ, robotFieldYawDeg));
            telemetry.addData("targetTagVisible", tagVisibleNow);
            telemetry.addData("targetTagFrames", tagVisibleFrames);
            telemetry.addData("stableVisibleNeed", DriveTuning.aimAcquireFrames);

            telemetry.addData("alignedFrames", alignedFrames);

            telemetry.addData("TagSeenId", targetSeenId);
            telemetry.addData("TagTX(deg)", String.format("%.2f", targetSeenTxDeg));
            telemetry.addData("ErrDeg", String.format("%.2f", targetErrorDeg));
            telemetry.addData("LastSeenErrDeg", String.format("%.2f", lastSeenErrorDeg));
            telemetry.addData("Turret", String.format("meas=%.1f target=%.1f err=%.1f power=%.2f",
                    turretMeasuredDeg, turretTargetDeg, turretErrorDeg, turretPower));
            telemetry.addData("TurretPID", String.format("P=%.3f I=%.3f D=%.3f rate=%.1f",
                    turretPidP, turretPidI, turretPidD, turretErrorRateDegPerSec));
            telemetry.addData("Shot", String.format("dist=%.2f horiz=%.2f angle=%.1f servo=%.3f",
                    targetDistanceM, targetHorizontalDistanceM, shotAngleDeg, shootAngleServoPos));
            telemetry.addData("FiltTurretErr", String.format("%.2f", filtErr));

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

    private void resetTurretController() {
        resetTurretPid();
        lastTurretPower = 0.0;
        turretPower = 0.0;
        lastTurretLogicalPower = 0.0;
    }

    private void resetTurretPid() {
        filtErr = 0.0;
        turretIntegral = 0.0;
        lastTurretPidError = 0.0;
        turretErrorRateDegPerSec = 0.0;
        turretPidHasLastError = false;
        turretPidP = 0.0;
        turretPidI = 0.0;
        turretPidD = 0.0;
    }

    private void updateTurretAndShotAngle(boolean autoAimHeld, double dt) {
        updateEstimatedTurretAngle(dt);
        if (!autoAimHeld && gamepad1.b) {
            turretMeasuredDeg = 0.0;
            resetTurretPid();
        }

        if (autoAimHeld) {
            if (!autoAimLastLoop) resetTurretController();

            boolean stablePose = poseVisibleFrames >= Math.max(1, DriveTuning.aimAcquireFrames);
            if (stablePose && update3DAimSetpoints()) {
                aimMode = "3DTrack";
                turretPower = calculateTurretPower(dt);
                shootAngleServo.setPosition(shootAngleServoPos);
            } else {
                aimMode = poseValidNow ? "3DAcquire" : "NoPose";
                alignedToTag = false;
                resetTurretPid();
                turretPower = 0.0;
            }
        } else {
            aimMode = "Manual";
            alignedToTag = false;
            resetTurretPid();
            double manualInput = gamepad2.left_stick_x;
            if (Math.abs(manualInput) <= 0.05) {
                manualInput = gamepad1.right_trigger - gamepad1.left_trigger;
            }
            double manualPower = manualInput * DriveTuning.turretManualPower;
            manualPower = applyTurretSoftLimits(manualPower);
            turretPower = DriveTuning.invertTurretPower ? -manualPower : manualPower;
        }

        if (autoAimHeld && Math.abs(turretPower) <= 0.0001) {
            lastTurretPower = 0.0;
            lastTurretLogicalPower = 0.0;
            turretServo.setPower(0.0);
            return;
        }

        turretPower = slew(turretPower, lastTurretPower, dt, DriveTuning.turretPowerSlewPerSec);
        lastTurretPower = turretPower;
        lastTurretLogicalPower = DriveTuning.invertTurretPower ? -turretPower : turretPower;
        turretServo.setPower(turretPower);
    }

    private boolean update3DAimSetpoints() {
        double yawRad = Math.toRadians(robotFieldYawDeg);
        double cos = Math.cos(yawRad);
        double sin = Math.sin(yawRad);

        double turretFieldX = robotFieldX
                + DriveTuning.turretOffsetForward_M * cos
                - DriveTuning.turretOffsetLeft_M * sin;
        double turretFieldY = robotFieldY
                + DriveTuning.turretOffsetForward_M * sin
                + DriveTuning.turretOffsetLeft_M * cos;
        double turretFieldZ = robotFieldZ + DriveTuning.shooterExitHeight_M;

        double dx = DriveTuning.targetFieldX_M - turretFieldX;
        double dy = DriveTuning.targetFieldY_M - turretFieldY;
        double dz = DriveTuning.targetFieldZ_M - turretFieldZ;

        targetHorizontalDistanceM = Math.hypot(dx, dy);
        targetDistanceM = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (targetHorizontalDistanceM < 0.05) return false;

        double fieldBearingDeg = Math.toDegrees(Math.atan2(dy, dx));
        turretTargetDeg = wrapDeg(fieldBearingDeg - robotFieldYawDeg - DriveTuning.turretZeroOffsetDeg);
        turretTargetDeg = clamp(turretTargetDeg, DriveTuning.turretMinDeg, DriveTuning.turretMaxDeg);
        turretErrorDeg = wrapDeg(turretTargetDeg - turretMeasuredDeg);

        shotAngleDeg = Math.toDegrees(Math.atan2(dz, targetHorizontalDistanceM))
                + DriveTuning.shotAngleOffsetDeg
                + DriveTuning.shotAngleDistanceKDegPerM * targetHorizontalDistanceM;
        shootAngleServoPos = angleDegToServo(shotAngleDeg);

        alignedToTag = Math.abs(turretErrorDeg) <= DriveTuning.turretDeadbandDeg;
        return true;
    }

    private double calculateTurretPower(double dt) {
        double alpha = clamp(DriveTuning.errAlpha, 0.0, 1.0);
        filtErr = alpha * turretErrorDeg + (1.0 - alpha) * filtErr;

        double deadband = Math.max(0.0, DriveTuning.turretDeadbandDeg);
        double rawAbsErr = Math.abs(turretErrorDeg);
        if (rawAbsErr <= deadband) {
            resetTurretPid();
            return 0.0;
        }

        if (Math.signum(filtErr) != Math.signum(turretErrorDeg)) {
            filtErr = turretErrorDeg;
            turretIntegral = 0.0;
        }

        double safeDt = Math.max(0.001, dt);
        double measuredErrorRate = turretPidHasLastError ? (filtErr - lastTurretPidError) / safeDt : 0.0;
        double derivativeAlpha = clamp(DriveTuning.turretDerivativeAlpha, 0.0, 1.0);
        turretErrorRateDegPerSec = derivativeAlpha * measuredErrorRate
                + (1.0 - derivativeAlpha) * turretErrorRateDegPerSec;
        lastTurretPidError = filtErr;
        turretPidHasLastError = true;

        double slowdown = Math.max(deadband + 0.1, DriveTuning.turretSlowdownDeg);
        double lookahead = Math.max(0.0, DriveTuning.turretStopLookaheadSec);
        double projectedError = filtErr + turretErrorRateDegPerSec * lookahead;
        if (rawAbsErr <= slowdown && Math.signum(projectedError) != Math.signum(turretErrorDeg)) {
            resetTurretPid();
            return 0.0;
        }

        double integralZone = Math.max(deadband, DriveTuning.turretIntegralZoneDeg);
        if (rawAbsErr <= integralZone) {
            turretIntegral += filtErr * safeDt;
            double integralLimit = Math.max(0.0, DriveTuning.turretIntegralLimitDegSec);
            turretIntegral = clamp(turretIntegral, -integralLimit, integralLimit);
        } else {
            turretIntegral = 0.0;
        }

        turretPidP = DriveTuning.turretKp * filtErr;
        turretPidI = DriveTuning.turretKi * turretIntegral;
        turretPidD = DriveTuning.turretKd * turretErrorRateDegPerSec;

        double cmd = turretPidP + turretPidI + turretPidD;
        double max = Math.abs(DriveTuning.turretMaxPower);
        double min = Math.min(Math.abs(DriveTuning.turretMinPower), max);
        double ramp = clamp((rawAbsErr - deadband) / (slowdown - deadband), 0.0, 1.0);
        double maxForError = min + (max - min) * ramp;
        double minForError = min * ramp;
        cmd = clamp(cmd, -maxForError, maxForError);

        if (Math.abs(cmd) < minForError) {
            double sign = Math.abs(cmd) > 0.0001 ? Math.signum(cmd) : Math.signum(filtErr);
            cmd = sign * minForError;
        }

        cmd = applyTurretSoftLimits(cmd);
        return DriveTuning.invertTurretPower ? -cmd : cmd;
    }

    private double applyTurretSoftLimits(double logicalPower) {
        if (turretMeasuredDeg >= DriveTuning.turretMaxDeg && logicalPower > 0.0) return 0.0;
        if (turretMeasuredDeg <= DriveTuning.turretMinDeg && logicalPower < 0.0) return 0.0;
        return logicalPower;
    }

    private void updateEstimatedTurretAngle(double dt) {
        double maxDegPerSec = Math.abs(DriveTuning.turretDegPerSecAtFullPower);
        turretMeasuredDeg += lastTurretLogicalPower * maxDegPerSec * dt;
        turretMeasuredDeg = clamp(turretMeasuredDeg, DriveTuning.turretMinDeg, DriveTuning.turretMaxDeg);
    }

    private double getTurretVelocityDegPerSec() {
        return lastTurretLogicalPower * Math.abs(DriveTuning.turretDegPerSecAtFullPower);
    }

    private double angleDegToServo(double angleDeg) {
        double pos = DriveTuning.angleServoAt0Deg + angleDeg * DriveTuning.angleServoDegToPos;
        return clamp(pos, DriveTuning.angleServoMinPos, DriveTuning.angleServoMaxPos);
    }

    private double slew(double requested, double previous, double dt, double maxPerSec) {
        double maxDelta = Math.max(0.0, maxPerSec) * Math.max(0.001, dt);
        return clamp(requested, previous - maxDelta, previous + maxDelta);
    }

    /**
     * 更新 Limelight 状态（含连续帧消抖 + 对正帧统计）
     */
    private void updateTagStatus() {
        tagVisibleNow = false;
        alignedToTag = false;
        poseValidNow = false;
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

            if (tagVisibleNow) updateBotposeStatus(result);
        }

        if (!tagVisibleNow) {
            double hold = Math.max(0.0, DriveTuning.tagHoldSeconds);
            if (hasTargetTag && tagSeenTimer.seconds() <= hold) hasTargetTag = true;
            else hasTargetTag = false;
        }

        // 连续可见帧数（控制用）
        if (tagVisibleNow) tagVisibleFrames++;
        else tagVisibleFrames = 0;

        if (poseValidNow) poseVisibleFrames++;
        else poseVisibleFrames = 0;
    }

    private void updateBotposeStatus(LLResult result) {
        botposeTagCount = result.getBotposeTagCount();
        botposeAvgDist = result.getBotposeAvgDist();

        if (botposeTagCount < Math.max(1, DriveTuning.poseMinTagCount)) return;
        if (result.getStaleness() > Math.max(20.0, DriveTuning.poseMaxStalenessMs)) return;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return;

        Position position = botpose.getPosition().toUnit(DistanceUnit.METER);

        robotFieldX = position.x;
        robotFieldY = position.y;
        robotFieldZ = position.z;
        robotFieldYawDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        poseValidNow = true;
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
    }

    private void ledRedBlinkNoSleep() {
        if (statusLed == null) return;
        double period = Math.max(0.1, DriveTuning.redBlinkPeriodSec);
        if (redBlinkTimer.seconds() >= period) {
            redBlinkTimer.reset();
            redBlinkOn = !redBlinkOn;
        }
        statusLed.setPosition(redBlinkOn ? DriveTuning.LED_RED_POS : 0.0);
    }
}
