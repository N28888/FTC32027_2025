package org.firstinspires.ftc.teamcode.teleop.red;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp(name = "JYF_RED_LIN", group = "TeleOp5")
public class JYF_RED_LIN extends LinearOpMode {

    // =========================
    // 你要识别/对准的 Tag number 在这里改：
    // -1 = 不限定
    // 5  = 只对准 5 号 tag
    // =========================
    private int targetTagId = 5;

    // =========================
    // goBILDA RGB Indicator Light（PWM 当 Servo）
    // =========================
    private Servo statusLed = null;
    private double LED_RED_POS   = 0.28;
    private double LED_GREEN_POS = 0.49;
    private double LED_BLUE_POS  = 0.611;

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

    // --- HuskyLens ---
    private HuskyLens husky;
    private static final int HUSKY_CENTER_X = 140;

    // ===== AutoAim 控制参数 =====
    private double aimKp = 0.006;
    private double aimKd = 0.0025;
    private double aimMin = 0.05;
    private double aimMax = 0.55;
    private int aimDeadbandPx = 16;

    // ===== 误差滤波 + dt =====
    private double errAlpha = 0.35;
    private double filtErr = 0.0;
    private double lastFiltErr = 0.0;
    private ElapsedTime aimDtTimer = new ElapsedTime();

    // ===== 目标丢失保持（只给 LED/UI 用，不再用于控制）=====
    private ElapsedTime tagSeenTimer = new ElapsedTime();
    private double tagHoldSeconds = 0.25;

    // ===== “对正”连续帧确认（防 LED 乱跳）=====
    private int alignedFrames = 0;
    private int alignedNeedFrames = 5;

    // ===== 扫描：来回扫 =====
    private ElapsedTime scanTimer = new ElapsedTime();
    private double scanPower = 0.14;
    private double scanPeriod = 1.2;

    // 方向开关：如果反了改 true
    private boolean invertAim = false;

    // 如果 TagSeenId 经常 = -1，先设 false
    private boolean strictIdFilter = true;

    // ===== 旋转输出限速（slew）=====
    private double lastRotCmd = 0.0;
    private double rotSlewPerSec = 1.6;

    // ===== 过冲抑制：误差符号翻转时短暂刹车 =====
    private int lastErrSign = 0;
    private ElapsedTime signFlipTimer = new ElapsedTime();
    private double flipBrakeSeconds = 0.08;

    // ===== Husky 状态 =====
    private boolean hasTargetTag = false;
    private boolean tagVisibleNow = false;
    private boolean alignedToTag = false;

    private int targetErrorPx = 0;
    private int targetSeenId = -1;
    private int targetSeenX = -1;

    // =========================
    // IMU（Control Hub 内建）
    // =========================
    private IMU imu;
    private boolean imuOk = false;

    // IMU 阻尼参数（你主要调这俩）
    private double yawRateDampK = 0.045;  // 0.03~0.08
    private double yawRateLimit = 180.0;  // deg/s: 120~220
    private double scanYawRateCmd = 60.0; // 丢目标扫描角速度 40~90

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
        statusLed.setPosition(LED_RED_POS);

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

        // 计时器 reset
        shooterReverseTimer.reset();
        pullupFrontReverseTimer.reset();
        autoMoveTimer.reset();
        aimDtTimer.reset();
        scanTimer.reset();
        tagSeenTimer.reset();
        signFlipTimer.reset();

        telemetry.addLine("Init done. Waiting for start...");
        telemetry.addData("IMU", imuOk ? "OK" : "NO");
        telemetry.update();

        waitForStart();

        // ========== 开始后再 reset 一次（更干净） ==========
        aimDtTimer.reset();
        scanTimer.reset();
        tagSeenTimer.reset();
        signFlipTimer.reset();

        while (opModeIsActive()) {

            // =========================
            // 更新 HuskyLens 状态
            // =========================
            updateTagStatus();

            // =========================
            // LED：Shooter 没开=红闪；开了且对正稳定=绿；否则蓝
            // =========================
            boolean alignedStable = alignedFrames >= alignedNeedFrames;
            if (!shootMotorStatus) led_red_blink();
            else statusLed.setPosition(alignedStable ? LED_GREEN_POS : LED_BLUE_POS);

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
            // D-pad Down 自动前后移动（沿用你逻辑）
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

                double rotCmd;

                if (tagVisibleNow) {
                    // 误差滤波
                    filtErr = errAlpha * targetErrorPx + (1.0 - errAlpha) * filtErr;

                    // D 项
                    double derr = (filtErr - lastFiltErr) / dt;
                    lastFiltErr = filtErr;

                    // PD
                    rotCmd = aimKp * filtErr + aimKd * derr;
                    if (invertAim) rotCmd = -rotCmd;

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

                    if (signFlipTimer.seconds() < flipBrakeSeconds) {
                        rotCmd *= 0.2;
                    }

                    // 接近中心不强制 aimMin
                    if (Math.abs(targetErrorPx) > (aimDeadbandPx + 6)) {
                        if (rotCmd > 0) rotCmd = Math.max(rotCmd, aimMin);
                        else            rotCmd = Math.min(rotCmd, -aimMin);
                    } else {
                        if (Math.abs(targetErrorPx) <= aimDeadbandPx) rotCmd = 0.0;
                    }

                    rotCmd = clamp(rotCmd, -aimMax, aimMax);

                    // IMU 阻尼：抑制过冲
                    if (imuOk) {
                        rotCmd -= yawRateDampK * yawRate;
                        if (Math.abs(yawRate) > yawRateLimit) rotCmd *= 0.35;
                    }

                } else {
                    // 没看到 tag：清状态 + 稳定扫描
                    filtErr *= 0.85;
                    lastFiltErr = filtErr;
                    lastRotCmd *= 0.85;

                    double dir = (Math.floor(scanTimer.seconds() / scanPeriod) % 2 == 0) ? 1.0 : -1.0;

                    if (imuOk) {
                        double desired = dir * scanYawRateCmd;
                        double errRate = desired - yawRate;
                        rotCmd = clamp(0.0045 * errRate, -scanPower, scanPower);
                    } else {
                        rotCmd = dir * scanPower;
                    }
                }

                // slew 限速
                double maxDelta = rotSlewPerSec * dt;
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

            double targetVelocity = 1000;
            double reverseVelocity = 1000;
            double shootReverseTime = 0.15;

            if (shootMotorStatus) {
                if (shooterReversing && shooterReverseTimer.seconds() < shootReverseTime) {
                    shootMotorL.setVelocity(-reverseVelocity);
                    shootMotorR.setVelocity(-reverseVelocity);
                } else {
                    shooterReversing = false;
                    shootMotorL.setVelocity(targetVelocity);
                    shootMotorR.setVelocity(targetVelocity);
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
            telemetry.addData("IMU", imuOk ? "OK" : "NO");
            telemetry.addData("Yaw(deg)", String.format("%.1f", yawDeg));
            telemetry.addData("YawRate(deg/s)", String.format("%.1f", yawRate));

            telemetry.addData("TargetTagId", targetTagId);
            telemetry.addData("tagVisibleNow", tagVisibleNow);
            telemetry.addData("aligned(raw)", alignedToTag);
            telemetry.addData("alignedFrames", alignedFrames);
            telemetry.addData("TagSeenId", targetSeenId);
            telemetry.addData("TagX", targetSeenX);
            telemetry.addData("ErrPx", targetErrorPx);
            telemetry.addData("FiltErr", String.format("%.1f", filtErr));
            telemetry.addData("rotCmd", String.format("%.2f", lastRotCmd));

            telemetry.addData("Shooter", shootMotorStatus ? "ON" : "OFF");
            telemetry.addData("Intake", IntakeMotorStatus ? "ON" : "OFF");
            telemetry.update();
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
     * 更新 HuskyLens 状态
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

                if (targetTagId != -1) {
                    if (strictIdFilter) {
                        if (id != targetTagId) continue;
                    } else {
                        if (id != -1 && id != targetTagId) continue;
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

            targetErrorPx = best.x - HUSKY_CENTER_X;
            alignedToTag = Math.abs(targetErrorPx) <= aimDeadbandPx;
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

    private void led_red_blink() {
        statusLed.setPosition(LED_RED_POS);
        sleep(500);
        statusLed.setPosition(0);
        sleep(500);
    }
}