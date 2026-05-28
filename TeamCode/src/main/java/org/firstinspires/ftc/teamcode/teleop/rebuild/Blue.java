package org.firstinspires.ftc.teamcode.teleop.rebuild;

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

import java.util.List;

@Config
@TeleOp(name="Rebuild Blue",group = "TeleOp3")
public class Blue extends LinearOpMode {

    public static int LIMELIGHT_PIPELINE = 0;
    public static int LIMELIGHT_POLL_RATE_HZ = 100;
    public static double TAG_RESULT_TIMEOUT_MS = 250.0;

    public static double P = 0.025;
    public static double I = 0.0;
    public static double D = 0.0015;
    public static double AIM_TOLERANCE_DEG = 1.0;
    public static double MIN_TURRET_POWER = 0.04;
    public static double MAX_TURRET_POWER = 1.0;
    public static double INTEGRAL_ZONE_DEG = 8.0;
    public static double INTEGRAL_LIMIT = 20.0;
    public static double DERIVATIVE_FILTER = 0.35;
    public static boolean INVERT_TURRET_POWER = false;

    public static boolean ENABLE_IMU_DRIVE_CORRECTION = false;
    public static double DRIVE_IMU_KP = 0.012;
    public static double DRIVE_IMU_MAX_CORRECTION = 0.12;
    public static double DRIVE_STICK_DEADBAND = 0.06;
    public static double DRIVE_ROTATE_DEADBAND = 0.08;
    public static double DRIVE_TRANSLATE_DEADBAND = 0.08;

    public static double SHOOT_ANGLE_MIN_POS = 0.20;
    public static double SHOOT_ANGLE_MAX_POS = 0.85;
    public static double SHOOT_ANGLE_DEFAULT_POS = 0.500;
    public static double SHOOT_ANGLE_MANUAL_STEP_PER_SEC = 0.25;
    public static double SHOOT_ANGLE_DPAD_FINE_STEP = 0.05;
    public static double CAMERA_HEIGHT_M = 0.30;
    public static double APRILTAG_CENTER_HEIGHT_M = 0.90;
    public static double CAMERA_PITCH_DEG = 0.0;
    public static double DISTANCE_SCALE = 2.45 / 1.59;
    public static double DISTANCE_OFFSET_M = 0.0;
    public static double REFERENCE_DISTANCE_M = 2.45;
    public static double REFERENCE_SERVO_POS = 0.500;
    public static double SERVO_POS_PER_METER = 0.0;
    public static double DISTANCE_FILTER_ALPHA = 0.45;

    //----- 硬件定义 -----
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    private DcMotor intake_front;
    private DcMotorEx shoot;

    private DcMotorEx vertical;
    private CRServo intake_midR;
    private CRServo intake_midL;
    private CRServo turret;
    private Servo shoot_angle;


    private Limelight3A limelight;

    private IMU imu;

    //----- 软件内部定义 -----

    private boolean imuOk = false;
    private boolean lastShooterToggleBtn = true;
    private boolean shootMotorStatus = false;
    private boolean lastIntakeButton = false;
    private boolean IntakeMotorStatus = false;
    private boolean lastVerticalButton = false;
    private boolean lastReverseButton = false;
    private boolean lastShootAngleDpadUp = false;
    private boolean lastShootAngleDpadDown = false;

    // ----- 动力变量 -----
    private double IntakePower = 1.0;
    private double ShootVelocity = 4000;
    private double VerticalShootVelocity = 6000;
    private double VerticalReverseSeconds = 0.5;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime verticalReverseTimer = new ElapsedTime();
    private double tag20TxDeg = 0.0;
    private double tag20TyDeg = 0.0;
    private double tag20Area = 0.0;
    private double rawTag20DistanceM = Double.NaN;
    private double correctedTag20DistanceM = Double.NaN;
    private double filteredTag20DistanceM = Double.NaN;
    private double shootAnglePosition = SHOOT_ANGLE_DEFAULT_POS;
    private String shootAngleMode = "Manual";
    private boolean tag20Visible = false;
    private boolean autoAimActive = false;
    private boolean alignedToTag20 = false;
    private double turretPower = 0.0;
    private double aimErrorDeg = 0.0;
    private double aimIntegral = 0.0;
    private double lastAimErrorDeg = 0.0;
    private double aimDerivativeDegPerSec = 0.0;
    private boolean hasLastAimError = false;
    private double aimPTerm = 0.0;
    private double aimITerm = 0.0;
    private double aimDTerm = 0.0;

    private double blockclose = 0.9;
    private double blockopen = 0.75;

    private double blockstatus = 0;
    private double driveHeadingTargetDeg = 0.0;
    private double driveYawDeg = 0.0;
    private double driveHeadingErrorDeg = 0.0;
    private double driveImuCorrection = 0.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        //----- 硬件识别 -----
        leftFrontMotor = hardwareMap.get(DcMotor.class,"LF Drive");
        leftBackMotor = hardwareMap.get(DcMotor.class, "LB Drive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RF Drive");
        rightBackMotor = hardwareMap.get(DcMotor.class,"RB Drive");

        shoot = hardwareMap.get(DcMotorEx.class,"shoot");
        intake_front = hardwareMap.get(DcMotor.class,"intake front");

        turret = hardwareMap.get(CRServo.class,"turret rotate");
        vertical = hardwareMap.get(DcMotorEx.class, "vertical");
        intake_midL = hardwareMap.get(CRServo.class, "intake middle left");
        intake_midR = hardwareMap.get(CRServo.class, "intake middle right");
        shoot_angle = hardwareMap.get(Servo.class, "shoot angle");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(LIMELIGHT_POLL_RATE_HZ);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

        //----- Motor 方向 ------
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        shoot.setDirection(DcMotor.Direction.FORWARD);
        intake_front.setDirection(DcMotor.Direction.REVERSE);

        turret.setDirection(CRServo.Direction.FORWARD);
        vertical.setDirection(DcMotorSimple.Direction.FORWARD);
        intake_midL.setDirection(CRServo.Direction.REVERSE);
        intake_midR.setDirection(CRServo.Direction.FORWARD);
        shoot_angle.setDirection(Servo.Direction.REVERSE);

        //----- 零动力锁死 -----
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //----- 射球电机行为 -----
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical.setVelocity(0.0);
        turret.setPower(0.0);
        shootAnglePosition = clamp(SHOOT_ANGLE_DEFAULT_POS, SHOOT_ANGLE_MIN_POS, SHOOT_ANGLE_MAX_POS);
        shoot_angle.setPosition(shootAnglePosition);


        initIMU();

        telemetry.addLine("Ready. Hold gamepad2 RB to aim AprilTag 20.");
        telemetry.addData("IMU", imuOk ? "OK" : "NO");
        telemetry.addData("Dashboard D", D);
        telemetry.update();

        loopTimer.reset();
        waitForStart();
        if (imuOk) {
            driveYawDeg = getYawDeg();
            driveHeadingTargetDeg = driveYawDeg;
        }
        while (opModeIsActive()){
            limelight.start();
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0.0 || dt > 0.25) {
                dt = 0.02;
            }

            //----- Drive -----
            double forward = applyDeadband(-gamepad1.right_stick_y, DRIVE_STICK_DEADBAND);
            double pan = applyDeadband(gamepad1.right_stick_x, DRIVE_STICK_DEADBAND);
            double rotateInput = applyDeadband(gamepad1.left_stick_x, DRIVE_ROTATE_DEADBAND);
            double rotate = updateImuDriveCorrection(forward, pan, rotateInput);

            double lf = forward + pan + rotate;
            double rf = forward - pan - rotate;
            double lb = forward - pan + rotate;
            double rb = forward + pan - rotate;

            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lb), Math.abs(rb)));
            if (max > 1.0) {
                lf /= max;
                rf /= max;
                lb /= max;
                rb /= max;
            }
            leftFrontMotor.setPower(lf);
            rightFrontMotor.setPower(rf);
            leftBackMotor.setPower(lb);
            rightBackMotor.setPower(rb);

            //----- reverse feed -----
            boolean reverseHeld = gamepad1.dpad_left;
            boolean reverseReleased = !reverseHeld && lastReverseButton;
            lastReverseButton = reverseHeld;

            if (reverseReleased) {
                IntakeMotorStatus = false;
            }

            //----- intake -----
            boolean intakePressed = gamepad1.y && !lastIntakeButton;
            lastIntakeButton = gamepad1.y;

            if (intakePressed) IntakeMotorStatus = !IntakeMotorStatus;
            if (reverseHeld) {
                setIntakePower(-1.0);
            } else {
                setIntakePower(IntakeMotorStatus ? IntakePower : 0.0);
            }

            //----- shoot -----
            boolean shootPressed = gamepad1.x && !lastShooterToggleBtn;
            lastShooterToggleBtn = gamepad1.x;

            if (shootPressed) {
                shootMotorStatus = !shootMotorStatus;
            }

            if (shootMotorStatus) {
                shoot.setVelocity(ShootVelocity);
            } else {
                shoot.setVelocity(0);
            }

            //----- vertical -----
            boolean verticalHeld = gamepad1.left_bumper;
            boolean verticalPressed = verticalHeld && !lastVerticalButton;
            lastVerticalButton = verticalHeld;

            if (verticalPressed) {
                verticalReverseTimer.reset();
            }

            if (reverseHeld) {
                vertical.setVelocity(-Math.abs(VerticalShootVelocity));
            } else if (!verticalHeld) {
                vertical.setVelocity(0);
            } else if (verticalReverseTimer.seconds() < Math.max(0.0, VerticalReverseSeconds)) {
                vertical.setVelocity(-Math.abs(VerticalShootVelocity));
                if (IntakeMotorStatus) {
                    setIntakePower(0.0);
                }
            } else {
                vertical.setVelocity(Math.abs(VerticalShootVelocity));
                IntakeMotorStatus = true;
                setIntakePower(IntakePower);
            }

            updateAprilTag20Aim(dt);

            //----- manual turret / shoot angle -----
            if (!autoAimActive) {
                turretPower = applyDeadband(gamepad2.left_stick_x, 0.05);
                turret.setPower(turretPower);
                updateManualShootAngle(dt);
            }
            updateTelemetry();
        }

        turret.setPower(0.0);
        vertical.setVelocity(0.0);
        limelight.stop();
    }

    private void updateAprilTag20Aim(double dt) {
        autoAimActive = gamepad1.right_bumper;
        LLResultTypes.FiducialResult tag20 = findAprilTag20();
        tag20Visible = tag20 != null;

        if (tag20Visible) {
            tag20TxDeg = tag20.getTargetXDegrees();
            tag20TyDeg = tag20.getTargetYDegrees();
            tag20Area = tag20.getTargetArea();
            updateShotDistanceFromTag(tag20TyDeg);
        } else {
            tag20TxDeg = 0.0;
            tag20TyDeg = 0.0;
            tag20Area = 0.0;
            rawTag20DistanceM = Double.NaN;
            correctedTag20DistanceM = Double.NaN;
            filteredTag20DistanceM = Double.NaN;
        }

        if (!autoAimActive || !tag20Visible) {
            resetAimPid();
            turretPower = 0.0;
            alignedToTag20 = false;
            if (!autoAimActive) {
                shootAngleMode = "Manual";
            }
            if (autoAimActive) {
                turret.setPower(0.0);
                shootAngleMode = "NoTag";
            }
            return;
        }

        updateTurretAim(dt);
        updateShootAngleAim();
    }

    private void updateTurretAim(double dt) {
        aimErrorDeg = tag20TxDeg;

        double safeDt = Math.max(dt, 0.001);
        if (Math.abs(aimErrorDeg) <= Math.max(0.0, AIM_TOLERANCE_DEG)) {
            aimIntegral = 0.0;
            aimDerivativeDegPerSec = 0.0;
            aimPTerm = 0.0;
            aimITerm = 0.0;
            aimDTerm = 0.0;
            turretPower = 0.0;
            turret.setPower(0.0);
            alignedToTag20 = true;
            hasLastAimError = false;
            return;
        }

        alignedToTag20 = false;

        if (Math.abs(aimErrorDeg) <= Math.max(0.0, INTEGRAL_ZONE_DEG)) {
            aimIntegral += aimErrorDeg * safeDt;
            aimIntegral = clamp(aimIntegral, -Math.abs(INTEGRAL_LIMIT), Math.abs(INTEGRAL_LIMIT));
        } else {
            aimIntegral = 0.0;
        }

        double rawDerivative = hasLastAimError ? (aimErrorDeg - lastAimErrorDeg) / safeDt : 0.0;
        double derivativeAlpha = clamp(DERIVATIVE_FILTER, 0.0, 1.0);
        aimDerivativeDegPerSec = derivativeAlpha * rawDerivative
                + (1.0 - derivativeAlpha) * aimDerivativeDegPerSec;
        lastAimErrorDeg = aimErrorDeg;
        hasLastAimError = true;

        aimPTerm = P * aimErrorDeg;
        aimITerm = I * aimIntegral;
        aimDTerm = D * aimDerivativeDegPerSec;

        double command = aimPTerm + aimITerm + aimDTerm;
        command = clamp(command, -Math.abs(MAX_TURRET_POWER), Math.abs(MAX_TURRET_POWER));
        if (Math.abs(command) > 0.0 && Math.abs(command) < Math.abs(MIN_TURRET_POWER)) {
            command = Math.copySign(Math.abs(MIN_TURRET_POWER), command);
        }

        turretPower = INVERT_TURRET_POWER ? -command : command;
        turret.setPower(turretPower);
    }

    private void updateShootAngleAim() {
        if (Double.isNaN(filteredTag20DistanceM)) {
            shootAngleMode = "NoDistance";
            return;
        }

        shootAnglePosition = distanceToShootAngleServo(filteredTag20DistanceM);
        shoot_angle.setPosition(shootAnglePosition);
        shootAngleMode = "TagDistance";
    }

    private void updateManualShootAngle(double dt) {
        double input = -applyDeadband(gamepad2.right_stick_y, 0.05);
        boolean dpadUpPressed = gamepad1.dpad_up && !lastShootAngleDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !lastShootAngleDpadDown;
        lastShootAngleDpadUp = gamepad1.dpad_up;
        lastShootAngleDpadDown = gamepad1.dpad_down;

        if (input == 0.0 && !dpadUpPressed && !dpadDownPressed) {
            return;
        }

        if (input != 0.0) {
            shootAnglePosition += input * SHOOT_ANGLE_MANUAL_STEP_PER_SEC * dt;
        }
        if (dpadUpPressed) {
            shootAnglePosition += SHOOT_ANGLE_DPAD_FINE_STEP;
        }
        if (dpadDownPressed) {
            shootAnglePosition -= SHOOT_ANGLE_DPAD_FINE_STEP;
        }

        shootAnglePosition = clamp(shootAnglePosition, SHOOT_ANGLE_MIN_POS, SHOOT_ANGLE_MAX_POS);
        shoot_angle.setPosition(shootAnglePosition);
        shootAngleMode = "Manual";
    }

    private double distanceToShootAngleServo(double distanceM) {
        double pos = REFERENCE_SERVO_POS + (distanceM - REFERENCE_DISTANCE_M) * SERVO_POS_PER_METER;
        return clamp(pos, SHOOT_ANGLE_MIN_POS, SHOOT_ANGLE_MAX_POS);
    }

    private double estimateHorizontalDistanceFromTy(double targetYDeg) {
        double heightDelta = APRILTAG_CENTER_HEIGHT_M - CAMERA_HEIGHT_M;
        double angleRad = Math.toRadians(CAMERA_PITCH_DEG + targetYDeg);
        double tan = Math.tan(angleRad);
        if (Math.abs(tan) < 0.001) {
            return Double.NaN;
        }

        double distance = heightDelta / tan;
        return distance > 0.0 ? distance : Double.NaN;
    }

    private void updateShotDistanceFromTag(double targetYDeg) {
        rawTag20DistanceM = estimateHorizontalDistanceFromTy(targetYDeg);
        correctedTag20DistanceM = Double.isNaN(rawTag20DistanceM)
                ? Double.NaN
                : rawTag20DistanceM * DISTANCE_SCALE + DISTANCE_OFFSET_M;
        if (Double.isNaN(correctedTag20DistanceM)) {
            return;
        }

        double alpha = clamp(DISTANCE_FILTER_ALPHA, 0.0, 1.0);
        if (Double.isNaN(filteredTag20DistanceM)) {
            filteredTag20DistanceM = correctedTag20DistanceM;
        } else {
            filteredTag20DistanceM = alpha * correctedTag20DistanceM
                    + (1.0 - alpha) * filteredTag20DistanceM;
        }
    }

    private LLResultTypes.FiducialResult findAprilTag20() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        if (result.getStaleness() > TAG_RESULT_TIMEOUT_MS) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 20) {
                return fiducial;
            }
        }

        return null;
    }

    private void resetAimPid() {
        aimIntegral = 0.0;
        lastAimErrorDeg = 0.0;
        aimDerivativeDegPerSec = 0.0;
        hasLastAimError = false;
        aimPTerm = 0.0;
        aimITerm = 0.0;
        aimDTerm = 0.0;
        aimErrorDeg = 0.0;
    }

    private void updateTelemetry() {
        telemetry.addData("Auto Aim", autoAimActive ? "RB Held" : "Off");
        telemetry.addData("AprilTag 20", tag20Visible ? "Visible" : "Not Found");
        telemetry.addData("Aligned", alignedToTag20);
        telemetry.addData("Tag20 tx/ty", "%.2f / %.2f deg", tag20TxDeg, tag20TyDeg);
        telemetry.addData("Tag20 area", "%.2f", tag20Area);
        telemetry.addData("Tag20 distance", "raw=%.2f cal=%.2f filt=%.2f m",
                rawTag20DistanceM, correctedTag20DistanceM, filteredTag20DistanceM);
        telemetry.addData("Shoot angle", "mode=%s pos=%.3f", shootAngleMode, shootAnglePosition);
        telemetry.addData("Shoot ref", "%.2f m -> %.3f slope=%.3f",
                REFERENCE_DISTANCE_M, REFERENCE_SERVO_POS, SERVO_POS_PER_METER);
        telemetry.addData("Turret power", "%.3f", turretPower);
        telemetry.addData("PID Error", "%.2f deg", aimErrorDeg);
        telemetry.addData("P", "%.4f", aimPTerm);
        telemetry.addData("I", "%.4f", aimITerm);
        telemetry.addData("D", "%.4f", aimDTerm);
        telemetry.addData("block status", blockstatus);
        telemetry.addData("IMU", imuOk ? "OK" : "NO");
        telemetry.addData("Drive yaw/target", "%.1f / %.1f deg", driveYawDeg, driveHeadingTargetDeg);
        telemetry.addData("Drive IMU correction", "err=%.2f power=%.3f",
                driveHeadingErrorDeg, driveImuCorrection);
        telemetry.addData("Reverse feed", gamepad1.dpad_left ? "HELD" : "OFF");
        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) > Math.abs(deadband) ? value : 0.0;
    }

    private void setIntakePower(double power) {
        intake_front.setPower(power);
        intake_midL.setPower(power);
        intake_midR.setPower(power);
    }

    private double updateImuDriveCorrection(double forward, double pan, double rotateInput) {
        if (!ENABLE_IMU_DRIVE_CORRECTION || !imuOk) {
            if (imuOk) {
                driveYawDeg = getYawDeg();
                driveHeadingTargetDeg = driveYawDeg;
            }
            driveHeadingErrorDeg = 0.0;
            driveImuCorrection = 0.0;
            return rotateInput;
        }

        driveYawDeg = getYawDeg();
        boolean driverRotating = Math.abs(rotateInput) > Math.abs(DRIVE_ROTATE_DEADBAND);
        boolean translating = Math.hypot(forward, pan) > Math.abs(DRIVE_TRANSLATE_DEADBAND);

        if (driverRotating || !translating) {
            driveHeadingTargetDeg = driveYawDeg;
            driveHeadingErrorDeg = 0.0;
            driveImuCorrection = 0.0;
            return rotateInput;
        }

        driveHeadingErrorDeg = wrapDegrees(driveHeadingTargetDeg - driveYawDeg);
        driveImuCorrection = clamp(driveHeadingErrorDeg * DRIVE_IMU_KP,
                -Math.abs(DRIVE_IMU_MAX_CORRECTION), Math.abs(DRIVE_IMU_MAX_CORRECTION));
        return driveImuCorrection;
    }

    private double getYawDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double wrapDegrees(double degrees) {
        while (degrees > 180.0) {
            degrees -= 360.0;
        }
        while (degrees <= -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    private void initIMU() {
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
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                );

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
    }

}
