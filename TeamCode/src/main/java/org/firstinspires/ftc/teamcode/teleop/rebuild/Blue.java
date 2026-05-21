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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    //----- 硬件定义 -----
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    private DcMotor intake_front;
    private DcMotorEx shoot;

    private DcMotor vertical_right;
    private DcMotor vertical_left;
    private CRServo intake_midR;
    private CRServo intake_midL;
    private CRServo turret;
    private CRServo shoot_angle;

    private Limelight3A limelight;

    private IMU imu;

    //----- 软件内部定义 -----

    private boolean imuOk = false;
    private boolean lastShooterToggleBtn = false;
    private boolean shootMotorStatus = false;
    private boolean lastIntakeButton = false;
    private boolean IntakeMotorStatus = false;
    private boolean lastVerticalButton = false;
    private boolean VerticalMotorStatus = false;

    private boolean lastVerticalToggleButton = false;
    private boolean VerticalToggleMotorStatus = false;

    // ----- 动力变量 -----
    private double IntakePower = 1.0;
    private double VerticalPower = 1.0;
    private double ShootVelocity = 6000;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double tag20TxDeg = 0.0;
    private double tag20TyDeg = 0.0;
    private double tag20Area = 0.0;
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
        vertical_left = hardwareMap.get(DcMotor.class, "vertical left");
        vertical_right = hardwareMap.get(DcMotor.class, "vertical right");
        intake_midL = hardwareMap.get(CRServo.class, "intake middle left");
        intake_midR = hardwareMap.get(CRServo.class, "intake middle right");
        shoot_angle = hardwareMap.get(CRServo.class, "shoot angle");

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
        vertical_left.setDirection(CRServo.Direction.REVERSE);
        vertical_right.setDirection(CRServo.Direction.FORWARD);
        intake_midL.setDirection(CRServo.Direction.REVERSE);
        intake_midR.setDirection(CRServo.Direction.FORWARD);
        shoot_angle.setDirection(CRServo.Direction.FORWARD);

        //----- 零动力锁死 -----
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vertical_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //----- 射球电机行为 -----
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //initIMU();

        telemetry.addLine("Ready. Hold gamepad2 RB to aim AprilTag 20.");
        telemetry.addData("Dashboard D", D);
        telemetry.update();

        loopTimer.reset();
        waitForStart();
        while (opModeIsActive()){
            limelight.start();
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0.0 || dt > 0.25) {
                dt = 0.02;
            }

            //----- Drive -----
            double forward = -gamepad1.right_stick_y;
            double pan = gamepad1.right_stick_x;
            double rotate = gamepad1.left_stick_x;

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

            //----- intake -----
            boolean intakePressed = gamepad1.dpad_up && !lastIntakeButton;
            lastIntakeButton = gamepad1.dpad_up;

            if (intakePressed) IntakeMotorStatus = !IntakeMotorStatus;
            intake_front.setPower(IntakeMotorStatus ? IntakePower : 0.0);
            intake_midL.setPower(IntakeMotorStatus ? IntakePower : 0.0);
            intake_midR.setPower(IntakeMotorStatus ? IntakePower : 0.0);

            //----- vertical -----
            if (gamepad2.left_bumper){
                vertical_left.setPower(VerticalPower);
                vertical_right.setPower(VerticalPower);
            } else {
                vertical_right.setPower(0);
                vertical_left.setPower(0);
            }

            boolean verticaltoggle = gamepad1.dpad_right && !lastVerticalToggleButton;
            lastVerticalToggleButton = gamepad1.dpad_right;

            if (verticaltoggle) VerticalToggleMotorStatus = !VerticalToggleMotorStatus;
            {
                vertical_left.setPower(VerticalToggleMotorStatus ? 1:0);
                vertical_right.setPower(VerticalToggleMotorStatus ? 1:0);
            }

            //----- shoot -----
            boolean shootPressed = gamepad2.x && !lastShooterToggleBtn;
            lastShooterToggleBtn = gamepad2.x;

            if (shootPressed) {
                shootMotorStatus = !shootMotorStatus;
            }

            if (shootMotorStatus) {
                shoot.setVelocity(ShootVelocity);
            } else {
                shoot.setVelocity(0);
            }

            updateAprilTag20Aim(dt);
            updateTelemetry();

            //----- shoot angle -----
            shoot_angle.setPower(gamepad2.right_stick_y);
            turret.setPower(gamepad2.left_stick_x);
        }

        turret.setPower(0.0);
        limelight.stop();
    }

    private void updateAprilTag20Aim(double dt) {
        autoAimActive = gamepad2.right_bumper;
        LLResultTypes.FiducialResult tag20 = findAprilTag20();
        tag20Visible = tag20 != null;

        if (tag20Visible) {
            tag20TxDeg = tag20.getTargetXDegrees();
            tag20TyDeg = tag20.getTargetYDegrees();
            tag20Area = tag20.getTargetArea();
        } else {
            tag20TxDeg = 0.0;
            tag20TyDeg = 0.0;
            tag20Area = 0.0;
        }

        if (!autoAimActive || !tag20Visible) {
            resetAimPid();
            turretPower = 0.0;
            turret.setPower(0.0);
            alignedToTag20 = false;
            return;
        }

        aimErrorDeg = tag20TxDeg;

        double safeDt = Math.max(dt, 0.001);
        if (Math.abs(aimErrorDeg) <= Math.max(0.0, AIM_TOLERANCE_DEG)) {
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
        telemetry.addData("Turret power", "%.3f", turretPower);
        telemetry.addData("PID Error", "%.2f deg", aimErrorDeg);
        telemetry.addData("P", "%.4f", aimPTerm);
        telemetry.addData("I", "%.4f", aimITerm);
        telemetry.addData("D", "%.4f", aimDTerm);
        telemetry.update();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
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
