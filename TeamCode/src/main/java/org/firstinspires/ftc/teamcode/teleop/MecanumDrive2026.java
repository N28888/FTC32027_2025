package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumDrive2026", group = "TeleOp4")
public class MecanumDrive2026 extends OpMode {
    boolean shootMotorStatus = false;
    boolean IntakeMotorStatus = false;
    boolean pullUpStatus = false;

    private boolean lastIntakeButton = false;   // 现在：D-pad Up
    private boolean lastPullUpButton = false;   // 现在：Left Bumper
    private boolean lastRightBumper = false;    // Shoot Motor（不变）

    // --- 射球电机“预反转” ---
    private boolean shooterReversing = false;
    private ElapsedTime shooterReverseTimer = new ElapsedTime();

    // --- PullUp 前轮“预反转” ---
    private boolean pullupFrontReversing = false;
    private ElapsedTime pullupFrontReverseTimer = new ElapsedTime();

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor shootMotorL;
    private DcMotor shootMotorR;
    private DcMotor intake;

    private CRServo pullUp_back;
    private CRServo pullUp_mid;
    private CRServo pullUp_frontL;
    private CRServo pullUp_frontR;

    @Override
    public void init() {
        shootMotorStatus = false;
        IntakeMotorStatus = false;
        pullUpStatus = false;

        shooterReversing = false;
        pullupFrontReversing = false;

        // --- Hardware ---
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightfront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftback");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightback");

        shootMotorL = hardwareMap.get(DcMotor.class, "shootL");
        shootMotorR = hardwareMap.get(DcMotor.class, "shootR");
        intake = hardwareMap.get(DcMotor.class, "intake");

        pullUp_back   = hardwareMap.get(CRServo.class, "pullup_back");
        pullUp_frontL= hardwareMap.get(CRServo.class, "pullup_frontL");
        pullUp_frontR= hardwareMap.get(CRServo.class, "pullup_frontR");
        pullUp_mid   = hardwareMap.get(CRServo.class, "pullup_mid");

        // --- Directions ---
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

        shootMotorL.setPower(0.65);
        shootMotorR.setPower(0.65);
        intake.setPower(0);
        pullUp_back.setPower(0);
        pullUp_frontL.setPower(0);
        pullUp_frontR.setPower(0);
        pullUp_mid.setPower(0);
    }

    @Override
    public void loop() {
        // -------- DRIVE --------
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
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(rf);
        leftBackMotor.setPower(lb);
        rightBackMotor.setPower(rb);

        // -------- SHOOTER --------
        boolean shootPressed = gamepad1.right_bumper && !lastRightBumper;
        lastRightBumper = gamepad1.right_bumper;

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

        double shootPower = 0.6;
        double shootReversePower = -0.25;
        double shootReverseTime = 0.15;

        if (shootMotorStatus) {
            if (shooterReversing && shooterReverseTimer.seconds() < shootReverseTime) {
                shootMotorL.setPower(shootReversePower);
            } else {
                shooterReversing = false;
                shootMotorL.setPower(shootPower);
            }
        } else {
            shooterReversing = false;
            shootMotorL.setPower(0);
        }

        shootMotorR.setPower(shootMotorL.getPower());

        // -------- INTAKE （现在：D-pad UP）--------
        boolean intakePressed = gamepad1.dpad_up && !lastIntakeButton;
        lastIntakeButton = gamepad1.dpad_up;

        if (intakePressed) {
            IntakeMotorStatus = !IntakeMotorStatus;
        }

        intake.setPower(IntakeMotorStatus ? 1.0 : 0.0);

        // -------- PULLUP （现在：LEFT BUMPER）--------
        boolean pullUpPressed = gamepad1.left_bumper && !lastPullUpButton;
        lastPullUpButton = gamepad1.left_bumper;

        if (pullUpPressed) {
            boolean prev = pullUpStatus;
            pullUpStatus = !pullUpStatus;

            if (!prev && pullUpStatus && !shootMotorStatus) {
                pullupFrontReversing = true;
                pullupFrontReverseTimer.reset();
            }
        }

        double pullPower = 0.7;
        double frontReversePower = -0.7;
        double frontReverseTime = 0.15;

        pullUp_back.setPower(pullUpStatus ? pullPower : 0);
        pullUp_mid.setPower(pullUp_back.getPower());

        double frontOutput;

        if (pullupFrontReversing && pullupFrontReverseTimer.seconds() < frontReverseTime) {
            frontOutput = frontReversePower;
        } else {
            pullupFrontReversing = false;
            frontOutput = pullUp_back.getPower();
        }

        pullUp_frontL.setPower(frontOutput);
        pullUp_frontR.setPower(frontOutput);

        // -------- TELEMETRY --------
        telemetry.addData("Shooter", shootMotorStatus ? "ON" : "OFF");
        telemetry.addData("PullUp", pullUpStatus ? "ON" : "OFF");
        telemetry.addData("Intake", IntakeMotorStatus ? "ON" : "OFF");
        telemetry.addData("Shooter Reversing", shooterReversing);
        telemetry.addData("PullUp Front Reversing", pullupFrontReversing);
        telemetry.update();
    }

    public void forwardBackward(){
        double forward = -1;


    }
}
