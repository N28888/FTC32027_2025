package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TankDrive2026", group = "TeleOp3")
public class TankDrive2026 extends OpMode {
    boolean shootMotorStatus;
    boolean IntakeServoStatus;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor shootMotor;
    private CRServo intakeL;
    private CRServo intakeR;


    @Override
    public void init() {
        shootMotorStatus = false;
        IntakeServoStatus = false;

        // --- Hardware Map ---

        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightfront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftback");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightback");
        shootMotor = hardwareMap.get(DcMotor.class, "shoot");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");


        // Set motor directions

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeL.setDirection(CRServo.Direction.FORWARD);
        intakeR.setDirection(CRServo.Direction.REVERSE);


        // Set motor behavior when power is zero (BRAKE stops actively, FLOAT coasts)
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Motor Directions MUST be Verified!");
        telemetry.update();

        // ---Other Init---
        shootMotor.setPower(0);
    }

    @Override
    public void loop() {
        // --- Gamepad Input ---
        double forward = -gamepad1.right_stick_y;
        double pan = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        // --- Mecanum Drive Calculation ---
        double leftFrontPower = forward + pan + rotate;
        double rightFrontPower = forward - pan - rotate;
        double leftBackPower = forward - pan + rotate;
        double rightBackPower = forward + pan - rotate;

        // --- Normalize Motor Powers ---
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // --- Set Drive Motor Powers ---
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);

        // --- Set Shoot Motor Powers ---
        if (gamepad2.right_bumper && !shootMotorStatus){
            shootMotor.setPower(1);
            shootMotorStatus = true;
        } else if (gamepad2.right_bumper && shootMotorStatus) {
            shootMotor.setPower(0);
            shootMotorStatus = false;
        }

        // --- Ball Pickup Servo
        if (gamepad2.right_bumper && !IntakeServoStatus) {
            double intakePower = 1;
            intakeL.setPower(intakePower);
            intakeR.setPower(intakePower);
            IntakeServoStatus = true;
        } else if (gamepad2.right_bumper && IntakeServoStatus) {
            intakeL.setPower(0);
            intakeR.setPower(0);
            IntakeServoStatus = false;
        }


        // --- Telemetry ---
        telemetry.addData("Input Y (Forward)", "%.2f", forward);
        telemetry.addData("Input X (Pan)", "%.2f", pan);
        telemetry.addData("Input R (Rotate)", "%.2f", rotate);
        telemetry.addData("---", "---");
        telemetry.addData("Calculated LF Power", "%.2f", leftFrontPower);
        telemetry.addData("Calculated RF Power", "%.2f", rightFrontPower);
        telemetry.addData("Calculated LB Power", "%.2f", leftBackPower);
        telemetry.addData("Calculated RB Power", "%.2f", rightBackPower);
        //Display actual power being sent (can differ due to internal limits/control loops)
        telemetry.addData("Actual LF Power", "%.2f", leftFrontMotor.getPower());
        telemetry.addData("Actual RF Power", "%.2f", rightFrontMotor.getPower());
        telemetry.addData("Actual LB Power", "%.2f", leftBackMotor.getPower());
        telemetry.addData("Actual RB Power", "%.2f", rightBackMotor.getPower());
        telemetry.addData("Shoot Left Power", "%.2f", shootMotor.getPower());
        telemetry.update();

    }
}
