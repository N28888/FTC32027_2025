package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDrive2026", group = "TeleOp3")
public class TankDrive2026 extends OpMode {
    boolean shootMotorStatus;
    boolean IntakeServoStatus;
    private DcMotor leftfrontMotor;
    private DcMotor rightfrontMotor;
    private DcMotor leftbackMotor;
    private DcMotor rightbackMotor;
    private DcMotor shootMotor;
    private Servo intakeL;
    private Servo intakeR;
    private double intakePower = 0.7;



    @Override
    public void init() {
        shootMotorStatus = false;
        IntakeServoStatus = false;

        // --- Hardware Map ---

        leftfrontMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightfrontMotor = hardwareMap.get(DcMotor.class, "rightfront");
        leftbackMotor = hardwareMap.get(DcMotor.class, "leftback");
        rightbackMotor = hardwareMap.get(DcMotor.class, "rightback");
        shootMotor = hardwareMap.get(DcMotor.class, "shoot");
        intakeL = (Servo) hardwareMap.get(CRServo.class, "intakeL");
        intakeR = (Servo) hardwareMap.get(CRServo.class, "intakeR");


        // Set motor directions

        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setDirection(DcMotor.Direction.FORWARD);


        // Set motor behavior when power is zero (BRAKE stops actively, FLOAT coasts)
        leftfrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Motor Directions MUST be Verified!");
        telemetry.update();

        // ---Other Init---
        shootMotor.setPower(0);
        intakeL.setPosition(0);
        intakeR.setPosition(0);
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
        leftfrontMotor.setPower(leftFrontPower);
        rightfrontMotor.setPower(rightFrontPower);
        leftbackMotor.setPower(leftBackPower);
        rightbackMotor.setPower(rightBackPower);

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
            intakeL.setPosition(0.8);  // 正转
            intakeR.setPosition(0.3);
            IntakeServoStatus = true;
        } else if (gamepad2.right_bumper && IntakeServoStatus) {
            intakeL.setPosition(0.5);  // 松开后停止
            intakeR.setPosition(0.5);
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
        telemetry.addData("Actual LF Power", "%.2f", leftfrontMotor.getPower());
        telemetry.addData("Actual RF Power", "%.2f", rightfrontMotor.getPower());
        telemetry.addData("Actual LB Power", "%.2f", leftbackMotor.getPower());
        telemetry.addData("Actual RB Power", "%.2f", rightbackMotor.getPower());
        telemetry.addData("Shoot Left Power", "%.2f", shootMotor.getPower());
        telemetry.update();
    }
}
