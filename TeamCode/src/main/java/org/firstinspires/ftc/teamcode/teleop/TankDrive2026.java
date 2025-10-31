package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TankDrive2026", group = "TeleOp4")
public class TankDrive2026 extends OpMode {
    boolean shootMotorStatus = false;
    boolean IntakeMotorStatus = false;
    boolean pullUpStatus = false;
    private boolean lastLeftBumper = false; // Ball PickUp Motor
    private boolean lastRightBumper = false; // Ball shoot Motor
    private boolean lastdepad_up = false; // Pull Up CRServo

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor shootMotor;
    private DcMotor intake;
    private CRServo pullUp;



    @Override
    public void init() {
        shootMotorStatus = false;
        IntakeMotorStatus = false;
        pullUpStatus = false;



        // --- Hardware Map ---

        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightfront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftback");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightback");
        shootMotor = hardwareMap.get(DcMotor.class, "shoot");
        intake = hardwareMap.get(DcMotor.class, "intake");
        pullUp = hardwareMap.get(CRServo.class, "pullup");




        // Set motor directions

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setDirection(DcMotor.Direction.FORWARD);

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
        intake.setPower(0.0);
        pullUp.setPower(0.0);
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
//        if (gamepad2.right_bumper && !shootMotorStatus){
//            shootMotor.setPower(1);
//            shootMotorStatus = true;
//        } else if (gamepad2.right_bumper && shootMotorStatus) {
//            shootMotor.setPower(0);
//            shootMotorStatus = false;
//        }
        boolean pressedRBumper = gamepad2.right_bumper && !lastRightBumper;
        lastRightBumper = gamepad2.right_bumper;
        if (pressedRBumper) {
            shootMotorStatus = !shootMotorStatus; // 开/关切换
        }
        double shootPower = 1.0;
        shootMotor.setPower(shootMotorStatus ? shootPower : 0.0);




        // --- Ball Pickup Motor
        boolean pressedLBumper = gamepad2.left_bumper && !lastLeftBumper;
        lastLeftBumper = gamepad2.left_bumper;
        if (pressedLBumper) {
            IntakeMotorStatus = !IntakeMotorStatus; // 开/关切换
        }
        double intakePower = 1.0;
        intake.setPower(IntakeMotorStatus ? intakePower : 0.0);





        //-- Ball Pull Up CRServo
        boolean pressedDpadUp = gamepad2.dpad_up && !lastdepad_up;
        lastdepad_up = gamepad2.dpad_up;
        if (pressedDpadUp) {
            pullUpStatus = !pullUpStatus; // 开/关切换
        }
        double pullUpPower = 0.7;
        pullUp.setPower(pullUpStatus ? pullUpPower : 0.0);


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
        telemetry.addData("Intake Power", "%.2f", intake.getPower());
        telemetry.addData("Intake Status", IntakeMotorStatus ? "ON" : "OFF");
        telemetry.update();

    }
}
