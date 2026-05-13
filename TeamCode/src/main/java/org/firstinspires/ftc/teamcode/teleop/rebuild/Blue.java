package org.firstinspires.ftc.teamcode.teleop.rebuild;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name="Rebuild Blue",group = "TeleOp3")
public class Blue extends LinearOpMode {

    //----- 硬件定义 -----
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    private DcMotor intake_front;
    private DcMotorEx shoot;

    private CRServo vertical_right;
    private CRServo vertical_left;
    private CRServo intake_midR;
    private CRServo intake_midL;
    private CRServo turret;
    private CRServo shoot_angle;
    private CRServo intake_back;

    private Limelight3A limelight;

    private IMU imu;

    //----- 软件内部定义 -----

    private boolean imuOk = false;
    private boolean lastShooterToggleBtn = false;
    private boolean shootMotorStatus = false;
    private boolean lastIntakeButton = false;
    private boolean IntakeMotorStatus = false;
    private boolean lastVerticalButton = false;
    private boolean VerticalServoStatus = false;

    // ----- 动力变量 -----
    private double IntakePower = 1.0;
    private double VerticalPower = 1.0;
    private double ShootVelocity = 6000;

    @Override
    public void runOpMode() {
        //----- 硬件识别 -----
        leftFrontMotor = hardwareMap.get(DcMotor.class,"LF Drive");
        leftBackMotor = hardwareMap.get(DcMotor.class, "LB Drive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RF Drive");
        rightBackMotor = hardwareMap.get(DcMotor.class,"RB Drive");

        shoot = hardwareMap.get(DcMotorEx.class,"shoot");
        intake_front = hardwareMap.get(DcMotor.class,"intake front");

        turret = hardwareMap.get(CRServo.class,"turret rotate");
        intake_back = hardwareMap.get(CRServo.class, "intake back");
        vertical_left = hardwareMap.get(CRServo.class, "vertical left");
        vertical_right = hardwareMap.get(CRServo.class, "vertical right");
        intake_midL = hardwareMap.get(CRServo.class, "intake middle left");
        intake_midR = hardwareMap.get(CRServo.class, "intake middle right");
        shoot_angle = hardwareMap.get(CRServo.class, "shoot angle");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //----- Motor 方向 ------
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        shoot.setDirection(DcMotor.Direction.FORWARD);
        intake_front.setDirection(DcMotor.Direction.REVERSE);

        turret.setDirection(CRServo.Direction.FORWARD);
        intake_back.setDirection(CRServo.Direction.REVERSE);
        vertical_left.setDirection(CRServo.Direction.FORWARD);
        vertical_right.setDirection(CRServo.Direction.REVERSE);
        intake_midL.setDirection(CRServo.Direction.FORWARD);
        intake_midR.setDirection(CRServo.Direction.FORWARD);
        shoot_angle.setDirection(CRServo.Direction.FORWARD);

        //----- 零动力锁死 -----
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //----- 射球电机行为 -----
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        initIMU();

        waitForStart();
        while (opModeIsActive()){
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
            intake_back.setPower(IntakeMotorStatus ? IntakePower : 0.0);

            //----- vertical -----
            boolean verticalPressed = gamepad1.left_bumper && !lastVerticalButton;
            lastVerticalButton = gamepad1.left_bumper;

            if (verticalPressed) VerticalServoStatus = !VerticalServoStatus;
            vertical_left.setPower(VerticalServoStatus ? VerticalPower : 0.0);
            vertical_right.setPower(VerticalServoStatus ? VerticalPower : 0.0);

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
        }
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
