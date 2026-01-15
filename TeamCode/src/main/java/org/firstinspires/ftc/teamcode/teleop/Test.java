package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.IMU;


@TeleOp(name = "Test", group = "TeleOp2")
public class Test extends OpMode {
    // --- Mecanum Wheels ---
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private HuskyLens lens;
    private IMU imu;

    private VisionPortal visionPortal;




    @Override
    public void init() {
        // --- Hardware init ---
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightfront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftback");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightback");
        lens = hardwareMap.get(HuskyLens.class,"lens");
        imu = hardwareMap.get(IMU.class, "imu");

        // --- Direction ---
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // --- Zero Power Behavior ---
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    @Override
    public void loop() {
        // -------- DRIVE 输入 --------
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
    }
}
