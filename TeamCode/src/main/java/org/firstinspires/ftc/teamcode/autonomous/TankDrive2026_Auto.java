package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="TankDrive2026_auto", group="Auto1")
public class TankDrive2026_Auto extends LinearOpMode {
    DcMotor leftFront, rightFront, leftBack, rightBack;
    @Override public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;

        setAll(0.5); sleep(1200); // drive forward ~1.2s
        setAll(0); sleep(150);
        // in-place CW turn
        leftFront.setPower(-0.4); leftBack.setPower(-0.4); rightFront.setPower(0.4); rightBack.setPower(0.4);
        sleep(600);
        setAll(0);
    }
    void setAll(double p){ leftFront.setPower(p); rightFront.setPower(p); leftBack.setPower(p); rightBack.setPower(p); }
}
