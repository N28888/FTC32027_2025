package org.firstinspires.ftc.teamcode.archive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "move_only", group = "Auto1")
public class move_only extends LinearOpMode {
    private DcMotor leftFront, rightFront, leftBack, rightBack;



    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setDrivePower(0);
        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;

        driveStraight(1, 300);
    }

        private void setDrivePower(double p) {
            leftFront.setPower(p);
            rightFront.setPower(p);
            leftBack.setPower(p);
            rightBack.setPower(p);
        }
        private void driveStraight(double p, long timeMs) {
            setDrivePower(p);
            sleep(timeMs);
            setDrivePower(0);
    }

}
