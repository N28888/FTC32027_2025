package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name="TankDrive2026_auto", group="Auto1")
public class Auto_2026_Right extends LinearOpMode {
    DcMotor leftFront, rightFront, leftBack, rightBack, shootL, shootR;
    Servo feed;
    @Override public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        shootL = hardwareMap.dcMotor.get("shootL");
        shootR = hardwareMap.dcMotor.get("shootR");
        feed = hardwareMap.servo.get("feed");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.REVERSE);
        shootR.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        setAll(1); sleep(1000);
        setAll(0); sleep(150);
        shootL.setPower(1);
        shootR.setPower(shootL.getPower());
        for (int i = 0; i <= 3; i++) {
            feed.setPosition(0);
            feed.setPosition(1);
        }
        setAll(0);

    }
    void setAll(double p){
        leftFront.setPower(p);
        rightFront.setPower(p);
        leftBack.setPower(p);
        rightBack.setPower(p);
    }
    void turnRight(){
        leftFront.setPower(1);
        leftBack.setPower(1);
        rightFront.setPower(-1);
        rightBack.setPower(-1);
    }
}
