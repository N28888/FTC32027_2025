//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@Autonomous(name="TankDrive2026_auto", group="Auto1")
//public class Auto_2026_Left extends LinearOpMode {
//    DcMotor leftFront, rightFront, leftBack, rightBack, shootL, shootR;
//    @Override public void runOpMode() {
//        leftFront = hardwareMap.dcMotor.get("leftFront");
//        rightFront = hardwareMap.dcMotor.get("rightFront");
//        leftBack = hardwareMap.dcMotor.get("leftBack");
//        rightBack = hardwareMap.dcMotor.get("rightBack");
//        shootL = hardwareMap.dcMotor.get("shootL");
//        shootR = hardwareMap.dcMotor.get("shootR");
//
//
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        shootL.setDirection(DcMotorSimple.Direction.REVERSE);
//        shootR.setDirection(DcMotorSimple.Direction.FORWARD);
//        if (isStopRequested()) return;
//
//        shootL.setPower(1);
//        shootR.setPower(shootL.getPower());
//
//        setAll(1); sleep(3000);
//        setAll(0); sleep(150);
//        // in-place CW turn
//        leftFront.setPower(1);
//        leftBack.setPower(1);
//        rightFront.setPower(-1);
//        rightBack.setPower(-1);
//        sleep(600);
//        setAll(0);
//
//    }
//    void setAll(double p){
//        leftFront.setPower(p);
//        rightFront.setPower(p);
//        leftBack.setPower(p);
//        rightBack.setPower(p);
//    }
//}
