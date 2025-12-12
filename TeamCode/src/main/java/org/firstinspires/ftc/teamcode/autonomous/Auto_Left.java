package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto_Left", group = "Auto1")
public class Auto_Left extends LinearOpMode {

    // Drive
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Shooter
    private DcMotorEx shootMotorL;
    private DcMotor shootMotorR;

    // Intake
    private DcMotor intake;

    // PullUp CRServo
    private CRServo pullUp_back;
    private CRServo pullUp_mid;
    private CRServo pullUp_frontL;
    private CRServo pullUp_frontR;

    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotor.class, "leftfront");
        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftback");
        rightBack  = hardwareMap.get(DcMotor.class, "rightback");

        shootMotorL = hardwareMap.get(DcMotorEx.class, "shootL");
        shootMotorR = hardwareMap.get(DcMotor.class, "shootR");

        intake = hardwareMap.get(DcMotor.class, "intake");

        pullUp_back   = hardwareMap.get(CRServo.class, "pullup_back");
        pullUp_mid    = hardwareMap.get(CRServo.class, "pullup_mid");
        pullUp_frontL = hardwareMap.get(CRServo.class, "pullup_frontL");
        pullUp_frontR = hardwareMap.get(CRServo.class, "pullup_frontR");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        shootMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotorR.setDirection(DcMotorSimple.Direction.FORWARD);

        pullUp_back.setDirection(CRServo.Direction.REVERSE);
        pullUp_frontR.setDirection(CRServo.Direction.REVERSE);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shootMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --------------------------
        setDrivePower(0);

        shootMotorL.setVelocity(0);
        shootMotorR.setPower(0);

        intake.setPower(0);

        setPullUpPower(0);


        telemetry.addLine("AUTO READY");
        telemetry.update();


        waitForStart();
        if (isStopRequested()) return;

        // Move back
        driveStraight(-1, 850);
        sleep(500);

        // shootMotor start
        startShooterWithPreReverse();
        intake.setPower(1);
        sleep(3000);

        // PullUp start
        runPullUp(580);
        pausePullUp(2000);
        runPullUp(600);

        inertialPullUp();
        adjustDirectionL(0.2,200);
        sleep(500);

        runPullUp(600);

        adjustDirectionR(0.3,1000);
        sleep(500);
        driveStraight(-1,500);

        // Stop all
        stopShooter();
        setPullUpPower(0);
        intake.setPower(0);
        setDrivePower(0);
    }

    // Mecanum Drive

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

    // Shooter

    private void startShooterWithPreReverse() {

        double targetVelocity = 1000;
        double reverseVelocity = 1000;
        long reverseTimeMs = 150;

        shootMotorL.setVelocity(-reverseVelocity);
        sleep(reverseTimeMs);

        shootMotorL.setVelocity(targetVelocity);
        shootMotorR.setPower(shootMotorL.getPower());
    }

    private void stopShooter() {
        shootMotorL.setVelocity(0);
        shootMotorR.setPower(0);
    }

    // PullUp CRServos

    private void setPullUpPower(double p) {
        pullUp_back.setPower(p);
        pullUp_mid.setPower(p);
        pullUp_frontL.setPower(p);
        pullUp_frontR.setPower(p);
    }

    private void pausePullUp(long pauseTimeMs) {
        double PausePower = 0;
        sleep(pauseTimeMs);
        setPullUpPower(PausePower);
    }


    private void runPullUp(long totalTimeMs) {

        double pullPower = 0.7;
        setPullUpPower(pullPower);
        sleep(totalTimeMs);
        setPullUpPower(0);

    }

    private void autoMoveForwardBack() {
        double p = 1.0;
        long durationMs = 500;

        // Move Forward
        setDrivePower(p);
        sleep(durationMs);

        // Move Backward
        setDrivePower(-p);
        sleep(durationMs+200);

        // Stop
        setDrivePower(0);
    }

    private void inertialPullUp(){
        double inertiaPower = 0.7;
        pullUp_back.setPower(inertiaPower);
        pullUp_mid.setPower(inertiaPower);
        autoMoveForwardBack();
        pullUp_back.setPower(0);
        pullUp_mid.setPower(0);
    }

    private void adjustDirectionL(double adjustPower, long TimeMs){

        leftFront.setPower(-adjustPower);
        leftBack.setPower(-adjustPower);
        rightFront.setPower(adjustPower);
        rightBack.setPower(adjustPower);
        sleep(TimeMs);
        setDrivePower(0);
    }

    private void adjustDirectionR(double adjustPower, long TimeMs){

        leftFront.setPower(adjustPower);
        leftBack.setPower(adjustPower);
        rightFront.setPower(-adjustPower);
        rightBack.setPower(-adjustPower);
        sleep(TimeMs);
        setDrivePower(0);
    }


}
