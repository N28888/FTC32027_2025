package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto_Test", group = "Auto1")
public class Auto_Test extends LinearOpMode {

    // --------- Drive ---------
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // --------- Shooter ---------
    private DcMotorEx shootMotorL;
    private DcMotor shootMotorR;

    // --------- Intake ---------
    private DcMotor intake;

    // --------- PullUp CRServos (4) ---------
    private CRServo pullUp_back;
    private CRServo pullUp_mid;
    private CRServo pullUp_frontL;
    private CRServo pullUp_frontR;

    @Override
    public void runOpMode() {

        // ================== 硬件映射 ==================
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


        // ================== 方向：完全对齐 TeleOp ==================
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        shootMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotorR.setDirection(DcMotorSimple.Direction.FORWARD);

        pullUp_back.setDirection(CRServo.Direction.REVERSE);
        pullUp_frontR.setDirection(CRServo.Direction.REVERSE);


        // ================== Brake ==================
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // ================== Shooter 模式 ==================
        shootMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // ================== 初始全部停止 ==================
        setDrivePower(0);

        shootMotorL.setVelocity(0);
        shootMotorR.setPower(0);

        intake.setPower(0);

        setPullUpPower(0);


        telemetry.addLine("AUTO READY");
        telemetry.update();


        waitForStart();
        if (isStopRequested()) return;

        // =================================================
        //                  AUTO SEQUENCE
        // =================================================

        // 1️⃣ 后退 1 秒
        driveStraight(-1.0, 850);
        sleep(500);

        // 2️⃣ 启动射球
        startShooterWithPreReverse();
        sleep(3000);   // 等枪口转速稳定

        // 3️⃣ 拉球 / 上球
        runPullUp(550);
        pausePullUp(2000);
        runPullUp(600);

        // 4️⃣ 停止全部
        stopShooter();
        setPullUpPower(0);
        intake.setPower(0);
        setDrivePower(0);
    }

    // =================================================
    //                      DRIVE
    // =================================================

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

    // =================================================
    //                      SHOOTER
    // =================================================

    private void startShooterWithPreReverse() {

        double targetVelocity  = 1000;
        double reverseVelocity = 1000;
        long   reverseTimeMs  = 150;

        // 预反转
        shootMotorL.setVelocity(-reverseVelocity);
        sleep(reverseTimeMs);

        // 恒速正转
        shootMotorL.setVelocity(targetVelocity);

        // 右边跟随
        shootMotorR.setPower(shootMotorL.getPower());
    }

    private void stopShooter() {
        shootMotorL.setVelocity(0);
        shootMotorR.setPower(0);
    }

    // =================================================
    //                      PULL UP
    // =================================================

    private void setPullUpPower(double p) {
        pullUp_back.setPower(p);
        pullUp_mid.setPower(p);
        pullUp_frontL.setPower(p);
        pullUp_frontR.setPower(p);
    }

    private void pausePullUp(long pauseTimeMs){
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
}
