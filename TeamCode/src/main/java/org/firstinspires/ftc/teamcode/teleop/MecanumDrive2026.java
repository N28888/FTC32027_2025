package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumDrive2026", group = "TeleOp1")
public class MecanumDrive2026 extends OpMode {
    boolean shootMotorStatus = false;
    boolean IntakeMotorStatus = false;
    boolean pullUpStatus = false;

    private boolean lastIntakeButton = false;// 现在：D-pad Up
    private boolean lastPullUpButton = false;   // 现在：Left Bumper
    private boolean lastRightBumper = false;    // Shoot Motor（不变）

    // --- 射球电机“预反转” ---
    private boolean shooterReversing = false;
    private ElapsedTime shooterReverseTimer = new ElapsedTime();

    // --- PullUp 前轮“预反转” ---
    private boolean pullupFrontReversing = false;
    private ElapsedTime pullupFrontReverseTimer = new ElapsedTime();

    // --- D-pad Down 自动前后移动 ---
    private boolean autoMoveRunning = false;
    private int autoMoveStage = 0;              // 0=未运行, 1=前进, 2=后退
    private ElapsedTime autoMoveTimer = new ElapsedTime();
    private boolean lastDpadDown = false;

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    // 左射球用 DcMotorEx 做恒速控制
    private DcMotorEx shootMotorL;
    // 右射球用普通 DcMotor，跟随左边功率
    private DcMotor shootMotorR;

    private DcMotor intake;

    private CRServo pullUp_back;
    private CRServo pullUp_mid;
    private CRServo pullUp_frontL;
    private CRServo pullUp_frontR;

    @Override
    public void init() {
        shootMotorStatus = false;
        IntakeMotorStatus = false;
        pullUpStatus = false;

        shooterReversing = false;
        pullupFrontReversing = false;
        autoMoveRunning = false;
        autoMoveStage = 0;

        // --- Hardware ---
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftfront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightfront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftback");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightback");

        // 左射球：DcMotorEx，用 setVelocity
        shootMotorL = hardwareMap.get(DcMotorEx.class, "shootL");
        // 右射球：普通 DcMotor，用 setPower 跟随
        shootMotorR = hardwareMap.get(DcMotor.class, "shootR");

        intake      = hardwareMap.get(DcMotor.class, "intake");

        pullUp_back   = hardwareMap.get(CRServo.class, "pullup_back");
        pullUp_frontL = hardwareMap.get(CRServo.class, "pullup_frontL");
        pullUp_frontR = hardwareMap.get(CRServo.class, "pullup_frontR");
        pullUp_mid    = hardwareMap.get(CRServo.class, "pullup_mid");

        // --- Directions ---
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        shootMotorL.setDirection(DcMotor.Direction.REVERSE);
        shootMotorR.setDirection(DcMotor.Direction.FORWARD);

        pullUp_back.setDirection(CRServo.Direction.REVERSE);
        pullUp_frontR.setDirection(CRServo.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 左射球使用编码器 + 速度模式 ---
        shootMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // 右射球不用编码器控制，普通功率模式
        shootMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 初始全部停止
        shootMotorL.setPower(0);
        shootMotorR.setPower(0);
        intake.setPower(0);
        pullUp_back.setPower(0);
        pullUp_frontL.setPower(0);
        pullUp_frontR.setPower(0);
        pullUp_mid.setPower(0);
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

        // -------- D-pad Down 自动前后移动 --------
        boolean dpadDownPressed = gamepad1.dpad_down && !lastDpadDown;
        lastDpadDown = gamepad1.dpad_down;

        if (dpadDownPressed && !autoMoveRunning) {
            autoMoveRunning = true;
            autoMoveStage = 1;           // 从前进开始
            autoMoveTimer.reset();
        }

        if (autoMoveRunning) {
            double p = 1;              // 自动移动速度（可调）
            double cmd = 0.0;

            if (autoMoveStage == 1) {
                // 阶段 1：前进 0.5 秒
                if (autoMoveTimer.seconds() < 0.5) {
                    cmd = p;
                } else {
                    autoMoveStage = 2;
                    autoMoveTimer.reset();
                    cmd = 0.0;           // 这一帧先停一下
                }
            } else if (autoMoveStage == 2) {
                // 阶段 2：后退 0.5 秒
                if (autoMoveTimer.seconds() < 0.5) {
                    cmd = -p;
                } else {
                    autoMoveRunning = false;
                    autoMoveStage = 0;
                    cmd = 0.0;           // 动作完成后停车
                }
            }

            // 自动动作时覆盖四个轮子的输出
            lf = cmd;
            rf = cmd;
            lb = cmd;
            rb = cmd;
        }

        // 最终把底盘功率写到电机
        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(rf);
        leftBackMotor.setPower(lb);
        rightBackMotor.setPower(rb);

        // -------- SHOOTER（左恒速 + 预反转，右跟随） --------
        boolean shootPressed = gamepad1.right_bumper && !lastRightBumper;
        lastRightBumper = gamepad1.right_bumper;

        if (shootPressed) {
            boolean prev = shootMotorStatus;
            shootMotorStatus = !shootMotorStatus;

            if (!prev && shootMotorStatus) {
                // 从关 -> 开：启动预反转阶段
                shooterReversing = true;
                shooterReverseTimer.reset();
            } else if (!shootMotorStatus) {
                // 关掉
                shooterReversing = false;
            }
        }

        // 左射球目标转速（tick/s），需要现场微调
        double targetVelocity   = 1000;  // 正常发射转速
        double reverseVelocity  = 1000;  // 预反转转速（绝对值）
        double shootReverseTime = 0.15;  // 预反转持续时间（秒）

        if (shootMotorStatus) {
            if (shooterReversing && shooterReverseTimer.seconds() < shootReverseTime) {
                // 预反转阶段：左边反转
                shootMotorL.setVelocity(-reverseVelocity);
            } else {
                // 恒速正转阶段
                shooterReversing = false;
                shootMotorL.setVelocity(targetVelocity);
            }
        } else {
            shooterReversing = false;
            shootMotorL.setVelocity(0);
        }

        // 右射球：简单跟随左射球当前功率
        double leftAppliedPower = shootMotorL.getPower();
        shootMotorR.setPower(leftAppliedPower);

        // -------- INTAKE （现在：D-pad UP）--------
        boolean intakePressed = gamepad1.dpad_up && !lastIntakeButton;
        lastIntakeButton = gamepad1.dpad_up;

        if (intakePressed) {
            IntakeMotorStatus = !IntakeMotorStatus;
        }

        intake.setPower(IntakeMotorStatus ? 1.0 : 0.0);

        // -------- PULLUP （现在：LEFT BUMPER，前轮带“预反转”）--------
        boolean pullUpPressed = gamepad1.left_bumper && !lastPullUpButton;
        lastPullUpButton = gamepad1.left_bumper;

        if (pullUpPressed) {
            boolean prev = pullUpStatus;
            pullUpStatus = !pullUpStatus;

            // 在 shooter 不转 且 pullUp 刚从关->开 时，前两个 CRServo 先反转一小会
            if (!prev && pullUpStatus && !shootMotorStatus) {
                pullupFrontReversing = true;
                pullupFrontReverseTimer.reset();
            }
        }

        double pullPower = 0.7;
        double frontReverseTime = 0.15;

        // 后面两个 & 中间：正常上拉
        pullUp_back.setPower(pullUpStatus ? pullPower : 0);
        pullUp_mid.setPower(pullUp_back.getPower());

        // 前两个：可能先短暂反转，然后再跟随 back
        double frontOutput;
        if (pullupFrontReversing && pullupFrontReverseTimer.seconds() < frontReverseTime) {
            frontOutput = -pullPower;  // 预反转
        } else {
            pullupFrontReversing = false;
            frontOutput = pullUp_back.getPower();
        }

        pullUp_frontL.setPower(frontOutput);
        pullUp_frontR.setPower(frontOutput);

        // -------- TELEMETRY --------
        telemetry.addData("Shooter", shootMotorStatus ? "ON" : "OFF");
        telemetry.addData("Shooter Reversing", shooterReversing);
        telemetry.addData("Shooter L vel", shootMotorL.getVelocity());
        telemetry.addData("Shooter L power", shootMotorL.getPower());
        telemetry.addData("Shooter R power", shootMotorR.getPower());

        telemetry.addData("PullUp", pullUpStatus ? "ON" : "OFF");

        telemetry.addData("Intake", IntakeMotorStatus ? "ON" : "OFF");

        telemetry.addData("AutoMove Running", autoMoveRunning);
        telemetry.addData("AutoMove Stage", autoMoveStage);

        telemetry.update();
    }

}
