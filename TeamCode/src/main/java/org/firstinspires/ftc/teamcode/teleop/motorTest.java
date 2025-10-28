package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "motorTest", group = "TeleOp4")
public class motorTest extends OpMode {

    private DcMotor intakeL;

    // 开关状态：true=一直转，false=停止
    private boolean IntakeMotorStatus = false;

    // 边沿检测：记录上一次 left_bumper 的状态
    private boolean lastLeftBumper = false;

    @Override
    public void init() {
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");

        // 上电先停止
        intakeL.setPower(0.0);

        telemetry.addLine("Init done. Left bumper: toggle intake ON/OFF.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 边沿触发：从未按 -> 按下 的那个瞬间，才翻转一次状态
        boolean pressed = gamepad2.left_bumper && !lastLeftBumper;
        lastLeftBumper = gamepad2.left_bumper;

        if (pressed) {
            IntakeMotorStatus = !IntakeMotorStatus; // 开/关切换
        }

        // 根据当前状态持续输出功率（使用三元运算符）
        // 可调速度（0.0~1.0），需要反向可改成负数或在 init() 里 setDirection(REVERSE)
        double intakePower = 1.0;
        intakeL.setPower(IntakeMotorStatus ? intakePower : 0.0);

        // --- Telemetry ---
        telemetry.addData("Intake Power", "%.2f", intakeL.getPower());
        telemetry.addData("Intake Status", IntakeMotorStatus ? "ON" : "OFF");
        telemetry.addData("Left Bumper (held)", gamepad2.left_bumper);
        telemetry.update();
    }
}
