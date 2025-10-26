package org.firstinspires.ftc.teamcode.teleop;

import androidx.lifecycle.GenericLifecycleObserver;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Thread;

@TeleOp(name = "test", group = "TeleOp4")
public class test extends OpMode {

    private CRServo intakeL;
    boolean IntakeServoStatus = false;


    @Override
    public void init() {
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
    }

    @Override
    public void loop() {
        if (gamepad2.left_bumper && !IntakeServoStatus) {
//            boolean IntakeServoStatus = false;
            intakeL.setPower(1);
//            intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
            IntakeServoStatus = true;
//            Thread.sleep(2000);

        }
        else if (gamepad2.left_bumper && IntakeServoStatus) {
            intakeL.setPower(0);
            IntakeServoStatus = false;
        }

        // --- Telemetry ---
        telemetry.addData("Servo", "%.2f", intakeL.getPower());

        telemetry.addData("bumper", gamepad2.left_bumper);
        telemetry.update();

    }
}
