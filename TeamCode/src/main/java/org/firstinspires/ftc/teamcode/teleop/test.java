package org.firstinspires.ftc.teamcode.teleop;

import androidx.lifecycle.GenericLifecycleObserver;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "test", group = "TeleOp4")
public class test extends OpMode {

    private CRServo intakeL;


    @Override
    public void init() {
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
    }

    @Override
    public void loop() {
//        if (gamepad2.right_bumper && !IntakeServoStatus) {;
//            intakeL.setPower(1);
//            IntakeServoStatus = true;
//        }
//
//        if (gamepad2.right_bumper && IntakeServoStatus) {
//            intakeL.setPower(-1);
//            IntakeServoStatus = false;
//        }

        if (gamepad2.left_bumper) {
            boolean IntakeServoStatus = false;
            intakeL.setPower(1);
            IntakeServoStatus = true;
        }

        // --- Telemetry ---
        telemetry.addData("Servo", "%.2f", intakeL.getPower());
        telemetry.update();

    }
}
