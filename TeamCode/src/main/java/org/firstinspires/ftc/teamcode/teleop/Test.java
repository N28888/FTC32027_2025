package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test", group = "TeleOp2")
public class Test extends OpMode {
    private DcMotor Motor;
    private Servo Servo;
    private CRServo CRservo;

    @Override
    public void init() {
        // --- Hardware init ---
        Motor = hardwareMap.get(DcMotor.class, "Motor");
        Servo = hardwareMap.get(Servo.class,"Servo");
        CRservo = hardwareMap.get(CRServo.class,"CRServo");
    }

    @Override
    public void loop() {
        // -------- DRIVE 输入 --------
        Motor.setPower(gamepad1.right_trigger);
        if (gamepad1.dpad_down) {
            Servo.setPosition(0.5);
        }
        if (gamepad1.dpad_up){
            Servo.setPosition(1);
        }
        CRservo.setPower(gamepad1.left_trigger);
    }
}
