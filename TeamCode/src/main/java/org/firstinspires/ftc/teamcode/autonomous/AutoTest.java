package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="AutoTest", group="Auto1")
public class AutoTest extends LinearOpMode {
    DcMotor leftFront;
    @Override public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;

        setAll(); sleep(500000); // drive forward ~1.2s
    }
    void setAll(){ leftFront.setPower(1); }
}
