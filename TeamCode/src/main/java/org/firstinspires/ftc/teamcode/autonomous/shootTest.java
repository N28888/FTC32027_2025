package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="shootTest", group="Auto1")
public class shootTest extends LinearOpMode {
    DcMotor shootL, shootR;
    @Override public void runOpMode() {
        shootL = hardwareMap.dcMotor.get("shootL");
        shootR = hardwareMap.dcMotor.get("shootR");
        shootL.setDirection(DcMotorSimple.Direction.REVERSE);
        shootR.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        if (isStopRequested()) return;

        setAll(); sleep(500000);
    }
    void setAll(){
        shootL.setPower(0.5);
        shootR.setPower(shootL.getPower());
    }

}
