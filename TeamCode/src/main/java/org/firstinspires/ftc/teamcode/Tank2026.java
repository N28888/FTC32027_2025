package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDrive2", group = "TeleOp")
class TankDrive2 extends OpMode {
    private DcMotor leftfrontmotor;
    private DcMotor leftbackmotor;
    private DcMotor rightfrontmotor;
    private DcMotor rightbackmotor;

    private Servo ballgetservoL;
    private Servo ballgetservoR;


    @Override
    public void init() {
        leftfrontmotor = hardwareMap.get(DcMotor.class, "leftfront");
        leftbackmotor = hardwareMap.get(DcMotor.class, "leftback");
        rightfrontmotor = hardwareMap.get(DcMotor.class, "rightfront");
        rightbackmotor = hardwareMap.get(DcMotor.class, "rightback");

    }

    @Override
    public void loop() {

    }
}
