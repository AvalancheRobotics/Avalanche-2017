package org.firstinspires.ftc.avalanche.education;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Keith's First Teleop", group = "Education")
public class KeithFirstTeleOp extends LinearOpMode {

    private DcMotor motorLeftFront;
    private DcMotor motorRightFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightBack;

    @Override
    public void runOpMode() throws InterruptedException {

        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightFront = hardwareMap.dcMotor.get("RightFront");
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            motorRightBack.setDirection(DcMotor.Direction.REVERSE);
            motorRightFront.setDirection(DcMotor.Direction.REVERSE);

            idle();

        }
    }
}

