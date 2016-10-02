package org.firstinspires.ftc.avalanche.education;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Cole's First Teleop", group = "Education")
public class ColeFirstTeleOp extends LinearOpMode {

    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;

    @Override
    public void runOpMode() throws InterruptedException {


        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorRightFront = hardwareMap.dcMotor.get("RightFront");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");

        waitForStart();


        // Go go gadget robot!
        while (opModeIsActive()) {

            motorRightFront.setDirection(DcMotor.Direction.REVERSE);
            motorRightBack.setDirection(DcMotor.Direction.REVERSE);

            if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
                motorLeftFront.setPower(gamepad1.left_stick_y);
                motorLeftBack.setPower(gamepad1.left_stick_y);
                motorRightFront.setPower(gamepad1.right_stick_y);
                motorRightBack.setPower(gamepad1.right_stick_y);
            }
            else if (gamepad1.y) {
                motorLeftFront.setPower(1);
                motorLeftBack.setPower(1);
                motorRightFront.setPower(1);
                motorRightBack.setPower(1);
            }
            else if (gamepad1.a) {
                motorLeftFront.setPower(-1);
                motorLeftBack.setPower(-1);
                motorRightFront.setPower(-1);
                motorRightBack.setPower(-1);

            }
            else
            {
                motorLeftFront.setPower(0);
                motorLeftBack.setPower(0);
                motorRightBack.setPower(0);
                motorRightFront.setPower(0);

            }

            telemetry.update();
            idle();

        }
    }
}

