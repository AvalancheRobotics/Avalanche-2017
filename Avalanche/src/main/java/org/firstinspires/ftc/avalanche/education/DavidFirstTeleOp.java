package org.firstinspires.ftc.avalanche.education;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;

@TeleOp(name = "First Teleop", group = "Education")
public class DavidFirstTeleOp extends LinearOpMode {

    private DcMotor motorLeftFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightFront;
    private DcMotor motorRightBack;

    @Override
    public void runOpMode() throws InterruptedException {

        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");
        motorRightFront = hardwareMap.dcMotor.get("RightFront");

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            motorRightFront.setDirection(DcMotor.Direction.REVERSE);
            motorRightBack.setDirection(DcMotor.Direction.REVERSE);
            if (gamepad1.a) {
                motorLeftBack.setPower(1);
                motorLeftFront.setPower(1);
                motorRightFront.setPower(1);
                motorRightBack.setPower(1);
            }
            else if(gamepad1.b){
                motorLeftBack.setPower(-1);
                motorLeftFront.setPower(-1);
                motorRightFront.setPower(-1);
                motorRightBack.setPower(-1);
            }
            else {
                motorLeftBack.setPower(0);
                motorLeftFront.setPower(0);
                motorRightFront.setPower(0);
                motorRightBack.setPower(0);
            }
        }
        idle();

    }
}


