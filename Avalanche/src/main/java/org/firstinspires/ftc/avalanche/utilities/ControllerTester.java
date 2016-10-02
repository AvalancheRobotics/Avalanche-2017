package org.firstinspires.ftc.avalanche.utilities;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Controller Tester", group = "Utilities")
public class ControllerTester extends LinearOpMode {


    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftBack;
    DcMotor motorRightBack;
    DriveTrainController driveTrain;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");
        motorRightFront = hardwareMap.dcMotor.get("RightFront");

        driveTrain = new DriveTrainController(motorLeftFront, motorRightFront, motorLeftBack, motorRightBack);

        // Reset encoders
        driveTrain.resetEncoders();

        playR2D2();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        // Wait for the game to start
        waitForStart();

        //Test our media player
        //sanic.start();

        // Go go gadget robot!
        while (opModeIsActive()) {

            driveTrain.setLeftDrivePower(gamepad1.left_stick_y);
            driveTrain.setRightDrivePower(gamepad1.right_stick_y);

            telemetry.addData("Gamepad 1 Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Gamepad 1 Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Gamepad 1 Right Stick Y", gamepad1.right_stick_x);
            telemetry.addData("Gamepad 1 Right Stick X", gamepad1.right_stick_x);

            telemetry.addData("Gamepad 2 Left Stick Y", gamepad2.left_stick_y);
            telemetry.addData("Gamepad 2 Left Stick X", gamepad2.left_stick_x);
            telemetry.addData("Gamepad 2 Right Stick Y", gamepad2.right_stick_x);
            telemetry.addData("Gamepad 2 Right Stick X", gamepad2.right_stick_x);

            if (gamepad1.a)
            {
                playR2D2();
            }

            if (gamepad1.b)
            {
                //not yet implemented
            }

            if (gamepad1.x)
            {
                //not yet implemented
            }

            if (gamepad1.y)
            {
                //not yet implemented
            }

            if (gamepad1.dpad_down)
            {
                //not yet implemented
            }

            if (gamepad1.dpad_left)
            {
                //not yet implemented
            }

            if (gamepad1.dpad_right)
            {
                //not yet implemented
            }

            if (gamepad1.dpad_up) {
                //not yet implemented
            }

            telemetry.update();
            idle();

        }
    }

    private void playR2D2()
    {
        MediaPlayer r2d2 = MediaPlayer.create(hardwareMap.appContext, org.firstinspires.ftc.avalanche.R.raw.r2d2a);

        int rand = (int)(Math.random()*5);

        switch (rand)
        {
            case 0:
                r2d2 = MediaPlayer.create(hardwareMap.appContext, org.firstinspires.ftc.avalanche.R.raw.r2d2a);
                break;
            case 1:
                r2d2 = MediaPlayer.create(hardwareMap.appContext, org.firstinspires.ftc.avalanche.R.raw.r2d2b);
                break;
            case 2:
                r2d2 = MediaPlayer.create(hardwareMap.appContext, org.firstinspires.ftc.avalanche.R.raw.r2d2c);
                break;
            case 3:
                r2d2 = MediaPlayer.create(hardwareMap.appContext, org.firstinspires.ftc.avalanche.R.raw.r2d2d);
                break;
            case 4:
                r2d2 = MediaPlayer.create(hardwareMap.appContext, org.firstinspires.ftc.avalanche.R.raw.r2d2e);
                break;
        }

        r2d2.start();
    }
}

