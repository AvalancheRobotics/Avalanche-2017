package org.firstinspires.ftc.avalanche.teleop;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;

/* A port of the BasicDrive Class used for development and fun. */
@TeleOp(name = "FunDrive", group = "TeleOp")
public class FunDrive extends LinearOpMode {

    private DcMotor motorLeftFront;
    private DcMotor motorRightFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightBack;
    private DriveTrainController driveTrain;

    private double driveSpeed = 1.0;
    private double turnSpeed = 0.2;

    private int countB = 0;
    private int countBAlt = 0;
    private int countX = 0;
    private int countXAlt = 0;

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
            telemetry.addData("Gamepad", gamepad1.toString());
            telemetry.addData("Turn Speed", turnSpeed);
            telemetry.addData("Drive Speed", driveSpeed);

            boolean modifierKey = gamepad1.dpad_down;

            driveTrain.manualDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);

            if (gamepad1.a)
            {
                    driveTrain.setLeftDrivePower(driveSpeed);
                    driveTrain.setRightDrivePower(driveSpeed);
            }

            if (gamepad1.b)
            {
                if (modifierKey)
                {
                    countBAlt++;
                    turnSpeed += 0.1;
                    if (Double.compare(turnSpeed, 1.0) > 0)
                    {
                        turnSpeed = 1.0;
                    }
                }
                else
                {
                    countB++;
                    driveSpeed += 0.1;
                    if (Double.compare(driveSpeed, 1.0) > 0)
                    {
                        driveSpeed = 1.0;
                    }
                }
            }
            else
            {
                countB = 0;
                countBAlt = 0;
            }

            if (gamepad1.x)
            {
                if (modifierKey)
                {
                    countXAlt++;
                    turnSpeed -= 0.1;
                    if (Double.compare(turnSpeed, 0.1) < 0)
                    {
                        turnSpeed = 0.1;
                    }
                }
                else
                {
                    countX++;
                    driveSpeed -= 0.1;
                    if (Double.compare(driveSpeed, 0.1) < 0)
                    {
                        driveSpeed = 0.1;
                    }
                }
            }
            else
            {
                countX = 0;
                countXAlt = 0;
            }

            if (gamepad1.y)
            {
                if (modifierKey)
                    playR2D2();
                else
                {
                    driveTrain.setLeftDrivePower(-driveSpeed);
                    driveTrain.setRightDrivePower(-driveSpeed);
                }
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

            if (gamepad1.dpad_up)
            {
                //not yet implemented
            }

            //turn right
            if (gamepad1.right_bumper) {
                driveTrain.setLeftDrivePower(turnSpeed);
                driveTrain.setRightDrivePower(-turnSpeed);
            }

            //turn left
            if (gamepad1.left_bumper) {
                driveTrain.setLeftDrivePower(-turnSpeed);
                driveTrain.setRightDrivePower(turnSpeed);
            }

            if (!gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.y && !gamepad1.a
                    && ScaleInput.scale(gamepad1.left_stick_y) == 0
                    && ScaleInput.scale(gamepad1.right_stick_y) == 0) {
                driveTrain.setLeftDrivePower(0);
                driveTrain.setRightDrivePower(0);
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

