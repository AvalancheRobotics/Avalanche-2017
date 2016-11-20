package org.firstinspires.ftc.avalanche.teleop;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.controls.SingleControllerControls;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.controls.ControllerConfig;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;

/**
 * The scratch teleop used for development and testing.
 *
 * @author Keith
 */
@TeleOp(name = "Scratch Teleop", group = "TeleOp")
public class ScratchTeleop extends LinearOpMode{

    private ControllerConfig controls;
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

        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        // Reset encoders
        driveTrain.resetEncoders();

        driveTrain.setControlMode(true);

        playR2D2();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        // Wait for the game to start
        waitForStart();

        controls = new SingleControllerControls(gamepad1, gamepad2);

        // Go go gadget robot!
        while (opModeIsActive())
        {
            boolean modifierKey = controls.modifierKey();

            if (ScaleInput.scale(controls.LTrack()) != 0
                    || ScaleInput.scale(controls.RTrack()) != 0)
            driveTrain.manualDrive(-controls.LTrack(), -controls.RTrack());

            if (controls.reverse())
            {
                    driveTrain.setLeftDrivePower(-driveSpeed);
                    driveTrain.setRightDrivePower(-driveSpeed);
            }

            if (controls.increaseSpeed())
            {
                if (modifierKey)
                {
                    countBAlt++;
                    if (countBAlt == 250)
                    {
                        turnSpeed += 0.1;
                        if (Double.compare(turnSpeed, 1.0) > 0)
                        {
                            turnSpeed = 1.0;
                        }
                        countBAlt = 0;
                    }
                }
                else
                {
                    countB++;
                    if (countB == 250) {
                        driveSpeed += 0.1;
                        if (Double.compare(driveSpeed, 1.0) > 0)
                        {
                            driveSpeed = 1.0;
                        }
                        countB = 0;
                    }
                }
            }
            else
            {
                countB = 0;
                countBAlt = 0;
            }

            if (controls.decreaseSpeed())
            {
                if (modifierKey)
                {
                    countXAlt++;
                    if (countXAlt == 250) {
                        turnSpeed -= 0.1;
                        if (Double.compare(turnSpeed, 0.1) < 0)
                        {
                            turnSpeed = 0.1;
                        }
                        countXAlt = 0;
                    }
                }
                else
                {
                    countX++;
                    if (countX == 250) {
                        driveSpeed -= 0.1;
                        if (Double.compare(driveSpeed, 0.1) < 0)
                        {
                            driveSpeed = 0.1;
                        }
                    }
                }
            }
            else
            {
                countX = 0;
                countXAlt = 0;
            }

            if (controls.forward())
            {
                if (modifierKey)
                    playR2D2();
                else
                {
                    driveTrain.setLeftDrivePower(driveSpeed);
                    driveTrain.setRightDrivePower(driveSpeed);
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
            if (controls.turnRight()) {
                driveTrain.setLeftDrivePower(-turnSpeed);
                driveTrain.setRightDrivePower(turnSpeed);
            }

            //turn left
            if (controls.turnLeft()) {
                driveTrain.setLeftDrivePower(turnSpeed);
                driveTrain.setRightDrivePower(-turnSpeed);
            }

            if (!controls.turnLeft() && !controls.turnRight() && !controls.forward() && !controls.reverse()
                    && ScaleInput.scale(controls.LTrack()) == 0
                    && ScaleInput.scale(controls.RTrack()) == 0) {
                driveTrain.setLeftDrivePower(0);
                driveTrain.setRightDrivePower(0);
            }

            driveSpeed = roundToOneDec(driveSpeed);
            turnSpeed = roundToOneDec(turnSpeed);

            telemetry.addData("countB", countB);
            telemetry.addData("countBAlt", countBAlt);
            telemetry.addData("countX", countX);
            telemetry.addData("countXAlt", countXAlt);
            telemetry.addData("Gamepad", gamepad1.toString());
            telemetry.addData("Turn Speed", turnSpeed);
            telemetry.addData("Drive Speed", driveSpeed);
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

    public static double roundToOneDec(double value)
    {
        return (double)Math.round(value * 10d) / 10d;
    }

}

