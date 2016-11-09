package org.firstinspires.ftc.avalanche.teleop;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ControllerConfig;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;

/**
 *
 *
 * Created by Nicholas on 11/7/16
 */
@TeleOp(name = "FunDrive", group = "TeleOp")
public class FullFunctionTeleOp extends LinearOpMode{

    private ControllerConfig controls;
    private DcMotor motorLeftFront;
    private DcMotor motorRightFront;
    private DcMotor motorLeftBack;
    private DcMotor motorRightBack;
    private DriveTrainController driveTrain;

    DcMotor motorHarvester;

    DcMotor motorShooter;

    private double driveSpeed = 1.0;
    private double turnSpeed = 0.2;

    private int countB = 0;
    private int countBAlt = 0;
    private int countX = 0;
    private int countXAlt = 0;

    Servo servoLock;

    private final double LOAD_LOCK = .2; //ARBITRARY VALUE

    private final double RELEASE_LOCK = .8; //ARBITRARY VALUE

    private final int ONE_SHOOTER_LOOP = 1120; //ARBITRARY VALUE

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

        //Initialize harvester
        motorHarvester = hardwareMap.dcMotor.get("Harvester");

        motorHarvester.setPower(0);

        motorHarvester.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorHarvester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize shooter
        motorShooter = hardwareMap.dcMotor.get("Shooter");

        motorShooter.setPower(0);

        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize lock
        servoLock = hardwareMap.servo.get("Lock");

        servoLock.setPosition(RELEASE_LOCK);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        // Wait for the game to start
        waitForStart();

        controls = new DefaultControls(gamepad1, gamepad2);

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
                driveTrain.setLeftDrivePower(driveSpeed);
                driveTrain.setRightDrivePower(driveSpeed);
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

            if (controls.LockPositionButtonPressed())
                servoLock.setPosition(LOAD_LOCK);
            else
                servoLock.setPosition(RELEASE_LOCK);

            if (controls.HarvesterButtonPressed())
            {
                motorHarvester.setPower(1);
            }
            else if (controls.ReverseHarvesterButtonPressed())
            {
                motorHarvester.setPower(-1);
            }
            else
            {
                motorHarvester.setPower(0);
            }

            if (controls.LoadAndShootButtonPressed()) {
                servoLock.setPosition(LOAD_LOCK);

                servoLock.setPosition(RELEASE_LOCK);

                launchOneBall();
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

    public static double roundToOneDec(double value)
    {
        return (double)Math.round(value * 10d) / 10d;
    }

    private void launchOneBall(){}

}

