package org.firstinspires.ftc.avalanche.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.controls.SingleControllerControls;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.controls.ControllerConfig;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;

/**
 * The main Teleop class used for production (competition) purposes.
 *
 * TODO: Implement Button Pressor
 *
 * Created by Nicholas on 11/7/16
 * Edited by Keith on 11/13/16
 */
@TeleOp(name = "Main Teleop", group = "TeleOp")
public class MainTeleop extends LinearOpMode{

    private ControllerConfig controls;
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

    Servo servoBeaconShuttle;

    Servo servoBeaconTilt;

    boolean singleController = false;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        driveTrain.reverseMotors();

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

        servoLock.setPosition(ValueStore.LOCK_LOAD);

        //Initialize beacon servos
        servoBeaconShuttle = hardwareMap.servo.get("BeaconShuttle");

        servoBeaconShuttle.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);

        servoBeaconTilt = hardwareMap.servo.get("BeaconTilt");

        servoBeaconTilt.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
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

            //driving
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

            //other components
            if (controls.LoadPositionButtonPressed())
                servoLock.setPosition(ValueStore.LOCK_LOAD);

            if (controls.ReleasePositionButtonPressed())
                servoLock.setPosition(ValueStore.LOCK_RELEASE);

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
                loadAndLaunch();
            }

            if (ScaleInput.scale(controls.ShootOneBall()) > 0) {
                motorShooter.setPower(Math.abs(ScaleInput.scale(controls.ShootOneBall())));
            } else {
                motorShooter.setPower(0);
            }

            /*if (controls.ShootOneBall() > 0.5)
            {
                launchOneBall();
            }*/

            driveSpeed = roundToOneDec(driveSpeed);
            turnSpeed = roundToOneDec(turnSpeed);

            idle();
        }
    }

    public static double roundToOneDec(double value)
    {
        return (double)Math.round(value * 10d) / 10d;
    }

    private void loadAndLaunch() throws InterruptedException {
        servoLock.setPosition(ValueStore.LOCK_LOAD);

        Thread.sleep(1000);

        servoLock.setPosition(ValueStore.LOCK_RELEASE);

        Thread.sleep(1000);

        servoLock.setPosition(ValueStore.LOCK_LOAD);

        Thread.sleep(500);

        launchOneBall();
    }

    private void launchOneBall() throws InterruptedException {
        motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorShooter.setTargetPosition(motorShooter.getCurrentPosition() + ValueStore.ONE_SHOOTER_LOOP);
        motorShooter.setPower(1);
        while (motorShooter.isBusy()) {
            idle();
        }
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}

