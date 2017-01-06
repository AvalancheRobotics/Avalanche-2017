package org.firstinspires.ftc.avalanche.autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.avalanche.enums.TeamColor;
import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;
import org.firstinspires.ftc.avalanche.subsystems.BeaconPresser;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;

/**
 * A simple autonomous class used for testing (and possibly production in the future).
 *
 * @author Keith
 */

@Autonomous(name = "AutoRed", group = "Autonomous")
public class Auto extends LinearOpMode {

    private static TeamColor teamColor = TeamColor.RED;

    private AutoDriveTrainController autoDriveTrain;
    ColorSensor lineLeft;
    ColorSensor lineRight;
    ColorSensor colorLeft;
    ColorSensor colorRight;
    ModernRoboticsI2cGyro gyro;
    DcMotor harvester;
    BeaconPresser beaconPresser;

    Servo servoLock;

    Servo servoBeaconShuttle;

    Servo servoBeaconTilt;

    DcMotor motorShooter;

    private void hardwareMapping() throws InterruptedException {
        telemetry.addData("start", "initializing");
        telemetry.update();

        lineLeft = hardwareMap.colorSensor.get("lineLeft");
        lineRight = hardwareMap.colorSensor.get("lineRight");
        colorLeft = hardwareMap.colorSensor.get("colorLeft");
        colorRight = hardwareMap.colorSensor.get("colorRight");
        harvester = hardwareMap.dcMotor.get("Harvester");
        motorShooter = hardwareMap.dcMotor.get("Shooter");
        servoLock = hardwareMap.servo.get("Lock");

        motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorShooter.setPower(0);

        this.lineRight.enableLed(true);

        this.lineLeft.enableLed(true);

        //Initialize beacon servos
        servoBeaconShuttle = hardwareMap.servo.get("BeaconShuttle");

        servoBeaconShuttle.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);

        servoBeaconTilt = hardwareMap.servo.get("BeaconTilt");

        servoBeaconTilt.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);

        lineRight.setI2cAddress(new I2cAddr(0x3c / 2));
        lineLeft.setI2cAddress(new I2cAddr(0x5c / 2));

        colorLeft.setI2cAddress(new I2cAddr(0x6c / 2));
        colorRight.setI2cAddress(new I2cAddr(0x4c / 2));
        gyro = (ModernRoboticsI2cGyro) (hardwareMap.gyroSensor.get("Gyro"));

        autoDriveTrain = new AutoDriveTrainController(lineLeft, lineRight, this, gyro, hardwareMap, harvester);

        beaconPresser = new BeaconPresser(this, teamColor, servoBeaconShuttle, servoBeaconTilt, colorLeft, colorRight);

        telemetry.addData("Done", "Initializing");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        autoDriveTrain.callAtBeginningOfOpModeAfterInit();


        if (teamColor.equals(TeamColor.RED)) {

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, 10, 0);
            autoDriveTrain.pivotToAngle(57, .6);
            autoDriveTrain.moveDistanceAtSpeedOnHeadingFloat(43, 57, 1000);
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.3, -3, 57);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, 5, 0);
            autoDriveTrain.driveToLine(6000, true);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.startButtonPress(5000, 0);
            beaconPresser.setPresserToDrivePosition();
            Thread.sleep(1000);
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, 42, 0);
            autoDriveTrain.driveToLine(6000, true);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.startButtonPress(5000, 0);
            beaconPresser.setPresserToDrivePosition();

            autoDriveTrain.pivotToAngle(37, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.7, -25, 37);

            launchOneBall();

            loadAndLaunch();

            autoDriveTrain.pivotToAngle(50, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -25, 50);
        } else {

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -10, 0);
            autoDriveTrain.pivotToAngle(-57, .6);
            autoDriveTrain.moveDistanceAtSpeedOnHeadingFloat(-45, -57, 1000);
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.3, 3, -57);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.driveToLine(6000, false);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.startButtonPress(5000, 0);
            Thread.sleep(1000);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.5, -38, 0);
            autoDriveTrain.driveToLine(6000, false);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.startButtonPress(5000, 0);
            beaconPresser.setPresserToDrivePosition();

            autoDriveTrain.pivotToAngle(127, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.7, -27, 127);

            launchOneBall();

            loadAndLaunch();

            autoDriveTrain.pivotToAngle(140, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -28, 140);
        }


       /* if (teamColor.equals(TeamColor.RED)) {

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, 10, 0);
            autoDriveTrain.pivotToAngle(62, .6);
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, 55, 61);
            autoDriveTrain.pivotToAngle(0, .6);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.driveToLine(6000, true);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.startButtonPress(4000, 50);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.5, 38, 0);
            autoDriveTrain.driveToLine(6000, true);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.startButtonPress(4000, 50);
            beaconPresser.setPresserToDrivePosition();

            autoDriveTrain.pivotToAngle(37, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.7, -25, 37);

            launchOneBall();

            loadAndLaunch();

            autoDriveTrain.pivotToAngle(50, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -25, 50);
        } else {

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -10, 0);
            autoDriveTrain.pivotToAngle(-61, .6);
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -51, -61);
            autoDriveTrain.pivotToAngle(0, .6);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.driveToLine(6000, false);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.startButtonPress(8000, 50);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.5, -38, 0);
            autoDriveTrain.driveToLine(6000, false);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.startButtonPress(8000, 50);
            beaconPresser.setPresserToDrivePosition();

            autoDriveTrain.pivotToAngle(127, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.7, -27, 127);

            launchOneBall();

            loadAndLaunch();

            autoDriveTrain.pivotToAngle(140, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -28, 140);
        }

        */

    }

    private void loadAndLaunch() throws InterruptedException {
        servoLock.setPosition(ValueStore.LOCK_RELEASE);

        Thread.sleep(1000);

        servoLock.setPosition(ValueStore.LOCK_LOAD);

        Thread.sleep(750);

        launchOneBall();
    }


    private void launchOneBall() throws InterruptedException {
        motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorShooter.setTargetPosition(motorShooter.getCurrentPosition() + ValueStore.ONE_SHOOTER_LOOP);
        motorShooter.setPower(1);
        while (!(motorShooter.getCurrentPosition() > motorShooter.getTargetPosition() - 10 && motorShooter.getCurrentPosition() < motorShooter.getTargetPosition() + 10)) {
            idle();
        }
        motorShooter.setPower(0);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
