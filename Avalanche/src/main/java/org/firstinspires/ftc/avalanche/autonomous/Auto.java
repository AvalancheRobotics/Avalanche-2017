package org.firstinspires.ftc.avalanche.autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.controls.ControllerConfig;
import org.firstinspires.ftc.avalanche.enums.TeamColor;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;
import org.firstinspires.ftc.avalanche.subsystems.BeaconPresser;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;

/**
 * A simple autonomous class used for testing (and possibly production in the future).
 *
 * @author Keith
 */

@Autonomous(name = "Auto", group = "Autonomous")
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
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        autoDriveTrain.callAtBeginningOfOpModeAfterInit();


        if (teamColor.equals(TeamColor.RED)) {

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, 10, 0);
            autoDriveTrain.pivotToAngle(61, .6);
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, 52.5, 61);
            autoDriveTrain.pivotToAngle(0, .6);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.driveToLine(6000, true);
            autoDriveTrain.pivotToAngle(0, .1);
            //beaconPresser.startButtonPress(8000, 0);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.5, 38, 0);
            autoDriveTrain.driveToLine(6000, true);
            autoDriveTrain.pivotToAngle(0, .1);
            //beaconPresser.startButtonPress(8000, 0);
            beaconPresser.setPresserToDrivePosition();

            autoDriveTrain.pivotToAngle(37, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.7, -30, 37);

            launchOneBall();

            loadAndLaunch();

            autoDriveTrain.pivotToAngle(50, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -30, 50);
        } else {

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -10, 0);
            autoDriveTrain.pivotToAngle(-61, .6);
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -52.5, -61);
            autoDriveTrain.pivotToAngle(0, .6);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.driveToLine(6000, false);
            autoDriveTrain.pivotToAngle(0, .1);
            //beaconPresser.startButtonPress(8000, 0);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.5, -38, 0);
            autoDriveTrain.driveToLine(6000, false);
            autoDriveTrain.pivotToAngle(0, .1);
            //beaconPresser.startButtonPress(8000, 0);
            beaconPresser.setPresserToDrivePosition();

            autoDriveTrain.pivotToAngle(127, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.7, -30, 127);

            launchOneBall();

            loadAndLaunch();

            autoDriveTrain.pivotToAngle(140, .6);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -30, 140);
        }



/* TOO SLOW - DOESN'T GET PAST BUTTON PRESSING
        if (teamColor.equals(TeamColor.RED)) {

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, 10, 0);
            autoDriveTrain.pivotToAngle(61, .6);
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, 52.5, 61);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.driveToLine(6000, true);
            autoDriveTrain.pivotToAngle(0, .1);
            //beaconPresser.startButtonPress(8000, 0);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.3, 36, 0);
            autoDriveTrain.driveToLine(6000, true);
            autoDriveTrain.pivotToAngle(0, .1);
            //beaconPresser.startButtonPress(8000, 0);
            beaconPresser.setPresserToDrivePosition();

            autoDriveTrain.pivotToAngle(37, .4);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.3, -30, 37);

            launchOneBall();

            loadAndLaunch();

            autoDriveTrain.pivotToAngle(50, .4);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.4, -30, 50);
        } else {

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -10, 0);
            autoDriveTrain.pivotToAngle(-61, .6);
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -52.5, -61);
            autoDriveTrain.pivotToAngle(0, .1);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.driveToLine(6000, false);
            autoDriveTrain.pivotToAngle(0, .1);
            //beaconPresser.startButtonPress(8000, 0);
            beaconPresser.setPresserToDrivePosition();
            autoDriveTrain.moveDistanceAtSpeedOnHeading(.3, -36, 0);
            autoDriveTrain.driveToLine(6000, false);
            autoDriveTrain.pivotToAngle(0, .1);
            //beaconPresser.startButtonPress(8000, 0);
            beaconPresser.setPresserToDrivePosition();

            autoDriveTrain.pivotToAngle(127, .4);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.3, -30, 127);

            launchOneBall();

            loadAndLaunch();

            autoDriveTrain.pivotToAngle(140, .4);

            autoDriveTrain.moveDistanceAtSpeedOnHeading(.4, -30, 140);
        }

        */


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        //autoDriveTrain.moveDistanceAtSpeedOnHeading(1, 48.0);        // Drive FWD 48 inches
        //autoDriveTrain.moveDistanceAtSpeedOnHeading(1, -48.0);       // Drive REV 48 inches

        //autoDriveTrain.moveDistanceAtSpeedOnHeading(autoDriveTrain.DRIVE_SPEED, 57.0, 45); //Drive FWD 57 inches 45 degrees
        //autoDriveTrain.moveDistanceAtSpeedOnHeading(autoDriveTrain.DRIVE_SPEED, 35.0, -45); //Drive FWD 35 inches -45 degrees
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
