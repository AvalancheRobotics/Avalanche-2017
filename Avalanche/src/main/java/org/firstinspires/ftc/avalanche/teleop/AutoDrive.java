package org.firstinspires.ftc.avalanche.teleop;

import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.enums.TeamColor;
import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;
import org.firstinspires.ftc.avalanche.subsystems.BeaconPresser;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ControllerConfig;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;

@TeleOp(name = "AutoDrive", group = "TeleOp")
public class AutoDrive extends LinearOpMode implements ControllerConfig{

    private AutoDriveTrainController autoDriveTrain;
    ColorSensor colorSensor;
    ModernRoboticsI2cGyro gyro;
    DcMotor odometer;
    double distance;

    TeamColor teamColor = TeamColor.BLUE;
    Servo beaconShuttle;
    Servo beaconTilt;
    ColorSensor colorLeft;
    ColorSensor colorRight;
    BeaconPresser beaconPresser;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {

        beaconShuttle = hardwareMap.servo.get("beaconShuttle");
        beaconTilt = hardwareMap.servo.get("beaconTilt");
        colorLeft = hardwareMap.colorSensor.get("colorLeft");
        colorRight = hardwareMap.colorSensor.get("colorRight");

        colorLeft.setI2cAddress(new I2cAddr(0x03c/2));
        colorRight.setI2cAddress(new I2cAddr(0x04c/2));

        colorRight.enableLed(false);
        colorLeft.enableLed(false);

        teamColor = TeamColor.BLUE;

        colorSensor = hardwareMap.colorSensor.get("ColorSensor");
        gyro = (ModernRoboticsI2cGyro)(hardwareMap.gyroSensor.get("Gyro"));
        odometer = hardwareMap.dcMotor.get("Odometer");
        beaconPresser = new BeaconPresser(this, teamColor, beaconShuttle, beaconTilt, colorLeft, colorRight);

        autoDriveTrain = new AutoDriveTrainController(colorSensor, this, gyro, hardwareMap, odometer);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();

        autoDriveTrain.callAtBeginningOfOpModeAfterInit();

        // Go go gadget robot!
        while (opModeIsActive()) {

            if (gamepad1.y) {
                autoDriveTrain.gyroDrive(.6 , distance, autoDriveTrain.getCorrectedHeading());
            }

            if (gamepad1.a) {
                autoDriveTrain.gyroDrive(.6 , -distance, autoDriveTrain.getCorrectedHeading());
            }

            if (gamepad1.b)
            {
                autoDriveTrain.pivotToAngle(90, .4);
            }

            if (gamepad1.x)
            {
                autoDriveTrain.pivotToAngle(-90, .4);
            }

            if (gamepad1.left_bumper)
            {
                distance -= .005;
            }

            if (gamepad1.right_bumper)
            {
                distance += .005;
            }

            if (gamepad1.dpad_up) {
                beaconPresser.setPresserToDrivePosition();
            }

            if (gamepad1.dpad_down) {
                beaconPresser.startButtonPress(8000, 0);
            }

            telemetry.addData("Distance", roundToOneDec(distance) + " inches");

            telemetry.addData("odom" , odometer.getCurrentPosition());

            telemetry.update();
            idle();
        }
    }

    public static double roundToOneDec(double value)
    {
        return (double)Math.round(value * 10d) / 10d;
    }
    public float LTrack() {return gamepad1.left_stick_y;}
    public float RTrack() {return gamepad1.right_stick_y;}
    public boolean LeftButtonPresserButtonPressed() {return gamepad2.dpad_left;}
    public boolean RightButtonPresserButtonPressed() {return gamepad2.dpad_right;}
    public boolean HarvesterButtonPressed() {return gamepad2.a;}
    public boolean ShooterButtonPressed() {return gamepad2.y;}
    public boolean turnLeft() {return gamepad1.left_bumper;}
    public boolean turnRight() {return gamepad1.right_bumper;}
    public boolean modifierKey() {return gamepad1.dpad_down;}
    public boolean increaseSpeed() {return gamepad1.b;}
    public boolean decreaseSpeed() {return gamepad1.x;}
    public boolean reverse() {return gamepad1.a;}
    public boolean forward() {return gamepad1.y;}

}

