package org.firstinspires.ftc.avalanche.testing;

import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.R;
import org.firstinspires.ftc.avalanche.enums.TeamColor;
import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;
import org.firstinspires.ftc.avalanche.subsystems.BeaconPresser;
import org.firstinspires.ftc.avalanche.teleop.DefaultControls;
import org.firstinspires.ftc.avalanche.utilities.ControllerConfig;

@TeleOp(name = "Auto DriveToLine Tester", group = "Testing")
public class AutoDriveToLineTest extends LinearOpMode{

    private ControllerConfig controls;
    private AutoDriveTrainController autoDriveTrain;
    ColorSensor colorSensor;
    ModernRoboticsI2cGyro gyro;
    DcMotor odometer;

    double distance;
    double speed;

    boolean lastDrive;

    int countSpeedUp;
    int countSpeedDown;

    TeamColor teamColor = TeamColor.BLUE;
    Servo beaconShuttle;
    Servo beaconTilt;
    ColorSensor colorLeft;
    ColorSensor colorRight;
    BeaconPresser beaconPresser;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {

        /*beaconShuttle = hardwareMap.servo.get("beaconShuttle");
        beaconTilt = hardwareMap.servo.get("beaconTilt");
        colorLeft = hardwareMap.colorSensor.get("colorLeft");
        colorRight = hardwareMap.colorSensor.get("colorRight");

        colorLeft.setI2cAddress(new I2cAddr(0x03c/2));
        colorRight.setI2cAddress(new I2cAddr(0x04c/2));

        colorRight.enableLed(false);
        colorLeft.enableLed(false);*/

        teamColor = TeamColor.BLUE;

        colorSensor = hardwareMap.colorSensor.get("ColorSensor");
        gyro = (ModernRoboticsI2cGyro)(hardwareMap.gyroSensor.get("Gyro"));
        odometer = hardwareMap.dcMotor.get("Odometer");
        /*beaconPresser = new BeaconPresser(this, teamColor, beaconShuttle, beaconTilt, colorLeft, colorRight);*/

        autoDriveTrain = new AutoDriveTrainController(colorSensor, this, gyro, hardwareMap, odometer);

        distance = 0;
        speed = 1;

        countSpeedUp = 0;
        countSpeedDown = 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        MediaPlayer initDone = MediaPlayer.create(hardwareMap.appContext, R.raw.imready);
        initDone.start();

        waitForStart();
        controls = new DefaultControls(gamepad1, gamepad2);

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

            if (gamepad1.dpad_right)
            {
                    countSpeedUp++;
                    if (countSpeedUp == 250) {
                        speed += 0.1;
                        speed = roundToOneDec(speed);
                        if (Double.compare(speed, 1.0) > 0) {
                            speed = 1.0;
                        }
                        countSpeedUp = 0;
                    }
            }
            else
            {
                countSpeedUp = 0;
            }

            if (gamepad1.dpad_left)
            {
                countSpeedDown++;
                if (countSpeedDown == 250) {
                    speed -= 0.1;
                    speed = roundToOneDec(speed);
                    if (Double.compare(speed, 0.1) < 0) {
                        speed = 0.1;
                    }
                }
            }
            else
            {
                countSpeedDown = 0;
            }

            if (gamepad1.dpad_up) {
                beaconPresser.setPresserToDrivePosition();
            }

            if (gamepad1.dpad_down) {
                beaconPresser.startButtonPress(8000, 0);
            }

            if (gamepad1.start) {
                lastDrive = autoDriveTrain.driveToLine(speed, 1000000000*1000000000);
            }

            telemetry.addData("Distance", roundToOneDec(distance) + " inches");
            telemetry.addData("Speed", speed);

            telemetry.addData("odom" , odometer.getCurrentPosition());

            telemetry.update();
            idle();
        }
    }

    public static double roundToOneDec(double value)
    {
        return (double)Math.round(value * 10d) / 10d;
    }

}

