package org.firstinspires.ftc.avalanche.testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.enums.TeamColor;
import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;
import org.firstinspires.ftc.avalanche.subsystems.BeaconPresser;
import org.firstinspires.ftc.avalanche.controls.DefaultControls;
import org.firstinspires.ftc.avalanche.controls.ControllerConfig;

@TeleOp(name = "TurnUsingLeftDrive", group = "Testing")
public class TurnUsingLeftDriveTester extends LinearOpMode
{
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
        speed = 0.4;

        countSpeedUp = 0;
        countSpeedDown = 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();
        controls = new DefaultControls(gamepad1, gamepad2);

        autoDriveTrain.callAtBeginningOfOpModeAfterInit();

        telemetry.addData("Done", "");
        telemetry.update();

        // Go go gadget robot!
        while (opModeIsActive()) {

            telemetry.addData("Gamepad 1", gamepad1.toString());
            telemetry.addData("Gamepad 2", gamepad2.toString());

            if (gamepad1.y)
            {
                autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, distance, autoDriveTrain.getCorrectedHeading());
            }

            if (gamepad1.a)
            {
                autoDriveTrain.moveDistanceAtSpeedOnHeading(.6, -distance, autoDriveTrain.getCorrectedHeading());
            }

            if (gamepad1.b)
            {
                autoDriveTrain.turnUsingLeftDrive(90, speed, 100000);
            }

            if (gamepad1.x)
            {
                autoDriveTrain.turnUsingLeftDrive(-90, speed, 100000);
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

            if (gamepad1.start)
            {
                lastDrive = autoDriveTrain.driveToLine(speed, 100000);
            }

            telemetry.addData("Speed", speed);

            telemetry.addData("lastDrive", lastDrive);

            telemetry.update();
            idle();
        }
    }

    public static double roundToOneDec(double value)
    {
        return (double)Math.round(value * 10d) / 10d;
    }

}

