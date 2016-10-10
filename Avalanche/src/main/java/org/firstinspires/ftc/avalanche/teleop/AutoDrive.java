package org.firstinspires.ftc.avalanche.teleop;

import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;

@TeleOp(name = "AutoDrive", group = "TeleOp")
public class AutoDrive extends LinearOpMode {


    private AutoDriveTrainController autoDriveTrain;
    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftBack;
    DcMotor motorRightBack;
    ColorSensor colorSensor;
    ModernRoboticsI2cGyro gyro;
    DcMotor odometer;
    double distance;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");
        motorRightFront = hardwareMap.dcMotor.get("RightFront");
        colorSensor = hardwareMap.colorSensor.get("ColorSensor");
        gyro = (ModernRoboticsI2cGyro)(hardwareMap.gyroSensor.get("Gyro"));
        odometer = hardwareMap.dcMotor.get("Odometer");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            if (gamepad1.y) {
                autoDriveTrain.gyroDrive(autoDriveTrain.DRIVE_SPEED, distance, 0.0);
            }

            if (gamepad1.a) {
                autoDriveTrain.gyroDrive(autoDriveTrain.DRIVE_SPEED, -distance, 0.0);
            }

            if (gamepad1.b)
            {
                autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, 90.0);
            }

            if (gamepad1.x)
            {
                autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, -90.0);
            }

            if (gamepad1.left_bumper)
            {
                distance += 0.1;
                distance = roundToOneDec(distance);
            }

            if (gamepad1.right_bumper)
            {
                distance -= 0.1;
                distance = roundToOneDec(distance);
            }

            telemetry.addData("Distance", distance + " inches");
            telemetry.update();
            idle();
        }
    }

    public static double roundToOneDec(double value)
    {
        return (double)Math.round(value * 10d) / 10d;
    }
}

