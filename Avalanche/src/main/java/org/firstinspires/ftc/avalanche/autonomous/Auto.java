package org.firstinspires.ftc.avalanche.autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;

/**
 * A simple autonomous class used for testing (and possibly production in the future).
 *
 * @author Keith
 */

@Autonomous(name="Auto", group="Autonomous")
public class Auto extends LinearOpMode {

    private AutoDriveTrainController autoDriveTrain;
    ColorSensor colorSensor;
    ModernRoboticsI2cGyro gyro;
    DcMotor odometer;

    private void hardwareMapping()
    {
        colorSensor = hardwareMap.colorSensor.get("ColorSensor");
        gyro = (ModernRoboticsI2cGyro)(hardwareMap.gyroSensor.get("Gyro"));
        odometer = hardwareMap.dcMotor.get("Odometer");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        //Initialize AutoDriveTrainController
        autoDriveTrain = new AutoDriveTrainController(colorSensor, this, gyro, hardwareMap, odometer);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        autoDriveTrain.callAtBeginningOfOpModeAfterInit();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        //autoDriveTrain.moveDistanceAtSpeedOnHeading(1, 48.0);        // Drive FWD 48 inches
        //autoDriveTrain.moveDistanceAtSpeedOnHeading(1, -48.0);       // Drive REV 48 inches

        //autoDriveTrain.moveDistanceAtSpeedOnHeading(autoDriveTrain.DRIVE_SPEED, 57.0, 45); //Drive FWD 57 inches 45 degrees
        //autoDriveTrain.moveDistanceAtSpeedOnHeading(autoDriveTrain.DRIVE_SPEED, 35.0, -45); //Drive FWD 35 inches -45 degrees
    }
}
