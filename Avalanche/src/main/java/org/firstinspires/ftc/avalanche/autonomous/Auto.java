package org.firstinspires.ftc.avalanche.autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;


@Autonomous(name="Auto", group="Autonomous")
public class Auto extends LinearOpMode {

    private AutoDriveTrainController autoDriveTrain;
    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftBack;
    DcMotor motorRightBack;
    ColorSensor colorSensor;
    ModernRoboticsI2cGyro gyro;
    DcMotor odometer;

    private void hardwareMapping()
    {
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

        //Initialize AutoDriveTrainController
        autoDriveTrain = new AutoDriveTrainController(colorSensor, this, gyro, motorLeftBack,
                motorRightBack, motorLeftFront, motorRightFront, odometer);

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {

        }

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        autoDriveTrain.gyroDrive(autoDriveTrain.DRIVE_SPEED, 48.0, 0.0);        // Drive FWD 48 inches
        autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, -45.0);              // Turn  CCW to -45 Degrees
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, -45.0, 0.5);         // Hold -45 Deg heading for a 1/2 second
        autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, 45.0);               // Turn  CW  to  45 Degrees
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, 45.0, 0.5);          // Hold  45 Deg heading for a 1/2 second
        autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, 0.0);                // Turn  CW  to   0 Degrees
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, 0.0, 1.0);           // Hold  0 Deg heading for a 1 second
        autoDriveTrain.gyroDrive(autoDriveTrain.DRIVE_SPEED, -48.0, 0.0);       // Drive REV 48 inches
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, 0.0, 0.5);           // Hold  0 Deg heading for a 1/2 second
        autoDriveTrain.driveToLine(autoDriveTrain.DRIVE_SPEED, 8000);           //Drive at the set speed until 8 seconds has passed or until you reach a white line

        while (opModeIsActive()) {

        }
    }
}
