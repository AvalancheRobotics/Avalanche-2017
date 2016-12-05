package org.firstinspires.ftc.avalanche.testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.controls.SingleControllerControls;
import org.firstinspires.ftc.avalanche.enums.TeamColor;
import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;
import org.firstinspires.ftc.avalanche.subsystems.BeaconPresser;
import org.firstinspires.ftc.avalanche.controls.ControllerConfig;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;

@TeleOp(name="AutoDriverTester", group="Testing")
public class AutoDrive extends LinearOpMode {

    private AutoDriveTrainController autoDriveTrain;
    ColorSensor lineLeft;
    ColorSensor lineRight;
    ColorSensor colorLeft;
    ColorSensor colorRight;
    ModernRoboticsI2cGyro gyro;
    DcMotor harvester;
    double distance;
    BeaconPresser beaconPresser;

    Servo servoLock;

    Servo servoBeaconShuttle;

    Servo servoBeaconTilt;

    private void hardwareMapping() throws InterruptedException {
        lineLeft = hardwareMap.colorSensor.get("lineLeft");
        lineRight = hardwareMap.colorSensor.get("lineRight");
        colorLeft = hardwareMap.colorSensor.get("colorLeft");
        colorRight = hardwareMap.colorSensor.get("colorRight");
        harvester = hardwareMap.dcMotor.get("Harvester");

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
        gyro = (ModernRoboticsI2cGyro)(hardwareMap.gyroSensor.get("Gyro"));

        autoDriveTrain = new AutoDriveTrainController(lineLeft, lineRight, this, gyro, hardwareMap, harvester);

        beaconPresser = new BeaconPresser(this, TeamColor.BLUE, servoBeaconShuttle, servoBeaconTilt, colorLeft, colorRight);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();

        autoDriveTrain.callAtBeginningOfOpModeAfterInit();

        // Go go gadget robot!
        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {
                telemetry.addData("align", "started");
                telemetry.update();
                autoDriveTrain.driveToLineAndAlign(8000);
            }
            if (gamepad1.dpad_right) {
                telemetry.addData("d2l", "started");
                telemetry.update();
                autoDriveTrain.driveToLine(8000, true);
            }

            if (gamepad1.y) {
                autoDriveTrain.moveDistanceAtSpeedOnHeading(.6 , distance, 0);
            }

            if (gamepad1.a) {
                autoDriveTrain.moveDistanceAtSpeedOnHeading(.6 , -distance, 0);
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


            telemetry.update();
            idle();
        }
    }

    public static double roundToOneDec(double value)
    {
        return (double)Math.round(value * 10d) / 10d;
    }

}

