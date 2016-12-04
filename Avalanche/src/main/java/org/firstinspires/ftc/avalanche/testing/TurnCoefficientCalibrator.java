package org.firstinspires.ftc.avalanche.testing;

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
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;

/**
 * Created by austinzhang on 12/3/16.
 */
@TeleOp(name = "Turn Calibrator", group = "Testing")
public class TurnCoefficientCalibrator extends LinearOpMode {

    private AutoDriveTrainController autoDriveTrain;
    ColorSensor lineLeft;
    ColorSensor lineRight;
    ModernRoboticsI2cGyro gyro;
    DcMotor harvester;
    int angle;
    double speed = .6;
    double floor = .05;
    int accuracy = 1;
    int timeout = 5000;
    double porpCoeff = .004;

    private void hardwareMapping() throws InterruptedException {
        telemetry.addData("Start", "Initializing");
        telemetry.update();


        lineLeft = hardwareMap.colorSensor.get("lineLeft");
        lineRight = hardwareMap.colorSensor.get("lineRight");
        harvester = hardwareMap.dcMotor.get("Harvester");

        lineRight.setI2cAddress(new I2cAddr(0x3c / 2));
        lineLeft.setI2cAddress(new I2cAddr(0x5c / 2));

        gyro = (ModernRoboticsI2cGyro) (hardwareMap.gyroSensor.get("Gyro"));

        autoDriveTrain = new AutoDriveTrainController(lineLeft, lineRight, this, gyro, hardwareMap, harvester);

        telemetry.addData("Done", "Initializing");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMapping();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                angle -= 10;
            }
            if (gamepad1.right_bumper) {
                angle += 10;
            }

            if (gamepad1.right_stick_x > .5) {
                angle += 1;
            }
            if (gamepad1.right_stick_x < -.5) {
                angle -= 1;
            }

            if (gamepad1.dpad_up) {
                porpCoeff += .0002;
            }
            if (gamepad1.dpad_down) {
                porpCoeff -= .0002;
            }

            if (gamepad1.dpad_left) {
                accuracy -= 1;
            }
            if (gamepad1.dpad_right) {
                accuracy += 1;
            }

            if (gamepad1.left_trigger > .99) {
                timeout = timeout - 100;
            }
            if (gamepad1.right_trigger > .99) {
                timeout = timeout + 100;
            }

            if (gamepad1.left_stick_y != 0 && gamepad1.b) {
                speed = gamepad1.left_stick_y;
            }

            if (gamepad1.left_stick_y != 0 && gamepad1.x) {
                floor = (gamepad1.left_stick_y + 1) / 8;
            }

            if (gamepad1.a) {
                autoDriveTrain.pivotToAngle(angle, speed, porpCoeff, floor, accuracy, timeout);
            }

            telemetry.addData("Current Speed", speed);
            telemetry.addData("Current Angle", autoDriveTrain.getCorrectedHeading());
            telemetry.addData("Current Floor", floor);
            telemetry.addData("Current Timeout", timeout);
            telemetry.addData("Current Accuracy", accuracy);
            telemetry.addData("Current porpCoeff", porpCoeff);

            telemetry.addData("Target Angle", angle);
            telemetry.addData("Target Speed", gamepad1.left_stick_y);
            telemetry.addData("Target Floor", (gamepad1.left_stick_y + 1) / 8);


            telemetry.update();


            Thread.sleep(300);
            idle();
        }
    }
}
