package org.firstinspires.ftc.avalanche.testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.MecanumDriveTrainController;

@TeleOp(name = "Mecanum Tester", group = "Testing")
public class MecanumTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDriveTrainController driveTrain = new MecanumDriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap), this);

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {
            if (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y && !gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                driveTrain.manualDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                double power = Math.sqrt(Math.pow(Math.abs(-gamepad1.left_stick_x), 2) + Math.pow(Math.abs(gamepad1.left_stick_y), 2));


                double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);

                telemetry.addData("power", power);
                telemetry.addData("angle", angle / Math.PI);

                telemetry.update();
            }
            else {
                if (gamepad1.a) {
                    driveTrain.driveAtSpeedOnAngle(1, 1 * Math.PI);
                }

                if (gamepad1.b) {
                    driveTrain.driveAtSpeedOnAngle(1, 1.5 * Math.PI);
                }

                if (gamepad1.y) {
                    driveTrain.driveAtSpeedOnAngle(1, 0 * Math.PI);
                }

                if (gamepad1.x) {
                    driveTrain.driveAtSpeedOnAngle(1, .5 * Math.PI);
                }

                if (gamepad1.dpad_right) {
                    driveTrain.driveAtSpeedOnAngle(1, 1.75 * Math.PI);
                }

                if (gamepad1.dpad_up) {
                    driveTrain.driveAtSpeedOnAngle(1, .25 * Math.PI);
                }

                if (gamepad1.dpad_left) {
                    driveTrain.driveAtSpeedOnAngle(1, .75 * Math.PI);
                }

                if (gamepad1.dpad_down) {
                    driveTrain.driveAtSpeedOnAngle(1, 1.25 * Math.PI);
                }
            }
        }
    }

}
