package org.firstinspires.ftc.avalanche.teleop;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;

@TeleOp(name = "BasicDrive", group = "TeleOp")
public class BasicDrive extends LinearOpMode {


    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftBack;
    DcMotor motorRightBack;
    DriveTrainController driveTrain;

    MediaPlayer sanic;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");
        motorRightFront = hardwareMap.dcMotor.get("RightFront");

        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        // Reset encoders
        driveTrain.resetEncoders();

        sanic = MediaPlayer.create(hardwareMap.appContext, org.firstinspires.ftc.avalanche.R.raw.sanic);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();

        sanic.start();

        // Go go gadget robot!
        while (opModeIsActive()) {
            if (gamepad1.a) {
                sanic.stop();
            }

            driveTrain.manualDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);

            if (gamepad1.left_bumper) {
                driveTrain.setLeftDrivePower(.5);
                driveTrain.setRightDrivePower(-.5);
            }

            if (gamepad1.right_bumper) {
                driveTrain.setLeftDrivePower(-.5);
                driveTrain.setRightDrivePower(.5);
            }

            idle();
        }
    }
}

