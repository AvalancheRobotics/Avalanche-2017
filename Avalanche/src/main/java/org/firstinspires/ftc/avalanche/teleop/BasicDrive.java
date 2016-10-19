package org.firstinspires.ftc.avalanche.teleop;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ControllerConfig;

@TeleOp(name = "BasicDrive", group = "TeleOp")
public class BasicDrive extends LinearOpMode implements ControllerConfig{


    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftBack;
    DcMotor motorRightBack;
    DriveTrainController driveTrain;


    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        motorLeftBack = hardwareMap.dcMotor.get("LeftBack");
        motorLeftFront = hardwareMap.dcMotor.get("LeftFront");
        motorRightBack = hardwareMap.dcMotor.get("RightBack");
        motorRightFront = hardwareMap.dcMotor.get("RightFront");

        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));
        // Reset encoders
        driveTrain.resetEncoders();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        waitForStart();


        // Go go gadget robot!
        while (opModeIsActive()) {


            driveTrain.manualDrive(LTrack(), RTrack());

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
    public float RTrack()
    {
        return gamepad1.right_stick_y;

    }
    public float LTrack()
    {
        return gamepad1.left_stick_y;
    }
    public boolean HarvesterButtonPressed()
    {
        return gamepad2.a;
    }
    public boolean ShooterButtonPressed()
    {
        return gamepad2.y;
    }
    public boolean LeftButtonPresserButtonPressed()
    {
        return gamepad2.dpad_left;
    }
    public boolean RightButtonPresserButtonPressed()
    {
        return gamepad2.dpad_right;
    }
}

