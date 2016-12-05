package org.firstinspires.ftc.avalanche.education;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;

/**
 * A bare bones teleop class used for teaching new members.
 */
@TeleOp(name = "First Teleop", group = "Education")
@Disabled
public class FirstTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            idle();

        }
    }
}

