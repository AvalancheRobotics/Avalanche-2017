package org.firstinspires.ftc.avalanche.education;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A bare bones teleop class used for teaching new members.
 */
@TeleOp(name = "Megan First Teleop", group = "Education")
public class MeganFirstTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            idle();

        }
    }
}

