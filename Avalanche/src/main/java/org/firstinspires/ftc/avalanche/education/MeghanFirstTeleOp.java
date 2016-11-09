package org.firstinspires.ftc.avalanche.education;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * A bare bones teleop class used for teaching new members.
 */
@TeleOp(name = "Meghan First Teleop", group = "Education")
public class MeghanFirstTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive()) {

            System.out.println("Delete this later.");
            idle();

        }
    }
}

