package org.firstinspires.ftc.avalanche.testing;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.avalanche.utilities.ColorReader;
import org.firstinspires.ftc.avalanche.utilities.Logger;

/**
 * A simple class used for testing and demonstration of Logger.
 *
 * @author Keith
 */
@TeleOp(name = "Logger Tester", group = "Testing")
public class LoggerTester extends LinearOpMode {

    Logger logger;

    //Initialize and Map All Hardware
    private void hardwareMapping() throws InterruptedException {
        logger = new Logger("avalanche");
        logger.setOpmode(this);
        logger.log("Hardware Mapping Finished");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();

        logger.log("Waiting for start...");
        telemetry.addLine("Waiting for start...");
        waitForStart();
        logger.log("started.");
        telemetry.addLine("started.");
        logger.log(new RuntimeException("Exception"));

        telemetry.update();

        // Go go gadget robot!
        while (opModeIsActive()) {
            idle();
        }
    }
}


