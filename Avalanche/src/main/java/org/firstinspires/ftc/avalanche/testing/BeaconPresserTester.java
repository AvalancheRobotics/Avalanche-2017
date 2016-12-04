package org.firstinspires.ftc.avalanche.testing;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.avalanche.enums.TeamColor;
import org.firstinspires.ftc.avalanche.subsystems.BeaconPresser;

/**
 * Created by austinzhang on 9/18/16.
 */

@TeleOp (name = "Beacon Presser Tester", group = "Utilities")
public class BeaconPresserTester extends LinearOpMode {

    Servo beaconShuttle;
    Servo beaconTilt;

    ColorSensor colorLeft;
    ColorSensor colorRight;

    TeamColor teamColor;

    @Override
    public void runOpMode() throws InterruptedException {
        beaconShuttle = hardwareMap.servo.get("beaconShuttle");
        beaconTilt = hardwareMap.servo.get("beaconTilt");
        colorLeft = hardwareMap.colorSensor.get("colorLeft");
        colorRight = hardwareMap.colorSensor.get("colorRight");

        colorLeft.setI2cAddress(new I2cAddr(0x03c/2));
        colorRight.setI2cAddress(new I2cAddr(0x04c/2));

        colorRight.enableLed(false);
        colorLeft.enableLed(false);

        teamColor = TeamColor.BLUE;

        waitForStart();

        while (opModeIsActive()) {
            BeaconPresser beaconPresser = new BeaconPresser(this, TeamColor.BLUE, beaconShuttle, beaconTilt, colorLeft, colorRight);

            if (gamepad1.a) {
                beaconPresser.setPresserToDrivePosition();
            }

            if (gamepad1.b) {
             beaconPresser.startButtonPress(8000, 50, hardwareMap.touchSensor.get("touchSensor"));
            }

            if (gamepad1.y) {
                if (teamColor.equals(TeamColor.RED)) {
                    teamColor = TeamColor.BLUE;
                    telemetry.addData("TeamColor: ", "Blue");
                }
                else {
                    teamColor = TeamColor.RED;
                    telemetry.addData("TeamColor: ", "Blue");
                }
                telemetry.update();
            }
        }

    }
}
