package org.firstinspires.ftc.avalanche.autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.avalanche.enums.TeamColor;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.AutoDriveTrainController;
import org.firstinspires.ftc.avalanche.subsystems.BeaconPresser;
import org.firstinspires.ftc.avalanche.subsystems.MecanumDriveTrainController;
import org.firstinspires.ftc.avalanche.utilities.ColorReader;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;

/**
 * A simple autonomous class used for testing (and possibly production in the future).
 *
 * @author Keith
 */

@Autonomous(name = "MecanumAuto", group = "Autonomous")
public class MecanumAuto extends LinearOpMode {

    private static TeamColor teamColor = TeamColor.BLUE;

    MecanumDriveTrainController mecanumDriveTrain;

    ColorSensor line;
    ColorSensor color;
    ColorSensor color2;
    ModernRoboticsI2cGyro gyro;

    Servo servoLock;

    Servo servoLiftRelease;

    Servo servoSpacerOne;

    Servo servoSpacerTwo;

    DcMotor motorShooter;

    int initLight;

    private void hardwareMapping() throws InterruptedException {
        telemetry.addData("start", "initializing");

        line = hardwareMap.colorSensor.get("line");
        color = hardwareMap.colorSensor.get("color");
        color2 = hardwareMap.colorSensor.get("color2");

        this.line.enableLed(false);
        this.color.enableLed(false);
        this.color2.enableLed(false);

        this.line.enableLed(true);


        telemetry.update();


        line.setI2cAddress(new I2cAddr(0x3c / 2));
        color.setI2cAddress(new I2cAddr(0x5c / 2));
        color2.setI2cAddress(new I2cAddr(0x4c / 2));



        mecanumDriveTrain = new MecanumDriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap), this);

        mecanumDriveTrain.setZeroPowerBehavior(true);

        motorShooter = hardwareMap.dcMotor.get("Shooter");
        motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorShooter.setPower(0);

        servoLock = hardwareMap.servo.get("Lock");
        servoLiftRelease = hardwareMap.servo.get("LiftRelease");
        servoSpacerOne = hardwareMap.servo.get("SpacerOne");
        servoSpacerTwo = hardwareMap.servo.get("SpacerTwo");

        servoLock.setPosition(ValueStore.LOCK_LOAD);
        servoLiftRelease.setPosition(ValueStore.LIFT_HELD);
        servoSpacerOne.setPosition(ValueStore.SPACER_ONE_STORE);
        servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_STORE);

        initLight = line.red() + line.green() + line.blue();



        telemetry.addData("Done", initLight);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMapping();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (teamColor.equals(TeamColor.BLUE)) {

            mecanumDriveTrain.driveSpeedAtAngleForDistance(.8, Math.PI, 15);

            servoSpacerOne.setPosition(ValueStore.SPACER_ONE_EXTENDED);
            servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_EXTENDED);


            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 95);

            telemetry.addData("pastStraight", "[");
            telemetry.update();

            mecanumDriveTrain.driveAtSpeedOnAngle(.6, -3 * Math.PI / 4);


            while (!ColorReader.isWhite(initLight, line.blue() + line.green() + line.red())) {
                idle();
            }
            mecanumDriveTrain.setPower(0);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 10);

            Thread.sleep(250);

            servoSpacerOne.setPosition(ValueStore.SPACER_ONE_STORE);
            servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_STORE);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 15);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 5);

            Thread.sleep(1000);

            if (ColorReader.isRed(color.red(), color.green(), color.blue()) || ColorReader.isRed(color2.red(),color2.green(), color2.blue())) {
                Thread.sleep(4500);
                mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 15);
            }

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 10);

            servoSpacerOne.setPosition(ValueStore.SPACER_ONE_EXTENDED);
            servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_EXTENDED);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -3 * Math.PI / 4, 15);

            mecanumDriveTrain.driveAtSpeedOnAngle(.6, -3 * Math.PI / 4);


            while (!ColorReader.isWhite(initLight, line.blue() + line.green() + line.red())) {
                idle();
            }
            mecanumDriveTrain.setPower(0);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 10);

            Thread.sleep(250);

            servoSpacerOne.setPosition(ValueStore.SPACER_ONE_STORE);
            servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_STORE);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 15);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 5);

            Thread.sleep(1000);

            if (ColorReader.isRed(color.red(), color.green(), color.blue()) || ColorReader.isRed(color2.red(), color2.green(), color2.blue())) {
                Thread.sleep(4500);
                mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 15);
            }

            mecanumDriveTrain.turnUsingEncoders(.8, 2000, true);

        }
        else {

            mecanumDriveTrain.driveSpeedAtAngleForDistance(.8, 0, 45);

            servoSpacerOne.setPosition(ValueStore.SPACER_ONE_EXTENDED);
            servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_EXTENDED);


            mecanumDriveTrain.turnUsingEncoders(.7, 800, false);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(.5, 0, 4);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 7);

            telemetry.addData("pastStraight", "[");
            telemetry.update();

            mecanumDriveTrain.driveAtSpeedOnAngle(.6, -1 * Math.PI / 8);


            while (!ColorReader.isWhite(initLight, line.blue() + line.green() + line.red())) {
                idle();
            }
            mecanumDriveTrain.setPower(0);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 10);

            Thread.sleep(250);

            servoSpacerOne.setPosition(ValueStore.SPACER_ONE_STORE);
            servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_STORE);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 15);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 5);

            Thread.sleep(1000);

            if (ColorReader.isBlue(color.red(), color.green(), color.blue()) || ColorReader.isBlue(color2.red(),color2.green(), color2.blue())) {
                Thread.sleep(4500);
                mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 15);
            }

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 10);

            servoSpacerOne.setPosition(ValueStore.SPACER_ONE_EXTENDED);
            servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_EXTENDED);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, 1 * -Math.PI / 8, 15);

            mecanumDriveTrain.driveAtSpeedOnAngle(.6, 1 * -Math.PI / 8);


            while (!ColorReader.isWhite(initLight, line.blue() + line.green() + line.red())) {
                idle();
            }
            mecanumDriveTrain.setPower(0);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 10);

            Thread.sleep(250);

            servoSpacerOne.setPosition(ValueStore.SPACER_ONE_STORE);
            servoSpacerTwo.setPosition(ValueStore.SPACER_TWO_STORE);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 15);

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 5);

            Thread.sleep(1000);

            if (ColorReader.isBlue(color.red(), color.green(), color.blue()) || ColorReader.isBlue(color2.red(), color2.green(), color2.blue())) {
                Thread.sleep(4500);
                mecanumDriveTrain.driveSpeedAtAngleForDistance(1, -Math.PI / 2, 15);
            }

            mecanumDriveTrain.driveSpeedAtAngleForDistance(1, Math.PI / 2, 5);

            mecanumDriveTrain.turnUsingEncoders(.8, 2000, true);

        }



    }

    private void loadAndLaunch() throws InterruptedException {
        servoLock.setPosition(ValueStore.LOCK_RELEASE);

        Thread.sleep(1000);

        servoLock.setPosition(ValueStore.LOCK_LOAD);

        Thread.sleep(750);

        launchOneBall();
    }


    private void launchOneBall() throws InterruptedException {
        motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorShooter.setTargetPosition(motorShooter.getCurrentPosition() + ValueStore.ONE_SHOOTER_LOOP);
        motorShooter.setPower(1);
        while (!(motorShooter.getCurrentPosition() > motorShooter.getTargetPosition() - 10 && motorShooter.getCurrentPosition() < motorShooter.getTargetPosition() + 10)) {
            idle();
        }
        motorShooter.setPower(0);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
