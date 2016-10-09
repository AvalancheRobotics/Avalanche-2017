package org.firstinspires.ftc.avalanche.autonomous.intelligentauto;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.subsystems.DriveTrainController;


/**
 * Created by austinzhang on 8/20/16.
 */
@Autonomous(name = "FunctionTester")
public class FunctionTester extends LinearOpMode {
    Store store;

    @Override
    public void runOpMode() throws InterruptedException {
        store = new Store();

        store.telemetry = telemetry;

        // Initialize sensors
        store.gyro = hardwareMap.gyroSensor.get("gyro");

        // Initalize Drivetrain
        store.driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap) , new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        telemetry.addData("size: ", store.driveTrain.size());
        telemetry.update();

        /////////////////////////////////////////
        store.gyro.calibrate();                //
        //
        while (store.gyro.isCalibrating())     // Calibrating Gyro
            Thread.sleep(50);                  //
        //
        Thread.sleep(5000);                    //
        store.drift = store.gyro.getHeading(); //
        /////////////////////////////////////////

        store.range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        telemetry.addData("Done calibrating", getCorrectedHeading());
        telemetry.update();

        waitForStart();
        store.startTime = System.nanoTime();

        store.offset = store.gyro.getHeading();

        //STARTING POSITION
        store.lastPosition = new Location(1, 1);

        moveToTarget();
        telemetry.addData("done", getCorrectedHeading());
        telemetry.update();

    }


    public void moveToTarget() throws InterruptedException {
        pivotToAngle(45);
        store.driveTrain.resetEncoders();
        while (opModeIsActive()) {
            moveForwardOnHeading();
            idle();
        }
    }


    public void pivotToAngle(int angle) {
        int heading = getCorrectedHeading();

        double power;
        double proportionalConst = 0.004;

        double topCeiling = 1;
        double bottomCeiling = -1;
        double topFloor = .2;
        double bottomFloor = -.2;

        int target = angle;
        while (target > 359)
            target = target - 360;
        while (target < 0)
            target = target + 360;

        while (heading != target) {

            //   telemetry.addData("angle: ", heading);
            //   telemetry.addData("ultrasonic: ", calibratedUltrasonic(store.rangeReader.getReadBuffer()[0]));
            //   telemetry.update();

            power = Math.abs((target - heading) * proportionalConst);

            if (power > topCeiling)
                power = topCeiling;
            else if (power < bottomCeiling)
                power = bottomCeiling;
            else if (power < topFloor && power > 0)
                power = topFloor;
            else if (power > bottomFloor && power < 0)
                power = bottomFloor;


            boolean tarGreater = target - heading > 0;

            if ((tarGreater && target - heading > 180) || (!tarGreater && target - heading < 180)) {
                store.driveTrain.setRightDrivePower(power);
                store.driveTrain.setLeftDrivePower(-power);
            } else {
                store.driveTrain.setRightDrivePower(-power);
                store.driveTrain.setLeftDrivePower(power);
            }


            heading = getCorrectedHeading();
        }

        store.driveTrain.setRightDrivePower(0);
        store.driveTrain.setLeftDrivePower(0);
    }

    public int calibratedUltrasonic(byte reading) {
        //convert byte to int
        int uncalibratedDistance = reading;
        if (uncalibratedDistance == -1) {
            return -1;
        }

        int calibratedDistance;

        if (uncalibratedDistance >= 0) {
            return (int) (((double) uncalibratedDistance / 9) * 10);
        } else {
            calibratedDistance = uncalibratedDistance * -1;
            calibratedDistance = 255 - calibratedDistance;
            return (int) (((double) calibratedDistance / 9) * 10);
        }
    }

    public void moveForwardOnHeading() {
        Location currentLocation = store.lastPosition;

        int distance = (10 - store.driveTrain.distanceTraveledBeforeReset()); //Math.round(Math.sqrt(Math.pow((targetLocation.getX() - currentLocation.getX()) * 3, 2) + Math.pow((targetLocation.getY() - currentLocation.getY()) * 3, 2)));

        int ticks = (int) (Store.TICKS_PER_CM * distance);

        telemetry.addData("dist", distance);


        if (distance > -2 && distance < 2) {
            store.driveTrain.setLeftDrivePower(0);
            store.driveTrain.setRightDrivePower(0);
        } else {

            double power;
            double proportionalConst = 0.0003;

            double topCeiling = 1;
            double bottomCeiling = -1;
            double topFloor = .08;
            double bottomFloor = -.08;

            power = ticks * proportionalConst;

            if (power > topCeiling)
                power = topCeiling;
            else if (power < bottomCeiling)
                power = bottomCeiling;
            else if (power < topFloor && power > 0)
                power = topFloor;
            else if (power > bottomFloor && power < 0)
                power = bottomFloor;


            for (int i = 0; i < store.driveTrain.size(); i++) {

                store.driveTrain.runToPosition(i, .4, ticks);
            }
            //store.driveTrain.setLeftDrivePower(power);
            //store.driveTrain.setRightDrivePower(power);
        }


        telemetry.addData("ticks", ticks);
        telemetry.addData("size", store.driveTrain.size());
        telemetry.update();


    }

    public int getCorrectedHeading() {
        double elapsedSeconds = (System.nanoTime() - store.startTime) / 1000000000.0;
        int totalDrift = (int) (elapsedSeconds / 5 * store.drift);
        int targetHeading = store.gyro.getHeading() - store.offset - totalDrift;
        while (targetHeading > 359)               //
            targetHeading = targetHeading - 360; // Allows value to "wrap around"
        while (targetHeading < 0)                 // since values can only be 0-359
            targetHeading = targetHeading + 360; //
        return targetHeading;
    }
}
