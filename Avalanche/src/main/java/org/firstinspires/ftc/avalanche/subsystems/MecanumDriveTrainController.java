package org.firstinspires.ftc.avalanche.subsystems;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.utilities.ScaleInput;

/**
 * Code for Mecanum wheel based DriveTrain
 * Initialize in main TeleOp class
 * The manual drive program changes depending on whether the DriveTrain is initialized to use a gyro.
 * call manualDrive in a recursive loop updating the left and right inputs
 */

public class MecanumDriveTrainController extends MotorController {

    private boolean usingGyro;

    private ModernRoboticsI2cGyro gyro;

    private long startTime;
    private int offset;
    private int drift;

    //Constructors

    public MecanumDriveTrainController(MotorLeftBack motorLeftBack, MotorRightBack motorRightBack, MotorLeftFront motorLeftFront, MotorRightFront motorRightFront) {


        add(motorLeftBack.getMotor());
        add(motorRightBack.getMotor());
        add(motorLeftFront.getMotor());
        add(motorRightFront.getMotor());

        //Reverse left motors because gearing is flipped
        reverseMotors(0);
        reverseMotors(2);

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setPower(0);

        setZeroPowerBehavior(true);

        usingGyro = false;
    }

    public MecanumDriveTrainController(MotorLeftBack motorLeftBack, MotorRightBack motorRightBack, MotorLeftFront motorLeftFront, MotorRightFront motorRightFront, ModernRoboticsI2cGyro gyro) throws InterruptedException {

        add(motorLeftBack.getMotor());
        add(motorRightBack.getMotor());
        add(motorLeftFront.getMotor());
        add(motorRightFront.getMotor());

        //Reverse left motors because gearing is flipped
        reverseMotors(1);
        reverseMotors(3);

        setZeroPowerBehavior(false);

        this.gyro = gyro;

        gyro.calibrate();
        while (gyro.isCalibrating()) {    // Calibrating Gyro
            Thread.sleep(50);
        }
        Thread.sleep(5000);
        drift = gyro.getHeading();

        gyro.resetZAxisIntegrator();

        usingGyro = true;
    }

    public void callAtBeginningOfOpModeAfterInit() {
        startTime = System.nanoTime();
        offset = gyro.getHeading();
    }

    public void reverseMotors() {
        reverseMotors(0);
        reverseMotors(1);
        reverseMotors(2);
        reverseMotors(3);
    }


    /** TO DO : MAKE THIS DEGREES INSTEAD OF RADIANS */
    public void driveAtSpeedOnAngle(double power, double angle) {

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        leftFrontPower = power * Math.sin(angle + (Math.PI / 4));

        rightFrontPower = power * Math.cos(angle + (Math.PI / 4));

        leftBackPower = power * Math.cos(angle + (Math.PI / 4));

        rightBackPower = power * Math.sin(angle + (Math.PI / 4));


        double[] powers = new double[4];
        powers[0] = leftBackPower;
        powers[1] = rightBackPower;
        powers[2] = leftFrontPower;
        powers[3] = rightFrontPower;

        double curHighest = Math.abs(powers[0]);

        for (double highest: powers) {

            double absHighest = Math.abs(highest);
            if (absHighest > curHighest) {
                curHighest = absHighest;
            }
        }

        if (curHighest > 1) {
            leftFrontPower = leftFrontPower / curHighest;
            rightFrontPower = rightFrontPower / curHighest;
            leftBackPower = leftBackPower / curHighest;
            rightBackPower = rightBackPower / curHighest;
        }

        setPower(0, leftBackPower);
        setPower(1, rightBackPower);
        setPower(2, leftFrontPower);
        setPower(3, rightFrontPower);
    }

    //Functions by taking inputs (most likely joystick) and squaring them, providing more precise controls when moving slowly
    //When joysticks are not in use, the wheels lock in place using PID
    public void manualDrive(float xInput, float yInput, float turnInput) {
        if (usingGyro) {
            return;
        } else {
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;

            double power = Math.sqrt(Math.pow(Math.abs(-yInput), 2) + Math.pow(Math.abs(xInput), 2));


            double angle = Math.atan2(xInput, -yInput);
            if (-yInput < 0) {
                angle = (2 * Math.PI + angle);
            }


            leftFrontPower = power * Math.sin(angle + (Math.PI / 4)) + turnInput;

            rightFrontPower = power * Math.cos(angle + (Math.PI / 4)) - turnInput;

            leftBackPower = power * Math.cos(angle + (Math.PI / 4)) + turnInput;

            rightBackPower = power * Math.sin(angle + (Math.PI / 4)) - turnInput;


            double[] powers = new double[4];
            powers[0] = leftBackPower;
            powers[1] = rightBackPower;
            powers[2] = leftFrontPower;
            powers[3] = rightFrontPower;

            double curHighest = Math.abs(powers[0]);

            for (double highest: powers) {

                double absHighest = Math.abs(highest);
                if (absHighest > curHighest) {
                    curHighest = absHighest;
                }
            }

            if (curHighest > 1) {
                leftFrontPower = leftFrontPower / curHighest;
                rightFrontPower = rightFrontPower / curHighest;
                leftBackPower = leftBackPower / curHighest;
                rightBackPower = rightBackPower / curHighest;
            }

            setPower(0, leftBackPower);
            setPower(1, rightBackPower);
            setPower(2, leftFrontPower);
            setPower(3, rightFrontPower);


        }
    }

    public void setLeftBackPower(double power) {
        setPower(0, power);
    }

    public void setRightBackPower(double power) {
        setPower(1, power);
    }

    public void setLeftFrontPower(double power) {
        setPower(2, power);
    }

    public void setRightFrontPower(double power) {
        setPower(3, power);
    }

    public int getLeftBackEncoder() {
        return getEncoderValue(0);
    }

    public int getRightBackEncoder() {
        return getEncoderValue(1);
    }

    public int getLeftFrontEncoder() {
        return getEncoderValue(2);
    }

    public int getRightFrontEncoder() {
        return getEncoderValue(3);
    }

    public void setLeftBackTarget(int target) {
        setTargetPosition(0, target);
    }

    public void setRightBackTarget(int target) {
        setTargetPosition(1, target);
    }

    public void setLeftFrontTarget(int target) {
        setTargetPosition(2, target);
    }

    public void setRightFrontTarget(int target) {
        setTargetPosition(3, target);
    }


    public void setLeftDrivePower(double power) {
        if (motors.size() > 2) {
            setPower(0, power);
            setPower(2, power);
        } else {
            setPower(0, power);
        }
    }

    public void setRightDrivePower(double power) {
        if (motors.size() > 2) {
            setPower(1, power);
            setPower(3, power);
        } else {
            setPower(1, power);
        }
    }


    public void setZeroPowerBehavior(boolean brake) {
        if (brake) {
            for (int i = 0; i < motors.size(); i++) {
                if (!motors.get(i).getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.BRAKE)) {
                    motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        } else {
            for (int i = 0; i < motors.size(); i++) {
                if (!motors.get(i).getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.FLOAT)) {
                    motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
        }
    }

}
