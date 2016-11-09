package org.firstinspires.ftc.avalanche.subsystems;

import android.media.MediaPlayer;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.avalanche.R;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftBack;
import org.firstinspires.ftc.avalanche.hardware.MotorLeftFront;
import org.firstinspires.ftc.avalanche.hardware.MotorRightBack;
import org.firstinspires.ftc.avalanche.hardware.MotorRightFront;
import org.firstinspires.ftc.avalanche.utilities.ColorReader;

/**
 * Created by Keith on 9/19/2016.
 * ODOMETER WHEEL NEEDS TO BE IN THE SAME DIRECTION AS THE WHEEL ENCODER
 * Ex. When the odometer wheel ticks increase the wheel encoder ticks increase as well
 *
 * Updated by Austin on 10/3/2016
 * Incorporated odometer wheel, added gyro and timeout to driveToLine
 *
 *
 * TO DO NEXT:
 * Could improve DriveToLine
 * - Instead of giving up after not being able to detect the line, could have the robot instead revert
 * to a preset location
 * for example, if you don't see the line and drive past it, afterwards you could have a preset drive back 5 inches were you think the line is
 *
 * Also needs testing to figure out whether or not the gyro without a high pass filter is accurate and precise enough.
 */

public class AutoDriveTrainController {

    static final double COUNTS_PER_ODOMETER_REV = 1440 ;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // This is < 1.0 if geared UP
    static final double ODOMETER_DIAMETER_INCHES = 2.0 ;     // For figuring circumference
    static final double WHEEL_DIAMETER = 4.0;
    static final double COUNTS_PER_INCH_ODOMETER = (COUNTS_PER_ODOMETER_REV) /
            (ODOMETER_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH_DRIVE_WHEEL = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER * Math.PI);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    public static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    public static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    private int initLight;
    private LinearOpMode linearOpMode;
    private ModernRoboticsI2cGyro gyro;
    private DriveTrainController driveTrain;
    private DcMotor odometer;
    private ColorSensor colorSensor;

    private long startTime;
    private int offset;
    private int drift;

    //Example of how to run the AutoDriveTrainController
    /*
    public void init() throws InterruptedException
    {
        //Initialize AutoDriveTrainController
        autoDriveTrain = new AutoDriveTrainController(colorSensor, this, gyro, leftBack, rightBack, leftFront, rightFront, odometer);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        autoDriveTrain.gyroDrive(autoDriveTrain.DRIVE_SPEED, 48.0, 0.0);        // Drive FWD 48 inches
        autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, -45.0);              // Turn  CCW to -45 Degrees
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, -45.0, 0.5);         // Hold -45 Deg heading for a 1/2 second
        autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, 45.0);               // Turn  CW  to  45 Degrees
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, 45.0, 0.5);          // Hold  45 Deg heading for a 1/2 second
        autoDriveTrain.gyroTurn(autoDriveTrain.TURN_SPEED, 0.0);                // Turn  CW  to   0 Degrees
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, 0.0, 1.0);           // Hold  0 Deg heading for a 1 second
        autoDriveTrain.gyroDrive(autoDriveTrain.DRIVE_SPEED, -48.0, 0.0);       // Drive REV 48 inches
        autoDriveTrain.gyroHold(autoDriveTrain.TURN_SPEED, 0.0, 0.5);           // Hold  0 Deg heading for a 1/2 second
        autoDriveTrain.driveToLine(autoDriveTrain.DRIVE_SPEED, 8000);           //Drive at the set speed until 8 seconds has passed or until you reach a white line
    }
    */


    //When adding motors to array list, as a general rule leftBack first, then rightBack, leftFront, and finally rightFront
    public AutoDriveTrainController(ColorSensor colorSensor, LinearOpMode linearOpMode, ModernRoboticsI2cGyro gyro, HardwareMap hardwareMap, DcMotor odometer) throws InterruptedException {
        this.colorSensor = colorSensor;
        this.colorSensor.enableLed(true);
        this.linearOpMode = linearOpMode;
        this.gyro = gyro;

        gyro.calibrate();
        while (gyro.isCalibrating()) {    // Calibrating Gyro
            Thread.sleep(50);
        }
        Thread.sleep(5000);
        drift = gyro.getHeading();

        driveTrain = new DriveTrainController(new MotorLeftBack(hardwareMap), new MotorRightBack(hardwareMap), new MotorLeftFront(hardwareMap), new MotorRightFront(hardwareMap));

        this.odometer = odometer;

        odometer.setPower(0);

        odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveTrain.hardResetEncoders();

        driveTrain.setZeroPowerBehavior(true);

        gyro.resetZAxisIntegrator();

        initLight = this.colorSensor.red() + this.colorSensor.green() + this.colorSensor.blue();
    }

    public void callAtBeginningOfOpModeAfterInit() {
        startTime = System.nanoTime();
        offset = gyro.getHeading();
    }

    //Program automatically drives to white line while always facing the same direction
    //Uses gyroscope to maintain the same direction so robot doesn't drift off course

    /**
     * Drive to the white line, used primary for button pressing.
     *
     * @param speed                 Max speed you want your robot to travel (from -1 to 1).
     * @param timeoutMillis         Max time in milliseconds you want your robot to drive before
     *                              giving up and stopping.
     * @throws InterruptedException If interrupted during idle.
     * @return boolean              Whether or not the robot reached the line before timeout.
     */
    public boolean driveToLine(double speed, double timeoutMillis) throws InterruptedException
    {
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  angle;

        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angle = getCorrectedHeading();

        driveTrain.setLeftDrivePower(speed);
        driveTrain.setRightDrivePower(speed);

        //if on line, move off
        while (ColorReader.isWhite(initLight,
                colorSensor.red() + colorSensor.green() + colorSensor.blue()))
        {
            // Allow time for other processes to run.
            linearOpMode.idle();
        }

        long startTime = System.currentTimeMillis();

        boolean timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

        while (linearOpMode.opModeIsActive() && !ColorReader.isWhite(initLight,
                colorSensor.red() + colorSensor.green() + colorSensor.blue())
                && !timeout)
        {
            // adjust relative speed based on heading error.
            /*error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if any one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            driveTrain.setLeftDrivePower(leftSpeed);
            driveTrain.setRightDrivePower(rightSpeed);*/

            // If true this means that we timed out before we detected the white tape, therefore we're not on white.
            timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

            // Allow time for other processes to run.
            linearOpMode.idle();
        }

        // keep looping while we are still active, we haven't taken too long (reached timeout)
        // and we haven't reached the white line yet.

        driveTrain.setLeftDrivePower(0);
        driveTrain.setRightDrivePower(0);

        return !timeout;
    }

    /**
     * Drive to the white line, used primary for button pressing.
     *
     * @param speed                 Max speed you want your robot to travel (from -1 to 1).
     * @throws InterruptedException If interrupted   during idle.
     * @return boolean              Whether or not the robot reached the line before timeout.
     */
    public boolean driveToLine(double speed) throws InterruptedException
    {
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  angle;

        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angle = getCorrectedHeading();

        driveTrain.setLeftDrivePower(speed);
        driveTrain.setRightDrivePower(speed);

        while (linearOpMode.opModeIsActive() && !ColorReader.isWhite(initLight,
                colorSensor.red() + colorSensor.green() + colorSensor.blue()))
        {
            // adjust relative speed based on heading error.
            /*error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if any one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            driveTrain.setLeftDrivePower(leftSpeed);
            driveTrain.setRightDrivePower(rightSpeed);*/

            // If true this means that we timed out before we detected the white tape, therefore we're not on white.
            // Allow time for other processes to run.
            linearOpMode.idle();
        }

        // keep looping while we are still active, we haven't taken too long (reached timeout)
        // and we haven't reached the white line yet.

        driveTrain.setLeftDrivePower(0);
        driveTrain.setRightDrivePower(0);

        return true;
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on odometer counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for
     *                   adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance
     *                   means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive (double speed, double distance, double angle) throws InterruptedException
    {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int totalOdometerTicks;
        int totalDriveWheelTicks;
        int odometerStartValue;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            driveTrain.resetEncoders();

            odometerStartValue = odometer.getCurrentPosition();

            totalOdometerTicks = (int) (distance * COUNTS_PER_INCH_ODOMETER);

            totalDriveWheelTicks = (int) (distance * COUNTS_PER_INCH_DRIVE_WHEEL);

            int ticksTraveledOdometer = odometer.getCurrentPosition() - odometerStartValue;

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = driveTrain.getEncoderValue(0) + totalDriveWheelTicks;
            newRightBackTarget = driveTrain.getEncoderValue(1) + totalDriveWheelTicks;
            newLeftFrontTarget = driveTrain.getEncoderValue(2) + totalDriveWheelTicks;
            newRightFrontTarget = driveTrain.getEncoderValue(3) + totalDriveWheelTicks;

            // Set Target and Turn On RUN_TO_POSITION
            driveTrain.setTargetPosition(0, newLeftBackTarget);
            driveTrain.setTargetPosition(1, newRightBackTarget);
            driveTrain.setTargetPosition(2, newLeftFrontTarget);
            driveTrain.setTargetPosition(3, newRightFrontTarget);

            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            driveTrain.setPower(speed);

            // keep looping while we are still active, and odometer has not yet reached its target position.
            // The +- 5 is for allowing a bit of error since we're not going to be able to hit exactly on the odometer ticks
            while (linearOpMode.opModeIsActive() && !(ticksTraveledOdometer > totalOdometerTicks - 5 && ticksTraveledOdometer < totalOdometerTicks + 5)){

                ticksTraveledOdometer = odometer.getCurrentPosition() - odometerStartValue;

                //Convert odometer ticks to wheel ticks
                int wheelTicksRemaining = (int) (((totalOdometerTicks - Math.abs(ticksTraveledOdometer)) / COUNTS_PER_INCH_ODOMETER) * COUNTS_PER_INCH_DRIVE_WHEEL);

                // Set Target and Turn On RUN_TO_POSITION

                if (ticksTraveledOdometer > 0) {
                    driveTrain.setTargetPosition(0, wheelTicksRemaining + driveTrain.getEncoderValue(0));
                    driveTrain.setTargetPosition(1, wheelTicksRemaining + driveTrain.getEncoderValue(1));
                    driveTrain.setTargetPosition(2, wheelTicksRemaining + driveTrain.getEncoderValue(2));
                    driveTrain.setTargetPosition(3, wheelTicksRemaining + driveTrain.getEncoderValue(3));
                }
                else {
                    driveTrain.setTargetPosition(0, -wheelTicksRemaining + driveTrain.getEncoderValue(0));
                    driveTrain.setTargetPosition(1, -wheelTicksRemaining + driveTrain.getEncoderValue(1));
                    driveTrain.setTargetPosition(2, -wheelTicksRemaining + driveTrain.getEncoderValue(2));
                    driveTrain.setTargetPosition(3, -wheelTicksRemaining + driveTrain.getEncoderValue(3));
                }

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                driveTrain.setLeftDrivePower(leftSpeed);
                driveTrain.setRightDrivePower(rightSpeed);

                linearOpMode.telemetry.addData("leftSpeed", leftSpeed);
                linearOpMode.telemetry.addData("rightSpeed", rightSpeed);
                linearOpMode.telemetry.addData("leftSpeed", leftSpeed);
                linearOpMode.telemetry.addData("error", error);
                linearOpMode.telemetry.addData("tickstraveled", ticksTraveledOdometer);
                linearOpMode.telemetry.addData("totalodomticks", totalOdometerTicks);
                linearOpMode.telemetry.addData("wheelticksremaining", wheelTicksRemaining);


                linearOpMode.telemetry.update();



                // Allow time for other processes to run.
                linearOpMode.idle();
            }

            // Stop all motion;
            driveTrain.setPower(0);
        }
    }

    public void gyroDriveEncodersOnly (double speed, double distance, double angle) throws InterruptedException
    {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH_DRIVE_WHEEL);

            newLeftBackTarget = driveTrain.getEncoderValue(0) + moveCounts;
            newRightBackTarget = driveTrain.getEncoderValue(1) + moveCounts;
            newLeftFrontTarget = driveTrain.getEncoderValue(2) + moveCounts;
            newRightFrontTarget = driveTrain.getEncoderValue(3) + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            driveTrain.setTargetPosition(0, newLeftBackTarget);
            driveTrain.setTargetPosition(1, newRightBackTarget);
            driveTrain.setTargetPosition(2, newLeftFrontTarget);
            driveTrain.setTargetPosition(3, newRightFrontTarget);

            driveTrain.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            driveTrain.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (linearOpMode.opModeIsActive() && driveTrain.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                driveTrain.setLeftDrivePower(leftSpeed);
                driveTrain.setRightDrivePower(rightSpeed);


                // Allow time for other processes to run.
                linearOpMode.idle();
            }

            // Stop all motion;
            driveTrain.setPower(0);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @throws InterruptedException
     */
    public void gyroTurn (double speed, double angle) throws InterruptedException
    {
        // keep looping while we are still active, and not on heading.
        while (linearOpMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF))
        {
            // Update telemetry & Allow time for other processes to run.
            linearOpMode.idle();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     * @throws InterruptedException
     */
    public void gyroHold( double speed, double angle, double holdTime)
            throws InterruptedException {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (linearOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            linearOpMode.idle();
        }

        // Stop all motion;
        driveTrain.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        linearOpMode.telemetry.addData("currentAngle", getError(angle));
        linearOpMode.telemetry.update();

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        //Set motor run mode
        driveTrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send desired speeds to motors.
        driveTrain.setLeftDrivePower(leftSpeed);
        driveTrain.setRightDrivePower(rightSpeed);

        // Display it for the driver.
        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range
        robotError = targetAngle - getCorrectedHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //Pivots to the set angle
    //NOTE: No other tasks will be performed while turning, LOCKS THREAD
    //NOTE: Angle is ABSOLUTE ANGLE- Angles are relative to starting position, not current position
    //Angle starts at 0 at the beginning of TeleOp, and all other angles are based off of that
    public void pivotToAngle(int angle, double speed) throws InterruptedException {
        int heading = getCorrectedHeading();

        double power;
        double proportionalConst = 0.004;

        double topCeiling = speed;
        double bottomCeiling = -speed;
        double topFloor = .05;
        double bottomFloor = -.05;

        int target = angle;

        while (! (heading > target - 1 && heading < target + 1) ) {

            long currentTime = System.currentTimeMillis();

            power = Math.abs((target - heading) * proportionalConst);

            if (power > topCeiling)
                power = topCeiling;
            else if (power < bottomCeiling)
                power = bottomCeiling;
            else if (power < topFloor && power > 0)
                power = topFloor;
            else if (power > bottomFloor && power < 0)
                power = bottomFloor;

            if (target > heading) {
                driveTrain.setLeftDrivePower(power);
                driveTrain.setRightDrivePower(-power);

            }
            else {
                driveTrain.setLeftDrivePower(-power);
                driveTrain.setRightDrivePower(power);
            }


            heading = getCorrectedHeading();

            linearOpMode.idle();

        }

        driveTrain.setLeftDrivePower(0);
        driveTrain.setRightDrivePower(0);

    }

    public void turnUsingLeftDrive(int angle, double speed, double timeoutMillis) throws InterruptedException
    {
        int heading = getCorrectedHeading();

        double power;
        double proportionalConst = 0.004;

        double topCeiling = speed;
        double bottomCeiling = -speed;
        double topFloor = .05;
        double bottomFloor = -.05;

        int target = angle;

        long startTime = System.currentTimeMillis();

        boolean timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

        while (!(heading > target - 1 && heading < target + 1)  && !timeout)
        {
            power = Math.abs((target - heading) * proportionalConst);

            if (power > topCeiling)
                power = topCeiling;
            else if (power < bottomCeiling)
                power = bottomCeiling;
            else if (power < topFloor && power > 0)
                power = topFloor;
            else if (power > bottomFloor && power < 0)
                power = bottomFloor;

            if (target > heading)
            {
                driveTrain.setLeftDrivePower(power);
                linearOpMode.telemetry.addData("Direction", "Left");
            }
            else
            {
                driveTrain.setLeftDrivePower(-power);
                linearOpMode.telemetry.addData("Direction", "Right");
            }

            linearOpMode.telemetry.addData("Old Heading", heading);

            heading = getCorrectedHeading();

            linearOpMode.telemetry.addData("New Heading", heading);
            linearOpMode.telemetry.addData("Power", power);

            linearOpMode.telemetry.addData("Time", System.currentTimeMillis() - startTime);

            linearOpMode.telemetry.update();

            timeout = System.currentTimeMillis() - startTime >= timeoutMillis;

            linearOpMode.idle();
        }

        driveTrain.setLeftDrivePower(0);
    }


    //Returns corrected gyro angle
    public int getCorrectedHeading() {
        double elapsedSeconds = (System.nanoTime() - startTime) / 1000000000.0;
        int totalDrift = (int) (elapsedSeconds / 5 * drift);
        int targetHeading = gyro.getIntegratedZValue() - offset - totalDrift;

        return targetHeading;
    }


    //hacky methods for testing, not for use in production
    public void goBackward()
    {
        driveTrain.setLeftDrivePower(-1);
        driveTrain.setRightDrivePower(-1);
    }

    public void stop()
    {
        driveTrain.setLeftDrivePower(0);
        driveTrain.setRightDrivePower(0);
    }

}
