package org.firstinspires.ftc.avalanche.subsystems;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.avalanche.utilities.ColorReader;

/**
 * Created by Keith on 9/19/2016.
 */

public class AutoDriveTrainController extends MotorController {

    static final double COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // This is < 1.0 if geared UP
    static final double ODOMETER_DIAMETER_INCHES = 2.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (ODOMETER_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    private int initLight;
    private LinearOpMode linearOpMode;
    private ModernRoboticsI2cGyro gyro;
    private int rightMotorBackIndex;
    private int rightMotorFrontIndex;
    private int leftMotorBackIndex;
    private int leftMotorFrontIndex;

    public void driveToLine(ColorSensor colorSensor, double speed, double timeoutMillis) throws InterruptedException
    {
        setPower(speed);

        long startTime = System.currentTimeMillis();

        while (ColorReader.isWhite(initLight, colorSensor.red() + colorSensor.blue()) && System.currentTimeMillis() - startTime < timeoutMillis)
        {
            linearOpMode.idle();
        }
        setPower(0);
    }

    public void init(ColorSensor colorSensor, LinearOpMode linearOpMode, ModernRoboticsI2cGyro g,
                     int rMBI, int rMFI, int lMBI, int lMFI) throws InterruptedException
    {
        initLight = colorSensor.red() + colorSensor.blue();
        this.linearOpMode = linearOpMode;
        this.gyro = g;
        this.rightMotorBackIndex = rMBI;
        this.rightMotorFrontIndex = rMFI;
        this.leftMotorBackIndex = lMBI;
        this.leftMotorFrontIndex = lMFI;
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating())  {
            Thread.sleep(50);
            linearOpMode.idle();
        }

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!linearOpMode.isStarted()) {
            linearOpMode.idle();
        }
        gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
        gyroTurn(TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        gyroHold(TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        gyroTurn(TURN_SPEED, 45.0);         // Turn  CW  to  45 Degrees
        gyroHold(TURN_SPEED, 45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        gyroTurn(TURN_SPEED, 0.0);         // Turn  CW  to   0 Degrees
        gyroHold(TURN_SPEED, 0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED, -48.0, 0.0);    // Drive REV 48 inches
        gyroHold(TURN_SPEED, 0.0, 0.5);    // Hold  0 Deg heading for a 1/2 second
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive (double speed, double distance, double angle) throws InterruptedException
    {
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = motors.get(leftMotorBackIndex).getCurrentPosition() + moveCounts;
            newRightTarget = motors.get(rightMotorBackIndex).getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            motors.get(leftMotorBackIndex).setTargetPosition(newLeftTarget);
            motors.get(leftMotorFrontIndex).setTargetPosition(newLeftTarget);
            motors.get(rightMotorBackIndex).setTargetPosition(newRightTarget);
            motors.get(rightMotorFrontIndex).setTargetPosition(newRightTarget);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (linearOpMode.opModeIsActive() &&
                    (motors.get(leftMotorFrontIndex).isBusy() &&
                            motors.get(leftMotorBackIndex).isBusy() &&
                            motors.get(rightMotorFrontIndex).isBusy() && motors.get(rightMotorBackIndex).isBusy())) {

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

                motors.get(leftMotorBackIndex).setPower(leftSpeed);
                motors.get(leftMotorFrontIndex).setPower(leftSpeed);
                motors.get(rightMotorBackIndex).setPower(rightSpeed);
                motors.get(rightMotorFrontIndex).setPower(rightSpeed);


                // Allow time for other processes to run.
                linearOpMode.idle();
            }

            // Stop all motion;
            setPower(0);

            // Turn off RUN_TO_POSITION
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

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

        // Send desired speeds to motors.
        setPower(leftSpeed);
        setPower(rightSpeed);

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
        robotError = targetAngle - gyro.getIntegratedZValue();
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

}
