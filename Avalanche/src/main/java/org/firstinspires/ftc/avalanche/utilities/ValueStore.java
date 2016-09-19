package org.firstinspires.ftc.avalanche.utilities;


/**
 * Use this class to store values, save them as public static final values so they
 * can be easily accessed from other classes
 */
public class ValueStore {
    //Arbitrary Means that a value has not been assigned and needs assigning
    public static final int ARBITRARYINT = 0;
    public static final double ARBITRARYDOUBLE = 0;

    //Servo Values

    /** Button Presser Values */

    //Distance to extend button presser to before reaching beacon
    public static final double BUTTON_PRESSER_DRIVING = 0.6;

    //Distance to extend button presser to press button
    public static final double BUTTON_PRESSER_PRESSED = 0.34;

    //Button presser's stowed position
    public static final double BUTTON_PRESSER_RETRACTED = 0.84;

    //Distance to extend button presser to measure colors
    public static final double BUTTON_PRESSER_MEASURE = 0.38;

    //Button presser's stowed angle
    public static final double BUTTON_PRESSER_STORE_ANGLE = 0.55;

    //Button presser's right button pressing angle
    public static final double BUTTON_PRESSER_RIGHT_ANGLE = 0.85;

    //Button presser's left pressing angle
    public static final double BUTTON_PRESSER_LEFT_ANGLE = 0.245;

    //Button presser's time to move from the measuring position to the pressing position
    public static final double TIME_TO_BUTTON_PRESS_FROM_MEASURE_DISTANCE_MILLIS = 2000; //ARBITRARY

    //Motor Positions

}
