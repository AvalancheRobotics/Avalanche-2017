package org.firstinspires.ftc.avalanche.utilities;

/**
 * Scales the inputs provided by the joysticks and triggers on the controllers.
 *
 * Created by Austin on 6/3/2016.
 */
public class ScaleInput {

    /**
     * Scales a float.
     * @param dVal The value to be scaled.
     * @return The scaled value.
     */
    public static double scale(float dVal) {
            if (dVal < .1 && dVal > -.1) {
                return 0;
            }
            if (dVal < -.95) {
                return -1;
            }
            if (dVal > .95) {
                return 1;
            }

            if (dVal > 0) {
                return Math.pow(dVal, 2);
            }
            else {
                return -Math.pow(dVal, 2);
            }

    }

    /**
     * Scales a double.
     * @param dVal The value to be scaled.
     * @return The scaled value.
     */
    public static double scale(double dVal) {
        if (dVal < .1 && dVal > -.1) {
            return 0;
        }
        if (dVal < -.95) {
            return -1;
        }
        if (dVal > .95) {
            return 1;
        }

        if (dVal > 0) {
            return Math.pow(dVal, 2);
        }
        else {
            return -Math.pow(dVal, 2);
        }

    }


    // Alternate code
    /*
    public static float scale2(float dVal) {
        float absVal = Math.abs(dVal);

        if (absVal < 0.1 || absVal > 0.95) {
            absVal = absVal < 0.1 ? 0.0 : 1.0;
        } else {
            absVal = absVal * absVal;
        }

        return (dVal > 0) ? absVal : -absVal;
    }
    */

}
