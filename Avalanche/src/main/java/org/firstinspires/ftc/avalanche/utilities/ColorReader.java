package org.firstinspires.ftc.avalanche.utilities;

/**
 * A class to that takes in values supplied by a color sensor and determines what colors they most
 * likely correspond with.
 *
 * Created by Keith on 9/14/2016.
 * Change by Austin on 9/17/2016: Modified detection algorithm for color detection to be based
 * off of %RGB instead of static RGB values. - Needs calibration
 *
 * TODO: CENTRALIZE PERCENTAGES - move the percentage threshold values and other
 * arbitrary values determined through calibration
 * to ValueStore, our central value storing location so if we want to change values
 * we don't have to look through multiple classes to find the correct value.
 */
public class ColorReader {

    /**
     * Returns whether or not the supplied RGB values correspond to red.
     * @param red The red value.
     * @param green The green value.
     * @param blue The blue value.
     * @return If the supplied values suggest a red color.
     */
    public static boolean isRed(int red, int green, int blue)
    {
        //Percentage of total light required to be red to be considered red
        int percentageThreshold = 40;


        return isColor(percentageThreshold, 4, red, green, blue);
    }

    /**
     * Returns whether or not the supplied RGB values correspond to blue.
     * @param red The red value.
     * @param green The green value.
     * @param blue The blue value.
     * @return If the supplied values suggest a red color.
     */
    public static boolean isBlue(int red, int green, int blue)
    {
        //Percentage of total light required to be blue to be considered blue
        int percentageThreshold = 40;


        return isColor(percentageThreshold, 4, blue, green, red);
    }

    /**
     * Whether or not the supplied light values indicated the sensor detecting a white color.
     *
     * Different algorithm due to measuring change in total amount of light, not color.
     * originalLight is the light value at initialization.
     * currentLight is the light value at the current time.
     *
     * @param originalLight The original light level.
     * @param currentLight The current light level.
     * @return If the supplied light levels correspond to white.
     */
    public static boolean isWhite(int originalLight, int currentLight)
    {
        //Change this value to calibrate program
        int percentIncreaseThreshold = 200;



        if (originalLight == 0) {
            originalLight = 1;
        }

        int percentIncrease = ((currentLight * 100) / (originalLight)) - 100;

        return percentIncrease > percentIncreaseThreshold;
    }


    //Algorithm for deciding color
    private static boolean isColor(int percentageThreshold, int minimumLight, int targetColor, int firstColor, int secondColor) {
        int totalLight = targetColor + firstColor + secondColor;

        if (minimumLight > totalLight) {
            return false;
        }

        int percentageTarget = (targetColor * 100) / totalLight;
        return (percentageTarget >= percentageThreshold);
    }

}
