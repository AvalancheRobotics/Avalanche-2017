package org.firstinspires.ftc.avalanche.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.avalanche.enums.BeaconState;
import org.firstinspires.ftc.avalanche.enums.TeamColor;
import org.firstinspires.ftc.avalanche.utilities.ColorReader;
import org.firstinspires.ftc.avalanche.utilities.ValueStore;

/**
 * Created by austin zhang on 9/17/16: Created button presser subsystem
 * which is used to press the beacons.
 * Call setPresserToDrivePosition to extend the presser after auto starts
 * and call startButtonPress after lining up with the beacon.
 * Redundancy makes sure that a flash of light or temporary glitch in the
 * color sensor doesn't trigger a false positive.
 * Redundancy makes it so that detecting that a beacon is blue or red once isn't good enough
 * you would need to detect that the beacon is red multiple times
 * before it is accepted that the beacon is actually red.
 *
 * Updated by austin zhang on 9/19/16:
 * Made redundancy time based instead of cycle based.
 */
public class BeaconPresser {

    //Declare hardware
    private Servo extendServo;
    private Servo selectorServo;
    private ColorSensor colorLeft;
    private ColorSensor colorRight;
    private TeamColor teamColor;
    private LinearOpMode operation;

    public BeaconPresser(LinearOpMode opMode, TeamColor teamColor, Servo extendServo, Servo selectorServo, ColorSensor leftSensor, ColorSensor rightSensor) {
        //Set team
        this.teamColor = teamColor;

        this.operation = opMode;

        //Initialize hardware
        this.extendServo = extendServo;
        this.selectorServo = selectorServo;
        this.colorLeft = leftSensor;
        this.colorRight = rightSensor;

        //Set initial values
        this.extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
        this.selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);

        this.colorLeft.enableLed(false);
        this.colorRight.enableLed(false);
    }

    public void setPresserToDrivePosition() {
        extendServo.setPosition(ValueStore.BUTTON_PRESSER_DRIVING);
    }


    public void startButtonPress(int timeoutMilliseconds, long redundancy, TouchSensor touchSensor) throws InterruptedException {
        // Begin extending presser
        extendServo.setPosition(ValueStore.BUTTON_PRESSER_MEASURE);


        // Figure out what the current beacon state is.
        // If the robot can't figure it out in time, the robot quits and moves on.
        long startTime = System.currentTimeMillis();

        BeaconState currentState = beaconState(redundancy);

        while (currentState.equals(BeaconState.UNDECIDED) && (System.currentTimeMillis() - startTime) < timeoutMilliseconds && !touchSensor.isPressed()) {
            currentState = beaconState(redundancy);
            operation.idle();
        }

        if (currentState.equals(BeaconState.UNDECIDED)) {
            selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
            extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
            return;
        }


        // After you figure out the beacon state
        // move in to press the button and
        // tilt the selector servo to hit the button you want.
        if (teamColor.equals(TeamColor.RED)) {
            if (currentState.equals(BeaconState.BLUE_LEFT_BLUE_RIGHT) || currentState.equals(BeaconState.BLUE_LEFT_RED_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_RIGHT_ANGLE);
            } else if (currentState.equals(BeaconState.RED_LEFT_BLUE_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_LEFT_ANGLE);
            }
        } else {
            if (currentState.equals(BeaconState.RED_LEFT_RED_RIGHT) || currentState.equals(BeaconState.RED_LEFT_BLUE_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_RIGHT_ANGLE);
            } else if (currentState.equals(BeaconState.BLUE_LEFT_RED_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_LEFT_ANGLE);
            }
        }

        extendServo.setPosition(ValueStore.BUTTON_PRESSER_PRESSED);


        // Wait for the colors change,
        // if the colors change as desired, the robot quits and moves on.
        while ((System.currentTimeMillis() - startTime) < timeoutMilliseconds && !touchSensor.isPressed()) {
            BeaconState curState = beaconState(redundancy);

            if (teamColor.equals(TeamColor.RED) && curState.equals(BeaconState.RED_LEFT_RED_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
                extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
                return;
            } else if (curState.equals(BeaconState.BLUE_LEFT_BLUE_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
                extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
                return;
            }
            operation.idle();
        }


        // If the robot simply missed the button, there's not much we can do.
        // In this scenario, until we can figure out a better approach, we give up and move on.
        if (teamColor.equals(TeamColor.RED)) {
            if (!beaconState(redundancy).equals(BeaconState.BLUE_LEFT_BLUE_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
                extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
                return;
            }
        } else {
            if (!beaconState(redundancy).equals(BeaconState.RED_LEFT_RED_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
                extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
                return;
            }
        }


        // If the robot didn't miss the button, and we simply pressed the button too quickly after
        // our alliance partner selected the wrong color,
        // wait a bit longer and attempt to press again.
        // Depending on what we set as our timeout, this code may be completely unnecessary.
        // However if it's unnecessary it won't impact performance so we'll keep it here anyways
        // to cover all cases.
        extendServo.setPosition(ValueStore.BUTTON_PRESSER_MEASURE);

        // Just in case we missed the button on the first try, we'll try to press the other side
        // since in this case we can hit either button.
        if (selectorServo.getPosition() == ValueStore.BUTTON_PRESSER_RIGHT_ANGLE) {
            selectorServo.setPosition(ValueStore.BUTTON_PRESSER_LEFT_ANGLE);
        } else {
            selectorServo.setPosition(ValueStore.BUTTON_PRESSER_RIGHT_ANGLE);
        }

        int remainingTimeoutMillis = 5000 - timeoutMilliseconds;

        long startTimeTwo = System.currentTimeMillis();

        while (remainingTimeoutMillis > (System.currentTimeMillis() - startTimeTwo)) {
            operation.idle();
        }

        extendServo.setPosition(ValueStore.BUTTON_PRESSER_PRESSED);


        long startTimeThree = System.currentTimeMillis();
        while (ValueStore.TIME_TO_BUTTON_PRESS_FROM_MEASURE_DISTANCE_MILLIS > (System.currentTimeMillis() - startTimeThree)) {
            operation.idle();
        }

        extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
        selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);

    }


    public void startButtonPress(int timeoutMilliseconds, long redundancy) throws InterruptedException {
        // Begin extending presser
        extendServo.setPosition(ValueStore.BUTTON_PRESSER_MEASURE);


        // Figure out what the current beacon state is.
        // If the robot can't figure it out in time, the robot quits and moves on.
        long startTime = System.currentTimeMillis();

        BeaconState currentState = beaconState(redundancy);

        while (currentState.equals(BeaconState.UNDECIDED) && (System.currentTimeMillis() - startTime) < timeoutMilliseconds) {
            currentState = beaconState(redundancy);
            operation.idle();
        }

        if (currentState.equals(BeaconState.UNDECIDED)) {
            selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
            extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
            return;
        }


        // After you figure out the beacon state
        // move in to press the button and
        // tilt the selector servo to hit the button you want.
        if (teamColor.equals(TeamColor.RED)) {
            if (currentState.equals(BeaconState.BLUE_LEFT_BLUE_RIGHT) || currentState.equals(BeaconState.BLUE_LEFT_RED_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_RIGHT_ANGLE);
            } else if (currentState.equals(BeaconState.RED_LEFT_BLUE_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_LEFT_ANGLE);
            }
        } else {
            if (currentState.equals(BeaconState.RED_LEFT_RED_RIGHT) || currentState.equals(BeaconState.RED_LEFT_BLUE_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_RIGHT_ANGLE);
            } else if (currentState.equals(BeaconState.BLUE_LEFT_RED_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_LEFT_ANGLE);
            }
        }

        extendServo.setPosition(ValueStore.BUTTON_PRESSER_PRESSED);


        // Wait for the colors change,
        // if the colors change as desired, the robot quits and moves on.
        while ((System.currentTimeMillis() - startTime) < timeoutMilliseconds) {
            BeaconState curState = beaconState(redundancy);

            if (teamColor.equals(TeamColor.RED) && curState.equals(BeaconState.RED_LEFT_RED_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
                extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
                return;
            } else if (curState.equals(BeaconState.BLUE_LEFT_BLUE_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
                extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
                return;
            }
            operation.idle();
        }


        // If the robot simply missed the button, there's not much we can do.
        // In this scenario, until we can figure out a better approach, we give up and move on.
        if (teamColor.equals(TeamColor.RED)) {
            if (!beaconState(redundancy).equals(BeaconState.BLUE_LEFT_BLUE_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
                extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
                return;
            }
        } else {
            if (!beaconState(redundancy).equals(BeaconState.RED_LEFT_RED_RIGHT)) {
                selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);
                extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
                return;
            }
        }


        // If the robot didn't miss the button, and we simply pressed the button too quickly after
        // our alliance partner selected the wrong color,
        // wait a bit longer and attempt to press again.
        // Depending on what we set as our timeout, this code may be completely unnecessary.
        // However if it's unnecessary it won't impact performance so we'll keep it here anyways
        // to cover all cases.
        extendServo.setPosition(ValueStore.BUTTON_PRESSER_MEASURE);

        // Just in case we missed the button on the first try, we'll try to press the other side
        // since in this case we can hit either button.
        if (selectorServo.getPosition() == ValueStore.BUTTON_PRESSER_RIGHT_ANGLE) {
            selectorServo.setPosition(ValueStore.BUTTON_PRESSER_LEFT_ANGLE);
        } else {
            selectorServo.setPosition(ValueStore.BUTTON_PRESSER_RIGHT_ANGLE);
        }

        int remainingTimeoutMillis = 5000 - timeoutMilliseconds;

        long startTimeTwo = System.currentTimeMillis();

        while (remainingTimeoutMillis > (System.currentTimeMillis() - startTimeTwo)) {
            operation.idle();
        }

        extendServo.setPosition(ValueStore.BUTTON_PRESSER_PRESSED);


        long startTimeThree = System.currentTimeMillis();
        while (ValueStore.TIME_TO_BUTTON_PRESS_FROM_MEASURE_DISTANCE_MILLIS > (System.currentTimeMillis() - startTimeThree)) {
            operation.idle();
        }

        extendServo.setPosition(ValueStore.BUTTON_PRESSER_RETRACTED);
        selectorServo.setPosition(ValueStore.BUTTON_PRESSER_STORE_ANGLE);

    }



    // Returns the state of the beacon.
    // If the robot is unable to detect the colors for both sides
    // of the beacon then the beacon state is considered undecided.
    // Redundancy makes sure that the state is the same for multiple
    // runs, ensuring that a temporary flash of light or glitch in the
    // color sensor doesn't cause a false positive.
    // Redundancy is time based.
    private BeaconState beaconState(long redundancy) {
        int leftRed = colorLeft.red();
        int leftGreen = colorLeft.green();
        int leftBlue = colorLeft.blue();

        int rightRed = colorRight.red();
        int rightGreen = colorRight.green();
        int rightBlue = colorRight.blue();

        BeaconState firstState = BeaconState.UNDECIDED;

        if (ColorReader.isBlue(leftRed, leftGreen, leftBlue)) {
            if (ColorReader.isBlue(rightRed, rightGreen, rightBlue)) {
                firstState = BeaconState.BLUE_LEFT_BLUE_RIGHT;
            }

            if (ColorReader.isRed(rightRed, rightGreen, rightBlue)) {
                firstState = BeaconState.BLUE_LEFT_RED_RIGHT;
            }
        }

        if (ColorReader.isRed(leftRed, leftGreen, leftBlue)) {
            if (ColorReader.isBlue(rightRed, rightGreen, rightBlue)) {
                firstState = BeaconState.RED_LEFT_BLUE_RIGHT;
            }

            if (ColorReader.isRed(rightRed, rightGreen, rightBlue)) {
                firstState = BeaconState.RED_LEFT_RED_RIGHT;
            }
        }

        long redundancyStartTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - redundancyStartTime < redundancy) {
            if (ColorReader.isBlue(leftRed, leftGreen, leftBlue)) {
                if (ColorReader.isBlue(rightRed, rightGreen, rightBlue)) {
                    if (!firstState.equals(BeaconState.BLUE_LEFT_BLUE_RIGHT)) {
                        return BeaconState.UNDECIDED;
                    }
                }

                if (ColorReader.isRed(rightRed, rightGreen, rightBlue)) {
                    if (!firstState.equals(BeaconState.BLUE_LEFT_RED_RIGHT)) {
                        return BeaconState.UNDECIDED;
                    }
                }
            }

            if (ColorReader.isRed(leftRed, leftGreen, leftBlue)) {
                if (ColorReader.isBlue(rightRed, rightGreen, rightBlue)) {
                    if (!firstState.equals(BeaconState.RED_LEFT_BLUE_RIGHT)) {
                        return BeaconState.UNDECIDED;
                    }
                }

                if (ColorReader.isRed(rightRed, rightGreen, rightBlue)) {
                    if (!firstState.equals(BeaconState.RED_LEFT_RED_RIGHT)) {
                        return BeaconState.UNDECIDED;
                    }
                }
            }
        }

        return firstState;
    }

}
