package frc.robot.util;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.XboxController;

/** Checks Xbox controller buttons that are not included in the default stuff */
public class ControllerUtils {
    private static final int dpadAngleTolerance = 25;
    private static final double triggerTolerance = 0.2;

    // If pov (D-Pad) is not pressed, then .getPOV() returns -1.
    /**
     * Returns if the up button is pressed.
     * 
     * @param controller the controller
     * @return if the up button is pressed.
     */
    public static boolean dPadUp(XboxController controller) {
        return controller.getPOV() != -1 ? MathUtil.isNear(0, controller.getPOV(), dpadAngleTolerance, 0, 360) : false;
    }

    /**
     * Returns if the right button is pressed.
     * 
     * @param controller the controller
     * @return if the right button is pressed.
     */
    public static boolean dPadRight(XboxController controller) {
        return controller.getPOV() != -1 ? MathUtil.isNear(90, controller.getPOV(), dpadAngleTolerance, 0, 360) : false;
    }

    /**
     * Returns if the down button is pressed.
     * 
     * @param controller the controller
     * @return if the down button is pressed.
     */
    public static boolean dPadDown(XboxController controller) {
        return controller.getPOV() != -1 ? MathUtil.isNear(180, controller.getPOV(), dpadAngleTolerance, 0, 360) : false;
    }

    /**
     * Returns if the left button is pressed.
     * 
     * @param controller the controller
     * @return if the left button is pressed.
     */
    public static boolean dPadLeft(XboxController controller) {
        return controller.getPOV() != -1 ? MathUtil.isNear(270, controller.getPOV(), dpadAngleTolerance, 0, 360) : false;
    }

    /**
     * Returns if the right trigger is pressed.
     * 
     * @param controller the controller
     * @return if the right trigger is pressed.
     */
    public static boolean rightTrigger(XboxController controller) {
        return controller.getRightTriggerAxis() < triggerTolerance;
    }
}