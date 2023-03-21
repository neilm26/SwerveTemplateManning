package frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public interface SwerveConstants {
    public static final int MODULE_COUNT = 4;
    public static final String CANIVORENAME = "canivore1";

    public static final double xOffsetFrontRight = 0.381; //example value
    public static final double yOffsetFrontRight = 0.381; //example value

    public static final double xOffsetFrontLeft = -0.381; //example value
    public static final double yOffsetFrontLeft = 0.381; //example value

    public static final double WHEEL_DIAMETER = 3; //(inches)
    public static final double TICKS_PER_REV_CIM_CODER = 1024;
    public static final double TICKS_PER_REV_ANALOG_CODER = 4096;


    public static final double DESIRED_VEL_X_AXIS = 5.0; // m/s left-right
    public static final double DESIRED_VEL_Y_AXIS = 5.0; // m/s up-down
    public static final double DESIRED_RAD_SPEED = 0.5; // rad/s


    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(xOffsetFrontRight, yOffsetFrontRight);
    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(xOffsetFrontLeft, yOffsetFrontLeft);
    public static final Translation2d BACK = new Translation2d(xOffsetFrontLeft, yOffsetFrontLeft);

    public static final Translation2d[] OFFSET_ARRAY = new Translation2d[] {FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET, BACK, BACK};


    public static final PIDController DRIVE_PID_CONTROLLER = new PIDController(0, 0, 0);
    public static final PIDController ANGULAR_PID_CONTROLLER = new PIDController(0, 0, 0);

    public enum ModuleNames {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        // BACK_RIGHT
    }

}