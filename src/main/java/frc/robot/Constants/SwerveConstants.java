package frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface SwerveConstants {
    public int MODULE_COUNT = 4;
    public String CANIVORENAME = "canivore1";

    public double xOffsetFrontRight = 0.381; // example value
    public double yOffsetFrontRight = 0.381; // example value

    public double xOffsetFrontLeft = -0.381; // example value
    public double yOffsetFrontLeft = 0.381; // example value

    public double WHEEL_DIAMETER = 3; // (inches)
    public double TICKS_PER_REV_CIM_CODER = 1024;
    public double TICKS_PER_REV_ANALOG_CODER = 4096;

    public double HOME_ANALOG_ENC_POS_FRONT_RIGHT = -0.06944444444;

    public double HOME_ANALOG_ENC_POS_FRONT_LEFT = 0.1111111;

    public double A = -0.06944444;

    public double DESIRED_VEL_X_AXIS = 5.0; // m/s left-right
    public double DESIRED_VEL_Y_AXIS = 5.0; // m/s up-down
    public double DESIRED_RAD_SPEED = 0.5; // rad/s
    public double MAX_SPEED = 10.0;
    public double MAX_ACCEL = 5.0;

    //Driving encoder gear of module MK1 is 48 teeth.
 

    public Translation2d FRONT_RIGHT_OFFSET = new Translation2d(xOffsetFrontRight, yOffsetFrontRight);
    public Translation2d FRONT_LEFT_OFFSET = new Translation2d(xOffsetFrontLeft, yOffsetFrontLeft);
    public Translation2d BACK = new Translation2d(xOffsetFrontLeft, yOffsetFrontLeft);

    public Translation2d[] OFFSET_ARRAY = new Translation2d[] { FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET, BACK, BACK };

    public PIDController DRIVE_PID_CONTROLLER = new PIDController(1, 0, 0);
    public ProfiledPIDController ANGULAR_PID_CONTROLLER = new ProfiledPIDController(2, 0.05, 0, 
                            new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCEL));
    public SimpleMotorFeedforward TURN_FEEDFORWARD = new SimpleMotorFeedforward(1, 0.5);
    public SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(1, 0.5);


    public enum ModuleNames {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        // BACK_RIGHT
    }

}