package frc.robot.Subsystems.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants;

public class SwerveMath implements SwerveConstants {

    public static double ticksToAngle(double ticks, double gearRatio) {
        double angle = (ticks % TICKS_PER_REV_ANALOG_CODER) / gearRatio;

        double result = (angle / (TICKS_PER_REV_ANALOG_CODER / 2)) * 180;

        if (result > 180) {
            result -= 360;
        }

        return result;
    }

    public static double absolutePositionToAngle(double absPos) {
        return absPos * 360;
    }

    public static double[] findFastestTurnDirection(double currAngle, double targetAngle, double speed) {
        double dir =  targetAngle - currAngle; //only applies if angles are not clamped between 0-360
        double clampedSpeed = Utilities.constrain(speed, -1, 1);
        if (Math.abs(dir) >= 180) { //if conditional is valid, we inverse control
            //dir = 180 + (-(Math.signum(dir) * 360) + dir); //the inverse of the direction (signum)
            dir =  -(Math.signum(dir) * 360) + dir;
            clampedSpeed *= -1; //inverse speed as well so it remains relative to field.
        }
        return new double[] { dir, clampedSpeed};
    }

    public static double clamp(double encPos) {
        if (encPos > 0.5) {
            return encPos - 1;
        } else if (encPos < -0.5) {
            return encPos + 1;
        }
        return encPos;
    }
}
