package frc.robot;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.SensorConstants;

public class Utilities {
    public static void attemptToConfigure(ErrorCode errorCode, String errorMsg) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(errorMsg + " " + errorCode, false);
        }
    }

    public static void attemptToConfigureThrow(ErrorCode errorCode, String errorMsg) {
        if (errorCode != ErrorCode.OK) {
            throw new RuntimeException(errorMsg + " " + errorCode);
        }
    }

    public static Object convertAxesToDegrees(double xAxis, double yAxis) {
        if (Math.abs(xAxis) < SensorConstants.JOYSTICK_DEAD_ZONE && 
            Math.abs(yAxis) < SensorConstants.JOYSTICK_DEAD_ZONE) {
            return null;
        }
        double angle = Math.toDegrees(Math.atan2(yAxis, xAxis))*-1;

        return angle = angle < 0 ? angle+=360 : angle;
    }
}
