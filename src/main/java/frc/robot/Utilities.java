package frc.robot;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.DriverStation;

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
}
