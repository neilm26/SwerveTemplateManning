package frc.robot.Subsystems.SwerveModule;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Drivetrain;

public class SwerveModule {
    
    private WPI_TalonFX driveMotor;
    private CANSparkMax turnMotor;

    private PIDController drivePID;
    private PIDController angularPID;

    private Supplier<Double> initialVelo, initialAngle;

    private double targetAngle;
    private double targetVelo;
    private String moduleName;
    private Translation2d offset;


    public SwerveModule(int driveId, int turnId, String moduleName, Translation2d offset) {
        driveMotor = new WPI_TalonFX(driveId, SwerveConstants.CANIVORENAME);
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);

        initialVelo = () -> driveMotor.getSelectedSensorVelocity();
        initialAngle = () -> turnMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        this.moduleName = moduleName;
        this.offset = offset;
    }

    public void initalize() {
        //your typical kp, ki, kd, etcs.
        updateDrivePIDs(new PIDController(0, 0, 0), 
                             new PIDController(0, 0, 0)); 
        
        Drivetrain.preAssignedModules.add(this);
        Drivetrain.identityMap.put(moduleName, offset);
    }

    public void updateDrivePIDs(PIDController drivePID, PIDController angularPID) {
        this.drivePID = drivePID;
        this.angularPID = angularPID;
    }

    public void driveTo(SwerveModuleState moduleState) {
        if (moduleState.speedMetersPerSecond != 0) {

            targetVelo = moduleState.speedMetersPerSecond;
            targetAngle = moduleState.angle.getDegrees();
        }
        else {
            targetVelo = 0;
        }
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(initialVelo.get(), Rotation2d.fromDegrees(initialAngle.get()));
    }

    public void resetHeading() {
        initialAngle = () -> turnMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

    public double getTargetVel() {
        return targetVelo;
    }

    public double getTargetAng() {
        return targetAngle;
    }

    public double getCurrVel() {
        return initialVelo.get();
    }

    public double getCurrAng() {
        return initialAngle.get();
    }

    public String getName() {
        return moduleName;
    }

    public Translation2d getModuleOffset() {
        return offset;
    }
}
