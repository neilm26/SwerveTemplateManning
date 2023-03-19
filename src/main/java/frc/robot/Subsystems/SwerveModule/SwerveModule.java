package frc.robot.Subsystems.SwerveModule;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkEntry;

public class SwerveModule extends SubsystemBase{
    
    private WPI_TalonFX driveMotor;
    private CANSparkMax turnMotor;
    

    private PIDController drivePID = SwerveConstants.DRIVE_PID_CONTROLLER;
    private PIDController angularPID = SwerveConstants.ANGULAR_PID_CONTROLLER;

    private Supplier<Double> initialVelo, initialAngle;

    private double targetVelo;

    private double currAngle;
    private double currVelo;
    private double prevTargetHeading;

    private ModuleNames moduleName;
    private Translation2d offset;

    private TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    private NetworkEntry swerveModuleHeading, headingSlider;

    public SwerveModule(int driveId, int turnId, ModuleNames moduleName, Translation2d offset) {        
        driveMotor = new WPI_TalonFX(driveId, SwerveConstants.CANIVORENAME);
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);

        initialVelo = () -> driveMotor.getSelectedSensorVelocity();
        initialAngle = () -> turnMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        this.moduleName = moduleName;
        this.offset = offset;
    }

    public void initalize() {
        //your typical kp, ki, kd, etcs.
        updateDrivePIDs(drivePID, 
                             angularPID); 
        
        Drivetrain.preAssignedModules.add(this);
        Drivetrain.identityMap.put(moduleName, offset);

        configureSwerveModule();

        try {
            headingSlider = 
            new NetworkEntry(tab, 
            "slider", 
            BuiltInWidgets.kNumberSlider, Map.of("min",0,"max",360), initialAngle.get(), moduleName.toString());

            swerveModuleHeading = 
            new NetworkEntry(tab, 
            "target heading view", 
            BuiltInWidgets.kGyro, null, headingSlider.getNetworkTblValue(), moduleName.toString());
   
        } 
        catch (NullPointerException e) {
            // TODO: handle exception
        }
    }

    private void configureSwerveModule() {
        driveConfiguration.closedloopRamp = 0.8;

        Utilities.attemptToConfigure(driveMotor.configAllSettings(driveConfiguration), 
                    "Cannot calibrate initial settings");
    }

    public void updateDrivePIDs(PIDController drivePID, PIDController angularPID) {
        this.drivePID = drivePID;
        this.angularPID = angularPID;
    }

    public void driveTo(SwerveModuleState moduleState) {
        if (moduleState.speedMetersPerSecond != 0) {

            targetVelo = moduleState.speedMetersPerSecond;
            swerveModuleHeading.getEntry().setDouble(moduleState.angle.getDegrees());
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

    public void setTargetVel(double newTargetVel) {
        targetVelo = newTargetVel;
    }

    public double getTargetAng() {
        return swerveModuleHeading.getEntry().getDouble(initialAngle.get());
    }

    public void setTargetAng(double newTargetAng) {
        swerveModuleHeading.getEntry().setDouble(newTargetAng);
    }

    public String getName() {
        return moduleName.toString();
    }

    public double[] getModuleOffset() {
        double[] vectorArray2d = new double[] {offset.getX(), offset.getY()};

        return vectorArray2d;
    }

    @Override
    public void periodic() {
        double currTargetHeading = headingSlider.getEntry().getDouble(0);

        if (currTargetHeading != prevTargetHeading) {
            setTargetAng(currTargetHeading);
            prevTargetHeading = currTargetHeading;
        }
    }
}
