package frc.robot.Subsystems.SwerveModule;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkEntry;
import frc.robot.Subsystems.Networking.NetworkTableContainer;

public class SwerveModule extends SubsystemBase{
    
    private WPI_TalonFX driveMotor;
    private CANSparkMax turnMotor; //change to SRX and Victors on Loki
    private Encoder cimCoder;
    

    private PIDController drivePID = SwerveConstants.DRIVE_PID_CONTROLLER;
    private PIDController angularPID = SwerveConstants.ANGULAR_PID_CONTROLLER;

    private Supplier<Double> initialVelo, initialAngle;

    private double prevTargetHeading;

    private ModuleNames moduleName;
    private Translation2d offset;

    private TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    private NetworkEntry swerveModuleHeading, headingSlider, moduleOutput;

    public SwerveModule(int driveId, int turnId, ModuleNames moduleName, Translation2d offset, Tuple2<Integer> encoderPins) {        
        driveMotor = new WPI_TalonFX(driveId, SwerveConstants.CANIVORENAME);
        turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);
        
        cimCoder = new Encoder(encoderPins.get_0(), encoderPins.get_1());
        cimCoder.setDistancePerPulse(Math.PI*SwerveConstants.WHEEL_DIAMETER/SwerveConstants.TICKS_PER_REV_CIM_CODER);

        initialVelo = () -> driveMotor.getSelectedSensorVelocity();
        initialAngle = () -> turnMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        this.moduleName = moduleName;
        this.offset = offset;

        initalize();
    }

    public void initalize() {
        //your typical kp, ki, kd, etcs.
        updateDrivePIDs(drivePID, 
                             angularPID); 
        
        Drivetrain.preAssignedModules.add(this);
        Drivetrain.identityMap.put(moduleName, offset);
        Drivetrain.moduleWheelPos.put(moduleName, getWheelPosition());

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

            moduleOutput = 
            new NetworkEntry(tab, 
            "motor output velocity", 
            BuiltInWidgets.kTextView, null, initialVelo.get(), moduleName.toString());

            NetworkTableContainer.entries.putAll(
                Map.of(headingSlider.toString(), headingSlider,
                        swerveModuleHeading.toString(), swerveModuleHeading,
                        moduleOutput.toString(), moduleOutput));
   
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

            setTargetVel(moduleState.speedMetersPerSecond);
            setTargetAng(moduleState.angle.getDegrees());
        }
        else {
            moduleOutput.getEntry().setDouble(0);
        }
    }

    public SwerveModuleState getCurrentState(double speed, double angle) {
        return new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));
    }

    public SwerveModulePosition getWheelPosition() {
        return new SwerveModulePosition(cimCoder.getDistance(), Rotation2d.fromDegrees(initialAngle.get()));
    }

    public void resetHeading() {
        initialAngle = () -> turnMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

    public double getTargetVel() {
        return moduleOutput.getEntry().getDouble(initialVelo.get());
    }

    public void setTargetVel(double newTargetVel) {
        moduleOutput.getEntry().setDouble(newTargetVel);
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
