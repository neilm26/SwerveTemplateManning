package frc.robot.Subsystems.SwerveModule;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkEntry;
import frc.robot.Subsystems.Networking.NetworkTableContainer;

public class SwerveModule extends SubsystemBase implements SwerveConstants {
    
    private TalonSRX driveMotor;
    private TalonSRX turnMotor; //change to SRX and Victors on Loki
    private Encoder cimCoder;
    private AnalogEncoder analogEncoder;
    

    private PIDController drivePID = DRIVE_PID_CONTROLLER;
    private PIDController angularPID = ANGULAR_PID_CONTROLLER;

    private Supplier<Double> initialVelo, initialAngle;

    private double prevTargetHeading;

    private ModuleNames moduleName;
    private Translation2d offset;

    private TalonSRXConfiguration driveConfiguration = new TalonSRXConfiguration();
    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    private NetworkEntry swerveModuleTargetHeading, headingSlider, moduleOutput, moduleState, swerveModuleHeading;

    public SwerveModule(int driveId, int turnId, ModuleNames moduleName, Translation2d offset, Tuple2<Integer> encoderPins) {        
        driveMotor = new TalonSRX(driveId);
        turnMotor = new TalonSRX(turnId);
        
        //cimCoder = new Encoder(encoderPins.get_0(), encoderPins.get_1());
        
        //cimCoder.setDistancePerPulse(Math.PI*SwerveConstants.WHEEL_DIAMETER/SwerveConstants.TICKS_PER_REV_CIM_CODER);

        analogEncoder = new AnalogEncoder(encoderPins.get_0());

        initialVelo = () -> driveMotor.getSelectedSensorVelocity();
        initialAngle = () -> ticksToAngle(analogEncoder.getAbsolutePosition()); //needs to be changed.
        this.moduleName = moduleName;
        this.offset = offset;

        initalize();
    }

    public void initalize() {
        //your typical kp, ki, kd, etcs.
        updateDrivePIDs(drivePID, 
                             angularPID); 
        
        Drivetrain.preAssignedModules.add(this);
        Drivetrain.moduleWheelPos.put(moduleName, getWheelPosition());

        configureSwerveModule();

        try {
            headingSlider = 
            new NetworkEntry(tab, 
            "slider", 
            BuiltInWidgets.kNumberSlider, Map.of("min",0,"max",360), initialAngle.get(), moduleName.toString());

            swerveModuleTargetHeading = 
            new NetworkEntry(tab, 
            "target heading view", 
            BuiltInWidgets.kGyro, null, headingSlider.getNetworkTblValue(), moduleName.toString());

            moduleOutput = 
            new NetworkEntry(tab, 
            "motor output velocity", 
            BuiltInWidgets.kTextView, null, initialVelo.get(), moduleName.toString());

            moduleState = 
            new NetworkEntry(tab, 
            "module state", 
            BuiltInWidgets.kTextView, null, "", moduleName.toString());

            swerveModuleHeading = 
            new NetworkEntry(tab, 
            "current heading view", 
            BuiltInWidgets.kGyro, null, initialAngle.get(), moduleName.toString());

            NetworkTableContainer.entries.putAll(
                Map.of(headingSlider.toString(), headingSlider,
                        swerveModuleTargetHeading.toString(), swerveModuleTargetHeading,
                        moduleOutput.toString(), moduleOutput,
                        moduleState.toString(), moduleState,
                        swerveModuleHeading.toString(), swerveModuleHeading));
   
        } 
        catch (NullPointerException e) {
            // TODO: handle exception
        }
    }

    private void configureSwerveModule() {
        driveConfiguration.closedloopRamp = 0.2;

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

    public double ticksToAngle(double ticks) {
        double angle = ticks % TICKS_PER_REV_ANALOG_CODER;

        double result = (angle / (TICKS_PER_REV_ANALOG_CODER /2))*180;

        if (result > 180) {
            result -= 360;
        }

        return result;
    }

    public void easyDrive(double drive) {
        driveMotor.set(ControlMode.PercentOutput, drive);
    }
    public void easyTurn (double turn) {
        turnMotor.set(ControlMode.PercentOutput, turn);
    }

    public SwerveModulePosition getWheelPosition() {
        return new SwerveModulePosition(analogEncoder.getDistance(), Rotation2d.fromDegrees(initialAngle.get()));
    }

    public void resetHeading() {
        initialAngle = () -> ticksToAngle(analogEncoder.getAbsolutePosition());
    }

    public double getTargetVel() {
        return moduleOutput.getEntry().getDouble(initialVelo.get());
    }

    public void setTargetVel(double newTargetVel) {
        moduleOutput.getEntry().setDouble(newTargetVel);
    }

    public double getTargetAng() {
        return swerveModuleTargetHeading.getEntry().getDouble(initialAngle.get());
    }

    public void setTargetAng(double newTargetAng) {
        swerveModuleTargetHeading.getEntry().setDouble(newTargetAng);
    }

    public ModuleNames getModuleName() {
        return moduleName;
    }

    public double[] getModuleOffset() {
        double[] vectorArray2d = new double[] {offset.getX(), offset.getY()};

        return vectorArray2d;
    }

    public void updateModuleState(SwerveModuleState state) {
        if (state != null) {
            moduleState.setNetworkEntryValue(state.toString());
        }
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
