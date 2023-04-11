package frc.robot.Subsystems.SwerveModule;

import java.util.Map;
import java.util.function.Supplier;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.SwerveDrivetrain;
import frc.robot.Subsystems.Networking.NetworkEntry;
import frc.robot.Subsystems.Networking.NetworkTableContainer;

public class SwerveModule extends SubsystemBase implements SwerveConstants {

    private TalonSRX driveMotor;
    private TalonSRX turnMotor; // change to SRX and Victors on Loki
    private Encoder cimCoder;
    private AnalogEncoder analogEncoder;

    private PIDController drivePID = DRIVE_PID_CONTROLLER;
    private ProfiledPIDController angularPID = ANGULAR_PID_CONTROLLER;

    private SimpleMotorFeedforward driveFeedForward = DRIVE_FEEDFORWARD;
    private SimpleMotorFeedforward angularFeedForward = TURN_FEEDFORWARD;

    private Supplier<Double> initialVelo, initialAngle;

    private ModuleNames moduleName;
    private double encoderOffset;

    private TalonSRXConfiguration driveConfiguration = new TalonSRXConfiguration();
    private TalonSRXConfiguration turnConfiguration = new TalonSRXConfiguration();

    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    private NetworkEntry swerveModuleTargetHeading, headingSlider, moduleOutput, moduleState, swerveModuleHeading;

    public SwerveModule(int driveId, int turnId, boolean isReversed, ModuleNames moduleName,
            Supplier<Double> encoderOffset, Tuple2<Integer> encoderPins) {
        driveMotor = new TalonSRX(driveId);
        turnMotor = new TalonSRX(turnId);

        driveMotor.setInverted(isReversed);

        // cimCoder = new Encoder(encoderPins.get_0(), encoderPins.get_1());

        // cimCoder.setDistancePerPulse(Math.PI*SwerveConstants.WHEEL_DIAMETER/SwerveConstants.TICKS_PER_REV_CIM_CODER);

        analogEncoder = new AnalogEncoder(encoderPins.get_0());

        initialVelo = () -> driveMotor.getSelectedSensorVelocity();
        initialAngle = () -> getModuleAngle();
        this.moduleName = moduleName;
        this.encoderOffset = encoderOffset.get();

        initalize();
    }

    public void initalize() {
        // your typical kp, ki, kd, etcs.
        updateDrivePIDs(drivePID,
                angularPID);

        SwerveDrivetrain.preAssignedModules.add(this);
        SwerveDrivetrain.moduleWheelPos.put(moduleName, getWheelPosition());

        configureSwerveModule();

        try {
            headingSlider = new NetworkEntry(tab,
                    "slider",
                    BuiltInWidgets.kNumberSlider, Map.of("min", 0, "max", 360), initialAngle.get(),
                    moduleName.toString());

            swerveModuleTargetHeading = new NetworkEntry(tab,
                    "target heading view",
                    BuiltInWidgets.kGyro, null, initialAngle.get(), moduleName.toString());

            moduleOutput = new NetworkEntry(tab,
                    "motor output velocity",
                    BuiltInWidgets.kTextView, null, initialVelo.get(), moduleName.toString());

            moduleState = new NetworkEntry(tab,
                    "module state",
                    BuiltInWidgets.kTextView, null, "", moduleName.toString());

            swerveModuleHeading = new NetworkEntry(tab,
                    "current heading view",
                    BuiltInWidgets.kGyro, null, initialAngle.get(), moduleName.toString());

        } catch (NullPointerException e) {
            // TODO: handle exception
            DriverStation.reportWarning("Module Entries cannot be setup!", true);
        }
    }

    private void configureSwerveModule() {
        driveMotor.configFactoryDefault();
        turnMotor.configFactoryDefault();

        driveConfiguration.closedloopRamp = 0.08;
        driveConfiguration.peakCurrentLimit = 30;
        driveConfiguration.peakOutputForward = 0.4;
        driveConfiguration.peakOutputReverse = -0.4;
        driveConfiguration.peakCurrentDuration = 250;

        
        turnConfiguration.peakCurrentDuration = 30;
        turnConfiguration.peakCurrentDuration = 250;
        turnConfiguration.feedbackNotContinuous = false;

        Utilities.attemptToConfigure(driveMotor.configAllSettings(turnConfiguration),
                "Cannot calibrate initial turn settings");
        Utilities.attemptToConfigure(driveMotor.configAllSettings(driveConfiguration),
                "Cannot calibrate initial drive settings");
    }

    public void updateDrivePIDs(PIDController drivePID, ProfiledPIDController angularPID) {
        this.drivePID = drivePID;
        this.angularPID = angularPID;
    }

    public void easyMotion(double drive, double turn) {
        driveMotor.set(ControlMode.PercentOutput, drive);
        turnMotor.set(ControlMode.PercentOutput, turn);
    }

    public SwerveModulePosition getWheelPosition() {
        return new SwerveModulePosition(analogEncoder.getDistance(), Rotation2d.fromDegrees(initialAngle.get()));
    }

    public double getModuleAngle() {
        return (SwerveMath.clamp(analogEncoder.getAbsolutePosition() - encoderOffset)) * 360;
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

    public NetworkEntry getModuleState() {
        return moduleState;
    }

    public void 
    setDesiredState(SwerveModuleState currState) {
        // there is no selected sensor yet...
        double target = currState.angle.getDegrees();
        setTargetAng(target);
        
        swerveModuleHeading.getEntry().setDouble(getModuleAngle());

        final double driveOutput = drivePID.calculate(driveMotor.getSelectedSensorVelocity(),
                currState.speedMetersPerSecond);
        final double driveFF = driveFeedForward.calculate(currState.speedMetersPerSecond);
        final double[] constrainedTurning = SwerveMath.calculateFastestTurn(
                getModuleAngle(),
                target, driveOutput);

        final double turnOutput = angularPID.calculate(constrainedTurning[0] / 360);
        final double turnFF = angularFeedForward.calculate(angularPID.getSetpoint().velocity);

        turnMotor.set(ControlMode.PercentOutput, turnOutput);
        driveMotor.set(ControlMode.PercentOutput, constrainedTurning[1]);
    }

    public boolean throwLostEncoderException() {
        Utilities.attemptToConfigureThrow(ErrorCode.SensorNotPresent, String.valueOf(analogEncoder.getChannel()));
        return true;
    }

    public void singlePointTo() {
        double currTargetHeading = headingSlider.getEntry().getDouble(0);
        swerveModuleHeading.getEntry().setDouble(getModuleAngle());

        setTargetAng(currTargetHeading);
        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(currTargetHeading));

        setDesiredState(state);
    }

    @Override
    public void periodic() {
        swerveModuleHeading.getEntry().setDouble(getModuleAngle());
    }
}
