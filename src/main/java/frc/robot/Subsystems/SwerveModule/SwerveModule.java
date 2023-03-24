package frc.robot.Subsystems.SwerveModule;

import java.util.Map;
import java.util.function.Supplier;

import org.opencv.core.Mat.Tuple2;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Drivetrain;
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

    private double prevTargetHeading;

    private ModuleNames moduleName;
    private Translation2d offset;
    private double encoderOffset;

    private TalonSRXConfiguration driveConfiguration = new TalonSRXConfiguration();
    private TalonSRXConfiguration turnConfiguration = new TalonSRXConfiguration();

    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    private NetworkEntry swerveModuleTargetHeading, headingSlider, moduleOutput, moduleState, swerveModuleHeading;

    public SwerveModule(int driveId, int turnId, ModuleNames moduleName, Translation2d offset,
            Supplier<Double> encoderOffset, Tuple2<Integer> encoderPins) {
        driveMotor = new TalonSRX(driveId);
        turnMotor = new TalonSRX(turnId);

        // cimCoder = new Encoder(encoderPins.get_0(), encoderPins.get_1());

        // cimCoder.setDistancePerPulse(Math.PI*SwerveConstants.WHEEL_DIAMETER/SwerveConstants.TICKS_PER_REV_CIM_CODER);

        analogEncoder = new AnalogEncoder(encoderPins.get_0());

        initialVelo = () -> driveMotor.getSelectedSensorVelocity();
        initialAngle = () -> SwerveMath.clamp(analogEncoder.getAbsolutePosition() - analogEncoder.getPositionOffset())
                * 360; // needs to be changed.
        this.moduleName = moduleName;
        this.offset = offset;
        this.encoderOffset = encoderOffset.get();

        initalize();
    }

    public void initalize() {
        // your typical kp, ki, kd, etcs.
        updateDrivePIDs(drivePID,
                angularPID);

        Drivetrain.preAssignedModules.add(this);
        Drivetrain.moduleWheelPos.put(moduleName, getWheelPosition());

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
        driveConfiguration.closedloopRamp = 0.2;

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

    public void driveTo(double targetAngle) {
        // double out = -angularPID.calculate(getAbsoluteAngularPositionOffsetted(),
        // SwerveMath.findFastestTurnDirection(getAbsoluteAngularPositionOffsetted(),
        // targetAngle)/360);
        // SmartDashboard.putNumber("out", out);

        // turnMotor.set(ControlMode.PercentOutput, out);
    }

    public void easyMotion(double drive, double turn) {
        driveMotor.set(ControlMode.PercentOutput, drive);
        turnMotor.set(ControlMode.PercentOutput, turn);
    }

    public SwerveModulePosition getWheelPosition() {
        return new SwerveModulePosition(analogEncoder.getDistance(), Rotation2d.fromDegrees(initialAngle.get()));
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

    public void setDesiredState(SwerveModuleState currState) {
        // there is no selected sensor yet...
        final double driveOutput = drivePID.calculate(driveMotor.getSelectedSensorVelocity(),
                currState.speedMetersPerSecond);
        final double driveFF = driveFeedForward.calculate(currState.speedMetersPerSecond);
        final double[] constrainedTurning = SwerveMath.findFastestTurnDirection(
                SwerveMath.clamp(analogEncoder.getAbsolutePosition() - encoderOffset) * 360,
                currState.angle.getDegrees(), driveOutput);

        final double turnOutput = angularPID.calculate(constrainedTurning[0] / 360);
        final double turnFF = angularFeedForward.calculate(angularPID.getSetpoint().velocity);

        turnMotor.set(ControlMode.PercentOutput, turnOutput);
        driveMotor.set(ControlMode.PercentOutput, constrainedTurning[1]);
    }

    public double[] getModuleOffset() {
        double[] vectorArray2d = new double[] { offset.getX(), offset.getY() };

        return vectorArray2d;
    }

    @Override
    public void periodic() {
        Boolean overrideControls = (Boolean) NetworkTableContainer.entries.get("Override Target Heading")
                .getNetworkTblValue();

        double currTargetHeading = headingSlider.getEntry().getDouble(0);
        swerveModuleHeading.getEntry().setDouble(
                SwerveMath.clamp(analogEncoder.getAbsolutePosition() - encoderOffset) * 360);
                
        if (overrideControls) {
            setTargetAng(currTargetHeading);
            SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(currTargetHeading));

            setDesiredState(state);
        }
    }
}
