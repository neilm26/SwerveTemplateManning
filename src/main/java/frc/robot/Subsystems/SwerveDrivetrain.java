// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class SwerveDrivetrain extends SubsystemBase implements SwerveConstants {
  /** Creates a new Drivetrain. */
  public static SwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule;
  public static List<SwerveModule> preAssignedModules = new ArrayList<SwerveModule>();

  public static Map<ModuleNames, SwerveModuleState> stateMap = new HashMap<>();
  public static Map<ModuleNames, SwerveModulePosition> moduleWheelPos = new HashMap<ModuleNames, SwerveModulePosition>();

  private SwerveDriveKinematics driveKinematics;
  private SwerveDriveOdometry driveOdometry;
  private ChassisSpeeds chassisSpeeds;
  private ProfiledPIDController angularPID = ANGULAR_PID_CONTROLLER; //defaults.
  private PIDController drivePID = DRIVE_PID_CONTROLLER;
  private WPI_Pigeon2 pigeon2;

  public SwerveDrivetrain() {
    frontRightSwerveModule = new SwerveModule(1, 0, false,
        ModuleNames.FRONT_RIGHT,
        () -> HOME_ANALOG_ENC_POS_FRONT_RIGHT,
        new Tuple2<Integer>(0, 99));

    frontLeftSwerveModule = new SwerveModule(2, 3, true,
        ModuleNames.FRONT_LEFT,
        () -> HOME_ANALOG_ENC_POS_FRONT_LEFT,
        new Tuple2<Integer>(1, 99));

    backLeftSwerveModule = new SwerveModule(5, 4, true, 
        ModuleNames.BACK_LEFT,
        () -> -0.07777777777, //change to backleft!
        new Tuple2<Integer>(3, 99));
    
    backRightSwerveModule = new SwerveModule(7, 6, false,
        ModuleNames.BACK_RIGHT,
        () -> -0.0, //change to backleft!
        new Tuple2<Integer>(2, 99));


    pigeon2 = new WPI_Pigeon2(SensorConstants.PIGEON_ID);

    SmartDashboard.putData("angularPID", angularPID);
    SmartDashboard.putData("drivePID", drivePID);

    initializeAllModules();
  }

  private void initializeAllModules() {
    try {
      driveKinematics = new SwerveDriveKinematics(OFFSET_ARRAY);
      // driveOdometry = new SwerveDriveOdometry(driveKinematics,
      // new Rotation2d(getRobotHeading()),
      // moduleWheelPos.values().toArray(new
      // SwerveModulePosition[moduleWheelPos.size()]));
      setCentralMotion(new ChassisSpeeds(0, 0, 0));

    } catch (NullPointerException e) {
      DriverStation.reportError("Cannot initialize modules! Please verify that module name(s) exist!", true);
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  public SwerveDriveOdometry getOdometry() {
    return driveOdometry;
  }

  public double getRobotHeading() {
    return pigeon2.getAbsoluteCompassHeading();
  }

  public void setCentralMotion(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;

    SwerveModuleState[] states = driveKinematics.toSwerveModuleStates(this.chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);

    stateMap = IntStream.range(0, ModuleNames.values().length).boxed().collect(Collectors.toMap(
        i -> ModuleNames.values()[i],
        i -> states[i]));
  }

  public List<SwerveModule> getModules() {
    return preAssignedModules;
  }

  public SwerveModule getModule(ModuleNames name) {
    switch (name) {

      case FRONT_LEFT:
        return frontLeftSwerveModule;
      case FRONT_RIGHT:
        return frontRightSwerveModule;
      case BACK_LEFT:
        return backLeftSwerveModule;
      case BACK_RIGHT:
        return backRightSwerveModule;

      default:
        break;
    }
    return null;
  }

  public SwerveModuleState updateModuleState(double speed, double orientation, SwerveModule module) {
    SwerveModuleState state = new SwerveModuleState(speed, Rotation2d.fromDegrees(orientation));
    if (state != null) {
      module.getModuleState().setNetworkEntryValue(state.toString());
      stateMap.replace(module.getModuleName(), state);
    }
    
    return state;
  }

  public SwerveModuleState getModuleState(ModuleNames moduleName) {
    return stateMap.getOrDefault(moduleName, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (SwerveModule module : getModules()) {
      // try and update each module's state unless it cannot be found, then return an
      // empty module state.
      module.updateDrivePIDs(drivePID, angularPID);
    }
  }
}
