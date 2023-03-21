// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Utilities;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Networking.NetworkTableContainer;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class Drivetrain extends SubsystemBase implements SwerveConstants {
  /** Creates a new Drivetrain. */  
  public static SwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule;
  public static List<SwerveModule> preAssignedModules = new ArrayList<SwerveModule>();

  public static Map<ModuleNames, SwerveModuleState> stateMap = new HashMap<>();
  public static Map<ModuleNames, SwerveModulePosition> moduleWheelPos = new HashMap<ModuleNames, SwerveModulePosition>();

  

  private SwerveDriveKinematics driveKinematics;
  private SwerveDriveOdometry driveOdometry;
  private ChassisSpeeds chassisSpeeds;

  private WPI_Pigeon2 pigeon2;

  public Drivetrain() {
    frontRightSwerveModule = new SwerveModule(3,5,
              ModuleNames.FRONT_RIGHT, FRONT_RIGHT_OFFSET, new Tuple2<Integer>(2, 1));

    frontLeftSwerveModule = new SwerveModule(4,0,
              ModuleNames.FRONT_LEFT, FRONT_LEFT_OFFSET, new Tuple2<Integer>(3, 0));

    backLeftSwerveModule = new SwerveModule(1,2,
              ModuleNames.BACK_LEFT, BACK, new Tuple2<Integer>(4, 5));

    // backRightSwerveModule = new SwerveModule(6,7,
    //           ModuleNames.BACK_RIGHT, BACK, new Tuple2<Integer>(6, 7));
    
    //pigeon2 = new WPI_Pigeon2(SensorConstants.PIGEON_ID);
    
    initializeAllModules();
  }

  private void initializeAllModules() {
    SmartDashboard.putData(new PIDController(0, 0, 0));
    try {
      driveKinematics = new SwerveDriveKinematics(OFFSET_ARRAY);
      // driveOdometry = new SwerveDriveOdometry(driveKinematics, 
      //                 new Rotation2d(getRobotHeading()), 
      //                 moduleWheelPos.values().toArray(new SwerveModulePosition[moduleWheelPos.size()]));
      setCentralMotion(new ChassisSpeeds(0, 0, 0));
                      
    } catch (NullPointerException e) {
      DriverStation.reportError("Cannot initialize modules! Please verify that module name(s) exist!", true);
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  public Map<ModuleNames, SwerveModuleState> getSwerveStates() {
    return stateMap;
  }

  public SwerveDriveOdometry getOdometry() {
    return driveOdometry;
  }

  public double getRobotHeading() {
    return pigeon2.getAbsoluteCompassHeading();
  }

  public void setCentralMotion(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds; 

    stateMap = IntStream.range(0, ModuleNames.values().length).boxed().collect(Collectors.toMap(
                i->ModuleNames.values()[i], 
                i->driveKinematics.toSwerveModuleStates(this.chassisSpeeds)[i]));
  }

  public List<SwerveModule> getModules() {
    return preAssignedModules;
  }
  

  public SwerveModule getModule(ModuleNames name) {
    switch (name) {

      case FRONT_LEFT: return frontLeftSwerveModule;
      case FRONT_RIGHT: return frontRightSwerveModule;
      case BACK_LEFT: return backLeftSwerveModule;

      default:
        break;
    }
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (SwerveModule module: getModules()) { 
        module.updateModuleState(stateMap.getOrDefault(module.getModuleName(), null));
    }
  }
}
