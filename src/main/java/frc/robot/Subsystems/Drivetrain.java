// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */  
  public static SwerveModule frontLeftSwerveModule, frontRightSwerveModule, backLeftSwerveModule, backRightSwerveModule;
  public static List<SwerveModule> preAssignedModules = new ArrayList<SwerveModule>();

  public static Map<ModuleNames, Translation2d> identityMap = new HashMap<ModuleNames, Translation2d>();
  public static Map<ModuleNames, SwerveModulePosition> moduleWheelPos = new HashMap<ModuleNames, SwerveModulePosition>();

  

  private SwerveDriveKinematics driveKinematics;
  private SwerveDriveOdometry driveOdometry;

  private WPI_Pigeon2 pigeon2;

  public Drivetrain() {
    frontRightSwerveModule = new SwerveModule(1,2,
              ModuleNames.FRONT_RIGHT, SwerveConstants.FRONT_RIGHT_OFFSET, new Tuple2<Integer>(0, 1));

    frontLeftSwerveModule = new SwerveModule(0,3,
              ModuleNames.FRONT_LEFT, SwerveConstants.FRONT_LEFT_OFFSET, new Tuple2<Integer>(2, 3));

    backLeftSwerveModule = new SwerveModule(4,5,
              ModuleNames.BACK_LEFT, SwerveConstants.BACK, new Tuple2<Integer>(4, 5));

    backRightSwerveModule = new SwerveModule(6,7,
              ModuleNames.BACK_RIGHT, SwerveConstants.BACK, new Tuple2<Integer>(6, 7));
    
    pigeon2 = new WPI_Pigeon2(SensorConstants.PIGEON_ID);
    
    initializeAllModules();
  }

  private void initializeAllModules() {
    SmartDashboard.putData(new PIDController(0, 0, 0));
    try {
      driveKinematics = new SwerveDriveKinematics(identityMap.values().toArray(new Translation2d[identityMap.size()]));
      driveOdometry = new SwerveDriveOdometry(driveKinematics, 
                      new Rotation2d(pigeon2.getAbsoluteCompassHeading()), 
                      moduleWheelPos.values().toArray(new SwerveModulePosition[moduleWheelPos.size()]));
                      
    } catch (NullPointerException e) {
      DriverStation.reportError("Cannot initialize modules! Please verify that module name(s) exist!", true);
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  public List<SwerveModule> getModules() {
    return preAssignedModules;
  }

  public SwerveModule getModule(SwerveConstants.ModuleNames name) {
    switch (name) {

      case FRONT_LEFT: return frontLeftSwerveModule;
      case FRONT_RIGHT: return frontRightSwerveModule;

      default:
        break;
    }
    return null;
  }
  
  public Translation2d getModuleOffset(ModuleNames id) {
    return identityMap.getOrDefault(id, null);
  }

  public Map<ModuleNames, Translation2d> getIdentityMap() {
    return identityMap;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("fr_readings:", getModule(ModuleNames.FRONT_RIGHT).getTargetAng());
    SmartDashboard.putNumber("fl_readings:", getModule(ModuleNames.FRONT_LEFT).getTargetAng());

    SmartDashboard.putNumber("fl_velo_readings:", getModule(ModuleNames.FRONT_LEFT).getTargetVel());
  }
}
