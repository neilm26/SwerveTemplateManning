// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */  
  public static SwerveModule frontLeftSwerveModule;
  public static SwerveModule frontRightSwerveModule;
  public static List<SwerveModule> preAssignedModules = new ArrayList<SwerveModule>();
  public static Map<ModuleNames, Translation2d> identityMap = new HashMap<ModuleNames, Translation2d>();
  

  private SwerveDriveKinematics driveKinematics;

  public Drivetrain() {
    frontRightSwerveModule = new SwerveModule(1,2,
              ModuleNames.FRONT_RIGHT, SwerveConstants.FRONT_RIGHT_OFFSET);

    frontLeftSwerveModule = new SwerveModule(0,3,
              ModuleNames.FRONT_LEFT, SwerveConstants.FRONT_LEFT_OFFSET);
    
    initializeAllModules();
  }

  private void initializeAllModules() {
    try {
      for (ModuleNames name: ModuleNames.values()) {
        getModule(name).initalize();
      }
      driveKinematics = new SwerveDriveKinematics(identityMap.values().toArray(new Translation2d[identityMap.size()]));
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

  }
}
