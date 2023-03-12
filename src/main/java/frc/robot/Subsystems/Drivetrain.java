// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */  
  public static SwerveModule frontLeftSwerveModule;
  public static SwerveModule frontRightSwerveModule;
  public static List<SwerveModule> preAssignedModules = new ArrayList<SwerveModule>();
  public static Map<String, Translation2d> identityMap = new HashMap<String, Translation2d>();

  public Drivetrain() {
    frontLeftSwerveModule = new SwerveModule(0,3,
              "FrontLeftSwerve", SwerveConstants.FRONT_LEFT_OFFSET);

    frontRightSwerveModule = new SwerveModule(1,2,
              "FrontRightSwerve", SwerveConstants.FRONT_RIGHT_OFFSET);
    
    initializeAllModules();
  }

  private void initializeAllModules() {
    for (ModuleNames name: ModuleNames.values()) {
      getModule(name).initalize();
    }
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
  
  public Translation2d getModuleOffset(String id) {
    return identityMap.getOrDefault(id, null);
  }

  public Map<String, Translation2d> getIdentityMap() {
    return identityMap;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
