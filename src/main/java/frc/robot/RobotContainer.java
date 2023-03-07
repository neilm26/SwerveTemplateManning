// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Networking.NetworkTables;
import frc.robot.Subsystems.SwerveModule.SwerveConstructor;
import frc.robot.Subsystems.SwerveModule.SwerveModule;
import frc.robot.Constants.*;

public class RobotContainer {

  private SwerveConstructor swerveConstructor;

  private SwerveModule parentModule;

  Tuple2[] swerveOffsets = new Tuple2[] {SwerveConstants.tmp, SwerveConstants.tmp2};

  private HashMap<String, Object> temporary = new HashMap<>();
  

  public RobotContainer() {
    configureBindings();
    constructSwerveModules();

    temporary.put("Configured Modules?", true);
    temporary.put("Configured?", 9999);
    temporary.put("running!", new double[] {3,3});


    for (int i=0;i<swerveConstructor.GetIDMap().size();i++) {
      Object id = swerveConstructor.GetIDMap().keySet().toArray()[i];
      Translation2d translation = (Translation2d) swerveConstructor.GetIDMap().values().toArray()[i];

      temporary.put(id.toString()+"Module", translation);
    }

    NetworkTables table = constructTableTab("Swerve", temporary);
    SmartDashboard.putNumberArray("AHHHHH", (double[]) table.getEntry("running!"));
  }

  private void constructSwerveModules() {
    swerveConstructor = new SwerveConstructor(() -> parentModule, swerveOffsets);

    SmartDashboard.putNumber("X:", swerveConstructor.GetModuleOffset(1).getX());
  }

  public NetworkTables constructTableTab(String tabName, HashMap<String, ?> details) {
    NetworkTables localTable = new NetworkTables(tabName, details);
    
    return localTable;
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
