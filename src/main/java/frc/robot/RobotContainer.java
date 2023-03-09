// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.TreeSet;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkEntry;
import frc.robot.Subsystems.Networking.NetworkTables;

public class RobotContainer {

  private TreeSet<NetworkEntry> temporary = new TreeSet<>();

  private Drivetrain drivetrain = new Drivetrain();
  

  public RobotContainer() {
    configureBindings();
    temporary.add(new NetworkEntry("motors:", null, drivetrain.frontLeftSwerveModule.getTargetVel()));
    for (int i=0;i<drivetrain.getIdentityMap().size();i++) {
      String id = (String) drivetrain.getIdentityMap().keySet().toArray()[i];

      temporary.add(new NetworkEntry(id.toString(), null, drivetrain.getModuleOffset(id)));
    }

    temporary.add(new NetworkEntry("widgets built!", BuiltInWidgets.kTextView, true));

    NetworkTables table = constructTableTab("Swerve", temporary);
  }

  public NetworkTables constructTableTab(String tabName, TreeSet<NetworkEntry> details) {
    NetworkTables localTable = new NetworkTables(tabName, details);
    
    return localTable;
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
