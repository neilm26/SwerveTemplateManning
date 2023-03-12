// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.TreeSet;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkEntry;
import frc.robot.Subsystems.Networking.NetworkTables;

public class RobotContainer {

  private TreeSet<NetworkEntry> temporary = new TreeSet<>();

  private Drivetrain drivetrain = new Drivetrain();
  

  public RobotContainer() {
    configureBindings();
    temporary.add(new NetworkEntry("motors:", null, drivetrain.getModule(ModuleNames.FRONT_LEFT).getName()));
    for (int i=0;i<drivetrain.getIdentityMap().size();i++) {
      String id = (String) drivetrain.getIdentityMap().keySet().toArray()[i];

      temporary.add(new NetworkEntry(id.toString(), null, drivetrain.getModuleOffset(id)));
    }

    temporary.add(new NetworkEntry("widgets built!", BuiltInWidgets.kTextView, true));

    constructTableTab("Swerve", temporary);
  }

  public void constructTableTab(String tabName, TreeSet<NetworkEntry> details) {
    new NetworkTables(tabName, details);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
