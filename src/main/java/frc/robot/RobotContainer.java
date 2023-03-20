// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.DriverControls;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;

public class RobotContainer {
  private XboxController driverController = new XboxController(0);
  private Drivetrain drivetrain = new Drivetrain();

  //Commands
  private DriverControls driverControls = new DriverControls(drivetrain, 
                                          driverController::getLeftX, 
                                          driverController::getLeftY);

  public RobotContainer() {
    configureBindings();

    NetworkTableContainer.insertGlobalEntries();

    //kinda hard to read.
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(driverControls);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
