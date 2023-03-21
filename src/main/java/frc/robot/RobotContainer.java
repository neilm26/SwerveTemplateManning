// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.DriverControls;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;

public class RobotContainer implements SwerveConstants {
  private XboxController driverController = new XboxController(0);
  private Drivetrain drivetrain = new Drivetrain();

  //Commands
  private DriverControls driverControls = new DriverControls(drivetrain, 
                                          driverController::getLeftX, 
                                          driverController::getLeftY,
                                          driverController::getRightX,
                                          driverController::getLeftTriggerAxis,
                                          driverController::getRightTriggerAxis);

  public RobotContainer() {
    NetworkTableContainer.insertGlobalEntries();

    configureDriveRelative();
    configureBindings();

    //kinda hard to read.
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(driverControls);
  }

  private void configureDriveRelative() {
    boolean isFieldCentric = (Boolean) NetworkTableContainer.entries.get("Field Centric").getNetworkTblValue();

    if (isFieldCentric) {
      drivetrain.setCentralMotion(ChassisSpeeds.fromFieldRelativeSpeeds(
              DESIRED_VEL_X_AXIS,
              DESIRED_VEL_Y_AXIS,
              DESIRED_RAD_SPEED, 
              Rotation2d.fromDegrees(drivetrain.getRobotHeading())));
    }
    else {
      drivetrain.setCentralMotion(new ChassisSpeeds(
              DESIRED_VEL_X_AXIS,
              DESIRED_VEL_Y_AXIS,
              DESIRED_RAD_SPEED
      ));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
