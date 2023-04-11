// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ChassisControl;
import frc.robot.Commands.DriverControls;
import frc.robot.Commands.OrientateModules;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.SwerveDrivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class RobotContainer implements SwerveConstants {
  private CommandXboxController driverController = new CommandXboxController(0);
  private SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  // Commands
  private DriverControls driverControls = new DriverControls(drivetrain,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getLeftTriggerAxis,
      driverController::getRightTriggerAxis);

  private OrientateModules orientateModules = new OrientateModules(drivetrain,
      driverController::getLeftTriggerAxis,
      driverController::getRightTriggerAxis);

  private ChassisControl chassisControl = new ChassisControl(drivetrain,
      driverController::getLeftX,
      driverController::getLeftY, 
      () -> driverController.getRawAxis(3),
      () -> driverController.getRawAxis(4), () -> true);

  private final EventLoop eventLoop = new EventLoop();

  public RobotContainer() {
    NetworkTableContainer.insertGlobalEntries();

    configureDriveRelative();
    configureBindings();
    configureZeroHeading();
  }

  private void configureZeroHeading() {
    drivetrain.getPigeon().setYaw(0);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(chassisControl);

    // BooleanEvent pointToEvent = new BooleanEvent(eventLoop,
    //     () -> Utilities.notWithin(driverController.getRightX(), -0.1, 0.1))
    //     .debounce(0.1);

    // Trigger pointToTrigger = pointToEvent.castTo(Trigger::new);

    // pointToTrigger.whileTrue(orientateModules);
  }

  public void configureDriveRelative() {
    // so far redundant.
    // ChassisSpeeds needs to be tested eventually.
    boolean isFieldCentric = (Boolean) NetworkTableContainer.entries.get("Field Centric").getNetworkTblValue();
    SmartDashboard.putBoolean("FieldCentric", isFieldCentric);

    for (SwerveModule module : drivetrain.getModules()) {
      module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(180.0)));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void pollEvent() {
    eventLoop.poll();
  }
}
