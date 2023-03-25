// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class OrientateModules extends CommandBase {
  /** Creates a new OrientateModules. */
  private Supplier<Double>  leftXAxis, leftYAxis;
  private Drivetrain drivetrain;

  private SwerveModule frontLeft, frontRight, backLeft;

  private double angularOrientation=0;

  public OrientateModules(Drivetrain drivetrain, Supplier<Double> leftXAxis,
    Supplier<Double> leftYAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.leftXAxis = leftXAxis;
    this.leftYAxis = leftYAxis;

    frontLeft = drivetrain.getModule(ModuleNames.FRONT_LEFT);
    frontRight = drivetrain.getModule(ModuleNames.FRONT_RIGHT);
    backLeft = drivetrain.getModule(ModuleNames.BACK_LEFT);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angularOrientation = (Double) Objects.requireNonNullElse(
      Utilities.convertAxesToDegrees(leftXAxis.get(), leftYAxis.get()), angularOrientation);
    
    SwerveModuleState FRstate = drivetrain.updateModuleState(1, angularOrientation, frontRight);
    SwerveModuleState FLstate = drivetrain.updateModuleState(1, angularOrientation, frontLeft);

    Boolean overrideControls = (Boolean) NetworkTableContainer.entries.get("Override Target Heading")
        .getNetworkTblValue();

    if (overrideControls == false) {
      frontRight.setDesiredState(FRstate);
      frontLeft.setDesiredState(FLstate);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
