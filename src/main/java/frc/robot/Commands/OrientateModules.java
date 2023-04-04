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
import frc.robot.Subsystems.SwerveDrivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class OrientateModules extends CommandBase {
  /** Creates a new OrientateModules. */
  private Supplier<Double> leftTrigger, rightTrigger;
  private SwerveDrivetrain drivetrain;

  private SwerveModule frontLeft, frontRight, backLeft;

  private double output = 0;

  public OrientateModules(SwerveDrivetrain drivetrain, Supplier<Double> leftTrigger, Supplier<Double> rightTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;

    frontLeft = drivetrain.getModule(ModuleNames.FRONT_LEFT);
    frontRight = drivetrain.getModule(ModuleNames.FRONT_RIGHT);
    backLeft = drivetrain.getModule(ModuleNames.BACK_LEFT);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output = leftTrigger.get()-rightTrigger.get();

    SwerveModuleState FRstate = drivetrain.updateModuleState(output, 45, frontRight);
    SwerveModuleState FLstate = drivetrain.updateModuleState(output, -45, frontLeft);
    SwerveModuleState BLstate = drivetrain.updateModuleState(output, -45, backLeft);

    frontRight.setDesiredState(FRstate);
    frontLeft.setDesiredState(FLstate);
    backLeft.setDesiredState(BLstate);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
