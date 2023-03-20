// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkTableContainer;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class DriverControls extends CommandBase {
  /** Creates a new DriverControls. */
  private Drivetrain drivetrain;
  private Supplier<Double> leftXAxis, leftYAxis;

  public DriverControls(Drivetrain drivetrain, Supplier<Double> leftXAxis, Supplier<Double> leftYAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    this.leftXAxis = leftXAxis;
    this.leftYAxis = leftYAxis;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isFieldCentric = (Boolean) NetworkTableContainer.entries.get("Field Centric").getNetworkTblValue();
    boolean overrideTargetHeading = (Boolean)  NetworkTableContainer.entries.get("Override Target Heading").getNetworkTblValue();

    if (isFieldCentric) {
        
    }

    if (overrideTargetHeading) {
      for (SwerveModule module: drivetrain.getModules()) {
        Object angle = Utilities.convertAxesToDegrees(leftXAxis.get(), leftYAxis.get());
        if (angle != null) {
          SmartDashboard.putNumber("Joystick angle", (Double) angle);
          module.setTargetAng((Double) angle);
        }
        else {
          //angle should be set to the current robot angle (avoid continuing travel)
        }
      }
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
