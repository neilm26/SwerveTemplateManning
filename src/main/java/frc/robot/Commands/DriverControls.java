// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Networking.NetworkEntry;
import frc.robot.Subsystems.Networking.NetworkTableContainer;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class DriverControls extends CommandBase {
  /** Creates a new DriverControls. */
  private Drivetrain drivetrain;
  private Supplier<Double> leftXAxis, leftYAxis, rightXAxis, leftTrigger, rightTrigger;

  private double angularOrientation=0, vXOutput=0, vYOutput=0;

  public DriverControls(Drivetrain drivetrain, 
                        Supplier<Double> leftXAxis, 
                        Supplier<Double> leftYAxis, 
                        Supplier<Double> rightXAxis,
                        Supplier<Double> leftTrigger,
                        Supplier<Double> rightTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    this.leftXAxis = leftXAxis;
    this.leftYAxis = leftYAxis;
    this.rightXAxis = rightXAxis;
    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //vYOutput = Math.sqrt(Math.pow(leftXAxis.get(),2)+Math.pow(leftYAxis.get(), 2));
    vYOutput = leftTrigger.get()-rightTrigger.get();
    vXOutput = rightXAxis.get();

    boolean overrideTargetHeading = (Boolean)  NetworkTableContainer.entries.get("Override Target Heading").getNetworkTblValue();

    //different ways of doing the same thing.
    NetworkTableContainer.entries.get("Forward Output").setNetworkEntryValue(vYOutput);

    angularOrientation = (Double) Objects.requireNonNullElse(
        Utilities.convertAxesToDegrees(leftXAxis.get(), leftYAxis.get()),angularOrientation);

    if (overrideTargetHeading) {
      for (SwerveModule module: drivetrain.getModules()) {
          module.setTargetAng(angularOrientation);
      }
      drivetrain.getModule(ModuleNames.FRONT_RIGHT).easyDrive(vYOutput);
      drivetrain.getModule(ModuleNames.FRONT_RIGHT).easyTurn(vXOutput);

      drivetrain.getModule(ModuleNames.FRONT_LEFT).easyDrive(vYOutput);
      drivetrain.getModule(ModuleNames.FRONT_LEFT).easyTurn(vXOutput);

      drivetrain.getModule(ModuleNames.BACK_LEFT).easyDrive(vYOutput);
      drivetrain.getModule(ModuleNames.BACK_LEFT).easyTurn(vXOutput);
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
