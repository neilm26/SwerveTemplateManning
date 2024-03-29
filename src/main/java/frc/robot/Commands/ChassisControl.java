// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Set;
import java.util.Map.Entry;
import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants.ModuleNames;
import frc.robot.Subsystems.SwerveDrivetrain;
import frc.robot.Subsystems.SwerveModule.SwerveModule;

public class ChassisControl extends CommandBase {
  /** Creates a new ChassisControl. */
  private SwerveDrivetrain drivetrain;
  private SwerveModule FLmodule, FRmodule, BLmodule, BRmodule;
  private Supplier<Double> leftXAxis, leftYAxis, leftTrigger, rightTrigger;
  private Supplier<Boolean> fieldCentric;
  public ChassisControl(SwerveDrivetrain drivetrain, 
                Supplier<Double> leftXAxis, 
                Supplier<Double> leftYAxis,
                Supplier<Double> leftTrigger,
                Supplier<Double> rightTrigger,
                Supplier<Boolean> fieldCentric) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.leftXAxis = leftXAxis;
    this.leftYAxis = leftYAxis;
    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;
    this.fieldCentric = fieldCentric;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    FLmodule = drivetrain.getModule(ModuleNames.FRONT_LEFT);
    FRmodule = drivetrain.getModule(ModuleNames.FRONT_RIGHT);
    BLmodule = drivetrain.getModule(ModuleNames.BACK_LEFT);
    BRmodule = drivetrain.getModule(ModuleNames.BACK_RIGHT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yaw = drivetrain.getPigeon().getYaw();
    if (fieldCentric.get()) {
      drivetrain.setCentralMotion(ChassisSpeeds.fromFieldRelativeSpeeds(leftXAxis.get(), leftYAxis.get(), 
                    (leftTrigger.get()-rightTrigger.get()) * Math.PI, Rotation2d.fromDegrees(yaw)));
      SmartDashboard.putNumber("Robot Heading relative to field", yaw);
    } else {
      drivetrain.setCentralMotion(new ChassisSpeeds(leftXAxis.get(), leftYAxis.get(), (leftTrigger.get()-rightTrigger.get()) * Math.PI));
    }
    for (Entry<ModuleNames, SwerveModuleState> state: SwerveDrivetrain.stateMap.entrySet()) {
      SmartDashboard.putNumberArray(state.getKey().toString(), new Double[] {state.getValue().angle.getDegrees(), state.getValue().speedMetersPerSecond});
    }

    FLmodule.setDesiredState(drivetrain.getModuleState(ModuleNames.FRONT_LEFT));
    FRmodule.setDesiredState(drivetrain.getModuleState(ModuleNames.FRONT_RIGHT));
    BLmodule.setDesiredState(drivetrain.getModuleState(ModuleNames.BACK_LEFT));
    BRmodule.setDesiredState(drivetrain.getModuleState(ModuleNames.BACK_RIGHT));
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
