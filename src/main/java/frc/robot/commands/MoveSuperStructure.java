// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.util.GameInfo;
import frc.robot.util.GameInfo.CoralScoringPosition;

public class MoveSuperStructure extends Command {

  CoralScoringPosition position;

  double runRollers = 0;

  public MoveSuperStructure(CoralScoringPosition position) {
    this.position = position;

    addRequirements(Arm.get(), Elevator.get(), Wrist.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.get().goToPosition(position);
    Wrist.get().goToPosition(position);
    Elevator.get().goToPosition(position);

    //if (Math.abs(Wrist.get().getPosition() - position.wristRot.in(Units.Rotation)) < 2 && Math.abs(Arm.get().getPosition() - position.armRot.in(Units.Rotation)) < 2 && Math.abs(Elevator.get().getPosition() - position.height) < 2 )
    //  EndEffector.get().run(runRollers);
    //else  
    //  EndEffector.get().stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.get().stop();
    Wrist.get().stop();
    Elevator.get().stop();
    
    //EndEffector.get().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean atPosition(CoralScoringPosition position){
    return Math.abs(Wrist.get().getPosition() - position.wristRot.in(Units.Rotation)) < 2 && Math.abs(Arm.get().getPosition() - position.armRot.in(Units.Rotation)) < 2 && Math.abs(Elevator.get().getPosition() - position.height) < 2 ;
  }
  public static Command autoScore(CoralScoringPosition position){
    return new MoveSuperStructure(position).andThen((new MoveSuperStructure(position).alongWith(EndEffector.get().spin(0.6)).withTimeout(1)));
  }
  public static Command autoSource(){
    return new MoveSuperStructure(GameInfo.Source).alongWith(EndEffector.get().spin(-0.6)).unless(EndEffector.get()::isStalling);
  }
}
