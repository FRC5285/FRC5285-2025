package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climbMotor;

    public ClimberSubsystem() {
        climbMotor = new TalonFX(ClimberConstants.motorID);
        climbMotor.setPosition(0);
        new Trigger(() -> Math.abs(climbMotor.getPosition().getValue().in(Rotations)) >= ClimberConstants.climbRotations).onTrue(
            stopClimb()
        );
    }

    public Command doClimb() {
        return runOnce(() -> climbMotor.set(ClimberConstants.climbSpeed));
    }

    public Command stopClimb() {
        return runOnce(() -> climbMotor.stopMotor());
    }
}
