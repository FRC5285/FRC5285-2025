package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climbMotor;
    private final ClimberState climberstate;
    private PIDController climbPID = new PIDController(1.0, 0, 0);

    public ClimberSubsystem() {
        climbMotor = new TalonFX(ClimberConstants.motorID);
        climberstate = new ClimberState(climbMotor);
        climbMotor.setPosition(0);
    }

    public Command doClimb() {
        return runOnce(() -> {this.climbPID.reset(); this.climbPID.setSetpoint(ClimberConstants.climbRotations);})
        .andThen(run(() -> {
            climbMotor.set(Math.max(this.climbPID.calculate(climbMotor.getPosition().getValue().in(Rotations)), 0.0));
        }))
        .until(() -> Math.abs(climbMotor.getPosition().getValue().in(Rotations)) >= ClimberConstants.climbRotations)
        .andThen(this.stopClimb());
    }

    public Command stopClimb() {
        return runOnce(() -> climbMotor.stopMotor());
    }

    public class ClimberState implements Sendable{
        TalonFX theMotor;
        
        public ClimberState(TalonFX theClimbMotor) {
            this.theMotor = theClimbMotor;
            SendableRegistry.add(this, "ClimberState");
            SmartDashboard.putData(this);
        }

        @Override
        public void initSendable(SendableBuilder builder){
            builder.addDoubleProperty("Climber Amount", () -> Math.abs(theMotor.getPosition().getValue().in(Rotations)), null);
        }
    }
}
