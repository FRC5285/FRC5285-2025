package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climbMotor;

    public ClimberSubsystem() {
        climbMotor = new TalonFX(ClimberConstants.motorID);
        climbMotor.setPosition(0);
        // new Trigger(() -> Math.abs(climbMotor.getPosition().getValue().in(Rotations)) >= ClimberConstants.climbRotations).onTrue(
        //     stopClimb()
        // );
    }

    public Command doClimb() {
        return runOnce(() -> climbMotor.set(ClimberConstants.climbSpeed));
    }

    public Command stopClimb() {
        return runOnce(() -> climbMotor.stopMotor());
    }

    public class ClimberState implements Sendable{
        
        public ClimberState() {
            SendableRegistry.add(this, "ClimberState");
            SmartDashboard.putData(this);
        }

        @Override
        public void initSendable(SendableBuilder builder){
            builder.addDoubleProperty("Climber Amount", () -> Math.abs(climbMotor.getPosition().getValue().in(Rotations)), null);
        }
    }
}
