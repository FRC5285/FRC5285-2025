package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
// import edu.wpi.first.math.controller.PIDController;  
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climbMotor;
    private ProfiledPIDController climbPID = new ProfiledPIDController(2.0, 0, 0, new TrapezoidProfile.Constraints(40.0, 40.0));

    public ClimberSubsystem() {
        climbMotor = new TalonFX(ClimberConstants.motorID);
        climbMotor.setPosition(0);
        SendableRegistry.add(this, "Climber");
        SmartDashboard.putData(this);
    }

    public Command doClimb() {
        return runOnce(() -> {this.climbPID.reset(climbMotor.getPosition().getValue().in(Rotations)); this.climbPID.setGoal(ClimberConstants.climbRotations);})
        .andThen(run(() -> {
            climbMotor.set(Math.max(this.climbPID.calculate(climbMotor.getPosition().getValue().in(Rotations)), 0.0));
        }))
        .until(() -> Math.abs(climbMotor.getPosition().getValue().in(Rotations)) >= ClimberConstants.climbRotations)
        .andThen(this.stopClimb());
    }

    public Command stopClimb() {
        return runOnce(() -> climbMotor.stopMotor());
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("Climber Amount", () -> Math.abs(this.climbMotor.getPosition().getValue().in(Rotations)), null);
    }
}
