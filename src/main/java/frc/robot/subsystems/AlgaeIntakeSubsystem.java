package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final TalonFX mainMotor;
    private final TalonFX followerMotor;
    private final DigitalInput limitSwitch;

    public AlgaeIntakeSubsystem() {
        mainMotor = new TalonFX(AlgaeIntakeConstants.motorID1);
        followerMotor = new TalonFX(AlgaeIntakeConstants.motorID2);
        limitSwitch = new DigitalInput(AlgaeIntakeConstants.limitSwitchID);

        followerMotor.setControl(new Follower(mainMotor.getDeviceID(), true));
    }

    public boolean hasAlgae() {
        return !limitSwitch.get(); // Limit switch down when .get() returns false
    }

    public boolean noAlgae() {
        return !this.hasAlgae();
    }

    public Command doIntake() {
        return run(() -> mainMotor.set(AlgaeIntakeConstants.inSpeed))
        .until(this::hasAlgae)
        .withTimeout(AlgaeIntakeConstants.maxMotorTime)
        .andThen(() -> mainMotor.set(0));
    }

    public Command shootOut() {
        return runOnce(() -> mainMotor.set(AlgaeIntakeConstants.outSpeed))
        // .until(this::noAlgae) // Algae will still be held when limit switch is not down
        .andThen(new WaitCommand(AlgaeIntakeConstants.outMotorTime))
        .andThen(() -> mainMotor.set(0));
    }
}
