package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final TalonFX algaeIntakeMotor;
    private final TalonFX followerMotor;
    private final DigitalInput algaeIntakeSensor;

    @SuppressWarnings("unused")
    private final AlgaeIntakeState state;

    public AlgaeIntakeSubsystem() {
        algaeIntakeMotor = new TalonFX(AlgaeIntakeConstants.motorID2);
        followerMotor = new TalonFX(AlgaeIntakeConstants.motorID1);
        algaeIntakeSensor = new DigitalInput(AlgaeIntakeConstants.algaeIntakeSensorID);
        state = new AlgaeIntakeState();

        followerMotor.setControl(new Follower(algaeIntakeMotor.getDeviceID(), true));
    }

    public boolean hasAlgae() {
        return !algaeIntakeSensor.get(); 
    }

    public boolean noAlgae() {
        return !hasAlgae() && algaeIntakeMotor.get() == 0.0;
    }

    public Command doIntake() {
        return run(() -> algaeIntakeMotor.set(AlgaeIntakeConstants.inSpeed))
        .until(this::hasAlgae)
        .withTimeout(AlgaeIntakeConstants.maxMotorTime)
        .andThen(() -> algaeIntakeMotor.set(AlgaeIntakeConstants.normalSpeed));
    }

    public Command groundIntake() {
        return run(() -> algaeIntakeMotor.set(AlgaeIntakeConstants.inSpeed))
        .until(this::hasAlgae)
        .withTimeout(AlgaeIntakeConstants.maxGroundPickupTime)
        .andThen(() -> algaeIntakeMotor.set(AlgaeIntakeConstants.normalSpeed));
    }

    public Command stopIntake() {
        return runOnce(() -> algaeIntakeMotor.set(AlgaeIntakeConstants.normalSpeed));
    }

    public Command shootOut() {
        return runOnce(() -> algaeIntakeMotor.set(AlgaeIntakeConstants.outSpeed))
        // .until(this::noAlgae) // Algae will still be held when limit switch is not down
        .andThen(new WaitCommand(AlgaeIntakeConstants.outMotorTime))
        .andThen(() -> algaeIntakeMotor.set(AlgaeIntakeConstants.normalSpeed));
    }

    public class AlgaeIntakeState implements Sendable{
        
        public AlgaeIntakeState(){
        SendableRegistry.add(this, "AlgaeIntakeState");
        SmartDashboard.putData(this);
        }

        @Override
        public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("AlgaeIntakeMotor", ()-> algaeIntakeMotor.get(), null);
        builder.addBooleanProperty("Sensor", ()-> hasAlgae(), null);
        }
    }
}
