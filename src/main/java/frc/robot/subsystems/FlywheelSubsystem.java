package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Robot;

public class FlywheelSubsystem extends SubsystemBase {

  private final SparkMax flywheelMotor;
  private final DigitalInput intakeSensor;
  public final Trigger hasCoral;
  public final Trigger noCoral;
  @SuppressWarnings("unused")
  private final FlywheelState state;
  @SuppressWarnings("unused")
  private final FlywheelSimulation simulation;

  public FlywheelSubsystem() {
    flywheelMotor = new SparkMax(FlywheelConstants.flywheelMotorID, MotorType.kBrushless);
    intakeSensor = new DigitalInput(FlywheelConstants.intakeSensorID);
    hasCoral = new Trigger(this::hasCoral);
    noCoral = new Trigger(this::noCoral);
    state = new FlywheelState();

    simulation = Robot.isSimulation() ? new FlywheelSimulation() : null;

    SmartDashboard.putData(this);
  }

  private boolean hasCoral(){
    return !intakeSensor.get();
  }

  private boolean noCoral(){
    return !hasCoral() && flywheelMotor.get() == 0.0;
  }
  public Command intakeCoral(){
    return run(()-> flywheelMotor.set(FlywheelConstants.intakeSpeed))
      .onlyIf(this::noCoral)
      .until(this::hasCoral)
      .withTimeout(FlywheelConstants.intakeMaxTime)
      .andThen(()-> flywheelMotor.set(-0.05));
  }

  public Command shootCoral(){
    return run(()-> flywheelMotor.set(FlywheelConstants.shootSpeed))
      .onlyIf(this::hasCoral)
      .until(this::noCoral)
      .withTimeout(FlywheelConstants.shootDuration)
      .andThen(()->flywheelMotor.stopMotor());
  }

  public Command runIntake() {
    return runOnce(() -> flywheelMotor.set(0.1));
  }

  public Command stopIntake() {
    return runOnce(() -> flywheelMotor.stopMotor());
  }

  public class FlywheelSimulation {
    private final DIOSim device = new DIOSim(intakeSensor);
    private final Trigger intakeRunning = new Trigger(()-> flywheelMotor.get() < 0.0);

    public FlywheelSimulation() {
        intakeRunning.onTrue(Commands
            .waitSeconds(0.5)
            .andThen(()-> device.setValue(false)));
    }
  }

  public class FlywheelState implements Sendable {

    public FlywheelState(){
      SendableRegistry.add(this, "FlywheelState");
      SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder builder){
      builder.addDoubleProperty("Flywheel Motor", ()-> flywheelMotor.get(), null);
      builder.addBooleanProperty("Sensor", ()-> hasCoral(), null);
    }
  }
}