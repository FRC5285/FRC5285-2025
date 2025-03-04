package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor;
  private final TalonFX followerMotor;
  // private final DigitalInput topLimitSwitch;
  // private final DigitalInput bottomLimitSwitch;
  // private final Trigger atBottom;
  // private final Trigger atTop;
  private final ElevatorState elevatorState;
  private final Encoder elevatorEncoder;
  private final ProfiledPIDController elevatorPID;
  private final ElevatorFeedforward elevatorFeedforward;
  private final ElevatorSimulation elevatorSimulation;
  private boolean motorOverride = false;

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    followerMotor = new TalonFX(ElevatorConstants.followMotorID);
    // topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchID);
    // bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchID);
    elevatorEncoder = new Encoder(ElevatorConstants.encoderA, ElevatorConstants.encoderB);
    elevatorEncoder.setDistancePerPulse(ElevatorConstants.encoderPulseDist);
    elevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
      new TrapezoidProfile.Constraints(ElevatorConstants.maxV, ElevatorConstants.maxA)
    );
    elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    elevatorPID.setTolerance(ElevatorConstants.goalRange);
    elevatorPID.setGoal(0);
    
    followerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    elevatorState = new ElevatorState();
    elevatorSimulation = Robot.isSimulation() ? new ElevatorSimulation() : null;
    
    // atBottom = new Trigger(()-> !bottomLimitSwitch.get()); // Limit switch down is false, so must invert
    // atTop = new Trigger(()-> !topLimitSwitch.get());
    // atBottom.onTrue(hitBottomLimit());
    // atTop.onTrue(hitTopLimit());
    new Trigger(() -> elevatorEncoder.getDistance() >= ElevatorConstants.maxHeight).onTrue(hitTopLimit());
    // new Trigger(() -> elevatorEncoder.getDistance() <= ElevatorConstants.minHeight).onTrue(hitBottomLimit());

    SmartDashboard.putData(this);
  }

  @Override
  public void simulationPeriodic() {
      elevatorSimulation.updateSimulation();
  }

  public Command goToPosition(DoubleSupplier getTargetPosition) {
    return runOnce(() -> {
      motorOverride = false;
      elevatorPID.setGoal(getTargetPosition.getAsDouble());
    });
  }

  @Override
  public void periodic() {
    double pidCalc = elevatorPID.calculate(elevatorEncoder.getDistance());
    double ffwdCalc = elevatorFeedforward.calculate(elevatorPID.getSetpoint().velocity);
    if (motorOverride == false) this.elevatorMotor.setVoltage(pidCalc + ffwdCalc);
  }

  public boolean reachedGoal() {
    return elevatorPID.atGoal();
  }

  public Command goToLevel1Position(){
    return goToPosition(()-> elevatorState.getLevel1Position());
  }

  public Command goToLevel2Position(){
    return goToPosition(()-> elevatorState.getLevel2Position());
  }

  public Command goToLevel3Position(){
    return goToPosition(()-> elevatorState.getLevel3Position());
  }

  public Command goToLevel4Position(){
    return goToPosition(()-> elevatorState.getLevel4Position());
  }

  public Command goToIntakePosition(){
    return goToPosition(()-> elevatorState.getIntakePosition());
  }

  public Command goToFloorAlgaePosition() {
    return goToPosition(() -> elevatorState.getFloorAlgaePosition());
  }

  public Command goToProcessorPosition() {
    return goToPosition(() -> elevatorState.getProcessorHeight()); // Put other constants into Constants file later!! Also tune this one!!!
  }

  public Command goToBottomPosition(){
    return goToPosition(()-> 0.0);
  }

  public Command elevatorUp() {
    return runOnce(() -> elevatorPID.setGoal(Math.min(elevatorPID.getGoal().position + 0.05, ElevatorConstants.maxHeight)));
  }

  public Command elevatorDown() {
    return runOnce(() -> elevatorPID.setGoal(Math.max(elevatorPID.getGoal().position - 0.05, 0.0)));
  }

  // private Command hitBottomLimit(){
  //   return runOnce(()->{
  //     motorOverride = true;
  //     elevatorEncoder.reset();
  //     elevatorMotor.stopMotor();
  //   });
  // }

  private Command hitTopLimit(){
    return runOnce(()->{
      motorOverride = true;
      elevatorMotor.stopMotor();
    });
  }

  public Command stopElevator() {
    return runOnce(() -> {
      motorOverride = true;
      elevatorMotor.stopMotor();
    });
  }

  // public Command resetToHome(){
  //   return run(()-> {
  //     motorOverride = true;
  //     elevatorMotor.set(-0.1);
  //   }).until(atBottom);
  // }

    public class ElevatorSimulation {
        // Simulation classes help us simulate what's going on, including gravity.
        private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60(2);
        private final ElevatorSim m_elevatorSim = new ElevatorSim(
                m_elevatorGearbox,
                ElevatorConstants.elevatorGearing,
                ElevatorConstants.carriageMass,
                ElevatorConstants.elevatorGearRadius,
                0.0,
                elevatorState.getLevel4Position(),
                true,
                0,
                0.01,
                0.0);
        private final EncoderSim m_encoderSim = new EncoderSim(elevatorEncoder);
        private final TalonFXSimState m_motorSim = elevatorMotor.getSimState();

        public void updateSimulation() {
            // In this method, we update our simulation of what our elevator is doing
            // First, we set our "inputs" (voltages)
            m_elevatorSim.setInput(m_motorSim.getMotorVoltage());

            // Next, we update it. The standard loop time is 20ms.
            m_elevatorSim.update(0.020);

            // Finally, we set our simulated encoder's readings and simulated battery
            // voltage
            m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());

            // SimBattery estimates loaded battery voltages
            RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
        }
    }

  public class ElevatorState implements Sendable{

    private double level1Position = ElevatorConstants.level1Position;
    private double level2Position = ElevatorConstants.level2Position;
    private double level3Position = ElevatorConstants.level3Position;
    private double level4Position = ElevatorConstants.level4Position;
    private double maxHeight = ElevatorConstants.maxHeight;
    private double intakePosition = ElevatorConstants.intakePosition;
    private double processorHeight = ElevatorConstants.processorHeight;
    private double floorAlgaePosition = ElevatorConstants.floorAlgaePosition;

    public double getLevel1Position() {
      return level1Position;
    }

    public void setLevel1Position(double level1Position) {
      this.level1Position = level1Position;
    }

    public double getLevel2Position() {
      return level2Position;
    }

    public void setLevel2Position(double level2Position) {
      this.level2Position = level2Position;
    }

    public double getLevel3Position() {
      return level3Position;
    }

    public void setLevel3Position(double level3Position) {
      this.level3Position = level3Position;
    }

    public double getLevel4Position() {
      return level4Position;
    }

    public void setLevel4Position(double level4Position) {
      this.level4Position = level4Position;
    }

    public double getMaxHeight() {
      return maxHeight;
    }

    public void setMaxHeight(double maxHeight) {
      this.maxHeight = maxHeight;
    }

    public double getIntakePosition() {
      return intakePosition;
    }

    public void setIntakePosition(double intakePosition) {
      this.intakePosition = intakePosition;
    }

    public double getProcessorHeight() {
      return processorHeight;
    }

    public void setProcessorHeight(double processorHeight) {
      this.processorHeight = processorHeight;
    }

    public double getFloorAlgaePosition() {
      return this.floorAlgaePosition;
    }

    public void setFloorAlgaePosition(double floorAlgaePosition) {
      this.floorAlgaePosition = floorAlgaePosition;
    }

    public ElevatorState(){
      SendableRegistry.add(this, "Elevator State");
      SmartDashboard.putData(this);
    }
    
    @Override 
    public void initSendable(SendableBuilder builder){
      builder.setSmartDashboardType("RobotPreferences");

      builder.addDoubleProperty("Setpoint Height", () -> elevatorPID.getSetpoint().position, (newVal) -> {});
      builder.addDoubleProperty("Goal Height", ()-> elevatorPID.getGoal().position, null);
      builder.addDoubleProperty("Current Height", () -> elevatorEncoder.getDistance(), (newVal) -> {});
      builder.addBooleanProperty("Reached Goal", ()-> reachedGoal(), null);
      builder.addDoubleProperty("Level 1 Position", this::getLevel1Position, this::setLevel1Position);
      builder.addDoubleProperty("Level 2 Position", this::getLevel2Position, this::setLevel2Position);
      builder.addDoubleProperty("Level 3 Position", this::getLevel3Position, this::setLevel3Position);
      builder.addDoubleProperty("Level 4 Position", this::getLevel4Position, this::setLevel4Position);
      builder.addDoubleProperty("Max Height Position", this::getMaxHeight, this::setMaxHeight);
      builder.addDoubleProperty("Intake Position", this::getIntakePosition, this::setIntakePosition);
      builder.addDoubleProperty("Processor Height", this::getProcessorHeight, this::setProcessorHeight);
      builder.addDoubleProperty("Floor Algae Position", this::getFloorAlgaePosition, this::setFloorAlgaePosition);
    }
  }
}