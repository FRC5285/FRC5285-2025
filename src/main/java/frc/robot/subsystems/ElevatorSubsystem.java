package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor;
  private final TalonFX followerMotor;
  private final DigitalInput topLimitSwitch;
  private final DigitalInput bottomLimitSwitch;
  private final PositionVoltage goingUpVoltage;
  private final PositionVoltage goingDownVoltage;
  private final Trigger atBottom;
  private final Trigger atTop;
  private final ElevatorState elevatorState;

  public ElevatorSubsystem() {
    goingUpVoltage = new PositionVoltage(0).withSlot(0);
    goingDownVoltage = new PositionVoltage(0).withSlot(1);

    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    followerMotor = new TalonFX(ElevatorConstants.followMotorID);
    topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchID);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchID);
    
    followerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    Slot0Configs goingUpConfig = new Slot0Configs()
      .withKP(2.4)
      .withKI(0.1)
      .withKD(0.1);
    Slot1Configs goingDownConfig = new Slot1Configs()
      .withKP(2.0)
      .withKI(0.1)
      .withKD(0.1);
    TalonFXConfiguration config = new TalonFXConfiguration()
      .withSlot0(goingUpConfig)
      .withSlot1(goingDownConfig)
      .withVoltage(new VoltageConfigs()
        .withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-2)) // less volts going down
      );
    elevatorMotor.getConfigurator().apply(config);
    elevatorMotor.setPosition(0);
    elevatorState = new ElevatorState();
    
    atBottom = new Trigger(()-> bottomLimitSwitch.get());
    atTop = new Trigger(()-> topLimitSwitch.get());
    atBottom.onTrue(hitBottomLimit());
    atTop.onTrue(hitTopLimit());
  }

  private Command goToPosition(DoubleSupplier getTargetPosition) {
    return runOnce(() -> {
      double currentPosition = elevatorMotor.getPosition().getValueAsDouble(); //returns rotation
      double targetPosition = getTargetPosition.getAsDouble() / elevatorState.getDistancePerRotation(); //converts distance to rotations
      if (targetPosition > currentPosition) {
        elevatorMotor.setControl(goingDownVoltage.withPosition(targetPosition));
      }
      else if (targetPosition < currentPosition) {
        elevatorMotor.setControl(goingUpVoltage.withPosition(targetPosition));
      }
      else {
        elevatorMotor.stopMotor();
      }
    });
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

  public Command goToBottomPosition(){
    return goToPosition(()-> 0.0);
  }

  private Command hitBottomLimit(){
    return runOnce(()->{
      elevatorMotor.stopMotor();
      elevatorMotor.setPosition(0);
    });
  }

  private Command hitTopLimit(){
    return runOnce(()->{
      elevatorMotor.stopMotor();
      elevatorMotor.setPosition(elevatorState.getMaxHeight());
    });
  }

  public Command resetToHome(){
    return run(()-> elevatorMotor.set(-0.1)).until(atBottom);
  }

  public class ElevatorState implements Sendable{

    private double level1Position = 5;
    private double level2Position = 10;
    private double level3Position = 15;
    private double level4Position = 25;
    private double maxHeight = 30;
    private double intakePosition = 12;
    private double distancePerRotation = 0.5;

    public double getDistancePerRotation() {
      return distancePerRotation;
    }

    public void setDistancePerRotation(double distancePerRotation) {
      this.distancePerRotation = distancePerRotation;
    }

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

    public ElevatorState(){
      SendableRegistry.add(this, "Elevator State");
      SmartDashboard.putData(this);
    }
    
    @Override 
    public void initSendable(SendableBuilder builder){
      builder.setSmartDashboardType("RobotPreferences");

      builder.addDoubleProperty("Level 1 Position", this::getLevel1Position, this::setLevel1Position);
      builder.addDoubleProperty("Level 2 Position", this::getLevel2Position, this::setLevel2Position);
      builder.addDoubleProperty("Level 3 Position", this::getLevel3Position, this::setLevel3Position);
      builder.addDoubleProperty("Level 4 Position", this::getLevel4Position, this::setLevel4Position);
      builder.addDoubleProperty("Max Height Position", this::getMaxHeight, this::setMaxHeight);
      builder.addDoubleProperty("Intake Position", this::getIntakePosition, this::setIntakePosition);
      builder.addDoubleProperty("Distance Per Rotation", this::getDistancePerRotation, this::setDistancePerRotation);

    }
  }
}