package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TriggerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.ControllerUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

/*
 * Quite a lot of code was copied from here:
 * https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/RobotContainer.java
*/

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // The robot's subsystems and commands are defined here...
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final LEDSubsystem ledStrip = new LEDSubsystem();
    public final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // Chooses auto path
    private final SendableChooser<Command> autoChooser;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Defines commands for Auton
        NamedCommands.registerCommand("Wait for Elevator", new WaitUntilCommand(() -> elevator.reachedGoal()));
        NamedCommands.registerCommand("Wait for Elevator and Wrist", new WaitUntilCommand(() -> elevator.reachedGoal() && wrist.isAtSetpoint()));
        NamedCommands.registerCommand("Intake Coral", flywheel.intakeCoral());
        NamedCommands.registerCommand("Shoot Coral", flywheel.shootCoral());
        NamedCommands.registerCommand("Intake Algae", algaeIntake.doIntake());
        NamedCommands.registerCommand("Shoot Algae", algaeIntake.shootOut());
        NamedCommands.registerCommand("Elevator and Wrist to L1", elevator.goToLevel1Position().alongWith(wrist.goToLowShootPosition()));
        NamedCommands.registerCommand("Elevator and Wrist to L2", elevator.goToLevel2Position().alongWith(wrist.goToMidShootPosition()));
        NamedCommands.registerCommand("Elevator and Wrist to L3", elevator.goToLevel3Position().alongWith(wrist.goToMidShootPosition()));
        NamedCommands.registerCommand("Elevator and Wrist to L4", elevator.goToLevel4Position().alongWith(wrist.goToHighShootPosition()));
        NamedCommands.registerCommand("Elevator and Wrist to Intake", elevator.goToIntakePosition().alongWith(wrist.goToIntakePosition()));
        NamedCommands.registerCommand("Elevator to Reef Algae", elevator.goToPosition(() -> drivetrain.getAlgaeHeight()));
        NamedCommands.registerCommand("Elevator to Processor", elevator.goToProcessorPosition());

        // puts auto paths choices onto the Smart Dashboard
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        ledStrip.toNormal().schedule();

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(this.applyThrottle(-MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1) * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(this.applyThrottle(-MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1) * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(this.applyThrottle(-MathUtil.applyDeadband(m_driverController.getRightX(), 0.1) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );
        // Deposit coral
        m_driverController.x().debounce(TriggerConstants.debounceTime, DebounceType.kRising).onTrue(
            elevator.goToLevel1Position().alongWith(wrist.goToLowShootPosition())
        );
        m_driverController.a().debounce(TriggerConstants.debounceTime, DebounceType.kRising).onTrue(
            elevator.goToLevel2Position().alongWith(wrist.goToMidShootPosition())
        );
        m_driverController.b().debounce(TriggerConstants.debounceTime, DebounceType.kRising).onTrue(
            elevator.goToLevel3Position().alongWith(wrist.goToMidShootPosition())
        );
        m_driverController.y().debounce(TriggerConstants.debounceTime, DebounceType.kRising).onTrue(
            elevator.goToLevel4Position().alongWith(wrist.goToHighShootPosition())
        );
        m_driverController.a().or(m_driverController.b().or(m_driverController.x().or(m_driverController.y()))).debounce(TriggerConstants.debounceTime, DebounceType.kFalling).onFalse(
            drivetrain.depositReefBranch(flywheel, m_driverController.getHID(), elevator, wrist)
        );

        // Get coral from coral station
        new Trigger(() -> ControllerUtils.dPadDown(m_driverController.getHID())).debounce(TriggerConstants.debounceTime, DebounceType.kRising).onTrue(
            elevator.goToIntakePosition().alongWith(wrist.goToIntakePosition())
        );
        new Trigger(() -> ControllerUtils.dPadDown(m_driverController.getHID())).debounce(TriggerConstants.debounceTime, DebounceType.kFalling).onFalse(
            drivetrain.collectCoralStation(flywheel, m_driverController.getHID(), elevator, wrist)
            .alongWith(ledStrip.toAuton()).andThen(ledStrip.toNormal())
        );

        // Get algae from reef
        new Trigger(() -> ControllerUtils.dPadLeft(m_driverController.getHID())).debounce(TriggerConstants.debounceTime, DebounceType.kRising).onTrue(
            elevator.goToPosition(() -> drivetrain.getAlgaeHeight())
        );
        new Trigger(() -> ControllerUtils.dPadLeft(m_driverController.getHID())).debounce(TriggerConstants.debounceTime, DebounceType.kFalling).onFalse(
            drivetrain.collectAlgaeFromReef(elevator, algaeIntake)
            .alongWith(ledStrip.toAuton()).andThen(ledStrip.toNormal())
        );

        // Deposit algae into processor
        new Trigger(() -> ControllerUtils.dPadRight(m_driverController.getHID())).debounce(TriggerConstants.debounceTime, DebounceType.kRising).onTrue(
            elevator.goToProcessorPosition()
        );
        new Trigger(() -> ControllerUtils.dPadRight(m_driverController.getHID())).debounce(TriggerConstants.debounceTime, DebounceType.kFalling).onFalse(
            drivetrain.doProcessor(elevator, algaeIntake)
            .alongWith(ledStrip.toAuton()).andThen(ledStrip.toNormal())
        );

        // Do the deep climb
        new Trigger(() -> ControllerUtils.rightTrigger(m_driverController.getHID())).debounce(TriggerConstants.debounceTime, DebounceType.kRising).onTrue(
            drivetrain.doDeepClimb()
            .alongWith(ledStrip.toAuton()).andThen(ledStrip.toNormal())
        );
        new Trigger(() -> ControllerUtils.rightTrigger(m_driverController.getHID())).debounce(TriggerConstants.debounceTime, DebounceType.kFalling).onFalse(
            climber.doClimb()
        );

        // Emergency stops the aimbot (DO NOT USE UNLESS ABSOLUTELY NECESSARY)
        m_driverController.leftStick().onTrue(
            drivetrain.getDefaultCommand().alongWith(ledStrip.toNormal())
        );

        // LEDs for Auton
        new Trigger(() -> DriverStation.isAutonomous()).onTrue(ledStrip.toAuton());
        new Trigger(() -> DriverStation.isAutonomous()).onFalse(ledStrip.toNormal());

        /* Uncomment to test flywheel
        m_driverController.a().and(flywheel.noCoral).onTrue(flywheel.intakeCoral());
        m_driverController.a().and(flywheel.hasCoral).onTrue(flywheel.shootCoral());

        // Uncomment to test wrist
        m_driverController.y().onTrue(wrist.goToIntakePosition());
        m_driverController.b().onTrue(wrist.goToMidShootPosition());
        m_driverController.a().onTrue(wrist.goToHighShootPosition());
        */
    }

    private double applyThrottle(double speedVal) {
        return speedVal * (OperatorConstants.maxSpeedMultiplier * (1.0 - m_driverController.getLeftTriggerAxis() * OperatorConstants.throttleMaxReduction));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        return autoChooser.getSelected();
    }
}
