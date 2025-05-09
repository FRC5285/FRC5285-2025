package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AimbotCommands;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AprilTagCams;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.ControllerUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

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
    public final AimbotCommands abcs = new AimbotCommands(drivetrain, DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : true);
    public final AprilTagCams atCams = new AprilTagCams(drivetrain);
    public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final LEDSubsystem ledStrip = new LEDSubsystem();
    public final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_secondaryController = new CommandXboxController(OperatorConstants.kSecondaryControllerPort);

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
        NamedCommands.registerCommand("Shoot Coral", flywheel.runIntake());
        NamedCommands.registerCommand("Intake Algae", algaeIntake.doIntake());
        NamedCommands.registerCommand("Shoot Algae", algaeIntake.shootOut());
        NamedCommands.registerCommand("Elevator and Wrist to L1", elevator.goToLevel1Position().alongWith(wrist.goToLowShootPosition()));
        NamedCommands.registerCommand("Elevator and Wrist to L2", elevator.goToLevel2Position().alongWith(wrist.goToMidShootPosition()));
        NamedCommands.registerCommand("Elevator and Wrist to L3", elevator.goToLevel3Position().alongWith(wrist.goToMidShootPosition()));
        NamedCommands.registerCommand("Elevator and Wrist to L4", elevator.goToLevel4Position().alongWith(wrist.goToHighShootPosition()));
        NamedCommands.registerCommand("Elevator and Wrist to Intake", elevator.goToIntakePosition().alongWith(wrist.goToIntakePosition()));
        NamedCommands.registerCommand("Elevator to L2 Algae", elevator.goToPosition(() -> ElevatorConstants.L2AlgaeHeight));
        NamedCommands.registerCommand("Elevator to L3 Algae", elevator.goToPosition(() -> ElevatorConstants.L3AlgaeHeight));
        NamedCommands.registerCommand("Elevator to Processor", elevator.goToProcessorPosition());
        // Note: this was so that the wrist would go up before depositing coral. However, it doesn't work.
        // What would work instead is appending this to the "intake coral" command. (Note made 4/27/2025)
        NamedCommands.registerCommand("Wrist Up", wrist.goAllTheWayUp());

        // puts auto paths choices onto the Smart Dashboard
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        ledStrip.toNormal().schedule();

        // Configure the trigger bindings
        configureDrivetrainBinding();
        configureBindings();
    }

    private void configureTestBindings() {
        m_driverController.a().onTrue(ledStrip.toAuton());
        m_driverController.y().onTrue(ledStrip.toNormal());
    }

    private void configureDrivetrainBinding() {
        // drivetrain
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(drivetrain.getXVal(-MathUtil.applyDeadband(m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()), 0.1), this.m_driverController.getLeftTriggerAxis()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(drivetrain.getYVal(-MathUtil.applyDeadband(m_driverController.getLeftX() * Math.abs(m_driverController.getLeftX()), 0.1), this.m_driverController.getLeftTriggerAxis()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(drivetrain.getRotVal(-MathUtil.applyDeadband(m_driverController.getRightX() * Math.abs(m_driverController.getRightX()), 0.1), this.m_driverController.getLeftTriggerAxis()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }

    private void configureManualBindings() {
        // elevator
        m_secondaryController.leftBumper().and(() -> !ControllerUtils.dPadRight(m_secondaryController.getHID())).onFalse(
            elevator.elevatorDown()
        );
        m_secondaryController.rightBumper().and(() -> !ControllerUtils.dPadRight(m_secondaryController.getHID())).onFalse(
            elevator.elevatorUp()
        );
        new Trigger(() -> ControllerUtils.leftTrigger(m_secondaryController.getHID())).onFalse(
            elevator.goToPosition(() -> ElevatorConstants.L2AlgaeHeight).alongWith(wrist.goAllTheWayUp())
        );
        new Trigger(() -> ControllerUtils.rightTrigger(m_secondaryController.getHID())).onFalse(
            elevator.goToPosition(() -> ElevatorConstants.L3AlgaeHeight).alongWith(wrist.goAllTheWayUp())
        );
        new Trigger(() -> ControllerUtils.dPadDown(m_secondaryController.getHID())).onFalse(
            elevator.goToFloorAlgaePosition().alongWith(wrist.goAllTheWayUp())
        );
        new Trigger(() -> ControllerUtils.dPadUp(m_secondaryController.getHID())).onFalse(
            elevator.goToProcessorPosition()
        );

        // intake
        m_driverController.leftBumper().onTrue(
            flywheel.intakeCoral().alongWith(algaeIntake.groundIntake())
        );
        m_driverController.leftBumper().onFalse(
            flywheel.stopIntake().alongWith(algaeIntake.stopIntake())
        );
        m_driverController.rightBumper().onTrue(
            flywheel.shootCoral()
        );
        m_driverController.x().onTrue(
            algaeIntake.shootOut()
        );

        // deposit coral
        m_secondaryController.x().onFalse(
            elevator.goToLevel1Position().alongWith(wrist.goToLowShootPosition())
        );
        m_secondaryController.a().onFalse(
            elevator.goToLevel2Position().alongWith(wrist.goToLowShootPosition())
        );
        m_secondaryController.b().onFalse(
            elevator.goToLevel3Position().alongWith(wrist.goToMidShootPosition())
        );
        m_secondaryController.y().onFalse(
            elevator.goToLevel4Position().alongWith(wrist.goToHighShootPosition())
        );

        // Get coral from coral station
        new Trigger(() -> ControllerUtils.dPadLeft(m_secondaryController.getHID())).onTrue(
            elevator.goToIntakePosition().alongWith(wrist.goToIntakePosition())
        );

        // adjust intake
        m_secondaryController.leftBumper().and(() -> ControllerUtils.dPadRight(m_secondaryController.getHID())).onFalse(
            wrist.moveDown()
        );
        m_secondaryController.rightBumper().and(() -> ControllerUtils.dPadRight(m_secondaryController.getHID())).onFalse(
            wrist.moveUp()
        );

        // deep climb
        new Trigger(() -> ControllerUtils.rightTrigger(m_driverController.getHID())).onTrue(
            climber.doClimb()
        );
        new Trigger(() -> ControllerUtils.rightTrigger(m_driverController.getHID())).onFalse(
            climber.stopClimb()
        );
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
        flywheel.setDefaultCommand(flywheel.keepRunning());
        // Deposit coral
        m_secondaryController.x().onFalse(
            elevator.setToLevel1Position().alongWith(wrist.goToLowShootPosition())
        );
        m_secondaryController.a().onFalse(
            elevator.setToLevel2Position().alongWith(wrist.goToMidShootPosition())
        );
        m_secondaryController.b().onFalse(
            elevator.setToLevel3Position().alongWith(wrist.goToMidShootPosition())
        );
        m_secondaryController.y().onFalse(
            elevator.setToLevel4Position().alongWith(wrist.goToHighShootPosition())
        );
        m_driverController.y().onTrue(
            flywheel.doShootCoral().andThen(abcs.depositReefBranch(m_driverController.getHID(), flywheel, elevator, wrist))
            .alongWith(ledStrip.reefBranchColors(() -> elevator.goingToHeight)).andThen(ledStrip.toNormal())
        );

        // Get coral from coral station
        new Trigger(() -> ControllerUtils.dPadDown(m_secondaryController.getHID())).onFalse(
            elevator.goToIntakePosition().alongWith(wrist.goToIntakePosition())
        );
        m_driverController.a().onTrue(
            abcs.collectCoralStation(m_driverController.getHID(), flywheel, elevator, wrist)
            .alongWith(ledStrip.toAuton()).andThen(ledStrip.toNormal())
        );

        // Get algae from ground (in progress)
        new Trigger(() -> ControllerUtils.dPadUp(m_secondaryController.getHID())).onTrue(
            elevator.goToFloorAlgaePosition().alongWith(wrist.goAllTheWayUp()).andThen(algaeIntake.groundIntake())
        );
        new Trigger(() -> ControllerUtils.dPadUp(m_secondaryController.getHID())).onFalse(
            algaeIntake.stopIntake()
        );
        new Trigger(() -> ControllerUtils.dPadDown(m_driverController.getHID())).onTrue(
            algaeIntake.shootOut()
        );
        new Trigger(() -> ControllerUtils.rightTrigger(m_secondaryController.getHID())).onTrue(
            wrist.goAllTheWayUp()
        );

        // // Get algae from reef
        // new Trigger(() -> ControllerUtils.dPadLeft(m_secondaryController.getHID())).onFalse(
        //     elevator.goToPosition(() -> abcs.getAlgaeHeight()).alongWith(wrist.goAllTheWayUp())
        // );
        m_driverController.x().onTrue(
            abcs.collectAlgaeFromReef(elevator, algaeIntake)
            .alongWith(ledStrip.toAuton()).andThen(ledStrip.toNormal())
        );

        // Deposit algae into processor
        new Trigger(() -> ControllerUtils.dPadRight(m_secondaryController.getHID())).onFalse(
            elevator.goToProcessorPosition()
        );
        m_driverController.b().onTrue(
            abcs.doProcessor(elevator, algaeIntake)
            .alongWith(ledStrip.toAuton()).andThen(ledStrip.toNormal())
        );

        // Do the deep climb
        new Trigger(() -> ControllerUtils.rightTrigger(m_driverController.getHID())).onTrue(
            abcs.doDeepClimb()
            .alongWith(ledStrip.toAuton()).andThen(ledStrip.toNormal())
        );
        new Trigger(() -> ControllerUtils.dPadUp(m_driverController.getHID())).onTrue(
            climber.doClimb()
        );
        new Trigger(() -> ControllerUtils.dPadUp(m_driverController.getHID())).onFalse(
            climber.stopClimb()
        );

        // Emergency stops the aimbot (DO NOT USE UNLESS ABSOLUTELY NECESSARY)
        m_driverController.leftStick().onTrue(
            drivetrain.stopCurrentCommand().alongWith(abcs.stopCurrentCommand()).alongWith(ledStrip.toNormal()).alongWith(flywheel.dontShootCoral())
        );

        m_secondaryController.leftBumper().and(() -> ControllerUtils.leftTrigger(m_secondaryController.getHID())).onFalse(
            elevator.elevatorDown()
        );
        m_secondaryController.rightBumper().and(() -> ControllerUtils.leftTrigger(m_secondaryController.getHID())).onFalse(
            elevator.elevatorUp()
        );

        m_secondaryController.leftBumper().and(() -> ControllerUtils.dPadLeft(m_secondaryController.getHID())).onFalse(
            elevator.goToPosition(() -> ElevatorConstants.L2AlgaeHeight).alongWith(wrist.goAllTheWayUp())
        );
        m_secondaryController.rightBumper().and(() -> ControllerUtils.dPadLeft(m_secondaryController.getHID())).onFalse(
            elevator.goToPosition(() -> ElevatorConstants.L3AlgaeHeight).alongWith(wrist.goAllTheWayUp())
        );

        m_secondaryController.leftBumper().and(() -> !ControllerUtils.leftTrigger(m_secondaryController.getHID())).and(() -> !ControllerUtils.dPadLeft(m_secondaryController.getHID())).onFalse(
            wrist.moveDown()
        );
        m_secondaryController.rightBumper().and(() -> !ControllerUtils.leftTrigger(m_secondaryController.getHID())).and(() -> !ControllerUtils.dPadLeft(m_secondaryController.getHID())).onFalse(
            wrist.moveUp()
        );

        new Trigger(() -> ControllerUtils.dPadLeft(m_driverController.getHID())).onTrue(
            algaeIntake.groundIntake()
        );
        new Trigger(() -> ControllerUtils.dPadLeft(m_driverController.getHID())).onFalse(
            algaeIntake.stopIntake()
        );
        new Trigger(() -> ControllerUtils.dPadRight(m_driverController.getHID())).onTrue(
            algaeIntake.shootOut()
        );
        new Trigger(() -> ControllerUtils.dPadRight(m_driverController.getHID())).onFalse(
            algaeIntake.stopIntake()
        );
        

        m_secondaryController.leftStick().onTrue(flywheel.runIntake());
        m_secondaryController.leftStick().onFalse(flywheel.stopIntake());

        // // LEDs for Auton
        new Trigger(() -> DriverStation.isAutonomous()).onTrue(ledStrip.toAuton());
        new Trigger(() -> DriverStation.isAutonomous()).onFalse(ledStrip.toNormal());
    }

    public void resetSide() {
        DriverStation.getAlliance().ifPresent(allianceColor -> abcs.updateSide(allianceColor == Alliance.Blue));
        drivetrain.resetSide(abcs.getStartLoc());
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
