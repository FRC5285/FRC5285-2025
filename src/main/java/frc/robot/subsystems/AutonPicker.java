package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class for the autonomous chooser
 */
public class AutonPicker extends SubsystemBase {

    private SendableChooser<Integer> startPos = new SendableChooser<>();
    private SendableChooser<Integer> reefSide = new SendableChooser<>();
    private SendableChooser<Boolean> coralStation = new SendableChooser<>();
    private SendableChooser<Boolean> getCoral = new SendableChooser<>();
    private AimbotCommands abcs;
    private CommandSwerveDrivetrain drivetrain;

    /**
     * Inits the AutonPicker object
     */
    public AutonPicker(CommandSwerveDrivetrain drivetrain, AimbotCommands abcs) {
        this.drivetrain = drivetrain;
        this.abcs = abcs;
        this.startPos.setDefaultOption("0", 0);
        for (int i = 1; i <= 6; i ++) this.startPos.addOption("" + i, i);
        this.reefSide.setDefaultOption("0", 0);
        for (int i = 1; i <= 5; i ++) this.reefSide.addOption("" + i, i);
        this.coralStation.setDefaultOption("Left", true);
        this.coralStation.addOption("Right", false);
        this.getCoral.setDefaultOption("Yes", true);
        this.getCoral.addOption("No", false);
        SmartDashboard.putData("Start Position", this.startPos);
        SmartDashboard.putData("Reef Side", this.reefSide);
        SmartDashboard.putData("Coral Station", this.coralStation);
        SmartDashboard.putData("Get Coral?", this.getCoral);
    }

    /**
     * Builds the auton command, use before auton
     * 
     * @return The auton command object
     */
    public Command theAuton() {
        return runOnce(() -> {
            this.drivetrain.resetPose(this.abcs.getStartPosition(this.startPos.getSelected().intValue()));
        })
        .andThen(this.abcs.depositReefBranch(this.reefSide.getSelected().intValue(), true))
        .andThen(this.abcs.collectCoralStation(this.coralStation.getSelected().booleanValue(), false, false).onlyIf(() -> this.getCoral.getSelected().booleanValue() == true))
        .andThen(this.abcs.depositReefBranch(this.reefSide.getSelected().intValue(), false).onlyIf(() -> this.getCoral.getSelected().booleanValue() == true))
        .andThen(this.abcs.collectCoralStation(this.coralStation.getSelected().booleanValue(), false, false).onlyIf(() -> this.getCoral.getSelected().booleanValue() == true));
    }
}
