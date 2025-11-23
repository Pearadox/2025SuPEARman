package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisualizerConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
    private DoubleSupplier turretYawSupplier;
    private DoubleSupplier transferRollSupplier;
    private DoubleSupplier spindexerYawSupplier;
    private DoubleSupplier intakeRollSupplier;

    public RobotVisualizer(
            DoubleSupplier turretYawSupplier,
            DoubleSupplier transferRollSupplier,
            DoubleSupplier spindexerYawSupplier,
            DoubleSupplier intakeRollSupplier) {
        this.turretYawSupplier = turretYawSupplier;
        this.transferRollSupplier = transferRollSupplier;
        this.spindexerYawSupplier = spindexerYawSupplier;
        this.intakeRollSupplier = intakeRollSupplier;
    }

    public void periodic() {
        double turretYaw = turretYawSupplier.getAsDouble();
        double transferRoll = transferRollSupplier.getAsDouble();
        double spindexerYaw = spindexerYawSupplier.getAsDouble();
        double intakeRoll = -intakeRollSupplier.getAsDouble() + IntakeConstants.PIVOT_STARTING_ANGLE;

        Transform3d turret = new Transform3d(VisualizerConstants.M0_ZERO, new Rotation3d(0, 0, -turretYaw));
        Transform3d transfer = new Transform3d(VisualizerConstants.M1_ZERO, new Rotation3d(-transferRoll, 0, 0));
        Transform3d spindexer = new Transform3d(VisualizerConstants.M2_ZERO, new Rotation3d(0, 0, -spindexerYaw));
        Transform3d intakeStage1 = new Transform3d(VisualizerConstants.M3_ZERO, new Rotation3d(-intakeRoll, 0, 0));
        Transform3d intakeStage2 = new Transform3d(VisualizerConstants.M4_ZERO, new Rotation3d(-intakeRoll, 0, 0));
        Transform3d intakeStage3 =
                intakeStage1.plus(new Transform3d(VisualizerConstants.M5_OFFSET, new Rotation3d(0, 0, 0)));

        Logger.recordOutput(
                "FieldSimulation/Components2",
                new Transform3d[] {turret, transfer, spindexer, intakeStage1, intakeStage2, intakeStage3});
    }
}
