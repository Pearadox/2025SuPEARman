package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisualizerConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
    private DoubleSupplier turretYawSupplier;
    private DoubleSupplier intakeRollSupplier;

    public RobotVisualizer(DoubleSupplier turretYawSupplier, DoubleSupplier intakeRollSupplier) {
        this.turretYawSupplier = turretYawSupplier;
        this.intakeRollSupplier = intakeRollSupplier;
    }

    public void periodic() {
        double a = 0; // Math.sin(RobotController.getFPGATime() / 1e6);

        double turretYaw = turretYawSupplier.getAsDouble();
        double intakeRoll = -intakeRollSupplier.getAsDouble() + IntakeConstants.PIVOT_STARTING_ANGLE;

        Transform3d stage0 = new Transform3d(VisualizerConstants.M0_ZERO, new Rotation3d(0, 0, -turretYaw));
        Transform3d stage1 = new Transform3d(VisualizerConstants.M1_ZERO, new Rotation3d(-a, 0, 0));
        Transform3d stage2 = new Transform3d(VisualizerConstants.M2_ZERO, new Rotation3d(0, 0, -a));
        Transform3d stage3 = new Transform3d(VisualizerConstants.M3_ZERO, new Rotation3d(-intakeRoll, 0, 0));
        Transform3d stage4 = new Transform3d(VisualizerConstants.M4_ZERO, new Rotation3d(-intakeRoll, 0, 0));
        Transform3d stage5 = stage3.plus(new Transform3d(VisualizerConstants.M5_OFFSET, new Rotation3d(0, 0, 0)));

        Logger.recordOutput(
                "FieldSimulation/Components2", new Transform3d[] {stage0, stage1, stage2, stage3, stage4, stage5});
    }
}
