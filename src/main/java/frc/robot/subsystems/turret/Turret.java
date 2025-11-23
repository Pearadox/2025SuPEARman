package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private Supplier<Rotation2d> targetSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;

    public Turret(
            TurretIO io,
            Supplier<Rotation2d> robotCentricAngleSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.io = io;
        this.targetSupplier = robotCentricAngleSupplier;
        this.speedsSupplier = chassisSpeedsSupplier;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        double setpointRads = wrap(targetSupplier.get().getRadians());
        double setpointRots = setpointRads / TurretConstants.TURRET_P_COEFFICIENT;

        double ffVolts = getFF(setpointRads);

        io.runPosition(setpointRots, ffVolts);

        Logger.recordOutput("Turret/Setpoint Rots", setpointRots);
        Logger.recordOutput("Turret/FF Volts", ffVolts);
    }

    @AutoLogOutput
    public double getTurretAngleRads() {
        return inputs.turretData.position() * TurretConstants.TURRET_P_COEFFICIENT;
    }

    public double wrap(double target) {
        double current = getTurretAngleRads();

        double[] candidates = new double[] {target - 2 * Math.PI, target, target + 2 * Math.PI};

        double best = target;
        double bestDist = Double.POSITIVE_INFINITY;

        for (double c : candidates) {
            if (c > TurretConstants.TURRET_SAFE_MIN && c < TurretConstants.TURRET_SAFE_MAX) {
                double dist = Math.abs(current - c);
                if (dist < bestDist) {
                    best = c;
                    bestDist = dist;
                }
            }
        }

        return best;
    }

    public double getFF(double setpointRads) {
        double chassisAngularVelocity = speedsSupplier.get().omegaRadiansPerSecond;

        boolean shouldApplyFF = Math.abs(Rotation2d.fromRadians(setpointRads)
                                .minus(Rotation2d.fromRadians(getTurretAngleRads()))
                                .getRadians())
                        < TurretConstants.FF_ERROR_THRESHOLD
                && Math.abs(chassisAngularVelocity) < TurretConstants.FF_CHASSIS_ROT_VELOCITY_LIMIT;

        return shouldApplyFF ? chassisAngularVelocity * TurretConstants.K_OMEGA : 0;
    }
}
