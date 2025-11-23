package frc.robot.subsystems.spindexer;

import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    static class SpindexerIOInputs {
        public MotorData spindexerData = new MotorData();
    }

    default void updateInputs(SpindexerIOInputs inputs) {}

    default void runVolts(double volts) {}
}
