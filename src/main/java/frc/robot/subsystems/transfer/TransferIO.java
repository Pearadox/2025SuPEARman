package frc.robot.subsystems.transfer;

import frc.robot.util.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface TransferIO {
    @AutoLog
    static class TransferIOInputs {
        public MotorData leftData = new MotorData();
        public MotorData rightData = new MotorData();
    }

    default void updateInputs(TransferIOInputs inputs) {}

    default void runVolts(double volts) {}
}
