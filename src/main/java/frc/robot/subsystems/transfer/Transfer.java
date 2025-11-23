package frc.robot.subsystems.transfer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.Constants.TransferConstants.TransferState;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Transfer extends SubsystemBase {
    @AutoLogOutput
    @Getter
    @Setter
    TransferState state = TransferState.OFF;

    private TransferIO io;
    private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();

    public Transfer(TransferIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transfer", inputs);

        io.runVolts(state.volts);

        Logger.recordOutput("Transfer/Setpoint volts", state.volts);
    }

    @AutoLogOutput
    public double getRoll() {
        return inputs.leftData.position() * TransferConstants.TRANSFER_P_COEFFICIENT;
    }
}
