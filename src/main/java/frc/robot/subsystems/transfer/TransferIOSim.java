package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.TransferConstants;

public class TransferIOSim extends TransferIOTalonFX {
    private final DCMotorSim rollerPhysicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    TransferConstants.TRANSFER_MOTORS,
                    TransferConstants.TRANSFER_MOI,
                    TransferConstants.TRANSFER_GEAR_RATIO),
            TransferConstants.TRANSFER_MOTORS);

    private final TalonFXSimState leftSimState;

    public TransferIOSim() {
        leftSimState = leftMotor.getSimState();
    }

    @Override
    public void updateInputs(TransferIOInputs inputs) {
        super.updateInputs(inputs);

        leftSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

        rollerPhysicsSim.setInputVoltage(leftSimState.getMotorVoltage());
        rollerPhysicsSim.update(Constants.LOOP_PERIOD);

        leftSimState.setRawRotorPosition(
                rollerPhysicsSim.getAngularPositionRad() / TransferConstants.TRANSFER_P_COEFFICIENT);
        leftSimState.setRotorVelocity(
                rollerPhysicsSim.getAngularVelocityRadPerSec() / TransferConstants.TRANSFER_P_COEFFICIENT);
    }
}
