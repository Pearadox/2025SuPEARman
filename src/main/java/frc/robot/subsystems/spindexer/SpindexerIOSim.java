package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerIOSim extends SpindexerIOTalonFX {
    private final SingleJointedArmSim physicsSim = new SingleJointedArmSim(
            SpindexerConstants.SPINDEXER_MOTOR,
            SpindexerConstants.SPINDEXER_GEAR_RATIO,
            SpindexerConstants.SPINDEXER_MOI,
            SpindexerConstants.BUBBLE_TO_SPINDEXER, // unused
            Double.NEGATIVE_INFINITY,
            Double.POSITIVE_INFINITY,
            false,
            0.0);

    private final TalonFXSimState spindexerSimState;

    public SpindexerIOSim() {
        spindexerSimState = spindexerMotor.getSimState();
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        super.updateInputs(inputs);

        spindexerSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

        physicsSim.setInputVoltage(spindexerSimState.getMotorVoltage());
        physicsSim.update(Constants.LOOP_PERIOD);

        spindexerSimState.setRawRotorPosition(physicsSim.getAngleRads() / SpindexerConstants.SPINDEXER_P_COEFFICIENT);
        spindexerSimState.setRotorVelocity(
                physicsSim.getVelocityRadPerSec() / SpindexerConstants.SPINDEXER_P_COEFFICIENT);
    }
}
