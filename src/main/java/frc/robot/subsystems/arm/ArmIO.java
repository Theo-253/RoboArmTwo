package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
@AutoLog
  public static class ArmIOInputs {
    public boolean masterMotorConnected = true;
    public double masterPositionRads = 0.0;
    public double masterVelocityRadPerSec = 0.0;
    public double masterAppliedVolts = 0.0;
    public double masterSupplyCurrentAmps = 0.0;
    public double masterTempCelsius = 0.0;
  }

  public static enum ArmIOOutputMode {
    BRAKE,
    CLOSED_LOOP,
    VOLTAGE
  }

  public static class ArmIOOutputs {
    public ArmIOOutputMode mode = ArmIOOutputMode.BRAKE;
    public double positionRad = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double voltage = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void applyOutputs(ArmIOOutputs outputs) {}

  default void runOpenLoop(double decimalPercentage) {}
}