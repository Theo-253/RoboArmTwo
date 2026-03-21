package frc.robot.subsystems.arm;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.arm.ArmIO.ArmIOOutputMode;
import frc.robot.subsystems.arm.ArmIO.ArmIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends FullSubsystem {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final ArmIOOutputs outputs = new ArmIOOutputs();

  private Alert masterDisconnected;

  @RequiredArgsConstructor
 
   public enum Goal {
    IDLE(() -> 0.0),
    NINETY(() -> Units.degreesToRotations(90)),
    FORTYFIVE(() -> Units.degreesToRotations(45)),
    TWOSEVENTY(() -> Units.degreesToRotations(270)),
    ONEEIGHTY(() -> Units.degreesToRotations(180)),
    VOLTAGE(() -> -0.5);

    private final DoubleSupplier voltage;

    /** Returns the current target voltage for this goal state. */
    private double getGoal() {
      return voltage.getAsDouble();
    }
  }

  @AutoLogOutput(key = "Arm/Goal")
  private Goal currentGoal = Goal.IDLE;

  public ArmSubsystem(ArmIO io) {
    this.io = io;

    masterDisconnected = new Alert("Arm motor disconnected!", Alert.AlertType.kWarning);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Arm Idle"));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);


    // Force IDLE state if the robot is disabled so it doesn't snap to last arm angle on enable
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Re-poll the supplier every loop to handle new shot calculations
    if (currentGoal == Goal.IDLE) {
      stop();
    } else if (currentGoal == Goal.VOLTAGE) {
      runVoltage(currentGoal.getGoal());
    } else {
      runAngular(currentGoal.getGoal());
    }
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Arm/Mode", outputs.mode);
    io.applyOutputs(outputs);
  }

  /**
   * Sets the current goal state for the subsystem.
   *
   * @param desiredGoal The new goal to "transition" to.
   */
  private void setGoal(Goal desiredGoal) {
    this.currentGoal = desiredGoal;
  }

  @AutoLogOutput(key = "Arm/AtGoal")
  /**
   * Returns true if the arm is within the angular tolerance. Note: IDLE check catches the
   * exception because I'm lazy
   */
  public boolean atGoal() {
    return currentGoal == Goal.IDLE
        || Math.abs(getAngle() - currentGoal.getGoal())
            <= ArmConstants.closedLoopAngularTolerance;
  }

  /**
   * Update the io for angular closed loop control and applies the new angular setpoint
   *
   * @param angleRads the new angular setpoint.
   */
  private void runAngular(double angleRads) {
    outputs.mode = ArmIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = angleRads;
  }

  /**
   * Update the io for voltage control and applies the new voltage setpoint
   *
   * @param voltage the new voltage setpoint.
   */
  private void runVoltage(double voltage) {
    outputs.mode = ArmIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  @AutoLogOutput(key = "Arm/MeasuredAngleRads")
  public double getAngle() {
    return inputs.masterPositionRads;
  }



  public Command ninetyCommand() {
    return startEnd(() -> setGoal(Goal.NINETY), () -> setGoal(Goal.IDLE)).withName("Arm Juggle");
  }

  public Command fortyFiveCommand() {
    return startEnd(() -> setGoal(Goal.FORTYFIVE), () -> setGoal(Goal.IDLE)).withName("Arm Debug");
  }

  public Command oneEightyCommand() {
    return startEnd(() -> setGoal(Goal.ONEEIGHTY), () -> setGoal(Goal.IDLE)).withName("Arm Static");
  }

  public Command twoSeventyCommand() {
    return startEnd(() -> setGoal(Goal.TWOSEVENTY), () -> setGoal(Goal.IDLE))
        .withName("Arm Debug Voltage Up");
  }

    public Command voltageCommand() {
    return startEnd(() -> setGoal(Goal.VOLTAGE), () -> setGoal(Goal.IDLE))
        .withName("VOltage Command");
  }


  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command reZeroCommand() {
    return runOnce(this::reZero);
  }

  private void stop() {
    outputs.mode = ArmIOOutputMode.BRAKE;
  }

  private void reZero() {
    io.zeroEncoders();
  }
}

