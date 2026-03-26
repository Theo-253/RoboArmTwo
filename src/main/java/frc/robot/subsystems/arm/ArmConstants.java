package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static int sparkMasterArmCanId = 5;
    public static double sparkMasterArmkP = 0.1;
    public static double sparkMasterArmkI = 0.0;
    public static double sparkMasterArmkD = 0.065;
    public static double closedLoopAngularTolerance = Units.degreesToRadians(1);
}
