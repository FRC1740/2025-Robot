package frc.robot.constants;

import frc.robot.generated.TunerConstants;

public class CanIds {
    private static final int kFrontLeftDriveMotorId = TunerConstants.kFrontLeftDriveMotorId;
    private static final int kFrontLeftSteerMotorId = TunerConstants.kFrontLeftSteerMotorId;
    private static final int kFrontRightDriveMotorId = TunerConstants.kFrontRightDriveMotorId;
    private static final int kFrontRightSteerMotorId = TunerConstants.kFrontRightSteerMotorId;
    private static final int kBackRightDriveMotorId = TunerConstants.kBackRightDriveMotorId;
    private static final int kBackRightSteerMotorId = TunerConstants.kBackRightSteerMotorId;
    private static final int kBackLeftDriveMotorId = TunerConstants.kBackLeftDriveMotorId;
    private static final int kBackLeftSteerMotorId = TunerConstants.kBackLeftSteerMotorId;
    public static final int elevatorCanId = 9;
    public static final int wristCanId = 11;
    public static final int linearActuatorCanId = 12;

    public static final int[] canIds = new int[] {
        kFrontLeftDriveMotorId,
        kFrontLeftSteerMotorId,
        kFrontRightDriveMotorId,
        kFrontRightSteerMotorId,
        kBackRightDriveMotorId,
        kBackRightSteerMotorId,
        kBackLeftDriveMotorId,
        kBackLeftSteerMotorId,
        elevatorCanId,
        wristCanId,
        linearActuatorCanId,
    };

    public static Boolean validCanIds() {
        try {
            for (int i = 0; i < canIds.length; i++) {
                if (canIds[i] < 1 || canIds[i] > 63) {
                    throw new java.lang.Exception("CanId must be within 0-63: " + canIds[i]);
                }
                for (int j = 0; j < canIds.length; j++) {
                    if (i != j && canIds[i] == canIds[j]) {
                        throw new java.lang.Exception("CanId conflict detected between id " + canIds[i] + " and id " + canIds[j]);
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }finally{}

        return true;
    }
}
