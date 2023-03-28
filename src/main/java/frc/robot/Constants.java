package frc.robot;

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class MotorConstants {
        public static final int kRightFrontChannel = 0;
        public static final int kRightFrontFollowerChannel = 1;
        public static final int kRightRearChannel = 2;
        public static final int kRightRearFollowerChannel = 3;
      
        public static final int kLeftFrontChannel = 5;
        public static final int kLeftFrontFollowerChannel = 4;
        public static final int kLeftRearChannel = 6;
        public static final int kLeftRearFollowerChannel = 7;
    }

    public static class Drivetrain {
        public static final double KtTimeToMaxOutput = 0.25;
    }
}
