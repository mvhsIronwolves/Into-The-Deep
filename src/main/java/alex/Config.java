package alex;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Config {
    public static class Hardware {
        public static class Motor {
            public static String frontLeftMotorName = "frontLeftMotor";
            public static DcMotorEx.Direction frontLeftMotorDirection = DcMotorEx.Direction.REVERSE;
            public static String frontRightMotorName = "frontRightMotor";
            public static DcMotorEx.Direction frontRightMotorDirection = DcMotorEx.Direction.FORWARD;
            public static String backLeftMotorName = "backLeftMotor";
            public static DcMotorEx.Direction backLeftMotorDirection = DcMotorEx.Direction.REVERSE;
            public static String backRightMotorName = "backRightMotor";
            public static DcMotorEx.Direction backRightMotorDirection = DcMotorEx.Direction.FORWARD;
            public static String winchMotorName = "winchMotor";
            public static DcMotorEx.Direction winchMotorDirection = DcMotorEx.Direction.REVERSE;
            public static double moveVelo = 1.0;
            public static int winchUpPos = 8500;
            public static int winchDownPos = 0;
            public static double winchVelo = 1.0;
            public static String rightViperName = "rightViper";
            public static DcMotorEx.Direction rightViperDirection = DcMotorEx.Direction.REVERSE;
            public static double rightViperMoveVelo = 0.4;
            public static int rightViperStoredPos = 0;
            public static int rightViperUpPos = 2100;
            public static int rightViperLowPos = 257;
            public static int rightViperDownPos = 330;
            public static String leftViperName = "leftViper";
            public static DcMotorSimple.Direction leftViperDirection = DcMotorSimple.Direction.FORWARD;
            public static double leftViperMoveVelo = 0.2;
            public static int leftViperStoredPos = 0;
            public static int leftViperDownPos = 560;
            public static int leftViperLowPos = 375;
            public static int leftViperUpPos = 440;
            public static String elbowMoterName = "elbowMotor";
            public static DcMotorSimple.Direction elbowMoterDirection = DcMotorSimple.Direction.FORWARD;
            public static double elbowMoterMoveVelo = 0.2;
            public static int elbowMoterStoredPos = 0;
            public static int elbowMoterDownPos = 560;
            public static int elbowMoterLowPos = 375;
            public static int elbowMoterUpPos = 440;
            
            ////////////////////////////////////////////////////////////
            
            public static double driveMotorPPR =  ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double winchMotorPPR = ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double strafeMultiplier = 100d/89d;
            public static double rotateMultiplierLeft = -1050.0 / (Math.PI/2);
            public static double rotateMultiplierRight = 1000.0 / (Math.PI/2);

        }

        public static class Servo {
//            public static String clawServoLName = "clawServoL";
//            public static com.qualcomm.robotcore.hardware.Servo.Direction clawServoLDirection = com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
//            public static double clawLOpenPosition = 0.025;
//            public static double clawLClosedPosition = 0.2;
//            public static String clawServoRName = "clawServoR";
//            public static com.qualcomm.robotcore.hardware.Servo.Direction clawServoRDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
//            public static double clawROpenPosition = 0.025;
//            public static double clawRClosedPosition = 0.2;
//            public static String leftLimitServoName = "leftLimitServo";
//            public static com.qualcomm.robotcore.hardware.Servo.Direction leftLimitServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
//            public static double leftLimitStowed = 0.0;
//            public static double leftLimitDeployed = 0.385;
//            public static String rightLimitServoName = "rightLimitServo";
//            public static com.qualcomm.robotcore.hardware.Servo.Direction rightLimitServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
//            public static double rightLimitStowed = 0.0;
//            public static double rightLimitDeployed = 0.33;
//            public static String launcherName = "launcher";
//            public static com.qualcomm.robotcore.hardware.Servo.Direction launcherDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
//            public static double launcherStored = 0.0;
//            public static double launcherDeployed = 0.2;

            public static String intakeName = "intake";
            public static com.qualcomm.robotcore.hardware.Servo.Direction intakeDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
//            public static double intakeSuck = 0.025;
//            public static double intakeSpit = 0.2;
            
        }

        public static class Digital{
            public static String whiskerLName = "leftWhisker";
            public static String whiskerRName = "rightWhisker";
            public static String distanceName = "distance";
            public static double minDistance = 4.0;
            public static String gyroName = "gyro";


        }

        public static class Wheel {
            public static double wheelRadius = 0.048;
            static double gearRatio = 1d;
            static double distanceFromCenter = 0.1703;
            //42.6 degrees for 1 wheel rotation
            //1111 ticks per full rotation of robot
            //4.5 in to center on y
            //5 in to center x
            //0.048/(0.1703)*360*.42
        }
    }

    public static class Software{
        public static class PawnPosition{
            public static int centerLeft = 230;
            public static int centerRight = 430;
        }
        public static boolean withinErr(int input, int val, int buffer){
            if(val-buffer < input && input < val+buffer)
                return true;
            return false;
        }
        public static int degrees = 0;
    }
}
