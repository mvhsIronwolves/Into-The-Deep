//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode;

import alex.Config;
import alex.Config.Hardware.Digital;
import alex.Config.Hardware.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp(
        name = "MainTeleOp",
        group = "Linear Opmode"
)
public class MainTeleOp extends LinearOpMode {
    private GyroSensor gyro;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor winchMotor;
    private DcMotor rightViper;
    private DcMotor leftViper;
    private DcMotor elbowMotor;
    private Servo intakeServo;
    private Servo clawServoR;
    private Servo launcher;
    private Servo leftLimitServo;
    private Servo rightLimitServo;
    private TouchSensor leftWhisker;
    private TouchSensor rightWhisker;
    private boolean rightTouched = false;
    private boolean leftTouched = false;
    private boolean targetClawROpen = false;
    private boolean targetClawLOpen = false;
    private boolean launcherStored = true;
    ArmPositions armPos;
    private double targetClawLPosition;
    private double targetClawRPosition;
    private final Gamepad previousGamepad1;
    private final Gamepad currentGamepad1;

    private int armState = 0;


    public MainTeleOp() {
        this.armPos = MainTeleOp.ArmPositions.STORED;
//        this.targetClawLPosition = alex.Config.Hardware.Servo.clawLClosedPosition;
//        this.targetClawRPosition = alex.Config.Hardware.Servo.clawRClosedPosition;
        this.previousGamepad1 = new Gamepad();
        this.currentGamepad1 = new Gamepad();
    }

    public void runOpMode() {
        this.frontLeftMotor = (DcMotor) this.hardwareMap.dcMotor.get(Motor.frontLeftMotorName);
        this.backLeftMotor = (DcMotor) this.hardwareMap.dcMotor.get(Motor.backLeftMotorName);
        this.frontRightMotor = (DcMotor) this.hardwareMap.dcMotor.get(Motor.frontRightMotorName);
        this.backRightMotor = (DcMotor) this.hardwareMap.dcMotor.get(Motor.backRightMotorName);
        this.winchMotor = (DcMotor) this.hardwareMap.dcMotor.get(Motor.winchMotorName);
        this.rightViper = (DcMotor) this.hardwareMap.dcMotor.get(Motor.rightViperName);
        this.leftViper = (DcMotor) this.hardwareMap.dcMotor.get(Motor.leftViperName);
        this.elbowMotor = (DcMotor) this.hardwareMap.dcMotor.get(Motor.elbowMoterName);
        this.intakeServo = (Servo) this.hardwareMap.servo.get(Config.Hardware.Servo.intakeName);
//        this.clawServoR = (Servo)this.hardwareMap.servo.get(alex.Config.Hardware.Servo.clawServoRName);
//        this.leftLimitServo = (Servo)this.hardwareMap.servo.get(alex.Config.Hardware.Servo.leftLimitServoName);
//        this.rightLimitServo = (Servo)this.hardwareMap.servo.get(alex.Config.Hardware.Servo.rightLimitServoName);
//        this.launcher = (Servo)this.hardwareMap.servo.get(alex.Config.Hardware.Servo.launcherName);
        this.frontLeftMotor.setDirection(Motor.frontLeftMotorDirection);
        this.frontRightMotor.setDirection(Motor.frontRightMotorDirection);
        this.backLeftMotor.setDirection(Motor.backLeftMotorDirection);
        this.backRightMotor.setDirection(Motor.backRightMotorDirection);
        this.winchMotor.setDirection(Motor.winchMotorDirection);
        this.rightViper.setDirection(Motor.rightViperDirection);
        this.leftViper.setDirection(Motor.leftViperDirection);
        this.elbowMotor.setDirection(Motor.elbowMoterDirection);
        this.intakeServo.setDirection(Config.Hardware.Servo.intakeDirection);
//        this.clawServoR.setDirection(alex.Config.Hardware.Servo.clawServoRDirection);
//        this.leftLimitServo.setDirection(alex.Config.Hardware.Servo.leftLimitServoDirection);
//        this.rightLimitServo.setDirection(alex.Config.Hardware.Servo.rightLimitServoDirection);
//        this.launcher.setDirection(alex.Config.Hardware.Servo.launcherDirection);
//        this.launcher.setPosition(alex.Config.Hardware.Servo.launcherStored);
        this.frontLeftMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.rightViper.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.leftViper.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.winchMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        this.rightViper.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.leftViper.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.elbowMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.winchMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
//        this.leftLimitServo.setPosition(alex.Config.Hardware.Servo.leftLimitStowed);
//        this.rightLimitServo.setPosition(alex.Config.Hardware.Servo.rightLimitStowed);
        this.leftWhisker = (TouchSensor) this.hardwareMap.touchSensor.get(Digital.whiskerLName);
        this.rightWhisker = (TouchSensor) this.hardwareMap.touchSensor.get(Digital.whiskerRName);
        this.gyro = (GyroSensor) this.hardwareMap.gyroSensor.get(Digital.gyroName);
        this.waitForStart();
        if (!this.isStopRequested()) {
            while (this.opModeIsActive()) {
                try {
                    this.currentGamepad1.copy(this.gamepad1);
                } catch (Exception var9) {
                    Exception e = var9;
                    e.printStackTrace();
                }

                double y = (double) (-this.gamepad1.left_stick_y);
                double x = (double) this.gamepad1.left_stick_x * 1.1;
                double rx = (double) this.gamepad1.right_stick_x;
//                boolean p = this.gamepad1.b;
                this.telemetry.addData("rightViperPos", this.rightViper.getCurrentPosition());
                this.telemetry.addData("leftViperPos", this.leftViper.getCurrentPosition());
                this.telemetry.addData("gyro", this.gyro.getHeading());
                if (!this.previousGamepad1.ps && this.currentGamepad1.ps) {
                    this.gyro.calibrate();
                }

                this.hand();
                this.arm();
                this.winch();
                this.launcher();
                this.drive(x, y, rx, Motor.moveVelo);
                this.telemetry.update();

                try {
                    this.previousGamepad1.copy(this.currentGamepad1);
                } catch (Exception var8) {
                    Exception e = var8;
                    e.printStackTrace();
                }
            }

        }
    }

    public void hand() {
//        if (this.currentGamepad1.right_trigger > 0.0) {
////            this.targetClawLOpen = !this.targetClawLOpen;
////            this.targetClawLPosition = this.targetClawLOpen ? alex.Config.Hardware.Servo.clawLOpenPosition : alex.Config.Hardware.Servo.clawLClosedPosition;
//            this.intakeServo.setPosition(-0.0);
//        }
//        else if (this.currentGamepad1.left_trigger > 0.0) {
////            this.targetClawLOpen = !this.targetClawLOpen;
////            this.targetClawLPosition = this.targetClawLOpen ? alex.Config.Hardware.Servo.clawLOpenPosition : alex.Config.Hardware.Servo.clawLClosedPosition;
//            this.intakeServo.setPosition(-0.0);
//        }

        if (gamepad1.right_trigger != 0) {
            intakeServo.setPosition(0.00);
        } else if (gamepad1.left_trigger != 0) {
            intakeServo.setPosition(1.00);
        } else {
            intakeServo.setPosition(0.50);
        }


    }

    public void arm() {
        if (!this.previousGamepad1.y && this.currentGamepad1.y) {
            switch (this.armPos.ordinal()) {
                case 0:
                case 3:
                    this.rightViper.setTargetPosition(Motor.rightViperUpPos);
                    this.leftViper.setTargetPosition(Motor.rightViperUpPos);
                    this.armPos = MainTeleOp.ArmPositions.HIGH;
                case 1:
                default:
                    break;
                case 2:
                    this.rightViper.setTargetPosition(Motor.rightViperUpPos);
                    this.leftViper.setTargetPosition(Motor.rightViperUpPos);
                    this.armPos = MainTeleOp.ArmPositions.HIGH;
            }

            this.rightViper.setMode(RunMode.RUN_TO_POSITION);
            this.leftViper.setMode(RunMode.RUN_TO_POSITION);
            this.rightViper.setPower(Motor.rightViperMoveVelo);
            this.leftViper.setPower(Motor.rightViperMoveVelo);
        }

        if (!this.previousGamepad1.a && this.currentGamepad1.a) {
            switch (this.armPos.ordinal()) {
                case 0:
                case 1:
                case 2:
                    this.rightViper.setTargetPosition(Motor.rightViperStoredPos);
                    this.leftViper.setTargetPosition(Motor.rightViperStoredPos);
                    this.rightViper.setMode(RunMode.RUN_TO_POSITION);
                    this.leftViper.setMode(RunMode.RUN_TO_POSITION);
                    this.rightViper.setPower(2.0 * Motor.rightViperMoveVelo / 3.0);
                    this.leftViper.setPower(2.0 * Motor.rightViperMoveVelo / 3.0);
//                    this.clawServoR.setPosition(alex.Config.Hardware.Servo.clawRClosedPosition);
//                    this.intakeServo.setPosition(alex.Config.Hardware.Servo.clawLClosedPosition);
                    this.armPos = MainTeleOp.ArmPositions.STORED;
            }
        }





        ////////////////////////// TESTING IF THE ARM CAN BE MOVED ////////////////////////////////

//        if(!this.previousGamepad1.b && this.currentGamepad1.b) {
//            this.elbowMotor.setTargetPosition(450);
//            this.elbowMotor.setMode(RunMode.RUN_TO_POSITION);
//            this.elbowMotor.setPower(1.0);
//
//        }
//
//        if(!this.previousGamepad1.x && this.currentGamepad1.x) {
//            this.elbowMotor.setTargetPosition(0);
//            this.elbowMotor.setMode(RunMode.RUN_TO_POSITION);
//            this.elbowMotor.setPower(1.0);
//
//        }

        ////////////////////////// BASIC IDEA FOR SWITCHING THROUGH PRESET ARM POSITIONS ///////////////////////////

//        if (this.gamepad1.right_bumper) {
//            if (armState == 1) {
//                //Drive turns into Pickup
//
//                // Tilt Viper max
//                this.elbowMotor.setTargetPosition(360);
//
//                armState++;
//            } else if (armState == 2) {
//                //Pickup turns into Low Basket
//                this.leftViper.setTargetPosition(Motor.leftViperLowPos);
//                this.rightViper.setTargetPosition(Motor.rightViperLowPos);
//                // Tilt viper forward 60
//                this.elbowMotor.setTargetPosition(180);
//
//                armState++;
//            } else if (armState == 3) {
//                //Low Basket changes to High Basket
//
//                //tilt viper forward 60 from vertical
//
//                this.leftViper.setTargetPosition(Motor.leftViperUpPos);
//                this.rightViper.setTargetPosition(Motor.rightViperUpPos);
//                this.elbowMotor.setTargetPosition(180);
//
//                armState++;
//            } else if (armState == 4) {
//                //High basket turns into drive
//
//                //tilt viper forward 60 from vertical
//
//                this.leftViper.setTargetPosition(0);
//                this.rightViper.setTargetPosition(0);
//                this.elbowMotor.setTargetPosition(0);
//
//                armState = 1;
//            }
//        }
//
//        if (this.gamepad1.left_bumper) {
//            if (armState == 1) {
//                //Drive turns into High Basket
//
//                //tilt viper forward 60 from vertical
//
//                this.leftViper.setTargetPosition(Motor.leftViperUpPos);
//                this.rightViper.setTargetPosition(Motor.rightViperUpPos);
//                this.elbowMotor.setTargetPosition(180);
//
//
//                armState = 4;
//            } else if (armState == 2) {
//                //Pickup turns Drive
//
//                this.leftViper.setTargetPosition(0);
//                this.rightViper.setTargetPosition(0);
//                this.elbowMotor.setTargetPosition(0);
//
//                armState--;
//            } else if (armState == 3) {
//                //Low Basket changes to Pickup
//
//                // Tilt Viper max
//                this.elbowMotor.setTargetPosition(360);
//                this.leftViper.setTargetPosition(0);
//                this.rightViper.setTargetPosition(0);
//
//                armState--;
//            } else if (armState == 4) {
//                //High basket turns low basket
//
//                this.leftViper.setTargetPosition(Motor.leftViperLowPos);
//                this.rightViper.setTargetPosition(Motor.rightViperLowPos);
//                // Tilt viper forward 60
//                this.elbowMotor.setTargetPosition(180);
//
//                armState--;
//            }
//        }




    }


    public void winch() {
        if (this.gamepad1.dpad_up && this.winchMotor.getCurrentPosition() < Motor.winchUpPos) {
            this.winchMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
            this.winchMotor.setPower(Motor.winchVelo);
        }

        if (this.gamepad1.dpad_down && this.winchMotor.getCurrentPosition() > Motor.winchDownPos) {
            this.winchMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
            this.winchMotor.setPower(-Motor.winchVelo);
        }

        if (this.previousGamepad1.dpad_up != this.currentGamepad1.dpad_up || this.previousGamepad1.dpad_down != this.currentGamepad1.dpad_down || this.winchMotor.getCurrentPosition() > Motor.winchUpPos || this.winchMotor.getCurrentPosition() < Motor.winchDownPos) {
            this.winchMotor.setPower(0.0);
        }

    }

    public void launcher() {
        if (!this.previousGamepad1.right_bumper && this.currentGamepad1.right_bumper) {
            this.launcherStored = !this.launcherStored;
//            double targetLaunchPosition = !this.launcherStored ? alex.Config.Hardware.Servo.launcherDeployed : alex.Config.Hardware.Servo.launcherStored;
//            this.launcher.setPosition(targetLaunchPosition);
        }

    }

    public void drive(double x, double y, double rx, double _power) {
        double power = _power;

        int precise = 1;
//
//        if(p)
//            precise = 2;

        if (this.armPos == MainTeleOp.ArmPositions.HIGH) {
            power = 2.0 * power / 3.0;
        }

//        if ((double)this.gamepad1.left_trigger > 0.0) {
//            power *= -1.0;
//            rx *= -1.0;
//        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower = Math.pow(y + x + rx, 1.0) / denominator;
        double backLeftPower = Math.pow(y - x + rx, 1.0) / denominator;
        double frontRightPower = Math.pow(y - x - rx, 1.0) / denominator;
        double backRightPower = Math.pow(y + x - rx, 1.0) / denominator;
        if (frontLeftPower == 0.0 && frontRightPower == 0.0 && backLeftPower == 0.0 && backRightPower == 0.0) {
            this.frontLeftMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            this.backLeftMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            this.frontRightMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
            this.backRightMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }

        this.frontLeftMotor.setPower((frontLeftPower * power)/precise);
        this.backLeftMotor.setPower((backLeftPower * power)/precise);
        this.frontRightMotor.setPower((frontRightPower * power)/precise);
        this.backRightMotor.setPower((backRightPower * power)/precise);
    }

    private static enum ArmPositions {
        DOWN,
        LOW,
        HIGH,
        STORED;

        private ArmPositions() {
        }
    }
}
