package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Map;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// import for IMU (gyroscope)
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="OneController", group="Linear Opmode")

public class TeleOpOneController extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor FLMoto;
    private DcMotor FRMoto;
    private DcMotor BLMoto;
    private DcMotor BRMoto;
    private DcMotor IntakeMoto;
    private DcMotor ArmMoto;
    private DcMotor LaunchMoto;
    private Servo LWServes;
    
    private Servo ExtenderServes;
    private Servo DeployServes;
    private VoltageSensor ExpansionHub1_VoltageSensor;
    private DistanceSensor sensorDistance;
    
    private BNO055IMU imu;
    private double globalAngle, correction;
    private Orientation lastAngle = new Orientation();
    
    /*********************/
    /* Variables for imu */
    /*********************/
    
    Orientation   lastAngles = new Orientation();
    
    private static final Double wheelCircumference = 4.0 * Math.PI;
    private static final Double gearRatio = 1.0;
    private static final Double countsPerRotation = 2240.0;
    private static final Double scaleFactor = 0.50;
    private static final Double countsPerInch =  countsPerRotation / wheelCircumference / gearRatio * scaleFactor;
    private static Double noPower = 0.0;
    private static Double quarterPower = 0.25;
    private static Double halfPower = 0.5;
    private static Double threeQuartPower = 0.75;
    private static Double fullPower = 1.0;
    
    double voltage;
    private static double targetShooterPower = 0.46;
    private static Double batteryConst = 13.5;
    private static double powerConstant = batteryConst*targetShooterPower;
    
    
    @Override
    public void runOpMode() {
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FLMoto  = hardwareMap.get(DcMotor.class, "FLMoto");
        FRMoto = hardwareMap.get(DcMotor.class, "FRMoto");
        BLMoto  = hardwareMap.get(DcMotor.class, "BLMoto");
        BRMoto = hardwareMap.get(DcMotor.class, "BRMoto");
        IntakeMoto  = hardwareMap.get(DcMotor.class, "IntakeMoto");
        ArmMoto  = hardwareMap.get(DcMotor.class, "ArmMoto");
        LaunchMoto = hardwareMap.get(DcMotor.class, "LaunchMoto");
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "LaunchMoto");
        LWServes = hardwareMap.servo.get("LWServes");
        ExtenderServes = hardwareMap.servo.get("ExtenderServes");
        DeployServes = hardwareMap.servo.get("DeployServes");
        ExpansionHub1_VoltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 1");
        sensorDistance = hardwareMap.get(DistanceSensor.class,"CDSensor");
       
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FLMoto.setDirection(DcMotor.Direction.REVERSE);
        FRMoto.setDirection(DcMotor.Direction.FORWARD);
        BLMoto.setDirection(DcMotor.Direction.REVERSE);
        BRMoto.setDirection(DcMotor.Direction.FORWARD);
        IntakeMoto.setDirection(DcMotor.Direction.REVERSE);
        LaunchMoto.setDirection(DcMotor.Direction.FORWARD);
        ArmMoto.setDirection(DcMotor.Direction.REVERSE);
        
        
        //FLMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //FRMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //BLMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //ArmMoto.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        LWServes.setPosition(0.0);
        ExtenderServes.setPosition(0.5);
        
        
        double currentVelocity;
        double maxVelocity = 0.0;
        double kP = 1.32;
        double kI = 0.132;
        double kD = 0;
        double kF = 13.2;
        
        /************************/
        /* Setup IMU parameters */
        /************************/
    
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
    
        // Retrieve and initialize the IMU. The IMU should be attached to 
        // IC2 port on a Core Device Interface Module
        imu = hardwareMap.get(BNO055IMU.class, "imu");
           
        imu.initialize(parameters);
                  
        telemetry.addData("Mode","calibrating imu...." );
           
        // make sure the imu gyro is calibrated before continuing
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
         sleep(50);
         idle();
        }
        
        double targetShooterPower = 0.755;
        powerConstant = batteryConst*targetShooterPower;
        voltage = ExpansionHub1_VoltageSensor.getVoltage();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString() );
        telemetry.addData("Voltage", voltage);
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            telemetry.addData("Status","Running" );
            telemetry.addData("Servo Position",LWServes.getPosition());
            telemetry.addData("Servo Position",ExtenderServes.getPosition());
            telemetry.addData("Servo Position",DeployServes.getPosition());
             
            currentVelocity = motor.getVelocity();
              
            if(currentVelocity > maxVelocity){
              maxVelocity = currentVelocity;
            }
              
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Maximum Velocity", maxVelocity);
            
            // telemetry.addData("Target Power", tgtPower );
            telemetry.addData("Distance (cm)", 
                String.format(Locale.US,
                "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            
            telemetry.update();

            if(gamepad1.y) { // closes launch servo
            LWServes.setPosition (0.0);
            
            } else if (gamepad1.x){ // opens launch servo
            LWServes.setPosition (1);
        
            } else if (gamepad1.b){
            DeployServes.setPosition(0.04);
            sleep(100);
            ExtenderServes.setPosition(0.45);
            } 
            else if (gamepad1.a){
            ExtenderServes.setPosition(0);
            sleep(100);
            DeployServes.setPosition(0);
            }
            
            
          //**************************************************************//
          // Move robot forward and backward using joystick on game pad 1 //
          //**************************************************************//
          FLMoto.setPower(-gamepad1.left_stick_y);
          BLMoto.setPower(-gamepad1.left_stick_y);
          FRMoto.setPower(-gamepad1.right_stick_y);
          BRMoto.setPower(-gamepad1.right_stick_y);
          
          //************************************************************//
          // Straif robot using right and left trigger on game pad 1    //
          //************************************************************//
          // straif left 
          //FLMoto.setPower(-gamepad1.left_trigger);
          //BLMoto.setPower(Math.abs(gamepad1.left_trigger));
          //FRMoto.setPower(Math.abs(gamepad1.left_trigger));
          //BRMoto.setPower(-gamepad1.left_trigger);
          
          FLMoto.setPower(gamepad1.right_stick_x);
          BLMoto.setPower(-gamepad1.right_stick_x);
          FRMoto.setPower(-gamepad1.right_stick_x);
          BRMoto.setPower(gamepad1.right_stick_x);
          
          // straif right
          //FLMoto.setPower(Math.abs(gamepad1.right_trigger));
          //BLMoto.setPower(-gamepad1.right_trigger);
          //FRMoto.setPower(-gamepad1.right_trigger);
          //BRMoto.setPower(Math.abs(gamepad1.right_trigger));
          
          //************************************************************//
          // raise and lower arm using left joy stick on game pad 2      //
          // gravity resistance with right joy stick                     //
          //************************************************************//
          // contols arm
            if (gamepad1.left_bumper){
                ArmMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                ArmMoto.setPower(0.8);
            }
            else if (gamepad1.right_bumper){
                ArmMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                ArmMoto.setPower(-0.8);
            }
            else {
                ArmMoto.setPower(0);
                ArmMoto.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                
            }
            
          //************************************************************//
          // Intake, Sweep, and Launch Motors using joysticks on game pad 2//
          //************************************************************//
          IntakeMoto.setPower(gamepad1.right_trigger);
          
          if (gamepad1.start){
                IntakeMoto.setPower(-1);
                LaunchMoto.setPower(-0.1);
            }
            
          /**if  (gamepad1.dpad_right){
              IntakeMoto.setPower(1);
              sleep(2500);
          }**/
          
          if (gamepad1.dpad_up){
              //voltage = ExpansionHub1_VoltageSensor.getVoltage();
              //LaunchMoto.setPower(powerConstant/voltage);
              
              motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
              motor.setVelocity(1700);
            }
          if(gamepad1.dpad_down){
              //voltage = ExpansionHub1_VoltageSensor.getVoltage();
              //LaunchMoto.setPower(powerConstant/voltage);
              
              motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
              motor.setVelocity(1450);
          } 
          
          if(gamepad1.dpad_left){
  
              
             /*******************/
             /* Reset IMU angle */
             /*******************/
             resetAngle();
              
              motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
              motor.setVelocity(1500);
              sleep(1600);
              
              currentVelocity = motor.getVelocity();
              
              telemetry.addData("Current Velocity", currentVelocity);
              telemetry.addData("Maximum Velocity", maxVelocity);
              telemetry.update();
            
              double IntakePowerCont = ((batteryConst*1)/voltage);
              IntakeMoto.setPower(IntakePowerCont);
              sleep(600);
              
              IntakeMoto.setPower(0);
    
              DriveInch(7.0, -quarterPower, quarterPower, quarterPower, -quarterPower);
              
              currentVelocity = motor.getVelocity();
              
              telemetry.addData("Current Velocity", currentVelocity);
              telemetry.addData("Maximum Velocity", maxVelocity);
              telemetry.update();
              
              IntakeMoto.setPower(IntakePowerCont);
              sleep(1000);
            
              IntakeMoto.setPower(0);
    
              DriveInch(6.25, -quarterPower, quarterPower, quarterPower, -quarterPower);
              
              motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
              motor.setVelocity(1420);
              sleep(1000);
            
              currentVelocity = motor.getVelocity();
              
              telemetry.addData("Current Velocity", currentVelocity);
              telemetry.addData("Maximum Velocity", maxVelocity);
              telemetry.update();
              
              IntakeMoto.setPower(IntakePowerCont);
              sleep(1200);
              
              IntakeMoto.setPower(0);
              motor.setVelocity(0);
              
              resetEncoders();
              
              runWithoutEncoders();
              
              
          }
          
          if (gamepad1.b){
              LaunchMoto.setPower(0);
          }
          if(gamepad1.y){
              LaunchMoto.setPower(0);
          }
        }
    }
    
    private void resetEncoders() {
        FRMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMoto.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }    

    private void runUsingEncoders() {
        FRMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMoto.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
     private void runWithoutEncoders() {
        FRMoto.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        BRMoto.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLMoto.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        BLMoto.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
     }
    
    private void setDrivePower(double FLpower,double FRpower, Double BLpower, Double BRpower) {  
        FRMoto.setPower(FRpower);
        BRMoto.setPower(BRpower);
        FLMoto.setPower(FLpower);
        BLMoto.setPower(BLpower);
    }

    /************************************************************/
    /*                                                          */
    /* Function: DriveInch                                      */
    /* Returns: nothing                                         */
    /*                                                          */
    /* This function is called to have the robot move straigh   */
    /* in a forward or reverse direction.                       */
    /*                                                          */
    /************************************************************/

    private void DriveInch(Double inches,Double FLpower,Double FRpower, Double BLpower, Double BRpower) {
       
       Double counts = inches * countsPerInch;
           
       resetEncoders();
       runUsingEncoders();
           
       voltage = ExpansionHub1_VoltageSensor.getVoltage();
       double FLpowerCont = ((batteryConst*FLpower)/voltage);
       double FRpowerCont = ((batteryConst*FRpower)/voltage);
       double BLpowerCont = ((batteryConst*BLpower)/voltage);
       double BRpowerCont = ((batteryConst*BRpower)/voltage);
           
           setDrivePower(FLpowerCont, FRpowerCont, BLpowerCont, BRpowerCont);
        
           while (opModeIsActive() && 
              (Math.abs(FLMoto.getCurrentPosition()) + Math.abs(FRMoto.getCurrentPosition()) /2) 
                         < Math.abs(counts)) {       
                             
               // Use gyro to drive in a straight line.
               correction = checkDirection();
               
               // telemetry.addData("1 imu heading", lastAngles.firstAngle);
               // telemetry.addData("2 global heading", globalAngle);
               // telemetry.addData("3 correction", correction);
               // telemetry.update();
               
               setDrivePower(FLpower-correction, FRpower+correction, BLpower-correction, BRpower+correction);
               idle();
               
           }
     
           setDrivePower(noPower,noPower,noPower,noPower);     // Stop all motors
    }
    
           /************************************************************/
        /*                                                          */
        /* Function: DriveInchWithSlowing                           */
        /* Returns: nothing                                         */
        /*                                                          */
        /* This function is called to have the robot move straigh   */
        /* in a forward or reverse direction.                       */
        /*                                                          */
        /************************************************************/

        private void DriveInchWithSlowing(Double inches,Double FLpower,Double FRpower, Double BLpower, Double BRpower) {
           
           Double counts = inches * countsPerInch;
           Double slowDownCount = counts - counts/3; 
           
           resetEncoders();
           runUsingEncoders();
           setDrivePower(FLpower, FRpower, BLpower, BRpower);
        
           while (opModeIsActive() && 
              (Math.abs(FLMoto.getCurrentPosition()) + Math.abs(FRMoto.getCurrentPosition()) /2) 
                         < Math.abs(counts)) {       
                             
               // Use gyro to drive in a straight line.
               correction = checkDirection();
               
               // telemetry.addData("1 imu heading", lastAngles.firstAngle);
               // telemetry.addData("2 global heading", globalAngle);
               // telemetry.addData("3 correction", correction);
               // telemetry.update();
               
               if ((Math.abs(FLMoto.getCurrentPosition()) + Math.abs(FRMoto.getCurrentPosition()) /2) > slowDownCount) {
                         
                   FLpower = FLpower * 0.85;
                   FRpower = FRpower * 0.85;
                   BLpower = BLpower * 0.85;
                   BRpower = BRpower * 0.85;
                }
                
               setDrivePower(FLpower-correction, FRpower+correction, BLpower-correction, BRpower+correction);
               idle();
               
           }
     
           setDrivePower(noPower,noPower,noPower,noPower);     // Stop all motors
    
        }
        
        /***************************************************/
        /* Resets the cumulative angle tracking to zero.   */
        /***************************************************/
        private void resetAngle()
        {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    
            globalAngle = 0;
        }
        
        /************************************************************/
        /* Get current cumulative angle rotation from last reset.   */
        /* @return Angle in degrees. + = left, - = right.           */
        /************************************************************/

        private double getAngle()
        {
            // We experimentally determined the Z axis is the axis we want to use for heading angle.
            // We have to process the angle because the imu works in euler angles so the Z axis is
            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
    
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    
            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
    
            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;
    
            globalAngle += deltaAngle;
    
            lastAngles = angles;
    
            return globalAngle;
        }
    
         /**
         * See if we are moving in a straight line and if not return a power correction value.
         * @return Power adjustment, + is adjust left - is adjust right.
         */
        private double checkDirection()
        {
            // The gain value determines how sensitive the correction is to direction changes.
            // You will have to experiment with your robot to get small smooth direction changes
            // to stay on a straight line.
            double correction, angle, gain = .05;
    
            angle = getAngle();
    
            if (angle == 0)
                correction = 0;             // no adjustment.
            else
                correction = -angle;        // reverse sign of angle for correction.
    
            correction = correction * gain;
    
            return correction;
        }
    
        /**
         * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
         * @param degrees Degrees to turn, + is left - is right
         */
        private void rotate(int degrees, double power)
        {
            double  leftPower, rightPower;
    
            // restart imu movement tracking.
            resetAngle();
    
            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).
            if (degrees < 0)
            {   // turn right.
                leftPower = power;
                rightPower = -power;
            }
            else if (degrees > 0)
            {   // turn left.
                leftPower = -power;
                rightPower = power;
            }
            else return;
    
            // set power to rotate.
            setDrivePower(leftPower,rightPower,leftPower,rightPower);
    
            // rotate until turn is completed.
            if (degrees < 0)
            {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && getAngle() == 0) {}
    
                while (opModeIsActive() && getAngle() > degrees) {}
            }
            else    // left turn.
                while (opModeIsActive() && getAngle() < degrees) {}
           
            // turn the motors off.
            setDrivePower(0.0,0.0,0.0,0.0);
            
            // wait for rotation to stop.
            sleep(500);
    
            // reset angle tracking on new heading.
            resetAngle();
        }

}
