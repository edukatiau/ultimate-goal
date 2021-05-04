/**
 *
 * Programação desenvolvida pela equipe TchêStorm #16062
 *				  @tchestorm16062
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp (name = "MyTeste1", group = "TeleOp")
public class MyTeste1 extends LinearOpMode {

	//Declaração de Objetos
	DcMotor frontLeftMotor;
	DcMotor backLeftMotor;
	DcMotor frontRightMotor;
	DcMotor backRightMotor;
	
	DcMotor braco;
	
	Servo mao;
	
	//DcMotor coletor;
  //DcMotor lancador

	private BNO055IMU imu; //gyroscope
	private Orientation angles;
	
	@Override
	public void runOpMode() {
		//Instâncias dos Objetos
		frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
		backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
		frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
		backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
    
    //lancador = hardwareMap.get(DcMotor.class, "lancador");
		
		braco = hardwareMap.get(DcMotor.class, "braco");
		mao = hardwareMap.get(Servo.class, "mao");
		
		//coletor = hardwareMap.get(DcMotor.class, "coletor");

		frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    
    //lancador.setDirection(DcMotorSimple.Direction.REVERSE);

		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json";

		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);
		
		telemetry.addData("Status", "Aguardando Start");
		telemetry.addData("Bora da-lhe", "Tchê");

		//Aguarda o piloto pressionar START
		waitForStart();
		double minVelo = -.5;
		double maxVelo = .5;
		double veloBraco = 1;
		boolean GarraOn = false;
		
		while (opModeIsActive()){
			
			//Aumenta o Range
			if(gamepad1.right_bumper){
				maxVelo += 0.25;
				minVelo = -maxVelo;
				sleep(500);
			}
			//Reduz o Range
			if(gamepad1.left_bumper){
				maxVelo -= 0.25;
				minVelo = -maxVelo;
				sleep(500);
			}
			
			double y = -gamepad1.left_stick_y; //frente e trás
			double x = gamepad1.left_stick_x; //esquerda e direita
			double rx = gamepad1.right_stick_x; //360º no proprio eixo

			double frontLeftPower = Range.clip((y + x + rx),minVelo,maxVelo);
			double backLeftPower = Range.clip((y - x + rx),minVelo,maxVelo);
			double frontRightPower = Range.clip((y - x - rx),minVelo,maxVelo);
			double backRightPower = Range.clip((y + x - rx),minVelo,maxVelo);

			frontLeftMotor.setPower(frontLeftPower);
			backLeftMotor.setPower(backLeftPower);
			frontRightMotor.setPower(frontRightPower);
			backRightMotor.setPower(backRightPower);
			
			if(gamepad2.a && GarraOn == false){
				GarraOn = !GarraOn;
				telemetry.addData("Status","Abrindo");
				telemetry.update();
				mao.setPosition(1);
				sleep(500);
			}
			if(gamepad2.a && GarraOn == true){
				GarraOn = !GarraOn;
				telemetry.addData("Status","Fechando");
				telemetry.update();
				mao.setPosition(0);
				sleep(500);
			}
		
			braco.setPower(gamepad2.left_stick_y);
			
			angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
			telemetry.addData("Status: ", "Running");
			telemetry.addData("Right Motor Power: ", frontRightMotor.getPower());
			telemetry.addData("Left Motor Power: ", frontLeftMotor.getPower());
			telemetry.addData("Velocidade Maxima", maxVelo );
			telemetry.addData("Velocidade Minima", minVelo );
			telemetry.addData("Z: ", angles.firstAngle);
			telemetry.addData("X: ", angles.secondAngle);
			telemetry.addData("Y: ", angles.thirdAngle);
			telemetry.update();

		}
	}
}
