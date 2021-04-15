/**
 *
 * Programação desenvolvida pela equipe TchêStorm #16062
 *				  @tchestorm16062
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "TeleOp", group = "TeleOp")
public class TeleOperado extends LinearOpMode {

	//Declaração de Objetos
	DcMotor frontLeftMotor;
	DcMotor backLeftMotor;
	DcMotor frontRightMotor;
	DcMotor backRightMotor;
	
	DcMotor Coletor;

	Servo Garra;

	@Override
	public void runOpMode() {
		//Informa que está inicializando
		telemetry.addData("Status", "Inicializando");
		telemetry.update();

		//Instâncias dos Objetos
		frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
		backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
		frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
		backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
		
		Coletor = hardwareMap.get(DcMotor.class, "coletor");

		Garra = hardwareMap.get(Servo.class, "garra");

		//Define a direção dos motores
		frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		Coletor.setDirection(DcMotorSimple.Direction.FORWARD);

		//Define como os motores atuarão quando a potência for zero
		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		Coletor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		//Informa que a inicialização obteve êxito
		telemetry.addData("Status", "Aguardando Start");
		telemetry.addData("Bora da-lhe", "Tchê");
		telemetry.update();

		//Aguarda o piloto pressionar START
		waitForStart();

		telemetry.addData("Status", "Rodando");
		telemetry.update();
		boolean GarraOn = false;
		boolean ColetorOn = false;
		double minVelo = -.75;
		double maxVelo = .75;

		while (opModeIsActive()){
			
			double y = -gamepad1.left_stick_y; //frente e trás
			double x = gamepad1.left_stick_x; //esquerda e direita
			double rx = gamepad1.right_stick_x; //360º no proprio eixo
			
			//Aumenta o Range da velocidade
			if(gamepad1.right_bumper){
				maxVelo += 0.25;
				minVelo = -maxVelo;
				sleep(250);
			}
			//Reduz o Range da velocidade
			if(gamepad1.left_bumper){
				maxVelo -= 0.25;
				minVelo = -maxVelo;
				sleep(250);
			}

			/*
				Código para a movimentação
			 */
			double frontLeftPower = Range.clip((y + x + rx),minVelo,maxVelo);
			double backLeftPower = Range.clip((y - x + rx),minVelo,maxVelo);
			double frontRightPower = Range.clip((y - x - rx),minVelo,maxVelo);
			double backRightPower = Range.clip((y + x - rx),minVelo,maxVelo);

			frontLeftMotor.setPower(frontLeftPower);
			backLeftMotor.setPower(backLeftPower);
			frontRightMotor.setPower(frontRightPower);
			backRightMotor.setPower(backRightPower);

			/*
				Código para abrir e fechar a garra
			 */
			if(gamepad1.a && GarraOn == false){
				GarraOn = !GarraOn;
				telemetry.addData("Status","Abrindo");
				telemetry.update();
				Garra.setPosition(.6);
				sleep(250);
			}
			if(gamepad1.a && GarraOn == true){
				GarraOn = !GarraOn;
				telemetry.addData("Status","Fechando");
				telemetry.update();
				Garra.setPosition(.4);
				sleep(250);
			}
			/*
				Código para ligar e desligar o coletor
			 */
			if (gamepad1.b && ColetorOn == false){
				ColetorOn = !ColetorOn;
				Coletor.setPower(1);
				sleep(250);
			}
			if (gamepad1.b && ColetorOn == true){
				ColetorOn = !ColetorOn;
				Coletor.setPower(0);
				sleep(250);
			}
		}
	}
}
