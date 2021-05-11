/**
 *
 * Programação desenvolvida pela equipe TchêStorm #16062
 *				  @tchestorm16062
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "TELE_TeleOp", group = "Run1")
public class TELE_TeleOp extends LinearOpMode {
  
	//Declaração dos Objetos
	DcMotor			frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, Lancador;
	DcMotorEx		Braco;
	//DcMotor			Coletor;
	Servo			Garra;
	  
	@Override
	public void runOpMode() {
	
		//Informa que está inicializando
		telemetry.addData("Status", "Inicializando");
		telemetry.update();
	
		//Instâncias dos Objetos
		//HUB 1
		frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
		backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
		frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
		backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
		
		//HUB 2
		Braco = hardwareMap.get(DcMotorEx.class, "braco");
		Garra = hardwareMap.get(Servo.class, "garra");
		Lancador = hardwareMap.get(DcMotor.class, "lancador");
		//Coletor = hardwareMap.get(DcMotor.class, "coletor");
	
		//Define a direção dos motores
		frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
		backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
		frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
		backRightMotor.setDirection(DcMotor.Direction.FORWARD);
		Braco.setDirection(DcMotor.Direction.FORWARD);
		Lancador.setDirection(DcMotor.Direction.FORWARD);
		//Coletor.setDirection(DcMotorSimple.Direction.FORWARD);

		//Define como os motores atuarão quando a potência for zero
		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		Braco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//Coletor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		
		resetEncoder();
		
		//Informa que a inicialização obteve êxito
		telemetry.addData("Status", "Aguardando Start");
		telemetry.addData("Bora da-lhe", "Tchê");
		telemetry.update();

		//Aguarda o piloto pressionar START
		waitForStart();
		double minVelo = -.5;
		double maxVelo = .5;
		boolean GarraOn = false;
		int position = 0;
		double kP = .4;
		double BracoPower = 50;
		boolean BracoOn = false;
	
		while (opModeIsActive()){
	  
			double y = -gamepad1.left_stick_y; //frente e trás
			double x = gamepad1.left_stick_x; //esquerda e direita
			double rx = gamepad1.right_stick_x; //360º no proprio eixo
	  
			//Aumenta o Range da Velocidade
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
			if(gamepad2.a && GarraOn == false){
				GarraOn = !GarraOn;
				telemetry.addData("Status","Abrindo");
				telemetry.update();
				Garra.setPosition(1);
				sleep(500);
			}
			if(gamepad2.a && GarraOn == true){
				GarraOn = !GarraOn;
				telemetry.addData("Status","Fechando");
				telemetry.update();
				Garra.setPosition(.5);
				sleep(500);
			}
			
			/*
			Levantar e Descer o Braço
			*/
			if(gamepad2.b && BracoOn == false){
				BracoOn = !BracoOn;
				position = 245;
				sleep(100);
			}
			if(gamepad2.b && BracoOn == true){
				BracoOn = !BracoOn;
				position = 0;
				sleep(100);
			}
			
			Braco.setTargetPosition(position);
			BracoPower += 
				(Braco.getCurrentPosition() - position)*kP;
			
			Braco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			Braco.setVelocity(BracoPower);
			
			/*
			Liga o Lançador
			*/
			if (gamepad2.x){
				Lancador.setPower(1);
			}
			else{
				Lancador.setPower(0);
			}
			
			/*
			Código para ligar e desligar o coletor
			
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
			*/
	  
			telemetry.addData("FRight Motor Power: ", frontRightMotor.getPower());
			telemetry.addData("BRight Motor Power: ", backRightMotor.getPower());
			telemetry.addData("BLeft Motor Power: ", backLeftMotor.getPower());
			telemetry.addData("FLeft Motor Power: ", frontLeftMotor.getPower());
			telemetry.addData("Target", Braco.getCurrentPosition());
			telemetry.addData("Lancador", Lancador.getPower());
			telemetry.addData("Velocidade Maxima", maxVelo );
			telemetry.addData("Velocidade Minima", minVelo );
			telemetry.update();
	  
		}
	}
	private void resetEncoder(){
			Braco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			Braco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
}
