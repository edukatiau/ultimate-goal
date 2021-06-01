/**
 *
 * Programação desenvolvida pela equipe TchêStorm #16062
 *				  @tchestorm16062
 *					TELE OPERADO
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "TeleOp", group = "Run1")
public class RTeleOp extends LinearOpMode {

	//Declaração dos Objetos
	DcMotor			frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, Coletor, Transicao;
	DcMotorEx		Braco, Lancador;
	Servo			Garra;

	@Override
	public void runOpMode() {

		//Informa que está inicializando
		telemetry.addData("Status", "Inicializando");
		telemetry.update();

		/** Iteração com o Hub */
		//HUB 1
		frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
		backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
		frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
		backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

		//HUB 2
		Braco = hardwareMap.get(DcMotorEx.class, "braco");
		Lancador = hardwareMap.get(DcMotorEx.class, "lancador");
		Coletor = hardwareMap.get(DcMotor.class, "coletor");
		Transicao = hardwareMap.get(DcMotor.class, "transicao");
		Garra = hardwareMap.get(Servo.class, "garra");

		/** Define a direção dos motores */
		frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
		backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
		frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
		backRightMotor.setDirection(DcMotor.Direction.FORWARD);
		
		Braco.setDirection(DcMotor.Direction.FORWARD);
		Lancador.setDirection(DcMotor.Direction.REVERSE);
		Coletor.setDirection(DcMotor.Direction.REVERSE);
		Transicao.setDirection(DcMotor.Direction.REVERSE);

		/** Define como os motores atuarão quando a potência for zero */
		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		Braco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		Coletor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		Transicao.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		Lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		
		/** Define os coeficientes do Lançador */
		Lancador.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
		
		/** Reseta os Encoders */
		resetEncoder();

		/** Aguarda o piloto pressionar START */
		telemetry.addData("Status", "Aguardando Start");
		telemetry.addData("Bora da-lhe", "Tchê");
		telemetry.update();
		waitForStart();
		
		/** Define variáveis e constantes */
		double minVelo = -.5;
		double maxVelo = .5;
		boolean GarraOn = false;
		boolean ColetorOn = false;
		boolean LancadorOn = false;
		boolean TransicaoOn = false;
		int position = 0;
		double kP = .4;
		double BracoPower = 50;
		boolean BracoOn = false;

		while (opModeIsActive()){

			/** Define variáveis dos Joysticks do Piloto 1 */
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

			/** Código para a movimentação */
			double frontLeftPower = Range.clip((y + x + rx),minVelo,maxVelo);
			double backLeftPower = Range.clip((y - x + rx),minVelo,maxVelo);
			double frontRightPower = Range.clip((y - x - rx),minVelo,maxVelo);
			double backRightPower = Range.clip((y + x - rx),minVelo,maxVelo);

			frontLeftMotor.setPower(frontLeftPower);
			backLeftMotor.setPower(backLeftPower);
			frontRightMotor.setPower(frontRightPower);
			backRightMotor.setPower(backRightPower);

			/** Código para abrir e fechar a garra */
			if(gamepad2.a && GarraOn == false){
				GarraOn = !GarraOn;
				telemetry.addData("Status","Abrindo Garra");
				telemetry.update();
				Garra.setPosition(1);
				sleep(500);
			}
			if(gamepad2.a && GarraOn == true){
				GarraOn = !GarraOn;
				telemetry.addData("Status","Fechando Garra");
				telemetry.update();
				Garra.setPosition(.5);
				sleep(500);
			}

			/** Levantar e Descer o Braço */
			if(gamepad2.b && BracoOn == false){
				BracoOn = !BracoOn;
				telemetry.addData("Status","Descendo Braço");
				telemetry.update();
				position = -350;
				sleep(500);
			}
			if(gamepad2.b && BracoOn == true){
				BracoOn = !BracoOn;
				telemetry.addData("Status","Subindo Braço");
				telemetry.update();
				position = 0;
				sleep(500);
			}

			Braco.setTargetPosition(position);
			BracoPower +=
					(Braco.getCurrentPosition() - position)*kP;

			Braco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			Braco.setVelocity(BracoPower);
			
			
			/** Liga e desliga o Lançador */
			if (gamepad1.x && LancadorOn == false){
				LancadorOn = true;
				Lancador.setVelocity(rpmTicks(5500));
				sleep(250);
			}
			if(gamepad1.x && LancadorOn == true){
				LancadorOn = false;
				Lancador.setVelocity(0);
				sleep(250);
			}
			

			/** Código para ligar e desligar o coletor */
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
			
			/** Código para ligar e desligar a transicao */
			if (gamepad1.y && TransicaoOn == false){
				TransicaoOn = !TransicaoOn;
				Transicao.setPower(1);
				sleep(250);
			}
			if (gamepad1.y && TransicaoOn == true){
				TransicaoOn = !TransicaoOn;
				Transicao.setPower(0);
				sleep(250);
			}
			
			
			telemetry.addData("FRight Motor Power: ", frontRightMotor.getPower());
			telemetry.addData("BRight Motor Power: ", backRightMotor.getPower());
			telemetry.addData("BLeft Motor Power: ", backLeftMotor.getPower());
			telemetry.addData("FLeft Motor Power: ", frontLeftMotor.getPower());
			telemetry.addData("Target", Braco.getCurrentPosition());
			telemetry.addData("LancadorPower", Lancador.getPower());
			telemetry.addData("Velocidade Maxima", maxVelo );
			telemetry.addData("Velocidade Minima", minVelo );
			telemetry.update();

			}
		}
		public void resetEncoder(){
			Braco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			Braco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			Lancador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			Lancador.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
		public double rpmTicks(int rpm){
			rpm/=60;
			double rpmTick = rpm*28;
			return rpmTick;
		}
}
