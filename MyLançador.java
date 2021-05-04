package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "lançador", group = "Run1")
public class Mylancador extends LinearOpMode {

	//Declaração de Objetos
	DcMotor frontLeftMotor;
	DcMotor backLeftMotor;

	@Override
	public void runOpMode() {
		//Informa que está inicializando
		telemetry.addData("Status", "Inicializando");
		telemetry.update();

		//Instâncias dos Objetos
		frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
		backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");

		//Define a direção dos motores
		frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
		backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
		
		//Informa que a inicialização obteve êxito
		telemetry.addData("Status", "Aguardando Start");
		telemetry.update();

		//Aguarda o piloto pressionar START
		waitForStart();

		telemetry.addData("Status", "Rodando");
		telemetry.update();

		while (opModeIsActive()){

			if (gamepad2.a){
				frontLeftMotor.setPower(1);
				backLeftMotor.setPower(1);
			}
			else{
				frontLeftMotor.setPower(0);
				backLeftMotor.setPower(0);
			}
		}
	}
}
