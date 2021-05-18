/**
 *
 * Programação desenvolvida pela equipe TchêStorm #16062
 *				  @tchestorm16062
 *					 AUTONOMO
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "RAutonomo", group = "Autonomous", preselectTeleOp = "RTeleOp")
public class RAutonomo extends LinearOpMode {
	DcMotor		 frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, Lancador;
	//DcMotor			Coletor;
	DcMotorEx	   Braco;
	Servo		Garra,Dedo;
	BNO055IMU	   imu;
	Orientation	 lastAngles = new Orientation();
	double		  globalAngle = .30;

	double		  DIAMETRO_IN = 3.966;
	double		  POLEGADA = 2.54;
	double		  ENGRENAGEM = 2.6;
	int			  TICKS_REV = 1120;

	int			 position = 0;
	double		  kP = .4;
	double		  BracoPower = 50;

	private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
	private static final String LABEL_FIRST_ELEMENT = "Quad";
	private static final String LABEL_SECOND_ELEMENT = "Single";
	private static final String VUFORIA_KEY =
			"AYWpo1j/////AAABmXdvyto7jU+LuXGPiPaJ7eQ4FIrujbhvZmoi " +
					" KRcyjHFOYhPWujqUT8itJ5yl5d6xeQtRltWIaeULLDoE/zTbq+fGgveeiVmFzR45LGe6HWGjNi2twZhZqTPWFh" +
					" 8KGHueGcpX5am/wGJGKEp25ELJ+z9laddGkm0ykwJVAJ5NP47SSdBbAb/yzDCQmAUnuNvQMgSbm8fv0wE/tukSV" +
					" CgkhEaGuipkWgO9t6HDyh2E2UBsYeOjKwzZVsSBcn3hC2UyOimn5nkdyLqn08uu8l1eZBJWingstpU+YyRTwc0t" +
					" VDM7mK+GnS861EiN55nBYxXM2+XH4xqtgaA+0Wpum2J04BaNtg2vgs03PIK5Gw+bmUfM  ";

	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;

	@Override
	public void runOpMode() {

		/** Informa que está inicializando */
		telemetry.addData(">", "Inicializando");
		telemetry.update();

		/** Iteração com o Hub */
		// HUB 1
		frontLeftMotor = hardwareMap.dcMotor.get("front_left");
		backLeftMotor = hardwareMap.dcMotor.get("back_left");
		frontRightMotor = hardwareMap.dcMotor.get("front_right");
		backRightMotor = hardwareMap.dcMotor.get("back_right");

		// HUB 2
		Braco = hardwareMap.get(DcMotorEx.class, "braco");
		Lancador = hardwareMap.get(DcMotor.class, "lancador");
		Garra = hardwareMap.get(Servo.class, "garra");
		Dedo = hardwareMap.get(Servo.class, "dedo");
		//Coletor = hardwareMap.get(DcMotor.class, "coletor");

		/** Define a direção dos motores */
		frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
		backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
		frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
		backRightMotor.setDirection(DcMotor.Direction.FORWARD);
		Braco.setDirection(DcMotor.Direction.FORWARD);
		Lancador.setDirection(DcMotor.Direction.FORWARD);
		//Coletor.setDirection(DcMotorSimple.Direction.FORWARD);

		/** Define como os motores atuarão quando a potência for zero */
		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		Braco.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		Lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//Coletor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		/** Reseta os Encoders */
		resetEncoder();

		/** Define os parametros e inicializa o IMU */
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.loggingEnabled = false;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json";
		imu = hardwareMap.get(BNO055IMU.class, "imu");

		imu.initialize(parameters);

		/** Inicializa o Vuforia */
		initVuforia();
		initTfod();

		if (tfod != null) {
			tfod.activate();

			// Zoom e Resolução
			tfod.setZoom(1.5, 16.0 / 9.0);
		}

		/** Aguarda o piloto pressionar START */
		telemetry.addData(">", "Aguardando START");
		telemetry.update();
		waitForStart();
		
		telemetry.addData("Status", "Rodando");
		/** Define variáveis e constantes */
		
		
		
		/** Funções a executar abaixo */
		if (opModeIsActive()) {
			while (opModeIsActive()) {
				if (tfod != null) {
					List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
					if (updatedRecognitions != null) {
						telemetry.addData("# Object Detected", updatedRecognitions.size());
						if (updatedRecognitions.size() == 0 ) {
							// empty list.  no objects recognized.
							telemetry.addData("TFOD", "No items detected.");
							telemetry.addData("Target Zone", "A");
							zonaA();
							tfod.shutdown();
						} else {
						  // list is not empty.
						  // step through the list of recognitions and display boundary info.
						  int i = 0;
						  for (Recognition recognition : updatedRecognitions) {
							  telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
							  telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
									  recognition.getLeft(), recognition.getTop());
							  telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
									  recognition.getRight(), recognition.getBottom());

							  // check label to see which target zone to go after.
							  if (recognition.getLabel().equals("Single")) {
								  telemetry.addData("Target Zone", "B");
								  zonaB();
								  tfod.shutdown();
							  } else if (recognition.getLabel().equals("Quad")) {
								  telemetry.addData("Target Zone", "C");
								  zonaC();
								  tfod.shutdown();
							  } else {
								  telemetry.addData("Target Zone", "UNKNOWN");
							  }
							}
						}
						telemetry.update();
					}
				telemetry.addData("Angulo", getAngle());
				telemetry.addData("Target", Braco.getCurrentPosition());
				telemetry.addData("Velocity", BracoPower);
				telemetry.addData("Position", position);
				telemetry.update();
				}

			}
		}

		if (tfod != null) {
			tfod.shutdown();
		}
	}
	/**
	 * Initialize the Vuforia localization engine.
	 */
	private void initVuforia() {
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraDirection = CameraDirection.BACK;

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the TensorFlow Object Detection engine.
	}

	/**
	 * Initialize the TensorFlow Object Detection engine.
	 */
	private void initTfod() {
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minResultConfidence = 0.8f;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
	}

	/**
	 * Reseta o ângulo do imu.
	 */
	private void resetAngle()
	{
		lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		globalAngle = 0;
	}

	/**
	 * Recebe o ângulo.
	 */
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
	 * Rotaciona o robô em graus.
	 */
	private void rotate(int degrees, double power)
	{
		double  leftPower, rightPower;
		degrees -= 15; //erro de 15º

		// reseta o ângulo do imu
		resetAngle();

		int i = 0;
		do {
			if (degrees < 0) {   // gira para direita
				leftPower = power;
				rightPower = -power;
			} else if (degrees > 0) {   // gira para esquerda
				leftPower = -power;
				rightPower = power;
			} else return;

			// define a potência para o giro
			frontLeftMotor.setPower(leftPower);
			frontRightMotor.setPower(rightPower);
			backLeftMotor.setPower(leftPower);
			backRightMotor.setPower(rightPower);
			i++;

			while(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()) {
				telemetry.addData("Situação", "Girando");
				telemetry.addData("Posição/Rotação", frontRightMotor.getCurrentPosition());
				telemetry.addData("Angulo", getAngle());
				telemetry.update();
			}

		}while(i != 2);

		// rotate until turn is completed.
		if (degrees < 0)
		{
			// On right turn we have to get off zero first.
			while (opModeIsActive() && getAngle() == 0) {}

			while (opModeIsActive() && getAngle() > degrees) {}
		}
		else	// left turn.
			while (opModeIsActive() && getAngle() < degrees) {}

		// desliga os motores
		frontLeftMotor.setPower(0);
		frontRightMotor.setPower(0);
		backLeftMotor.setPower(0);
		backRightMotor.setPower(0);

		// aguarda a rotação para parar
		sleep(500);

		// reseta o ângulo do imu
		resetAngle();
	}

	/**
	 * Função para andar para frente com distância em cm.
	 */
	private void toFrente(int distancia, double power)
	{
		double CIRCUNFERENCIA = Math.PI * DIAMETRO_IN;
		double DISTANCIA_IN = distancia / POLEGADA;
		double ROTACOES = (DISTANCIA_IN / CIRCUNFERENCIA ) / ENGRENAGEM;
		int targetEncoder = (int)(ROTACOES*TICKS_REV);

		resetEncoder();

		frontRightMotor.setTargetPosition(targetEncoder);
		backRightMotor.setTargetPosition(targetEncoder);
		frontLeftMotor.setTargetPosition(targetEncoder);
		backLeftMotor.setTargetPosition(targetEncoder);

		frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		frontRightMotor.setPower(power);
		backRightMotor.setPower(power);
		frontLeftMotor.setPower(power);
		backLeftMotor.setPower(power);

		while(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy()) {
			telemetry.addData("Situação", "Andando");
			telemetry.addData("Posição/Rotação", frontRightMotor.getCurrentPosition());
			telemetry.addData("Angulo", getAngle());
			telemetry.update();
		}

		frontRightMotor.setPower(0);
		backRightMotor.setPower(0);
		frontLeftMotor.setPower(0);
		backLeftMotor.setPower(0);
	}

	/**
	 * Função de resetar os Encoders.
	 */
	private void resetEncoder()
	{
		frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		Braco.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		Lancador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		//Coletor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		Braco.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		Lancador.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		//Coletor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

	}

	private void OpenGarra()
	{
		Garra.setPosition(1);
	}
	private void CloseGarra()
	{
		Garra.setPosition(.5);
	}
	private void LancadorOn()
	{
		Lancador.setPower(1);
	}
	private void LancadorOff()
	{
		Lancador.setPower(0);
	}
	private void LevantaBraco()
	{
		CloseGarra();
		sleep(500);
		position = 200;
		Braco.setTargetPosition(position);
		BracoPower += (Braco.getCurrentPosition() - position) * kP;
		Braco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		Braco.setVelocity(BracoPower);
	}
	private void AbaixaBraco()
	{
		position = 0;
		Braco.setTargetPosition(position);
		BracoPower += (Braco.getCurrentPosition() - position) * kP;
		Braco.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		Braco.setVelocity(BracoPower);
		sleep(500);
		OpenGarra();
	}
	private void zonaC(){
		telemetry.addData("Msg", "To indo pra zona C");
		toFrente(180, .5);
		rotate(-45,.5);
		toFrente(30, .5);
		AbaixaBraco();
		OpenGarra();
		sleep(1000);
	}
	private void zonaB(){
		telemetry.addData("Msg", "To indo pra zona B");
		toFrente(150, .5);
		rotate(-45,.5);
		AbaixaBraco();
		OpenGarra();
		sleep(1000);
	}
	private void zonaA(){
		telemetry.addData("Msg", "To indo pra zona A");
		toFrente(90, .5);
		rotate(-45,.5);
		toFrente(30, .5);
		AbaixaBraco();
		OpenGarra();
		sleep(1000);
	}
}
