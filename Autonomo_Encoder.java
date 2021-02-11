/**
 *
 * Programação desenvolvida pela equipe TchêStorm #16062
 *				  @tchestorm16062
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Autonomo", group = "Autonomous")
public class Encoder extends LinearOpMode
{
	DcMotor			 frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
	BNO055IMU		 imu;
	Orientation		 lastAngles = new Orientation();
	double		   globalAngle = .30;

	double			  DIAMETRO_IN = 3.966;
	double			  DIAMETRO_CM = 10.16;
	double			  POLEGADA = 2.54;
	double			  ENGRENAGEM = 2.6;
	int				  TICKS_REV = 1120;


	public void runOpMode() throws InterruptedException
	{
		telemetry.addData("Status", "Inicializando");
		telemetry.update();

		frontLeftMotor = hardwareMap.dcMotor.get("front_left");
		backLeftMotor = hardwareMap.dcMotor.get("back_left");
		frontRightMotor = hardwareMap.dcMotor.get("front_right");
		backRightMotor = hardwareMap.dcMotor.get("back_right");

		frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
		backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

		parameters.angleUnit			= BNO055IMU.AngleUnit.DEGREES;
		parameters.loggingEnabled		= false;
		parameters.calibrationDataFile  = "BNO055IMUCalibration.json";

		imu = hardwareMap.get(BNO055IMU.class, "imu");

		imu.initialize(parameters);

		telemetry.addData("Status", "Aguardando START");
		telemetry.update();
		waitForStart();

		telemetry.addData("Status", "Rodando");


	   /*
		   Funções a executar abaixo
		*/
		while(opModeIsActive())
		{
			telemetry.addData("Angulo", getAngle());
			telemetry.update();
		}

	}

	private void resetAngle()
	{
		lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		globalAngle = 0;
	}

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

	private void toFrente(int distancia, double power)
	{
		double CIRCUNFERENCIA = Math.PI * DIAMETRO_CM;
		double ROTACOES = (distancia / CIRCUNFERENCIA ) / ENGRENAGEM;
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

	private void resetEncoder()
	{
		frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}
}
