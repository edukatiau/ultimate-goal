package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

@TeleOp(name = "TF_example")
public class TF_example extends LinearOpMode {

  private VuforiaCurrentGame vuforiaUltimateGoal;
  private TfodCurrentGame tfodUltimateGoal;

  Recognition recognition;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    List<Recognition> recognitions;
    double index;

    vuforiaUltimateGoal = new VuforiaCurrentGame();
    tfodUltimateGoal = new TfodCurrentGame();

    // Sample TFOD Op Mode
    // Initialize Vuforia.
    // This sample assumes phone is in landscape mode.
    // Rotate phone -90 so back camera faces "forward" direction on robot.
    // We need Vuforia to provide TFOD with camera images.
    vuforiaUltimateGoal.initialize(
        "", // vuforiaLicenseKey
        VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
        false, // useExtendedTracking
        false, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        0, // xAngle
        -90, // yAngle
        0, // zAngle
        true); // useCompetitionFieldTargetLocations
    // Set min confidence threshold to 0.7
    tfodUltimateGoal.initialize(vuforiaUltimateGoal, 0.7F, true, true);
    // Initialize TFOD before waitForStart.
    // Init TFOD here so the object detection labels are visible
    // in the Camera Stream preview window on the Driver Station.
    tfodUltimateGoal.activate();
    // Enable following block to zoom in on target.
    tfodUltimateGoal.setZoom(2.5, 16 / 9);
    telemetry.addData(">", "Press Play to start");
    telemetry.update();
    // Wait for start command from Driver Station.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // Get a list of recognitions from TFOD.
        recognitions = tfodUltimateGoal.getRecognitions();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (recognitions.size() == 0) {
          telemetry.addData("TFOD", "No items detected.");
          telemetry.addData("Zona Alvo", "A");
        } else {
          index = 0;
          // Iterate through list and call a function to
          // display info for each recognized object.
          for (Recognition recognition_item : recognitions) {
            recognition = recognition_item;
            // Display info.
            displayInfo(index);
            // Increment index.
            index = index + 1;
          }
          if (recognition.getLabel() == "Single"){
            telemetry.addData("Zona Alvo", "B");
          }
          else if (recognition.getLabel() == "Quad"){
            telemetry.addData("Zona Alvo", "C");
          }
        }
        telemetry.update();
      }
    }
    // Deactivate TFOD.
    tfodUltimateGoal.deactivate();

    vuforiaUltimateGoal.close();
    tfodUltimateGoal.close();
  }

  /**
   * Display info (using telemetry) for a recognized object.
   */
  private void displayInfo(double i) {
    // Display label info.
    // Display the label and index number for the recognition.
    telemetry.addData("label " + i, recognition.getLabel());
    // Display upper corner info.
    // Display the location of the top left corner
    // of the detection boundary for the recognition
    telemetry.addData("Left, Top " + i, recognition.getLeft() + ", " + recognition.getTop());
    // Display lower corner info.
    // Display the location of the bottom right corner
    // of the detection boundary for the recognition
    telemetry.addData("Right, Bottom " + i, recognition.getRight() + ", " + recognition.getBottom());
  }
}
