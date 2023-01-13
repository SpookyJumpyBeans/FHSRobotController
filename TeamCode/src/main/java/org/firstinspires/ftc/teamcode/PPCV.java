package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class PPCV {
    private int[] viabilityBounds = {700/*bottom*/, 225/*top*/, 200/*left*/, 700/*right*/};
    private static String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private boolean isStopped = true;
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static String[] LABELS = {
            /*Number of dots*/
            "ONE",
            "THREE",
            "TWO"
    };
    private static final String VUFORIA_KEY =
            "AVVrDYb/////AAABmT0TlZXDYE3gpf/zMjQrOgACsYT0LcTPCkhjAmq0XO3HT0RdGx2eP+Lwumhftz4e/g28CBGg1HmaFfy5kW9ioO4UGDeokDyxRfqWjNQwKG3BanmjCXxMxACaJ7iom5J3o4ylWNmuiyxsK8n1fFf2dVsTUsvUI7aRxqTahnIqqRJRsGmxld18eHy/ZhHfIjOyifi4svZUQiput21/jAloTx0sTnnrpR1Y/xGOz+68sGuXIgLZHpAQSoZnXiczGKdahGXOg3n6dXlQPIiASE1kHp253CTwO40l1HHN083m4wYjP4FCl/9TH3tb0Wj/Ccmlhfz2omhnZQKOBe7RsIxRk+PuEGkIe5hCs/lV9+yf9iBm";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public PPCV(){

    }

    public Recognition detect(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            int closestIndex = 0;
            int highestConfidenceIndex = 0;
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions == null || updatedRecognitions.size() == 0){
                return null;
            }
            else {
//                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                DbgLog.msg("Objects Detected", updatedRecognitions.size());
                double dist = Integer.MAX_VALUE;
                float confidence = 0;
                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    if(getDist( (viabilityBounds[3] + viabilityBounds[2])/2 , (viabilityBounds[1] + viabilityBounds[0])/2, updatedRecognitions.get(i)) < dist){
                        dist = getDist( (viabilityBounds[3] + viabilityBounds[2])/2 , (viabilityBounds[1] + viabilityBounds[0])/2, updatedRecognitions.get(i));
                        closestIndex = i;
                    }
                    if (updatedRecognitions.get(i).getConfidence() * 100 > confidence) {
                        confidence = updatedRecognitions.get(i).getConfidence();
                        highestConfidenceIndex = i;
                    }
                }
            }
            Recognition decision = choose(updatedRecognitions.get(closestIndex), updatedRecognitions.get(highestConfidenceIndex));
//            telemetry.addData("Closest Box Label: ", decision.getLabel());
//            telemetry.addData("Closest Box Confidence: ", 100 * decision.getConfidence());
//            telemetry.update();
            return decision;
        }
        return null;
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    public Recognition choose(Recognition dist, Recognition confidence){

        if(dist.getLabel().equals(confidence.getLabel())){
            return dist;
        }
        else if(viable(dist) && !viable(confidence)){
            return dist;
        }
        else if(!viable(dist) && viable(confidence)){
            return confidence;
        }
        else if(getDist( (viabilityBounds[2] + viabilityBounds[3])/2 , (viabilityBounds[0] + viabilityBounds[1])/2, dist) < getDist((viabilityBounds[2] + viabilityBounds[3])/2 , (viabilityBounds[0] + viabilityBounds[1])/2, confidence)){
            DbgLog.msg("Recognition Chosen: " + "Low Dist" + dist.getLabel());
            return dist;
        }
        else if(getDist( (viabilityBounds[2] + viabilityBounds[3])/2 , (viabilityBounds[0] + viabilityBounds[1])/2, dist) > getDist((viabilityBounds[2] + viabilityBounds[3])/2 , (viabilityBounds[0] + viabilityBounds[1])/2, confidence)){
            DbgLog.msg("Recognition Chosen: " + "Low Dist" + confidence.getLabel());
            return confidence;
        }
        else if(confidence.getConfidence() * 100 > dist.getConfidence() * 100){
            DbgLog.msg("Recognition Chosen: " + "High Confidence" + confidence.getLabel());
            return confidence;
        }
        else if(confidence.getConfidence() * 100 < dist.getConfidence() * 100){
            DbgLog.msg("Recognition Chosen: " + "High Confidence" + dist.getLabel());
            return dist;
        }
        else {
            //Nothing Satisfied, Just Choose One
            return dist;
        }
    }
    private boolean viable(Recognition i){
        return i.getBottom() < viabilityBounds[0] && i.getTop() > viabilityBounds[1] && i.getLeft() > viabilityBounds[2] && i.getRight() < viabilityBounds[3];
    }

    private double getDist(float x, float y, Recognition i){
        return Math.sqrt(Math.pow(x - (i.getRight() + i.getLeft())/2, 2) + Math.pow(y - (i.getTop() + i.getBottom())/2, 2));
    }

    public void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.00f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            isStopped = false;
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }
    public void setModel(String model){
        if(model.equals("XV")){
            TFOD_MODEL_ASSET = "Sleeve_Recognition_V7_Pole_Gear_Battery.tflite";
        }
        else{
            TFOD_MODEL_ASSET = "PowerPlay.tflite";
            LABELS = new String[]{
                    "ONE",
                    "TWO",
                    "THREE"
            };
        }
    }
    public void stop(){
        tfod.deactivate();
        isStopped = true;
    }
    public boolean isStopped(){
        return isStopped;
    }
}
