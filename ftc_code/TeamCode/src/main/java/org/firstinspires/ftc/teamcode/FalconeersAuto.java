package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by the Falconeers 10820
 */

@Autonomous(name="Final Auto", group="Period8")
public class FalconeersAuto extends LinearOpMode {
    private BasicHardware hw = new BasicHardware();

    private VuforiaLocalizer vuforia;
    private int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
    @Override
    public void runOpMode() throws InterruptedException {
        hw.initHardware(hardwareMap, telemetry);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWxGCVn/////AAAAGRMSxZqLZkJMrcxIzIw5leFjXs7UwzvXgUH+EdiuMWdDVuEC+AlMJ3TtsY4zhrZ1ZmuunWGkF+lrgZwG6pMwqp+i5mvskhbifpUR+nhQaAqwNHMOxx1tBS0yRVtcGLnZLOV/UOKMDC5Id+olaTqxyEKtXvq3POuxEJQ349eJ4EZxSU0PgCPeCJS0mASNTKUmmlnckjToG4bOJ7Zo5otKPwlf9R2jHEhAdZ0/v93utHct1MjbF/EdNJ7z1EQ30xIgz4cpzxBP/cB0rcJlFKn4EckCXshYQrB1FHQVsFQam4dnlrkULUxABDRLgiCZB7J9OMXSCor+9WyHe6kIYA0BsKzjgSOJpdaGmgu3EeMG179h";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();
        hw.drive(1,1);

        while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        telemetry.addData("Pictograph", "%s", vuMark);
        hw.drive(0,0);
    }

}


