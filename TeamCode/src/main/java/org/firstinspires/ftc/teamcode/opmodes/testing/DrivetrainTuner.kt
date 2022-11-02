package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

class DrivetrainTuner: LinearOpMode() {

  override fun runOpMode(){

    val fl = hardwareMap.dcMotor.get("fl")
    val fr = hardwareMap.dcMotor.get("fr")
    val bl = hardwareMap.dcMotor.get("bl")
    val br = hardwareMap.dcMotor.get("br")

    waitForStart()

    while(opModeIsActive()){
      telemetry.addData("fl", fl.currentPosition)
      telemetry.addData("fr", fr.currentPosition)
      telemetry.addData("bl", bl.currentPosition)
      telemetry.addData("br", br.currentPosition)
      telemetry.update()
    }
  }

}
