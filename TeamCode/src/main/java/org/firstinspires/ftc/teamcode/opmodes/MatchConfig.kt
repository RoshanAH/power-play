package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ReadWriteFile
import kotlinx.coroutines.yield
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.robotcore.internal.system.AppUtil


@TeleOp

class MatchConfig : LinearOpMode(){

  override fun runOpMode() { 
    runBlocking {
      waitForStart() 

      val alliance = option("blue", "red")
      val side = option("left", "right")

      telemetry.addData("alliance", alliance)
      telemetry.addData("side", side)
      telemetry.addLine("Press 'A' to save or 'B' to cancel")
      telemetry.update()

      val allianceFile = AppUtil.getInstance().getSettingsFile("alliance.txt")
      val sideFile = AppUtil.getInstance().getSettingsFile("side.txt")

      while(opModeIsActive()){
        if(gamepad1.a){
          ReadWriteFile.writeFile(allianceFile, alliance);
          ReadWriteFile.writeFile(sideFile, side);
          telemetry.addLine("Settings saved")
          telemetry.update()
          break
        }
        if(gamepad1.b) {
          telemetry.addLine("Settings cancelled")
          telemetry.update()
          break
        }
      }

      while(opModeIsActive()) yield()
    }
  }

  suspend fun option (x: String, b: String): String? {

    telemetry.addLine("$x 'X' or $b 'B'")
    telemetry.update()
    var out: String? = null
    while(opModeIsActive()){
      if(gamepad1.x){
        out = x
        break
      }
      if(gamepad1.b){
        out = b
        break
      }
      yield()
    }
    while (opModeIsActive() && (gamepad1.x || gamepad1.b)) yield()
    return out
  }

}
