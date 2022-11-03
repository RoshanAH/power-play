package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.CoroutineScope
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.core.BaseOpmode
import org.firstinspires.ftc.teamcode.core.Robot

class DrivetrainTuner : BaseOpmode() {

  private lateinit var dt: Mecanum

  override fun setRobot() =
      object : Robot() {
        override fun mapHardware(map: HardwareMap) {
          addComponents(dt)
        }
      }

  override fun onStart(scope: CoroutineScope) {}

  override fun onUpdate(scope: CoroutineScope) {
    telemetry.addData("fl", dt.flp)
    telemetry.addData("fr", dt.frp)
    telemetry.addData("bl", dt.blp)
    telemetry.addData("br", dt.brp)
    telemetry.update()
  }
}
