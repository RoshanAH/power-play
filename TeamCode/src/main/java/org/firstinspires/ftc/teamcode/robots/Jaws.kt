package org.firstinspires.ftc.teamcode.robots

import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.components.Slides
import com.qualcomm.robotcore.hardware.HardwareMap


class Jaws : Robot() {

  lateinit var drivetrain: Mecanum
  lateinit var slides: Slides

  val low = 0.45
  val medium = 0.7
  val high = 1.0

  override fun mapHardware(map: HardwareMap) {
    drivetrain = Mecanum(map, "fl", "fr", "bl", "br").apply {
      maxAccel = 5.0
      ticksPerInch = 100.0
    }

    slides = Slides(map, "left", "right", "claw").apply {
      maxticks = 1125
      p = 10.0
      d = 0.02
      gravity = 0.1

      open = 0.5
      close = 0.8
    }

    addComponents(slides, drivetrain)
  }

}
