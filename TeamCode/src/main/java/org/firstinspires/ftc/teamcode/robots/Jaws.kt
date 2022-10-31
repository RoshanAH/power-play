package org.firstinspires.ftc.teamcode.robots

import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import com.qualcomm.robotcore.hardware.HardwareMap


class Jaws : Robot() {

  lateinit var drivetrain: Mecanum

  override fun mapHardware(map: HardwareMap) {
    drivetrain = Mecanum(map, "fl", "fr", "bl", "br")

    addComponents(drivetrain)
  }

}
