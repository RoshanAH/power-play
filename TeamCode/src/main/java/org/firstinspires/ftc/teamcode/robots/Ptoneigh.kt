package org.firstinspires.ftc.teamcode.robots


import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor

import org.firstinspires.ftc.teamcode.components.*
import org.firstinspires.ftc.teamcode.core.Robot

class Ptoneigh(val active : () -> Boolean) : Robot(){

    lateinit var drivetrain : Mecanum;
    lateinit var turret : Turret;
    lateinit var claw : Claw;
    lateinit var camera : Webcam;
    lateinit var imu : IMU;

    private lateinit var voltageSensor: HardwareMap.DeviceMapping<VoltageSensor>
    override fun mapHardware(map: HardwareMap) {
        TODO("Not yet implemented")
    }
}