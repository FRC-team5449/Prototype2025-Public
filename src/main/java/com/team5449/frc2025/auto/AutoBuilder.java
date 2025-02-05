package com.team5449.frc2025.auto;

import com.team5449.frc2025.subsystems.arm.ArmSubsystem;
import com.team5449.frc2025.subsystems.drive.Drive;
import com.team5449.frc2025.subsystems.elevator.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoBuilder {
    private final Drive drive;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;

    public Command dummyFourLV3() {
        return Commands.none();
    }
}
