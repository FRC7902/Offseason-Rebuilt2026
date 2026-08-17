package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.belt.IndexerBeltSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.roller.RollerFloorSubsystem;

public class IndexerSystem extends SubsystemBase {

    private final IndexerBeltSubsystem m_indexerBelt;
    private final FeederSubsystem m_feeder;
    private final RollerFloorSubsystem m_rollerFloor;

    public IndexerSystem() {
        m_indexerBelt = new IndexerBeltSubsystem();
        m_feeder = new FeederSubsystem();
        m_rollerFloor = new RollerFloorSubsystem();
    }
}
