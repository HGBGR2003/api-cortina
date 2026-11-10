package trabalho.cortina.automatizada.cortina.domain;

import java.time.Instant;

public record TelemetrySnapshot(
    HardwareState currentState,
    int luminosity,
    boolean limitOpen,
    boolean limitClose,
    boolean online,
    Instant lastSync
) {
    public static TelemetrySnapshot offline() {
        return new TelemetrySnapshot(HardwareState.UNKNOWN, 0, false, false, false, null);
    }
}
