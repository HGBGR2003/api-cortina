package trabalho.cortina.automatizada.cortina.dto;

import trabalho.cortina.automatizada.cortina.domain.*;
import java.time.Instant;

public record StatusResponse(
    // Desired state
    String mode,
    String targetAction,
    String autoRule,
    int lightThreshold,
    // Telemetry
    String currentState,
    int luminosity,
    boolean limitOpen,
    boolean limitClose,
    boolean online,
    Instant lastSync
) {}
