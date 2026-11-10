package trabalho.cortina.automatizada.cortina.service;

import org.springframework.stereotype.Service;
import trabalho.cortina.automatizada.cortina.domain.*;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicReference;

@Service
public class CurtainStateService {

    private final AtomicReference<CurtainDesiredState> desired =
        new AtomicReference<>(CurtainDesiredState.defaultState());

    private final AtomicReference<TelemetrySnapshot> telemetry =
        new AtomicReference<>(TelemetrySnapshot.offline());

    // --- Desired state mutators ---

    public void applyCommand(CurtainAction action) {
        desired.updateAndGet(s -> s.withAction(action));
    }

    public void applyModeConfig(CurtainMode mode, AutoRule rule, int threshold) {
        desired.updateAndGet(s -> s.withAutoConfig(mode, rule, threshold));
    }

    // --- Telemetry updater (called by HW sync) ---

    public void updateTelemetry(int ldr, boolean limitOpen, boolean limitClose, HardwareState state) {
        TelemetrySnapshot snap = new TelemetrySnapshot(state, ldr, limitOpen, limitClose, true, Instant.now());
        telemetry.set(snap);
    }

    // --- Readers ---

    public CurtainDesiredState getDesired() {
        return desired.get();
    }

    public TelemetrySnapshot getTelemetry() {
        return telemetry.get();
    }

    // Build the compact command string for the hardware
    public String buildHwCommand() {
        CurtainDesiredState d = desired.get();
        String m = d.mode() == CurtainMode.AUTO ? "A" : "M";
        String r = d.autoRule() == AutoRule.STANDARD ? "S" : "I";
        String t = switch (d.targetAction()) {
            case OPEN  -> "O";
            case CLOSE -> "C";
            case STOP  -> "S";
        };
        return "CMD:M=%s,R=%s,T=%s,TH=%d".formatted(m, r, t, d.lightThreshold());
    }
}
