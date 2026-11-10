package trabalho.cortina.automatizada.cortina.controller;

import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import trabalho.cortina.automatizada.cortina.domain.*;
import trabalho.cortina.automatizada.cortina.dto.*;
import trabalho.cortina.automatizada.cortina.service.CurtainStateService;

import java.util.Map;

@RestController
@RequestMapping("/api/v1/curtain")
public class CurtainController {

    private final CurtainStateService service;

    public CurtainController(CurtainStateService service) {
        this.service = service;
    }

    @GetMapping("/status")
    public ResponseEntity<StatusResponse> getStatus() {
        CurtainDesiredState d = service.getDesired();
        TelemetrySnapshot t = service.getTelemetry();

        StatusResponse resp = new StatusResponse(
            d.mode().name(),
            d.targetAction().name(),
            d.autoRule().name(),
            d.lightThreshold(),
            t.currentState().name(),
            t.luminosity(),
            t.limitOpen(),
            t.limitClose(),
            t.online(),
            t.lastSync()
        );
        return ResponseEntity.ok(resp);
    }

    @PostMapping("/command")
    public ResponseEntity<Map<String, String>> postCommand(@RequestBody CommandRequest req) {
        CurtainAction action = switch (req.action().toUpperCase()) {
            case "OPEN"  -> CurtainAction.OPEN;
            case "CLOSE" -> CurtainAction.CLOSE;
            case "STOP"  -> CurtainAction.STOP;
            default -> throw new IllegalArgumentException("Unknown action: " + req.action());
        };
        service.applyCommand(action);
        return ResponseEntity.ok(Map.of("status", "ok", "action", action.name()));
    }

    @PostMapping("/mode")
    public ResponseEntity<Map<String, String>> postMode(@RequestBody ModeRequest req) {
        CurtainMode mode = CurtainMode.valueOf(req.mode().toUpperCase());
        AutoRule rule = req.autoRule() != null
            ? AutoRule.valueOf(req.autoRule().toUpperCase())
            : AutoRule.STANDARD;
        int threshold = req.lightThreshold() != null ? req.lightThreshold() : 700;

        service.applyModeConfig(mode, rule, threshold);
        return ResponseEntity.ok(Map.of("status", "ok", "mode", mode.name()));
    }

    @ExceptionHandler(IllegalArgumentException.class)
    public ResponseEntity<Map<String, String>> handleBadRequest(IllegalArgumentException ex) {
        return ResponseEntity.badRequest().body(Map.of("error", ex.getMessage()));
    }
}
