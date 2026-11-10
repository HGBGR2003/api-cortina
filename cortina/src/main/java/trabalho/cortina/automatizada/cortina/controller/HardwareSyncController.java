package trabalho.cortina.automatizada.cortina.controller;

import org.springframework.http.*;
import org.springframework.web.bind.annotation.*;
import trabalho.cortina.automatizada.cortina.domain.HardwareState;
import trabalho.cortina.automatizada.cortina.service.CurtainStateService;

@RestController
@RequestMapping("/api/v1/hw")
public class HardwareSyncController {

    private final CurtainStateService service;

    public HardwareSyncController(CurtainStateService service) {
        this.service = service;
    }

    /**
     * Ultra-compact sync endpoint for Arduino/ESP-01.
     * Query params: ldr=512&lo=0&lc=0&st=STOPPED
     * Response: CMD:M=A,R=S,T=S,TH=700
     */
    @GetMapping("/sync")
    public ResponseEntity<String> sync(
        @RequestParam(defaultValue = "0") int ldr,
        @RequestParam(defaultValue = "0") int lo,
        @RequestParam(defaultValue = "0") int lc,
        @RequestParam(defaultValue = "UNKNOWN") String st
    ) {
        HardwareState state;
        try {
            state = HardwareState.valueOf(st.toUpperCase());
        } catch (IllegalArgumentException e) {
            state = HardwareState.UNKNOWN;
        }

        service.updateTelemetry(ldr, lo == 1, lc == 1, state);
        String cmd = service.buildHwCommand();

        return ResponseEntity.ok()
            .contentType(MediaType.TEXT_PLAIN)
            .body(cmd);
    }
}
