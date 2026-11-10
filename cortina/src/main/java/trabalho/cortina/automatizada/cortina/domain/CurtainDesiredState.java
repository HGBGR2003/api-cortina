package trabalho.cortina.automatizada.cortina.domain;

public record CurtainDesiredState(
    CurtainMode mode,
    CurtainAction targetAction,
    AutoRule autoRule,
    int lightThreshold
) {
    public static CurtainDesiredState defaultState() {
        return new CurtainDesiredState(CurtainMode.AUTO, CurtainAction.STOP, AutoRule.STANDARD, 700);
    }

    public CurtainDesiredState withMode(CurtainMode newMode) {
        return new CurtainDesiredState(newMode, targetAction, autoRule, lightThreshold);
    }

    public CurtainDesiredState withAction(CurtainAction newAction) {
        return new CurtainDesiredState(CurtainMode.MANUAL, newAction, autoRule, lightThreshold);
    }

    public CurtainDesiredState withAutoConfig(CurtainMode newMode, AutoRule newRule, int newThreshold) {
        return new CurtainDesiredState(newMode, targetAction, newRule, newThreshold);
    }
}
