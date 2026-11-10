// ---- State ----
let authToken = localStorage.getItem('cortina_token') || '';
let currentAutoRule = 'STANDARD';
let isAutoMode = false;
let pollingInterval = null;

// ---- Init ----
document.addEventListener('DOMContentLoaded', () => {
  if (!authToken) showTokenModal();
  startPolling();
});

// ---- Token management ----
function showTokenModal() {
  const modal = document.getElementById('tokenModal');
  modal.style.display = 'flex';
  const inp = document.getElementById('tokenInput');
  inp.value = authToken;
}

function saveToken() {
  const inp = document.getElementById('tokenInput');
  authToken = inp.value.trim();
  localStorage.setItem('cortina_token', authToken);
  document.getElementById('tokenModal').style.display = 'none';
  fetchStatus();
}

// ---- API helpers ----
async function apiFetch(path, options = {}) {
  const headers = {
    'Content-Type': 'application/json',
    'X-Auth-Token': authToken,
    ...(options.headers || {})
  };
  const res = await fetch(path, { ...options, headers });
  return res;
}

// ---- Polling ----
function startPolling() {
  fetchStatus();
  if (pollingInterval) clearInterval(pollingInterval);
  pollingInterval = setInterval(fetchStatus, 2000);
}

async function fetchStatus() {
  try {
    const res = await apiFetch('/api/v1/curtain/status');
    if (!res.ok) {
      setOffline();
      return;
    }
    const data = await res.json();
    updateUI(data);
  } catch (e) {
    setOffline();
  }
}

// ---- UI Update ----
function updateUI(data) {
  // Connection status
  const dot = document.getElementById('statusDot');
  const txt = document.getElementById('statusText');
  if (data.online) {
    dot.className = 'status-dot online';
    txt.textContent = 'Arduino conectado';
  } else {
    dot.className = 'status-dot offline';
    txt.textContent = 'Arduino desconectado';
  }

  // Curtain state
  const stateEl = document.getElementById('curtainStateText');
  const visual   = document.getElementById('curtainVisual');
  const stateMap = {
    OPENED:  { label: 'aberta',   cls: 'curtain-open'    },
    OPENING: { label: 'abrindo',  cls: 'curtain-opening' },
    CLOSED:  { label: 'fechada',  cls: 'curtain-closed'  },
    CLOSING: { label: 'fechando', cls: 'curtain-closing' },
    STOPPED: { label: 'parada',   cls: ''                },
    UNKNOWN: { label: '...',      cls: ''                },
  };
  const s = stateMap[data.currentState] || { label: data.currentState, cls: '' };
  stateEl.textContent = s.label;
  visual.className = 'curtain-visual ' + s.cls;

  // Mode toggle
  const toggle = document.getElementById('autoModeToggle');
  isAutoMode = data.mode === 'AUTO';
  toggle.checked = isAutoMode;

  // Auto rule
  currentAutoRule = data.autoRule;
  updateRuleUI(currentAutoRule);

  // LDR
  const ldrPct = Math.round((data.luminosity / 1023) * 100);
  document.getElementById('ldrBar').style.width   = ldrPct + '%';
  document.getElementById('ldrValue').textContent = data.luminosity;

  // Limit switches
  setBadge('limitOpenBadge',  data.limitOpen,  'Ativo', 'Inativo');
  setBadge('limitCloseBadge', data.limitClose, 'Ativo', 'Inativo');

  // Last sync
  if (data.lastSync) {
    const d = new Date(data.lastSync);
    document.getElementById('lastSyncText').textContent = d.toLocaleTimeString('pt-BR');
  } else {
    document.getElementById('lastSyncText').textContent = 'nunca';
  }
}

function setOffline() {
  document.getElementById('statusDot').className  = 'status-dot offline';
  document.getElementById('statusText').textContent = 'Arduino desconectado';
}

function setBadge(id, active, activeLabel, inactiveLabel) {
  const el = document.getElementById(id);
  el.textContent  = active ? activeLabel : inactiveLabel;
  el.className    = 'sensor-badge ' + (active ? 'active' : 'inactive');
}

function updateRuleUI(rule) {
  const stdCard = document.getElementById('ruleStandard');
  const invCard = document.getElementById('ruleInverted');
  const stdRadio = document.getElementById('radioStandard');
  const invRadio = document.getElementById('radioInverted');

  if (rule === 'STANDARD') {
    stdCard.classList.add('selected');
    invCard.classList.remove('selected');
    stdRadio.textContent = '🔵';
    invRadio.textContent = '⚪';
  } else {
    invCard.classList.add('selected');
    stdCard.classList.remove('selected');
    invRadio.textContent = '🔵';
    stdRadio.textContent = '⚪';
  }
}

// ---- Commands ----
async function sendCommand(action) {
  try {
    const res = await apiFetch('/api/v1/curtain/command', {
      method: 'POST',
      body: JSON.stringify({ action })
    });
    if (!res.ok) console.error('Command failed', await res.text());
    else fetchStatus();
  } catch (e) {
    console.error('Command error', e);
  }
}

async function onModeToggle() {
  const toggle = document.getElementById('autoModeToggle');
  const mode = toggle.checked ? 'AUTO' : 'MANUAL';
  await postMode(mode, currentAutoRule);
}

async function selectRule(rule) {
  currentAutoRule = rule;
  updateRuleUI(rule);
  await postMode(isAutoMode ? 'AUTO' : 'MANUAL', rule);
}

async function postMode(mode, rule) {
  try {
    const res = await apiFetch('/api/v1/curtain/mode', {
      method: 'POST',
      body: JSON.stringify({
        mode,
        autoRule: rule,
        lightThreshold: 700
      })
    });
    if (!res.ok) console.error('Mode failed', await res.text());
    else fetchStatus();
  } catch (e) {
    console.error('Mode error', e);
  }
}
