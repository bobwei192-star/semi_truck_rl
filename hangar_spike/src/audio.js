/**
 * 告警音效引擎：音源全部为实时合成 + 浏览器 TTS，不含任何外部音频素材。
 * - 滴滴声：Web Audio 方波 + 包络，节奏随接近入库位加快（雷达告警式）
 * - 女声播报：speechSynthesis（zh-CN），台词为本项目自写短句
 */

const ALERT_G = 0.22;        // 铰接角告警阈值（rad，约 12.6°；fakeSim 峰值 16°，取中段可触发）
const BEEP_FROM = 0.55;      // 进度 ≥ 55% 开始滴滴
const BEEP_SLOW = 0.75;      // 起始间隔（s）
const BEEP_FAST = 0.16;      // 最急间隔（s）
const FINAL_WARN = 0.93;     // 「拉起 拉起」触发进度
const SAY_COOLDOWN_MS = 5000;

export function createAudioEngine() {
  let ctx = null;
  let muted = false;
  let nextBeepAt = 0;
  let prevPhase = 'idle';
  const fired = { final: false, done: false };
  const lastSaid = Object.create(null);

  function unlock() {
    if (!ctx) {
      const Ctor = window.AudioContext || window.webkitAudioContext;
      if (Ctor) ctx = new Ctor();
    }
    if (ctx && ctx.state === 'suspended') ctx.resume();
  }

  function tone(freq, dur, gain = 0.12, type = 'square') {
    if (muted || !ctx) return;
    const t0 = ctx.currentTime;
    const osc = ctx.createOscillator();
    const amp = ctx.createGain();
    osc.type = type;
    osc.frequency.setValueAtTime(freq, t0);
    amp.gain.setValueAtTime(0, t0);
    amp.gain.linearRampToValueAtTime(gain, t0 + 0.008);
    amp.gain.exponentialRampToValueAtTime(0.0001, t0 + dur);
    osc.connect(amp).connect(ctx.destination);
    osc.start(t0);
    osc.stop(t0 + dur + 0.02);
  }

  function pickZhVoice() {
    const voices = window.speechSynthesis?.getVoices?.() || [];
    return voices.find((v) => /zh[-_]CN/i.test(v.lang) && /xiaoxiao|xiaoyi|huihui|yaoyao|tingting|female/i.test(v.name))
      || voices.find((v) => /zh/i.test(v.lang))
      || null;
  }

  function say(text, { rate = 1.08, pitch = 0.8 } = {}) {
    if (muted || !('speechSynthesis' in window)) return;
    const now = performance.now();
    if (lastSaid[text] && now - lastSaid[text] < SAY_COOLDOWN_MS) return;
    lastSaid[text] = now;
    try {
      const u = new SpeechSynthesisUtterance(text);
      u.lang = 'zh-CN';
      u.rate = rate;
      u.pitch = pitch;
      const v = pickZhVoice();
      if (v) u.voice = v;
      window.speechSynthesis.speak(u);
    } catch {
      /* 无语音引擎时静默降级 */
    }
  }

  function update(state, phase, now) {
    if (phase === 'running' && prevPhase !== 'running') {
      fired.final = false;
      fired.done = false;
      nextBeepAt = now;
      say('开始倒车');
      tone(660, 0.08);
    }
    if (phase === 'done' && prevPhase === 'running' && !fired.done) {
      fired.done = true;
      say('入库完成');
      tone(880, 0.1);
      tone(1100, 0.1);
      setTimeout(() => tone(1320, 0.16), 140);
    }
    prevPhase = phase;
    if (phase !== 'running') return;

    const p = Math.min(Math.max(state.progress || 0, 0), 1);
    if (p >= BEEP_FROM && now >= nextBeepAt) {
      const k = (p - BEEP_FROM) / (1 - BEEP_FROM);
      tone(920, 0.06, 0.09);
      nextBeepAt = now + (BEEP_SLOW + (BEEP_FAST - BEEP_SLOW) * k) * 1000;
    }

    if (Math.abs(state.g || 0) > ALERT_G) {
      say('最大迎角');
      tone(1240, 0.14, 0.1);
    }

    if (p >= FINAL_WARN && !fired.final) {
      fired.final = true;
      say('拉起 拉起');
    }
  }

  function setMuted(v) {
    muted = Boolean(v);
    if (muted) window.speechSynthesis?.cancel?.();
    return muted;
  }

  return {
    unlock,
    update,
    say,
    tone,
    setMuted,
    toggleMute: () => setMuted(!muted),
    isMuted: () => muted,
  };
}
