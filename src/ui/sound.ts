/**
 * Sound effects, all synthesised with the Web Audio API (nothing is loaded):
 *  - cues: short one-shot sounds for editing actions (placing parts, joining, deleting, …);
 *  - impacts: thuds when something lands on the ground, clacks when a slider hits a stop;
 *  - machine voices: continuous sounds that follow the running mechanism: motors whine with
 *    their speed and strain with their torque, propellers whoosh and chop at their blade-pass
 *    rate, gear teeth tick and whir at their tooth rate, wheels rumble and skid, joints scrape
 *    along the ground, fast parts swish through the air and hot friction sizzles.
 * Browsers only allow audio after a user gesture, so the engine starts on the first click or key.
 */
import type { SimEvent } from '../mechanics/simulate';

export type Cue =
  | 'select'
  | 'place'
  | 'bar'
  | 'rack'
  | 'spring'
  | 'gear'
  | 'wheel'
  | 'propeller'
  | 'mesh'
  | 'pin'
  | 'free'
  | 'join'
  | 'unjoin'
  | 'delete'
  | 'undo'
  | 'play'
  | 'pause'
  | 'reset'
  | 'motorOn'
  | 'motorOff'
  | 'lever'
  | 'pid'
  | 'sensor'
  | 'tick'
  | 'lock'
  | 'unlock'
  | 'mass'
  | 'copy'
  | 'paste'
  | 'preset'
  | 'grab'
  | 'error';

/** What the running mechanism is doing, for its continuous sounds. */
export interface MachineState {
  motors: { id: number; speed: number; torque: number; stall: number }[];
  /** spin φ̇ (rad/s), thrust (N), blade count */
  propellers: { id: number; spin: number; thrust: number; blades: number }[];
  /** the tooth-passing rate (Hz) and pitch-line speed (m/s) of the fastest mesh, or null */
  mesh: { rate: number; speed: number } | null;
  /** from the ground contacts (m/s) */
  rolling: number;
  slip: number;
  scrape: number;
  /** the fastest joint speed (m/s) */
  wind: number;
  /** friction power (W) */
  friction: number;
  events: SimEvent[];
}

interface Voice {
  out: GainNode;
  sources: AudioScheduledSourceNode[];
  set: (s: Record<string, number>, now: number) => void;
}

const clamp = (x: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, x));

export class SoundEngine {
  private ctx: AudioContext | null = null;
  private master: GainNode | null = null;
  private noise: AudioBuffer | null = null;
  private voices = new Map<string, Voice>();
  private lastImpact = 0;
  private lastCue = new Map<Cue, number>();
  enabled: boolean;
  volume: number;

  constructor(private readonly storageKey = 'linkage-lab-sound') {
    let saved: { enabled?: boolean; volume?: number } = {};
    try {
      saved = JSON.parse(localStorage.getItem(storageKey) ?? '{}');
    } catch {
      saved = {};
    }
    this.enabled = saved.enabled ?? true;
    this.volume = saved.volume ?? 0.6;
  }

  private save() {
    localStorage.setItem(this.storageKey, JSON.stringify({ enabled: this.enabled, volume: this.volume }));
  }

  /** Start (or resume) audio; call from a user gesture. */
  unlock() {
    if (!this.enabled) return;
    if (!this.ctx) {
      const AC = window.AudioContext ?? (window as unknown as { webkitAudioContext?: typeof AudioContext }).webkitAudioContext;
      if (!AC) return;
      const ctx = new AC();
      const comp = ctx.createDynamicsCompressor(); // keeps many sounds at once from clipping
      comp.threshold.value = -14;
      comp.ratio.value = 6;
      comp.connect(ctx.destination);
      const master = ctx.createGain();
      master.gain.value = this.volume;
      master.connect(comp);
      // two seconds of white noise, looped or cut into bursts by every noisy sound
      const noise = ctx.createBuffer(1, ctx.sampleRate * 2, ctx.sampleRate);
      const data = noise.getChannelData(0);
      for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
      [this.ctx, this.master, this.noise] = [ctx, master, noise];
    }
    if (this.ctx.state === 'suspended') void this.ctx.resume();
  }

  setEnabled(on: boolean) {
    this.enabled = on;
    this.save();
    if (!on) {
      this.silence();
      void this.ctx?.suspend();
    } else this.unlock();
  }

  setVolume(v: number) {
    this.volume = clamp(v, 0, 1);
    this.save();
    if (this.ctx && this.master) this.master.gain.setTargetAtTime(this.volume, this.ctx.currentTime, 0.05);
  }

  private get ready(): boolean {
    return this.enabled && !!this.ctx && this.ctx.state === 'running';
  }

  // ---- building blocks ----------------------------------------------------------

  /** A gain that rises to `peak` in `attack` s and dies away over `decay` s, starting at t. */
  private envelope(t: number, attack: number, peak: number, decay: number): GainNode {
    const g = this.ctx!.createGain();
    g.gain.setValueAtTime(0.0001, t);
    g.gain.linearRampToValueAtTime(peak, t + attack);
    g.gain.exponentialRampToValueAtTime(0.0001, t + attack + decay);
    g.connect(this.master!);
    return g;
  }

  /** A tone gliding from f0 to f1, through an envelope. */
  private tone(t: number, f0: number, dur: number, peak: number, type: OscillatorType = 'sine', f1 = f0, attack = 0.004) {
    const ctx = this.ctx!;
    const o = ctx.createOscillator();
    o.type = type;
    o.frequency.setValueAtTime(f0, t);
    if (f1 !== f0) o.frequency.exponentialRampToValueAtTime(Math.max(f1, 1), t + dur);
    o.connect(this.envelope(t, attack, peak, dur));
    o.start(t);
    o.stop(t + attack + dur + 0.05);
  }

  /** A burst of filtered noise, its filter gliding from f0 to f1. */
  private burst(t: number, dur: number, peak: number, type: BiquadFilterType, f0: number, q = 1, f1 = f0, attack = 0.002) {
    const ctx = this.ctx!;
    const src = ctx.createBufferSource();
    src.buffer = this.noise;
    const f = ctx.createBiquadFilter();
    f.type = type;
    f.Q.value = q;
    f.frequency.setValueAtTime(f0, t);
    if (f1 !== f0) f.frequency.exponentialRampToValueAtTime(Math.max(f1, 1), t + dur);
    src.connect(f).connect(this.envelope(t, attack, peak, dur));
    src.start(t, Math.random() * 1.5);
    src.stop(t + attack + dur + 0.05);
  }

  // ---- cues -----------------------------------------------------------------------

  /** Play a one-shot sound for an action. */
  cue(name: Cue) {
    if (!this.ready) return;
    const t = this.ctx!.currentTime + 0.005;
    // the same cue fired twice at once (one action reported twice) plays once
    if (t - (this.lastCue.get(name) ?? -1) < (name === 'tick' ? 0.02 : 0.05)) return;
    this.lastCue.set(name, t);
    switch (name) {
      case 'select':
        this.tone(t, 1800, 0.025, 0.035);
        break;
      case 'place':
        this.tone(t, 1400, 0.04, 0.12);
        this.tone(t, 2100, 0.03, 0.05);
        break;
      case 'bar': // a wooden knock
        this.tone(t, 420, 0.09, 0.22, 'triangle', 300);
        this.burst(t, 0.03, 0.12, 'bandpass', 1300, 2);
        break;
      case 'rack': // a ratchet
        for (let k = 0; k < 4; k++) this.burst(t + k * 0.028, 0.014, 0.16, 'highpass', 3000);
        this.tone(t, 380, 0.08, 0.12, 'triangle');
        break;
      case 'spring': {
        // boing: a wobbling glide down
        const ctx = this.ctx!;
        const o = ctx.createOscillator();
        o.frequency.setValueAtTime(330, t);
        o.frequency.exponentialRampToValueAtTime(170, t + 0.4);
        const lfo = ctx.createOscillator();
        lfo.frequency.value = 16;
        const depth = ctx.createGain();
        depth.gain.value = 45;
        lfo.connect(depth).connect(o.frequency);
        o.connect(this.envelope(t, 0.005, 0.2, 0.4));
        o.start(t);
        lfo.start(t);
        o.stop(t + 0.5);
        lfo.stop(t + 0.5);
        break;
      }
      case 'gear': // a metallic ting (inharmonic partials)
        this.tone(t, 1568, 0.3, 0.08);
        this.tone(t, 2351, 0.22, 0.05);
        this.tone(t, 3920, 0.14, 0.03);
        this.burst(t, 0.015, 0.08, 'highpass', 4000);
        break;
      case 'wheel': // a rubbery bump
        this.tone(t, 150, 0.16, 0.3, 'sine', 70);
        this.burst(t, 0.06, 0.1, 'lowpass', 500);
        break;
      case 'propeller': // a whoosh
        this.burst(t, 0.32, 0.22, 'bandpass', 400, 1.2, 2400, 0.06);
        break;
      case 'mesh': // teeth clinking into mesh
        this.tone(t, 2637, 0.08, 0.06);
        this.tone(t + 0.04, 3520, 0.08, 0.05);
        break;
      case 'pin': // a snap
        this.burst(t, 0.012, 0.18, 'highpass', 2500);
        this.tone(t, 1800, 0.035, 0.1);
        this.tone(t + 0.03, 1200, 0.04, 0.08);
        break;
      case 'free':
        this.tone(t, 1200, 0.04, 0.08);
        this.tone(t + 0.035, 1800, 0.04, 0.06);
        break;
      case 'join': // a solid clunk
        this.tone(t, 170, 0.14, 0.3, 'sine', 120);
        this.burst(t, 0.05, 0.15, 'bandpass', 900, 1.5);
        this.tone(t, 950, 0.05, 0.06, 'triangle');
        break;
      case 'unjoin':
        this.tone(t, 950, 0.05, 0.08, 'triangle');
        this.tone(t + 0.05, 260, 0.1, 0.16, 'sine', 180);
        break;
      case 'delete': // a pop, falling away
        this.tone(t, 700, 0.15, 0.2, 'sine', 120);
        this.burst(t, 0.03, 0.08, 'bandpass', 1500, 1);
        break;
      case 'undo':
        this.tone(t, 900, 0.1, 0.12, 'triangle', 480);
        break;
      case 'play':
        this.tone(t, 523, 0.12, 0.12, 'triangle');
        this.tone(t + 0.09, 784, 0.18, 0.12, 'triangle');
        break;
      case 'pause':
        this.tone(t, 784, 0.12, 0.1, 'triangle');
        this.tone(t + 0.09, 523, 0.18, 0.1, 'triangle');
        break;
      case 'reset':
        this.tone(t, 600, 0.22, 0.12, 'sine', 200);
        this.burst(t, 0.2, 0.05, 'bandpass', 2000, 1, 500, 0.02);
        break;
      case 'motorOn': // whirring up
        this.tone(t, 70, 0.4, 0.12, 'sawtooth', 340, 0.05);
        this.tone(t, 140, 0.4, 0.05, 'square', 680, 0.05);
        break;
      case 'motorOff':
        this.tone(t, 340, 0.4, 0.12, 'sawtooth', 50, 0.01);
        break;
      case 'lever':
        for (let k = 0; k < 3; k++) this.burst(t + k * 0.04, 0.012, 0.14, 'highpass', 3200);
        this.tone(t + 0.12, 660, 0.08, 0.08, 'triangle');
        break;
      case 'pid': // a controller booting: digital beeps
        this.tone(t, 880, 0.05, 0.08, 'square');
        this.tone(t + 0.07, 1320, 0.05, 0.07, 'square');
        this.tone(t + 0.14, 1760, 0.08, 0.06, 'square');
        break;
      case 'sensor': // a sensor coming on: a bright blip
        this.tone(t, 1500, 0.08, 0.1, 'sine', 2600);
        this.tone(t + 0.09, 2600, 0.12, 0.06);
        break;
      case 'tick':
        this.burst(t, 0.008, 0.09, 'highpass', 3500);
        this.tone(t, 2600, 0.008, 0.03);
        break;
      case 'lock': // a latch
        this.burst(t, 0.02, 0.2, 'highpass', 2500);
        this.tone(t, 1200, 0.03, 0.1, 'square');
        this.tone(t + 0.025, 600, 0.06, 0.12, 'triangle');
        break;
      case 'unlock':
        this.tone(t, 600, 0.04, 0.1, 'triangle');
        this.tone(t + 0.04, 1200, 0.04, 0.08, 'square');
        break;
      case 'mass': // a heavy thunk
        this.tone(t, 95, 0.22, 0.35, 'sine', 50);
        this.burst(t, 0.06, 0.12, 'lowpass', 350);
        break;
      case 'copy':
        this.tone(t, 1000, 0.05, 0.1);
        break;
      case 'paste':
        this.tone(t, 700, 0.08, 0.12, 'sine', 1100);
        this.tone(t + 0.06, 1400, 0.05, 0.06);
        break;
      case 'preset':
        this.burst(t, 0.3, 0.1, 'bandpass', 500, 1, 2000, 0.05);
        this.tone(t + 0.12, 659, 0.25, 0.08, 'triangle');
        this.tone(t + 0.2, 988, 0.3, 0.06, 'triangle');
        break;
      case 'grab': // a pluck
        this.tone(t, 330, 0.12, 0.12, 'triangle', 300);
        break;
      case 'error':
        this.tone(t, 110, 0.2, 0.15, 'square', 90);
        break;
    }
  }

  // ---- impacts ----------------------------------------------------------------------

  /** A landing: a thud, lower and fuller for a heavier hit, louder for a faster one. */
  private thud(speed: number, mass: number) {
    const t = this.ctx!.currentTime + 0.002;
    const loud = clamp(0.05 + 0.12 * speed, 0, 0.55) * clamp(0.4 + 0.4 * Math.sqrt(mass), 0.4, 1.2);
    const f = clamp(190 - 40 * Math.log2(1 + mass), 55, 190);
    this.tone(t, f, 0.12 + 0.05 * Math.min(mass, 2), loud, 'sine', f * 0.55, 0.002);
    this.burst(t, 0.05, loud * 0.6, 'lowpass', clamp(2500 / (1 + mass), 400, 2500));
  }

  /** A slider hitting an end stop: a metallic clack. */
  private clack(speed: number) {
    const t = this.ctx!.currentTime + 0.002;
    const loud = clamp(0.05 + 0.1 * speed, 0, 0.4);
    this.burst(t, 0.015, loud, 'highpass', 2000);
    this.tone(t, 1900, 0.09, loud * 0.5);
    this.tone(t, 2870, 0.06, loud * 0.3);
  }

  // ---- machine voices -----------------------------------------------------------------

  private noiseSource(): AudioBufferSourceNode {
    const src = this.ctx!.createBufferSource();
    src.buffer = this.noise;
    src.loop = true;
    return src;
  }

  /** A continuous voice: built once, then steered with set(). */
  private voice(key: string, build: (out: GainNode) => Omit<Voice, 'out'>): Voice {
    let v = this.voices.get(key);
    if (!v) {
      const out = this.ctx!.createGain();
      out.gain.value = 0;
      out.connect(this.master!);
      v = { out, ...build(out) };
      const now = this.ctx!.currentTime;
      for (const s of v.sources) {
        if (s instanceof AudioBufferSourceNode) s.start(now, Math.random() * 1.5); // each noise from its own place in the buffer
        else s.start(now);
      }
      this.voices.set(key, v);
    }
    return v;
  }

  private motorVoice(key: string): Voice {
    return this.voice(key, (out) => {
      const ctx = this.ctx!;
      const saw = ctx.createOscillator();
      saw.type = 'sawtooth';
      const whine = ctx.createOscillator(); // the electrical whine, three times higher
      whine.type = 'sine';
      const lp = ctx.createBiquadFilter();
      lp.type = 'lowpass';
      lp.Q.value = 2;
      const wg = ctx.createGain();
      wg.gain.value = 0.25;
      saw.connect(lp).connect(out);
      whine.connect(wg).connect(out);
      return {
        sources: [saw, whine],
        set: ({ f, gain, cutoff }, now) => {
          saw.frequency.setTargetAtTime(f, now, 0.03);
          whine.frequency.setTargetAtTime(f * 3.02, now, 0.03);
          lp.frequency.setTargetAtTime(cutoff, now, 0.03);
          out.gain.setTargetAtTime(gain, now, 0.05);
        },
      };
    });
  }

  private propellerVoice(key: string): Voice {
    return this.voice(key, (out) => {
      const ctx = this.ctx!;
      const air = this.noiseSource();
      const bp = ctx.createBiquadFilter();
      bp.type = 'bandpass';
      bp.Q.value = 0.7;
      const chop = ctx.createGain(); // the blades chop the air at the blade-pass rate
      chop.gain.value = 0.55;
      const lfo = ctx.createOscillator();
      const depth = ctx.createGain();
      depth.gain.value = 0.45;
      lfo.connect(depth).connect(chop.gain);
      const hum = ctx.createOscillator(); // and hum at it
      hum.type = 'triangle';
      const hg = ctx.createGain();
      air.connect(bp).connect(chop).connect(out);
      hum.connect(hg).connect(out);
      return {
        sources: [air, lfo, hum],
        set: ({ f, rate, gain, humGain }, now) => {
          bp.frequency.setTargetAtTime(f, now, 0.05);
          lfo.frequency.setTargetAtTime(rate, now, 0.05);
          hum.frequency.setTargetAtTime(Math.max(rate, 20), now, 0.05);
          hg.gain.setTargetAtTime(humGain, now, 0.05);
          out.gain.setTargetAtTime(gain, now, 0.08);
        },
      };
    });
  }

  /** Filtered noise (rumble, scrape, wind, sizzle). */
  private noiseVoice(key: string, type: BiquadFilterType, q: number): Voice {
    return this.voice(key, (out) => {
      const src = this.noiseSource();
      const f = this.ctx!.createBiquadFilter();
      f.type = type;
      f.Q.value = q;
      src.connect(f).connect(out);
      return {
        sources: [src],
        set: ({ f: freq, gain }, now) => {
          f.frequency.setTargetAtTime(freq, now, 0.05);
          out.gain.setTargetAtTime(gain, now, 0.06);
        },
      };
    });
  }

  private meshVoice(): Voice {
    return this.voice('mesh', (out) => {
      const ctx = this.ctx!;
      // a square wave: at a few hertz each edge is a tooth's tick, faster it becomes a whir
      const sq = ctx.createOscillator();
      sq.type = 'square';
      const hp = ctx.createBiquadFilter();
      hp.type = 'highpass';
      hp.frequency.value = 700;
      sq.connect(hp).connect(out);
      return {
        sources: [sq],
        set: ({ f, gain }, now) => {
          sq.frequency.setTargetAtTime(f, now, 0.03);
          out.gain.setTargetAtTime(gain, now, 0.05);
        },
      };
    });
  }

  private squealVoice(): Voice {
    return this.voice('squeal', (out) => {
      const ctx = this.ctx!;
      const o = ctx.createOscillator();
      o.type = 'sawtooth';
      o.frequency.value = 950;
      const lfo = ctx.createOscillator();
      lfo.frequency.value = 7;
      const depth = ctx.createGain();
      depth.gain.value = 30;
      lfo.connect(depth).connect(o.frequency);
      const bp = ctx.createBiquadFilter();
      bp.type = 'bandpass';
      bp.frequency.value = 1000;
      bp.Q.value = 6;
      o.connect(bp).connect(out);
      return {
        sources: [o, lfo],
        set: ({ gain }, now) => out.gain.setTargetAtTime(gain, now, 0.04),
      };
    });
  }

  /**
   * Follow the running mechanism (call every frame), or fall quiet with null (editing, or
   * paused). Impacts in `state.events` are played and the list emptied.
   */
  update(state: MachineState | null) {
    if (!this.ready) {
      if (state) state.events.length = 0;
      return;
    }
    const now = this.ctx!.currentTime;
    const live = new Set<string>();
    const steer = (key: string, make: () => Voice, params: Record<string, number>) => {
      // start a voice once it is worth hearing, and let it go once it falls silent
      if (params.gain < (this.voices.has(key) ? 1e-4 : 2e-3)) return;
      live.add(key);
      make().set(params, now);
    };
    if (state) {
      for (const m of state.motors) {
        const w = Math.abs(m.speed);
        const strain = clamp(Math.abs(m.torque) / Math.max(m.stall, 1e-3), 0, 1);
        const on = w > 0.3 || Math.abs(m.torque) > 1e-3;
        const f = clamp(45 + 14 * w, 45, 2400);
        steer(`motor:${m.id}`, () => this.motorVoice(`motor:${m.id}`), { f, cutoff: clamp(300 + 3 * f + 1500 * strain, 300, 8000), gain: on ? 0.03 + 0.07 * strain + 0.02 * Math.min(1, w / 50) : 0 });
      }
      for (const p of state.propellers) {
        const w = Math.abs(p.spin);
        const rate = (p.blades * w) / (2 * Math.PI);
        const level = Math.min(1, w / 60);
        steer(`prop:${p.id}`, () => this.propellerVoice(`prop:${p.id}`), {
          f: clamp(150 + 5 * w, 150, 5000),
          rate,
          gain: level * clamp(0.06 + 0.05 * Math.sqrt(Math.abs(p.thrust)), 0, 0.35),
          humGain: rate > 25 ? 0.12 : 0,
        });
      }
      if (state.mesh) {
        const { rate, speed } = state.mesh;
        steer('mesh', () => this.meshVoice(), { f: clamp(rate, 1, 3000), gain: rate > 0.5 ? clamp(0.015 + 0.05 * speed, 0, 0.08) : 0 });
      }
      steer('roll', () => this.noiseVoice('roll', 'lowpass', 0.8), { f: clamp(90 + 140 * state.rolling, 90, 1600), gain: clamp(0.3 * (state.rolling / 3), 0, 0.3) });
      steer('squeal', () => this.squealVoice(), { gain: state.slip > 0.5 ? clamp(0.03 * (state.slip - 0.5), 0, 0.07) : 0 });
      steer('scrape', () => this.noiseVoice('scrape', 'bandpass', 1.5), { f: clamp(1200 + 400 * state.scrape, 1200, 4000), gain: state.scrape > 0.05 ? clamp(0.1 * state.scrape, 0, 0.18) : 0 });
      steer('wind', () => this.noiseVoice('wind', 'bandpass', 0.6), { f: clamp(250 + 90 * state.wind, 250, 2500), gain: clamp((0.12 * (state.wind - 1.5)) / 8, 0, 0.12) });
      steer('fric', () => this.noiseVoice('fric', 'highpass', 0.7), { f: 5000, gain: clamp(0.05 * (state.friction / 5), 0, 0.05) });
      // impacts, at most one every 15 ms (a whole chain landing at once is one crash, not a buzz)
      for (const e of state.events) {
        if (e.speed < 0.08 || now - this.lastImpact < 0.015) continue;
        this.lastImpact = now;
        if (e.kind === 'impact') this.thud(e.speed, e.mass);
        else this.clack(e.speed);
      }
      state.events.length = 0;
    }
    // voices nothing drives any more fade out, and go once quiet
    for (const [key, v] of this.voices) {
      if (live.has(key)) continue;
      v.out.gain.setTargetAtTime(0, now, 0.06);
      this.voices.delete(key);
      for (const s of v.sources) s.stop(now + 0.5);
    }
  }

  /** Stop every continuous sound at once. */
  silence() {
    if (!this.ctx) return;
    const now = this.ctx.currentTime;
    for (const v of this.voices.values()) {
      v.out.gain.setTargetAtTime(0, now, 0.03);
      for (const s of v.sources) s.stop(now + 0.3);
    }
    this.voices.clear();
  }
}
