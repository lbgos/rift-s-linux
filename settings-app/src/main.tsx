import { useCallback, useEffect, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import { invoke } from '@tauri-apps/api/core';
import './style.css';

type View = 'headset' | 'room' | 'settings';
type Point = [number, number, number];
type Quad = [Point, Point, Point, Point];
type Device = { tracking: string | null; battery: number | null };
interface Snapshot {
  devices?: Device[];
  refresh?: number | null;
  display?: [number, number] | null;
  frameTime?: number | null;
  dropped?: number | null;
  ipdSetting?: number | null;
  ipd?: number | null;
  rect?: Quad | null;
  walls?: Quad[];
  floor?: number | null;
  head?: [number, number] | null;
  dimensions?: { width: number; depth: number; area: number } | null;
  saved?: string | null;
  cameraExport?: boolean;
  cameraActive?: boolean;
  cameraRate?: number | null;
  logs?: { label: string; path: string; size: number | null }[];
  journalSize?: number | null;
  driver?: string | null;
  steamVersion?: string | null;
  mirrorOnStartup?: boolean | null;
}

const missing = '—';
function format(value: number | null | undefined, suffix: string, precision = 0) {
  return value == null ? missing : `${value.toFixed(precision)}${suffix}`;
}
function bytes(value: number | null | undefined) {
  if (value == null) return missing;
  return value >= 1_000_000 ? `${(value / 1_000_000).toFixed(1)} MB` : value >= 1000 ? `${(value / 1000).toFixed(1)} kB` : `${value} B`;
}
function currentView(): View {
  const hash = location.hash.slice(1);
  return hash === 'room' || hash === 'settings' ? hash : 'headset';
}
function Icon({ name }: { name: View }) {
  return <svg aria-hidden="true" viewBox="0 0 24 24">{name === 'headset' ? <><path d="M2.5 7h19v9h-5.5l-1.5-3h-4L8 16H2.5Z"/><path d="M7 10.5h2.5M14.5 10.5H17"/></> : name === 'room' ? <><path d="M5 8.5 18.5 6 20 17.5 4 19Z"/><path d="M9 14.5h6M12 11.5v6"/></> : <><path d="M3 6h18M3 12h18M3 18h18"/><path d="M8 3v6M16 9v6M10 15v6" strokeWidth="2.6"/></>}</svg>;
}
function Plan({ data }: { data: Snapshot }) {
  const boundary = data.walls?.flatMap(q => [q[0], q[3]]) ?? [];
  const points = boundary.length ? boundary : data.rect ?? [];
  if (!points.length) return <div className="well empty">{missing}</div>;
  const xs = points.map(p => p[0]), zs = points.map(p => p[2]);
  const minX = Math.min(...xs), maxX = Math.max(...xs), minZ = Math.min(...zs), maxZ = Math.max(...zs);
  const scale = Math.min(580 / Math.max(maxX - minX, .5), 390 / Math.max(maxZ - minZ, .5));
  const x = (value: number) => 500 + (value - (minX + maxX) / 2) * scale;
  const z = (value: number) => 320 + (value - (minZ + maxZ) / 2) * scale;
  const line = (q: Quad) => `M${x(q[0][0])} ${z(q[0][2])}L${x(q[3][0])} ${z(q[3][2])}`;
  const grid: string[] = [];
  for (let i = Math.floor(minX - 3); i < maxX + 3; i += .5) grid.push(`M${x(i)} 0v640`);
  for (let i = Math.floor(minZ - 3); i < maxZ + 3; i += .5) grid.push(`M0 ${z(i)}h1000`);
  const left = x(minX), right = x(maxX), top = z(minZ), bottom = z(maxZ);
  return <div className="well"><svg className="plan" viewBox="0 0 1000 640" role="img" aria-label="Top-down plan of the live SteamVR boundary">
    <path d={grid.join('')} stroke="#16181b" strokeWidth="1"/>
    <path d={`M0 ${z(0)}h1000M${x(0)} 0v640`} stroke="#2a2d33" strokeWidth="1"/>
    {data.rect && <polygon points={data.rect.map(p => `${x(p[0])},${z(p[2])}`).join(' ')} fill="#ffffff" fillOpacity=".06"/>}
    {data.walls?.length ? <path d={data.walls.map(line).join('')} fill="none" stroke="#fff" strokeWidth="2.5"/> : <polygon points={points.map(p => `${x(p[0])},${z(p[2])}`).join(' ')} fill="none" stroke="#fff" strokeWidth="2.5"/>}
    <path d={`M${x(0) - 12} ${z(0)}h24M${x(0)} ${z(0) - 12}v24`} stroke="#6cb4e4" strokeWidth="2.5"/>
    <text x={x(0) + 20} y={z(0) - 6}>Origin</text>
    {data.head && <><path d={`M${x(data.head[0])} ${z(data.head[1]) - 14}l-12 28h24Z`} fill="#ececee"/><text x={x(data.head[0]) + 22} y={z(data.head[1]) + 10}>Headset</text></>}
    <g stroke="#a0a2a9" strokeWidth="1.5" fill="none"><path d={`M${left} ${bottom + 55}H${right}M${left} ${bottom + 47}v16M${right} ${bottom + 47}v16M${left - 48} ${top}V${bottom}M${left - 56} ${top}h16M${left - 56} ${bottom}h16`}/></g>
    <text className="v" x={(left + right) / 2} y={bottom + 83} textAnchor="middle">{format(maxX - minX, ' m', 2)}</text>
    <text className="v" textAnchor="middle" transform={`translate(${left - 73} ${(top + bottom) / 2}) rotate(-90)`}>{format(maxZ - minZ, ' m', 2)}</text>
  </svg></div>;
}
function KeyValues({ rows }: { rows: [string, string][] }) {
  return <div className="t kv">{rows.map(([label, value]) => <div key={label}><span>{label}</span><span>{value}</span></div>)}</div>;
}

interface PreviewStatus { presented: number; width: number; height: number; error: string | null }

interface PairingStatus { stage: string; message: string; candidates: { id: string }[]; hand?: 'left' | 'right' }
function ControllerPairing() {
  const [hand, setHand] = useState<'left' | 'right'>('left');
  const [status, setStatus] = useState<PairingStatus>({ stage: '', message: '', candidates: [] });
  const [error, setError] = useState('');
  const [pollError, setPollError] = useState('');
  const [pending, setPending] = useState(false);
  const active = ['searching', 'ready', 'pairing'].includes(status.stage);
  useEffect(() => {
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout>;
    async function poll() {
      try {
        const next = await invoke<PairingStatus>('controller_pairing_status');
        if (!cancelled) {
          setStatus(next);
          if (['searching', 'ready', 'pairing'].includes(next.stage) && next.hand) setHand(next.hand);
          setPollError('');
        }
      } catch (failure) { if (!cancelled) setPollError(String(failure)); }
      if (!cancelled) timer = setTimeout(poll, 500);
    }
    void poll();
    return () => { cancelled = true; clearTimeout(timer); };
  }, []);
  async function command(name: string, args?: Record<string, string>) {
    setPending(true); setError('');
    try {
      await invoke(name, args);
      setStatus(await invoke<PairingStatus>('controller_pairing_status'));
    } catch (failure) { setError(String(failure)); }
    finally { setPending(false); }
  }
  return <div className="sec pairing">
    <h2>Pair controller</h2>
    <p>Close SteamVR. Put a battery in the controller, then hold {hand === 'left' ? 'Menu + Y' : 'Oculus + B'} until its light flashes. Remove the old controller's battery when replacing it.</p>
    <div className="pairing-controls">
      <label>Hand <select aria-label="Controller hand" value={hand} disabled={active || pending} onChange={event => setHand(event.target.value as 'left' | 'right')}><option value="left">Left</option><option value="right">Right</option></select></label>
      <button className="btn" disabled={active || pending} onClick={() => void command('controller_pairing_start', { hand })}>Find controller</button>
      {active && status.stage !== 'pairing' && <button className="btn" disabled={pending} onClick={() => void command('controller_pairing_cancel')}>Cancel</button>}
    </div>
    {status.stage === 'ready' && status.candidates.map(candidate => <div className="pairing-candidate" key={candidate.id}><code>{candidate.id}</code><button className="btn sm" disabled={pending} onClick={() => void command('controller_pairing_select', { id: candidate.id })}>Pair {hand} controller</button></div>)}
    <output className="fb" role="status">{error || pollError || status.message}</output>
    <p className="pairing-note">Existing pairings stay saved. Firmware is never updated.</p>
  </div>;
}

// The headset view is drawn natively: a Wayland subsurface owned by the Rust
// side covers the preview box and receives GPU frames at the headset refresh
// rate. This component only reports where the box is and shows status text
// in the box while no frame covers it.
function HeadsetPreview({ enabled, mirrorOnStartup }: { enabled: boolean; mirrorOnStartup?: boolean | null }) {
  const box = useRef<HTMLDivElement>(null);
  const [error, setError] = useState('');
  const [stats, setStats] = useState('');
  const [full, setFull] = useState(false);
  const initialized = useRef(false);
  const [saving, setSaving] = useState(false);
  async function savePreference(value: boolean) {
    setSaving(true);
    try { await invoke('preview_preference', { enabled: value }); }
    catch (failure) { setError(String(failure)); }
    finally { setSaving(false); }
  }
  useEffect(() => {
    const changed = () => setFull(document.fullscreenElement === box.current);
    document.addEventListener('fullscreenchange', changed);
    return () => document.removeEventListener('fullscreenchange', changed);
  }, []);
  const toggle = useCallback(() => {
    if (document.fullscreenElement) void document.exitFullscreen();
    else void box.current?.requestFullscreen();
  }, []);
  useEffect(() => {
    function keys(event: KeyboardEvent) {
      if (event.key === 'F11') { event.preventDefault(); toggle(); }
    }
    addEventListener('keydown', keys);
    return () => removeEventListener('keydown', keys);
  }, [toggle]);
  useEffect(() => {
    const element = box.current;
    if (!element) return;
    let last = '';
    function report(visible: boolean) {
      const r = element!.getBoundingClientRect();
      const layout = { x: Math.round(r.left), y: Math.round(r.top), width: Math.round(r.width), height: Math.round(r.height), visible: visible && !document.hidden };
      const key = JSON.stringify(layout);
      if (key === last) return;
      last = key;
      invoke('preview_layout', layout).catch((failure: unknown) => setError(String(failure)));
    }
    const update = () => report(enabled);
    const observer = new ResizeObserver(update);
    observer.observe(element);
    addEventListener('resize', update);
    document.addEventListener('visibilitychange', update);
    update();
    return () => {
      observer.disconnect();
      removeEventListener('resize', update);
      document.removeEventListener('visibilitychange', update);
      report(false);
    };
  }, [enabled]);
  useEffect(() => {
    if (!enabled) return;
    let previous: { presented: number; at: number } | null = null;
    async function poll() {
      try {
        if (!initialized.current) {
          await invoke('preview_preference');
          initialized.current = true;
        }
        const status = await invoke<PreviewStatus>('preview_status');
        const now = performance.now();
        setError(status.error ?? '');
        if (previous && !status.error) {
          const fps = (status.presented - previous.presented) * 1000 / (now - previous.at);
          setStats(`${Math.round(fps)} fps · ${status.width} × ${status.height}`);
        } else setStats('');
        previous = { presented: status.presented, at: now };
      } catch (failure) { setError(String(failure)); }
    }
    void poll();
    const timer = setInterval(() => void poll(), 1000);
    return () => clearInterval(timer);
  }, [enabled]);
  return <>
    <h2 className="pv">Preview <span><span className="meta" aria-label="Preview rate">{stats}</span><button className="btn sm" onClick={toggle} aria-label={full ? 'Exit fullscreen preview' : 'Fullscreen preview'}>{full ? 'Exit' : 'Fullscreen'}</button></span></h2>
    <label className="preview-preference"><input type="checkbox" checked={mirrorOnStartup === true} disabled={mirrorOnStartup == null || saving} onChange={event => void savePreference(event.target.checked)}/>Full resolution after next SteamVR start</label>
    <div className="well preview" ref={box} onDoubleClick={toggle} aria-label="Live headset view">
      {error && <output role="status">{error}</output>}
    </div>
  </>;
}

function App() {
  const [view, setView] = useState<View>(currentView);
  const [data, setData] = useState<Snapshot>({});
  const [error, setError] = useState('');
  const [feedback, setFeedback] = useState<Record<string, string>>({});
  const [busy, setBusy] = useState<string | null>(null);
  const [ipd, setIpd] = useState<number | null>(null);
  const editingIpd = useRef(false);
  const dialog = useRef<HTMLDialogElement>(null);
  useEffect(() => {
    const changed = () => setView(currentView());
    addEventListener('hashchange', changed);
    let cancelled = false;
    let timer: ReturnType<typeof setTimeout>;
    async function poll() {
      try {
        const next = await invoke<Snapshot>('snapshot');
        if (!cancelled) {
          setData(next); setError('');
          if (!editingIpd.current) setIpd(next.ipdSetting ?? next.ipd ?? null);
        }
      } catch (e) { if (!cancelled) setError(String(e)); }
      if (!cancelled) timer = setTimeout(poll, 1000);
    }
    void poll();
    return () => { cancelled = true; clearTimeout(timer); removeEventListener('hashchange', changed); };
  }, []);
  async function action(key: string, command: string, args?: Record<string, unknown>) {
    if (busy) return;
    setBusy(key);
    try {
      const result = await invoke<string>(command, args);
      setFeedback(f => ({ ...f, [key]: result }));
    } catch (e) { setFeedback(f => ({ ...f, [key]: String(e) })); }
    finally { setBusy(null); }
  }
  async function saveIpd() {
    if (ipd == null) return;
    try { await invoke('set_ipd', { mm: ipd }); setFeedback(f => ({ ...f, ipd: 'Software IPD saved.' })); }
    catch (e) { setFeedback(f => ({ ...f, ipd: String(e) })); }
    finally { editingIpd.current = false; }
  }
  const fb = (key: string) => <output className="fb" role="status">{feedback[key]}</output>;
  return <>
    <main>
      <section className={`view${view === 'headset' ? ' on' : ''}`} id="headset">
        <div className="col">
          <h2>Display</h2>
          <KeyValues rows={[
            ['Resolution', data.display?.join(' × ') ?? missing],
            ['Refresh', format(data.refresh, ' Hz')],
            ['Frame time', format(data.frameTime, ' ms', 1)],
            ['Dropped frames', data.dropped == null ? missing : String(data.dropped)],
          ]}/>
          <HeadsetPreview enabled={view === 'headset'} mirrorOnStartup={data.mirrorOnStartup}/>
        </div>
        <div className="col">
          <div className="sec"><h2>Devices</h2><div className="t dev">
            <div className="h"><span>Device</span><span>Tracking</span><span className="r">Battery</span></div>
            {['Headset', 'Left controller', 'Right controller'].map((name, i) => {
              const device = data.devices?.[i];
              const battery = device?.battery;
              return <div key={name}><b>{name}</b><span>{device?.tracking ?? missing}</span>{i === 0 ? <span className="r">{device?.tracking ? 'Wired' : missing}</span> : <span className="bat">{battery != null && <i style={{ '--v': `${battery * 100}%` } as React.CSSProperties}/>} {format(battery == null ? null : battery * 100, '%')}</span>}</div>;
            })}
          </div></div>
          <ControllerPairing/>
          <div className="sec"><h2>Cameras</h2><div className="t cam">
            <div className="h"><span>Position</span><span>State</span><span className="r">Rate</span></div>
            {['Front left', 'Front right', 'Top', 'Left', 'Right'].map((name, i) => <div key={name}><b>{name}</b><span>{i < 2 && data.cameraExport ? data.cameraActive ? 'Active' : 'No frames' : missing}</span><span className="r">{format(i < 2 ? data.cameraRate : null, ' Hz', 1)}</span></div>)}
          </div></div>
        </div>
      </section>
      <section className={`view${view === 'room' ? ' on' : ''}`} id="room">
        <div className="col"><h2>Playspace</h2><Plan data={data}/><div className="stats"><div><span>Grid</span><b>{data.rect ? '0.5 m' : missing}</b></div><div><span>Origin</span><b>{data.rect ? 'Floor centre' : missing}</b></div></div></div>
        <div className="col"><div className="sec"><h2>Boundary</h2><KeyValues rows={[
          ['Width', format(data.dimensions?.width, ' m', 2)], ['Depth', format(data.dimensions?.depth, ' m', 2)], ['Area', format(data.dimensions?.area, ' m²', 2)], ['Floor height', format(data.floor, ' m', 2)], ['Saved', data.saved ?? missing],
        ]}/></div><div className="sec"><button className="btn lit wide" disabled={busy !== null} onClick={() => void action('room', 'vr_action', { action: 'room' })}>Start room setup</button>{fb('room')}</div></div>
      </section>
      <section className={`view${view === 'settings' ? ' on' : ''}`} id="settings">
        <div className="col">
          <div className="sec"><h2>Lens</h2><div className="row"><div><b>Software IPD</b><small>Applies when SteamVR starts. The lenses stay fixed.</small></div><div className="range"><input type="range" min="58" max="72" step="0.5" value={ipd ?? 58} disabled={ipd == null} aria-label="Software IPD" style={{ '--p': `${((ipd ?? 58) - 58) / 14 * 100}%` } as React.CSSProperties} onChange={e => { editingIpd.current = true; setIpd(Number(e.target.value)); }} onPointerUp={() => void saveIpd()} onKeyUp={() => void saveIpd()} onBlur={() => { if (editingIpd.current) void saveIpd(); }}/><output>{format(ipd, ' mm', 1)}</output></div></div>{fb('ipd')}</div>
          <div className="sec"><h2>View</h2><div className="row act"><div><b>View origin</b><small>Sets forward to where you face.</small></div><button className="btn" disabled={busy !== null} onClick={() => void action('recenter', 'vr_action', { action: 'recenter' })}>Recenter</button></div>{fb('recenter')}</div>
        </div>
        <div className="col">
          <div className="sec"><h2>Logs</h2><div className="t kv">{data.logs?.map(log => <div key={log.path} title={log.path}><span>{log.label}</span><span>{bytes(log.size)}</span></div>)}<div title="Allocated journal files. Export contains up to 10000 VR journal entries from the last hour."><span>System journal</span><span>{bytes(data.journalSize)}</span></div></div>
            <div className="row act"><div><b>Logs bundle</b><small>Includes device serials and file paths.</small></div><button className="btn lit" disabled={busy !== null} onClick={() => void action('logs', 'export_logs')}>Export logs</button></div>{fb('logs')}
          </div>
          <div className="sec"><h2>System</h2><KeyValues rows={[
            ['Driver', data.driver ?? missing], ['SteamVR', data.steamVersion ?? missing],
          ]}/><div className="row act"><div><b>SteamVR session</b><small>Closes running VR apps.</small></div><button className="btn" disabled={busy !== null} onClick={() => dialog.current?.showModal()}>Restart SteamVR</button></div>{fb('restart')}</div>
        </div>
      </section>
    </main>
    <nav aria-label="Screens">{(['headset', 'room', 'settings'] as const).map(v => <a key={v} href={`#${v}`} aria-current={view === v ? 'page' : undefined}><Icon name={v}/>{v.charAt(0).toUpperCase() + v.slice(1)}</a>)}</nav>
    {error && <output className="error" role="status">{error}</output>}
    <dialog ref={dialog}><p>Restart SteamVR?</p><p>Running VR apps will close. The headset must be off and still.</p><div className="actions"><button className="btn" onClick={() => dialog.current?.close()}>Cancel</button><button className="btn lit" onClick={() => { dialog.current?.close(); void action('restart', 'restart_steamvr', { confirmed: true }); }}>Restart SteamVR</button></div></dialog>
  </>;
}

createRoot(document.getElementById('root')!).render(<App/>);
