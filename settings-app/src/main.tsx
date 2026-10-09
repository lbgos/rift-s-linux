import { useEffect, useRef, useState } from 'react';
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
  proximity?: boolean | null;
  idle?: boolean;
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
          <h2>Stream display <button className="btn sm" disabled={busy !== null} onClick={() => void action('mirror', 'vr_action', { action: 'mirror' })}>Open mirror window</button></h2>
          <div className="well empty">{feedback.mirror || 'Preview is available in the SteamVR mirror window.'}</div>
          <div className="stats">
            <div><span>Display</span><b>{data.display?.join(' × ') ?? missing}</b></div>
            <div><span>Refresh</span><b>{format(data.refresh, ' Hz')}</b></div>
            <div title="Total render GPU time for the latest compositor frame"><span>Frame time</span><b>{format(data.frameTime, ' ms', 1)}</b></div>
            <div title="Dropped frames in the latest compositor frame"><span>Dropped</span><b>{data.dropped ?? missing}</b></div>
          </div>
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
