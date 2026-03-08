import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import { Joystick, Zap, Gauge, Rotate3D, Navigation, Activity } from 'lucide-react';

function App() {
    const [connected, setConnected] = useState(false);
    const [mode, setMode] = useState('MANUAL');
    const [robotState, setRobotState] = useState('IDLE');
    const [missionActive, setMissionActive] = useState(false);

    // Telemetry
    const [speed, setSpeed] = useState(0.0);
    const [pitch, setPitch] = useState(0.0);
    const [roll, setRoll] = useState(0.0);
    const [yaw, setYaw] = useState(0.0);
    const [encoderDist, setEncoderDist] = useState(0.0);
    const [leftPower, setLeftPower] = useState(0.0);
    const [rightPower, setRightPower] = useState(0.0);
    const [liveImage, setLiveImage] = useState(null);
    const [triggerImage, setTriggerImage] = useState(null);

    // Ultrasonic [FC, FL, FR, CFL, CFR, CRL, CRR]
    const [ultrasonic, setUltrasonic] = useState([4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]);

    const ros = useRef(null);

    useEffect(() => {
        const rosConnection = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        rosConnection.on('connection', () => {
            console.log('Connected to websocket server.');
            setConnected(true);
            setupSubscribers(rosConnection);
        });

        rosConnection.on('error', (error) => {
            console.log('Error connecting to websocket server: ', error);
            setConnected(false);
        });

        rosConnection.on('close', () => {
            console.log('Connection to websocket server closed.');
            setConnected(false);
        });

        ros.current = rosConnection;

        return () => {
            if (rosConnection) rosConnection.close();
        };
    }, []);

    const setupSubscribers = (rosInstance) => {
        // 1. Robot State
        const stateSub = new ROSLIB.Topic({
            ros: rosInstance,
            name: '/robot_state',
            messageType: 'std_msgs/String'
        });
        stateSub.subscribe((msg) => {
            setRobotState(msg.data);
            if (msg.data !== 'IDLE' && msg.data !== 'MANUAL') {
                setMissionActive(true);
            } else {
                setMissionActive(false);
            }
        });

        // 2. Unity Telemetry (Float32MultiArray)
        // [0]:posX, [1]:posY, [2]:posZ, [3]:speed, [4]:yaw, [5]:pitch, [6]:roll, 
        // [7]:encoderDist, [8]:leftPower, [9]:rightPower
        const telemetrySub = new ROSLIB.Topic({
            ros: rosInstance,
            name: '/unity/telemetry',
            messageType: 'std_msgs/Float32MultiArray'
        });
        telemetrySub.subscribe((msg) => {
            if (msg.data && msg.data.length >= 10) {
                setSpeed(msg.data[3].toFixed(2));
                setYaw(msg.data[4].toFixed(1));
                setPitch(msg.data[5].toFixed(1));
                setRoll(msg.data[6].toFixed(1));
                setEncoderDist(msg.data[7].toFixed(2));
                setLeftPower(msg.data[8].toFixed(2));
                setRightPower(msg.data[9].toFixed(2));
            }
        });

        // 3. Ultrasonic Sensors
        const usSub = new ROSLIB.Topic({
            ros: rosInstance,
            name: '/sensors/ultrasonic',
            messageType: 'std_msgs/Float32MultiArray'
        });
        usSub.subscribe((msg) => {
            if (msg.data && msg.data.length >= 7) {
                setUltrasonic(msg.data.map(v => parseFloat(v.toFixed(2))));
            }
        });

        // 4. Live Processed Image
        const liveSub = new ROSLIB.Topic({
            ros: rosInstance,
            name: '/vision/processed_frame_base64',
            messageType: 'std_msgs/String'
        });
        liveSub.subscribe((msg) => {
            setLiveImage('data:image/jpeg;base64,' + msg.data);
        });

        // 5. Stage Trigger Image
        const triggerSub = new ROSLIB.Topic({
            ros: rosInstance,
            name: '/vision/stage_trigger_frame_base64',
            messageType: 'std_msgs/String'
        });
        triggerSub.subscribe((msg) => {
            setTriggerImage('data:image/jpeg;base64,' + msg.data);
        });
    };

    const sendCommand = (cmd) => {
        if (!ros.current || !connected) return;
        const cmdTopic = new ROSLIB.Topic({
            ros: ros.current,
            name: '/user_command',
            messageType: 'std_msgs/String'
        });
        const msg = new ROSLIB.Message({ data: cmd });
        cmdTopic.publish(msg);
    };

    const sendManualControl = (linearX, angularZ, applyBrake = false) => {
        if (!ros.current || !connected || mode !== 'MANUAL') return;
        const cmdVel = new ROSLIB.Topic({
            ros: ros.current,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/Twist'
        });
        const twist = new ROSLIB.Message({
            linear: { x: linearX, y: applyBrake ? 1.0 : 0.0, z: 0 },
            angular: { x: 0, y: 0, z: angularZ }
        });
        cmdVel.publish(twist);
    };

    useEffect(() => {
        const handleKeyDown = (e) => {
            if (mode !== 'MANUAL') return;
            switch (e.key.toLowerCase()) {
                case 'w': sendManualControl(1.0, 0); break;
                case 's': sendManualControl(-1.0, 0); break;
                case 'a': sendManualControl(0, 1.0); break;
                case 'd': sendManualControl(0, -1.0); break;
                case ' ': sendManualControl(0, 0, true); break; // FREN
                default: break;
            }
        };
        const handleKeyUp = (e) => {
            if (mode !== 'MANUAL') return;
            // Space bırakıldığında freni kaldır ve boşa al
            if (e.key === ' ') {
                sendManualControl(0, 0, false);
            }
        };

        window.addEventListener('keydown', handleKeyDown);
        window.addEventListener('keyup', handleKeyUp);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
            window.removeEventListener('keyup', handleKeyUp);
        };
    }, [mode, connected]);

    const handleStartMission = () => {
        if (mode === 'AUTO_NORMAL') sendCommand('START_AUTO_NORMAL');
        if (mode === 'AUTO_ACCEL') sendCommand('START_AUTO_ACCEL');
    };

    const handleStopMission = () => {
        sendCommand('STOP');
        setMissionActive(false);
    };

    const stageLabels = {
        'IDLE': 'Bekleniyor',
        'MANUAL': 'Manuel Kontrol',
        'BASLA': '1. Başla / Su Geçişi',
        'TASLI_YOL': '2. Taşlı Yol',
        'YAN_EGIM': '3. Yan Eğim',
        'DIK_ENGEL': '4. Dik Engel',
        'TRAFIK_KONILERI': '5. Trafik Konileri',
        'KAYAR_ENGEL': '6. Kayar Engel',
        'ENGEBELI_ARAZI': '7. Engebeli Arazi',
        'DIK_EGIM_CIKIS': '8. Dik Eğim (Çıkış)',
        'CIKIS_DURMA': '⛔ Eğimde Durma',
        'PLATFORM_ATIS': '9. Platform / Atış',
        'DIK_EGIM_INIS': '10. Dik Eğim (İniş)',
        'INIS_DURMA': '⛔ İnişte Durma (BİTİŞ)',
        'ACCEL_RUN': '🏎️ Hızlanma Testi',
    };

    const PowerBar = ({ value, label, color }) => {
        const pct = Math.abs(value) * 100;
        const isNeg = value < 0;
        return (
            <div className="mb-2">
                <div className="flex justify-between text-xs text-slate-400 mb-1">
                    <span>{label}</span>
                    <span className="font-mono">{value}</span>
                </div>
                <div className="h-3 bg-slate-700 rounded-full overflow-hidden relative">
                    <div className={`h-full rounded-full transition-all duration-200 ${isNeg ? 'bg-rose-500' : color}`}
                        style={{ width: `${Math.min(pct, 100)}%` }} />
                </div>
            </div>
        );
    };

    return (
        <div className="min-h-screen bg-ika-dark flex flex-col font-sans text-slate-100">
            {/* Header */}
            <header className="bg-slate-900/50 backdrop-blur border-b border-slate-700 p-4 flex justify-between items-center sticky top-0 z-50">
                <h1 className="text-2xl font-black tracking-tight text-white flex items-center gap-2">
                    <Joystick className="text-ika-accent" />
                    TEKNOFEST <span className="text-ika-accent">2026</span>
                </h1>
                <div className="flex items-center gap-6">
                    {/* Stage indicator in header */}
                    <div className="flex flex-col items-end">
                        <span className="text-[10px] text-slate-400 font-bold uppercase tracking-wider">STAGE</span>
                        <span className={`font-mono font-bold text-sm ${robotState === 'IDLE' ? 'text-slate-500' : 'text-amber-400'}`}>
                            {stageLabels[robotState] || robotState}
                        </span>
                    </div>
                    <div className="flex flex-col items-end">
                        <span className="text-[10px] text-slate-400 font-bold uppercase tracking-wider">SYSTEM</span>
                        <span className={`font-mono font-bold ${robotState === 'IDLE' ? 'text-slate-400' : 'text-ika-accent animate-pulse'}`}>
                            {robotState}
                        </span>
                    </div>
                    <div className={`px-3 py-1 rounded-full text-xs font-bold flex items-center gap-2 border ${connected ? 'bg-emerald-950/30 border-emerald-500/50 text-emerald-400' : 'bg-rose-950/30 border-rose-500/50 text-rose-400'}`}>
                        <div className={`w-2 h-2 rounded-full ${connected ? 'bg-emerald-400 animate-ping' : 'bg-rose-400'}`}></div>
                        {connected ? 'ONLINE' : 'OFFLINE'}
                    </div>
                </div>
            </header>

            {/* Main Layout */}
            <main className="flex-1 p-6 grid grid-cols-12 gap-6 max-w-[1600px] mx-auto w-full">

                {/* Left Panel: Video & Telemetry */}
                <div className="col-span-8 flex flex-col gap-4">
                    {/* Main Video Feed */}
                    <div className="bg-black rounded-xl aspect-video relative border border-slate-700/50 shadow-2xl overflow-hidden group">
                        <img
                            src="http://localhost:8080/stream"
                            alt="Unity Camera Feed"
                            className="w-full h-full object-cover absolute inset-0 z-10"
                            onLoad={(e) => {
                                const overlay = document.getElementById('no-signal-overlay');
                                if (overlay) overlay.style.display = 'none';
                            }}
                            onError={(e) => { e.target.style.display = 'none'; }}
                        />
                        <div className="absolute inset-0 flex items-center justify-center pointer-events-none" id="no-signal-overlay">
                            <div className="text-center">
                                <p className="text-slate-600 font-mono text-sm mb-2">WAITING FOR CAMERA...</p>
                                <p className="text-slate-700 text-xs">Unity Play tuşuna basınca görüntü gelecek</p>
                            </div>
                        </div>
                        <div className="absolute top-4 left-4 right-4 flex justify-between opacity-0 group-hover:opacity-100 transition-opacity z-20">
                            <span className="bg-black/50 backdrop-blur px-2 py-1 rounded text-xs font-mono text-white">CAM_01</span>
                            <span className="bg-red-500/20 text-red-500 border border-red-500/30 px-2 py-1 rounded text-xs font-bold animate-pulse">LIVE</span>
                        </div>

                        {liveImage && (
                            <div className="absolute bottom-4 left-4 w-40 aspect-video rounded-lg border-2 border-slate-700/50 overflow-hidden shadow-lg z-20">
                                <img src={liveImage} className="w-full h-full object-cover" alt="Live Processing" />
                                <span className="absolute top-1 left-1 bg-black/60 text-[8px] font-mono text-white px-1 rounded">PROCESSED</span>
                            </div>
                        )}

                        {triggerImage && (
                            <div className="absolute bottom-4 right-4 w-40 aspect-video rounded-lg border-2 border-amber-500/50 overflow-hidden shadow-lg z-20">
                                <img src={triggerImage} className="w-full h-full object-cover" alt="Stage Trigger" />
                                <span className="absolute top-1 left-1 bg-amber-500/80 text-[8px] font-mono text-black font-bold px-1 rounded">LAST DETECT</span>
                            </div>
                        )}
                    </div>

                    {/* Telemetry Grid - Row 1: Speed, Distance, Yaw */}
                    <div className="grid grid-cols-3 gap-3">
                        <div className="bg-slate-800/50 p-4 rounded-xl border border-slate-700/50 relative overflow-hidden">
                            <Gauge className="absolute -right-3 -bottom-3 text-slate-700 opacity-20 w-20 h-20" />
                            <h3 className="text-slate-400 text-[10px] font-bold tracking-wider mb-1">VELOCITY</h3>
                            <p className="text-3xl font-black tracking-tighter">{speed} <span className="text-sm font-medium text-slate-500">m/s</span></p>
                        </div>
                        <div className="bg-slate-800/50 p-4 rounded-xl border border-slate-700/50 relative overflow-hidden">
                            <Navigation className="absolute -right-3 -bottom-3 text-slate-700 opacity-20 w-20 h-20" />
                            <h3 className="text-slate-400 text-[10px] font-bold tracking-wider mb-1">DISTANCE</h3>
                            <p className="text-3xl font-black tracking-tighter text-cyan-400">{encoderDist} <span className="text-sm font-medium text-slate-500">m</span></p>
                        </div>
                        <div className="bg-slate-800/50 p-4 rounded-xl border border-slate-700/50 relative overflow-hidden">
                            <Rotate3D className="absolute -right-3 -bottom-3 text-slate-700 opacity-20 w-20 h-20" />
                            <h3 className="text-slate-400 text-[10px] font-bold tracking-wider mb-1">YAW</h3>
                            <p className="text-3xl font-black tracking-tighter text-purple-400">{yaw} <span className="text-sm font-medium text-slate-500">°</span></p>
                        </div>
                    </div>

                    {/* Telemetry Grid - Row 2: Pitch, Roll, Wheel Power */}
                    <div className="grid grid-cols-3 gap-3">
                        <div className="bg-slate-800/50 p-4 rounded-xl border border-slate-700/50">
                            <h3 className="text-slate-400 text-[10px] font-bold tracking-wider mb-1">PITCH</h3>
                            <p className="text-3xl font-black tracking-tighter">{pitch} <span className="text-sm font-medium text-slate-500">°</span></p>
                        </div>
                        <div className="bg-slate-800/50 p-4 rounded-xl border border-slate-700/50">
                            <h3 className="text-slate-400 text-[10px] font-bold tracking-wider mb-1">ROLL</h3>
                            <p className="text-3xl font-black tracking-tighter">{roll} <span className="text-sm font-medium text-slate-500">°</span></p>
                        </div>
                        <div className="bg-slate-800/50 p-4 rounded-xl border border-slate-700/50">
                            <h3 className="text-slate-400 text-[10px] font-bold tracking-wider mb-2">WHEEL POWER</h3>
                            <PowerBar value={leftPower} label="Sol" color="bg-blue-500" />
                            <PowerBar value={rightPower} label="Sağ" color="bg-emerald-500" />
                        </div>
                    </div>

                    {/* Ultrasonic Sensors */}
                    <div className="bg-slate-800/50 p-4 rounded-xl border border-slate-700/50">
                        <h3 className="text-slate-400 text-[10px] font-bold tracking-wider mb-3">ULTRASONIC SENSORS</h3>
                        <div className="grid grid-cols-7 gap-1 text-center">
                            {['FL', 'CFL', 'FC', 'CFR', 'FR', 'CRL', 'CRR'].map((label, i) => {
                                const idx = [1, 3, 0, 4, 2, 5, 6][i];
                                const val = ultrasonic[idx];
                                const pct = Math.min((val / 4.0) * 100, 100);
                                const color = val < 0.3 ? 'bg-red-500' : val < 0.6 ? 'bg-yellow-500' : val < 1.5 ? 'bg-green-500' : 'bg-cyan-500';
                                return (
                                    <div key={label} className="flex flex-col items-center">
                                        <span className="text-[9px] text-slate-500 mb-1">{label}</span>
                                        <div className="w-full h-16 bg-slate-700 rounded-md relative overflow-hidden">
                                            <div className={`absolute bottom-0 w-full ${color} transition-all duration-200 rounded-md`}
                                                style={{ height: `${pct}%` }} />
                                        </div>
                                        <span className={`text-[10px] font-mono mt-1 ${val < 0.3 ? 'text-red-400' : 'text-slate-400'}`}>{val}m</span>
                                    </div>
                                );
                            })}
                        </div>
                    </div>
                </div>

                {/* Right Panel: Controls */}
                <div className="col-span-4 flex flex-col gap-4">

                    {/* Mode Selection */}
                    <div className="bg-slate-800/80 backdrop-blur p-5 rounded-xl border border-slate-700/50 shadow-lg">
                        <h2 className="text-white text-sm font-bold tracking-wider mb-3 border-b border-slate-700 pb-2">OPERATING MODE</h2>
                        <div className="space-y-2">
                            <button
                                onClick={() => { setMode('MANUAL'); sendCommand('MANUAL'); }}
                                disabled={missionActive}
                                className={`w-full text-left p-3 rounded-lg font-bold transition-all text-sm
                                    ${mode === 'MANUAL'
                                        ? 'bg-ika-accent text-white shadow-lg shadow-blue-900/20'
                                        : 'bg-slate-700/50 text-slate-400 hover:bg-slate-700 hover:text-white'}
                                    ${missionActive ? 'opacity-50 cursor-not-allowed' : ''}`}
                            >
                                <div className="flex justify-between items-center">
                                    <span>🎮 MANUAL CONTROL</span>
                                    {mode === 'MANUAL' && <div className="w-2 h-2 bg-white rounded-full animate-pulse"></div>}
                                </div>
                            </button>

                            <div className="h-px bg-gradient-to-r from-transparent via-slate-700 to-transparent my-2"></div>

                            <button
                                onClick={() => setMode('AUTO_NORMAL')}
                                disabled={missionActive}
                                className={`w-full text-left p-3 rounded-lg font-bold transition-all text-sm
                                    ${mode === 'AUTO_NORMAL'
                                        ? 'bg-purple-600 text-white shadow-lg shadow-purple-900/20'
                                        : 'bg-slate-700/50 text-slate-400 hover:bg-slate-700 hover:text-white'}
                                    ${missionActive ? 'opacity-50 cursor-not-allowed' : ''}`}
                            >
                                <div className="flex justify-between items-center">
                                    <span>🤖 AUTO: NORMAL PARKUR</span>
                                    {mode === 'AUTO_NORMAL' && <div className="w-2 h-2 bg-white rounded-full animate-pulse"></div>}
                                </div>
                            </button>
                            <button
                                onClick={() => setMode('AUTO_ACCEL')}
                                disabled={missionActive}
                                className={`w-full text-left p-3 rounded-lg font-bold transition-all text-sm
                                    ${mode === 'AUTO_ACCEL'
                                        ? 'bg-orange-600 text-white shadow-lg shadow-orange-900/20'
                                        : 'bg-slate-700/50 text-slate-400 hover:bg-slate-700 hover:text-white'}
                                    ${missionActive ? 'opacity-50 cursor-not-allowed' : ''}`}
                            >
                                <div className="flex justify-between items-center">
                                    <span>🚀 AUTO: ACCELERATION</span>
                                    {mode === 'AUTO_ACCEL' && <div className="w-2 h-2 bg-white rounded-full animate-pulse"></div>}
                                </div>
                            </button>
                        </div>
                    </div>

                    {/* Action Area */}
                    <div className="flex-1 flex flex-col gap-3">
                        {mode === 'MANUAL' ? (
                            <div className="bg-slate-800/50 border border-dashed border-slate-600 rounded-xl p-6 text-center">
                                <div className="grid grid-cols-3 gap-2 w-28 mx-auto mb-3">
                                    <div className="col-start-2 border border-slate-500 rounded p-2 text-slate-300 font-mono text-xs">W</div>
                                    <div className="col-start-1 border border-slate-500 rounded p-2 text-slate-300 font-mono text-xs">A</div>
                                    <div className="col-start-2 border border-slate-500 rounded p-2 text-slate-300 font-mono text-xs">S</div>
                                    <div className="col-start-3 border border-slate-500 rounded p-2 text-slate-300 font-mono text-xs">D</div>
                                </div>
                                <p className="text-slate-400 text-sm font-medium">Remote Control Active</p>
                                <p className="text-slate-600 text-xs mt-1">SPACE = Brake</p>
                            </div>
                        ) : !missionActive ? (
                            <button
                                onClick={handleStartMission}
                                className="w-full py-5 bg-gradient-to-r from-emerald-600 to-emerald-500 hover:from-emerald-500 hover:to-emerald-400 text-white font-black rounded-xl text-lg shadow-xl shadow-emerald-900/30 active:scale-95 transition-all border border-emerald-400/20 group">
                                <span className="flex items-center justify-center gap-2">
                                    ▶ START MISSION
                                    <span className="group-hover:translate-x-1 transition-transform">→</span>
                                </span>
                            </button>
                        ) : (
                            <button
                                onClick={handleStopMission}
                                className="w-full py-5 bg-gradient-to-r from-amber-600 to-amber-500 hover:from-amber-500 hover:to-amber-400 text-white font-black rounded-xl text-lg shadow-xl shadow-amber-900/30 active:scale-95 transition-all border border-amber-400/20">
                                <span className="flex items-center justify-center gap-2">
                                    ⏹ STOP MISSION
                                </span>
                            </button>
                        )}

                        {/* Stage progress (only in auto mode) */}
                        {missionActive && (
                            <div className="bg-slate-800/50 p-4 rounded-xl border border-slate-700/50">
                                <h3 className="text-slate-400 text-[10px] font-bold tracking-wider mb-3">STAGE PROGRESS</h3>
                                <div className="space-y-1.5">
                                    {['NORMAL', 'TASLI_YOL', 'YAN_EGIM', 'DIK_ENGEL', 'TRAFIK_KONILERI', 'KAYAR_ENGEL'].map((s, i) => (
                                        <div key={s} className={`flex items-center gap-2 p-1.5 rounded text-xs font-mono ${robotState === s ? 'bg-ika-accent/20 text-ika-accent font-bold' : 'text-slate-500'}`}>
                                            <div className={`w-2 h-2 rounded-full ${robotState === s ? 'bg-ika-accent animate-pulse' : 'bg-slate-700'}`}></div>
                                            {i + 1}. {stageLabels[s]?.replace(/^\d+\.\s/, '') || s}
                                        </div>
                                    ))}
                                </div>
                            </div>
                        )}

                        {/* Emergency Stop always visible */}
                        <button
                            onClick={handleStopMission}
                            className="w-full py-5 bg-gradient-to-r from-rose-700 to-rose-600 hover:from-rose-600 hover:to-rose-500 text-white font-black rounded-xl text-lg shadow-xl shadow-rose-900/30 active:scale-95 transition-all border border-rose-400/20 mt-auto">
                            🚨 EMERGENCY STOP
                        </button>
                    </div>

                </div>
            </main>
        </div>
    )
}

export default App
