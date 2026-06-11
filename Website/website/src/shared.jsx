import { useState, useEffect } from "react";

export const TEAM_NAME = "ABG FINE SHYT";
export const PROJECT_NAME = "Fin Control System";
export const TAGLINE = "Active thrust vector control for high-power rocketry";

export const MISSION_STATEMENT = "We're building an open-source active fin control system for high-power rocketry. Our goal is to prove that low-cost, off-the-shelf components can deliver real-time thrust vector control — making guided rocketry accessible to student teams and hobbyists everywhere.";

export const goals = [
  { num: "01", title: "Stable Guided Flight", desc: "Achieve a controlled, TVC-guided ascent with real-time attitude correction using PID-driven fin actuation." },
  { num: "02", title: "Live Telemetry", desc: "Downlink full flight data — position, velocity, orientation, altitude — to a ground station in real time via 915MHz LoRa." },
  { num: "03", title: "Reproducible & Open", desc: "Document every component, every line of code, every wiring diagram so any team can build and fly this system." },
];

export const hardwareStack = [
  { name: "Raspberry Pi", role: "Flight Computer", desc: "Quad-core brain running the PID control loop, sensor fusion, and telemetry packaging", icon: "⬡" },
  { name: "ESP32", role: "Servo Controller", desc: "Dedicated MCU for real-time TVC servo actuation with deterministic timing", icon: "◈" },
  { name: "MPU6050", role: "IMU", desc: "6-axis accelerometer + gyroscope for attitude determination at 100Hz", icon: "◎" },
  { name: "BMP585", role: "Altimeter", desc: "High-precision barometric pressure sensor for altitude tracking", icon: "△" },
  { name: "Adafruit GPS V3", role: "Position & Velocity", desc: "66-channel GPS receiver for lat/lon, ground speed, and heading", icon: "⊕" },
  { name: "RFM95W LoRa", role: "Telemetry Downlink", desc: "915MHz long-range radio transmitting live flight data to ground station", icon: "◇" },
];

export const milestones = [
  { phase: "01", title: "Sensor Integration", status: "active", desc: "GPS + IMU reading live on ESP32 bridge" },
  { phase: "02", title: "Telemetry Link", status: "upcoming", desc: "LoRa radio transmitting to ground station" },
  { phase: "03", title: "Flight Software", status: "upcoming", desc: "PID control loop + sensor fusion on Pi" },
  { phase: "04", title: "Ground Station", status: "upcoming", desc: "Real-time dashboard on laptop" },
  { phase: "05", title: "Static Test", status: "upcoming", desc: "Full avionics bay bench test" },
  { phase: "06", title: "First Flight", status: "upcoming", desc: "Launch with live TVC telemetry" },
];

export const dataFlow = [
  { from: "BMP585", to: "Raspberry Pi", protocol: "I2C" },
  { from: "MPU6050", to: "Raspberry Pi", protocol: "I2C" },
  { from: "GPS V3", to: "Raspberry Pi", protocol: "UART" },
  { from: "Raspberry Pi", to: "ESP32", protocol: "UART" },
  { from: "Raspberry Pi", to: "RFM95W", protocol: "SPI" },
  { from: "ESP32", to: "TVC Servos", protocol: "PWM" },
  { from: "RFM95W", to: "Ground Station", protocol: "915MHz" },
];

// photo: place image in public/team/ and set to "/team/filename.jpg" (or null for initials)
export const teamMembers = [
  { name: "Bryan Pham",    role: "Role 1", focus: "Focus Area 1", photo: "/team/BP.jpg" },
  { name: "Isaiah Tracy",  role: "Role 2", focus: "Focus Area 2", photo:  "/team/IT.jpg"},
  { name: "Taanish Patel", role: "Role 3", focus: "Focus Area 3", photo: "/team/TP.png" },
  { name: "Tariq Akilah",  role: "Role 4", focus: "Focus Area 4", photo: "/team/TA.jpg" },
];

export const mono = "'JetBrains Mono', monospace";
export const display = "'Space Grotesk', sans-serif";
export const accent = "#00ffaa";
export const accentBlue = "#00aaff";

export function GridBackground() {
  return (
    <div style={{ position: "fixed", inset: 0, zIndex: 0, overflow: "hidden", pointerEvents: "none" }}>
      <div style={{
        position: "absolute", inset: 0,
        backgroundImage: `linear-gradient(rgba(0,255,170,0.03) 1px, transparent 1px), linear-gradient(90deg, rgba(0,255,170,0.03) 1px, transparent 1px)`,
        backgroundSize: "60px 60px",
      }} />
      <div style={{ position: "absolute", inset: 0, background: "radial-gradient(ellipse at 50% 0%, rgba(0,255,170,0.06) 0%, transparent 60%)" }} />
      <div style={{ position: "absolute", bottom: 0, left: 0, right: 0, height: "40%", background: "linear-gradient(to top, rgba(8,10,14,1) 0%, transparent 100%)" }} />
    </div>
  );
}

export function ScanLine() {
  return (
    <div style={{ position: "fixed", inset: 0, zIndex: 1, pointerEvents: "none", overflow: "hidden" }}>
      <div style={{
        position: "absolute", left: 0, right: 0, height: "2px",
        background: "linear-gradient(90deg, transparent, rgba(0,255,170,0.08), transparent)",
        animation: "scanline 8s linear infinite",
      }} />
      <style>{`
        @keyframes scanline { 0% { top: -2px; } 100% { top: 100%; } }
        @keyframes pulse-glow { 0%,100% { opacity: 0.4; } 50% { opacity: 1; } }
        @keyframes float-up { 0% { transform: translateY(0); opacity:0.6; } 100% { transform: translateY(-40px); opacity:0; } }
        @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.3} }
      `}</style>
    </div>
  );
}

export function Ticker() {
  const [time, setTime] = useState(new Date());
  useEffect(() => { const i = setInterval(() => setTime(new Date()), 1000); return () => clearInterval(i); }, []);
  return <span style={{ fontFamily: mono, fontSize: "12px", color: accent, opacity: 0.7 }}>{time.toISOString().replace("T", " ").slice(0, 19)} UTC</span>;
}

export function StatusBadge({ status }) {
  const colors = {
    active: { bg: "rgba(0,255,170,0.12)", border: "rgba(0,255,170,0.4)", text: accent, label: "IN PROGRESS" },
    complete: { bg: "rgba(0,170,255,0.12)", border: "rgba(0,170,255,0.4)", text: accentBlue, label: "COMPLETE" },
    upcoming: { bg: "rgba(255,255,255,0.04)", border: "rgba(255,255,255,0.1)", text: "rgba(255,255,255,0.35)", label: "UPCOMING" },
  };
  const c = colors[status] || colors.upcoming;
  return (
    <span style={{ display: "inline-block", padding: "3px 10px", borderRadius: "3px", fontSize: "10px", fontFamily: mono, fontWeight: 600, letterSpacing: "1.5px", background: c.bg, border: `1px solid ${c.border}`, color: c.text }}>
      {c.label}
    </span>
  );
}

export function SectionLabel({ children }) {
  return <div style={{ fontFamily: mono, fontSize: "11px", letterSpacing: "4px", color: accent, marginBottom: "12px" }}>{children}</div>;
}

export function SectionTitle({ children }) {
  return <h2 style={{ fontFamily: display, fontSize: "36px", fontWeight: 600, color: "#fff", margin: "0 0 16px 0" }}>{children}</h2>;
}

export function RocketIllustration() {
  return (
    <svg viewBox="0 0 120 320" width="120" style={{ filter: "drop-shadow(0 0 20px rgba(0,255,170,0.15))" }}>
      <path d="M60 10 L45 80 L75 80 Z" fill="none" stroke={accent} strokeWidth="1.2" opacity="0.8" />
      <path d="M60 10 L45 80 L75 80 Z" fill="rgba(0,255,170,0.04)" />
      <rect x="45" y="80" width="30" height="140" rx="2" fill="none" stroke={accent} strokeWidth="1.2" opacity="0.6" />
      <rect x="45" y="80" width="30" height="140" rx="2" fill="rgba(0,255,170,0.03)" />
      <rect x="47" y="120" width="26" height="30" fill="rgba(0,255,170,0.08)" stroke={accent} strokeWidth="0.8" opacity="0.5" />
      <text x="60" y="138" textAnchor="middle" fill={accent} fontSize="6" fontFamily={mono} opacity="0.6">AV BAY</text>
      <path d="M45 200 L25 260 L45 240" fill="none" stroke={accent} strokeWidth="1.2" opacity="0.6" />
      <path d="M75 200 L95 260 L75 240" fill="none" stroke={accent} strokeWidth="1.2" opacity="0.6" />
      <path d="M45 200 L25 260 L45 240" fill="rgba(0,255,170,0.04)" />
      <path d="M75 200 L95 260 L75 240" fill="rgba(0,255,170,0.04)" />
      <rect x="48" y="220" width="24" height="20" fill="rgba(0,255,170,0.06)" stroke={accent} strokeWidth="0.8" opacity="0.4" />
      <ellipse cx="60" cy="248" rx="8" ry="12" fill="rgba(0,255,170,0.1)" style={{ animation: "pulse-glow 2s ease-in-out infinite" }} />
      <ellipse cx="60" cy="255" rx="4" ry="18" fill="rgba(0,255,170,0.06)" style={{ animation: "pulse-glow 2s ease-in-out infinite 0.3s" }} />
      <line x1="105" y1="10" x2="105" y2="240" stroke="rgba(255,255,255,0.1)" strokeWidth="0.5" />
      <line x1="102" y1="10" x2="108" y2="10" stroke="rgba(255,255,255,0.1)" strokeWidth="0.5" />
      <line x1="102" y1="240" x2="108" y2="240" stroke="rgba(255,255,255,0.1)" strokeWidth="0.5" />
    </svg>
  );
}

export function Footer() {
  return (
    <footer style={{ padding: "60px 24px", borderTop: "1px solid rgba(255,255,255,0.04)", textAlign: "center" }}>
      <div style={{ fontFamily: mono, fontSize: "11px", letterSpacing: "4px", color: "rgba(255,255,255,0.2)" }}>▲ {TEAM_NAME} — {new Date().getFullYear()}</div>
      <div style={{ fontFamily: mono, fontSize: "11px", color: "rgba(255,255,255,0.1)", marginTop: "8px" }}>BUILT FOR THE SKY</div>
    </footer>
  );
}
