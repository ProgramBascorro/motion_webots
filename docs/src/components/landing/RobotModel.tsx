'use client';

import { Suspense, useState } from 'react';
import { Canvas, useThree, useFrame } from '@react-three/fiber';
import { OrbitControls, useGLTF, Environment, Center, Html } from '@react-three/drei';

// Toggle this to enable/disable debug mode
const DEBUG_MODE = false;

function Model() {
  const { scene } = useGLTF('/ROBOTIS-OP3_part_asm.glb');
  return (
    <Center>
      <primitive object={scene} scale={1} />
    </Center>
  );
}

function CameraDebugger() {
  const { camera } = useThree();
  const [info, setInfo] = useState({
    position: { x: 0, y: 0, z: 0 },
    rotation: { x: 0, y: 0, z: 0 },
    fov: 0,
    zoom: 1,
  });

  useFrame(() => {
    setInfo({
      position: {
        x: Number(camera.position.x.toFixed(2)),
        y: Number(camera.position.y.toFixed(2)),
        z: Number(camera.position.z.toFixed(2)),
      },
      rotation: {
        x: Number((camera.rotation.x * (180 / Math.PI)).toFixed(1)),
        y: Number((camera.rotation.y * (180 / Math.PI)).toFixed(1)),
        z: Number((camera.rotation.z * (180 / Math.PI)).toFixed(1)),
      },
      fov: 'fov' in camera ? Number((camera as any).fov.toFixed(1)) : 0,
      zoom: Number(camera.zoom.toFixed(2)),
    });
  });

  return (
    <Html position={[0, 100, 0]} style={{ pointerEvents: 'none' }}>
      <div
        style={{
          position: 'fixed',
          top: 10,
          left: 10,
          background: 'rgba(0,0,0,0.85)',
          color: '#00ff00',
          padding: '12px 16px',
          borderRadius: '8px',
          fontFamily: 'monospace',
          fontSize: '11px',
          lineHeight: '1.6',
          minWidth: '200px',
          border: '1px solid #333',
          pointerEvents: 'auto',
        }}
      >
        <div style={{ color: '#fff', fontWeight: 'bold', marginBottom: '8px', borderBottom: '1px solid #444', paddingBottom: '4px' }}>
          Camera Debug
        </div>
        <div>
          <span style={{ color: '#888' }}>Position:</span>
          <br />
          &nbsp;&nbsp;x: <span style={{ color: '#ff6b6b' }}>{info.position.x}</span>
          &nbsp;&nbsp;y: <span style={{ color: '#4ecdc4' }}>{info.position.y}</span>
          &nbsp;&nbsp;z: <span style={{ color: '#ffe66d' }}>{info.position.z}</span>
        </div>
        <div style={{ marginTop: '6px' }}>
          <span style={{ color: '#888' }}>Rotation (deg):</span>
          <br />
          &nbsp;&nbsp;x: <span style={{ color: '#ff6b6b' }}>{info.rotation.x}°</span>
          &nbsp;&nbsp;y: <span style={{ color: '#4ecdc4' }}>{info.rotation.y}°</span>
          &nbsp;&nbsp;z: <span style={{ color: '#ffe66d' }}>{info.rotation.z}°</span>
        </div>
        <div style={{ marginTop: '6px' }}>
          <span style={{ color: '#888' }}>FOV:</span> <span style={{ color: '#a29bfe' }}>{info.fov}°</span>
        </div>
        <div style={{ marginTop: '4px' }}>
          <span style={{ color: '#888' }}>Zoom:</span> <span style={{ color: '#a29bfe' }}>{info.zoom}</span>
        </div>
        <div style={{ marginTop: '8px', paddingTop: '8px', borderTop: '1px solid #444', color: '#666', fontSize: '10px' }}>
          Copy for config:
          <br />
          <code style={{ color: '#888' }}>
            position: [{info.position.x}, {info.position.y}, {info.position.z}]
          </code>
        </div>
      </div>
    </Html>
  );
}

export default function RobotModel() {
  return (
    <Canvas
      camera={{ position: [-67.56, 395.59, 515.72], fov: 50 }}
      style={{ background: 'transparent' }}
    >
      <ambientLight intensity={0.5} />
      <spotLight position={[10, 10, 10]} angle={0.15} penumbra={1} />
      <Suspense fallback={null}>
        <Model />
        <Environment preset="studio" />
      </Suspense>
      <OrbitControls
        enablePan={true}
        enableZoom={true}
        enableRotate={true}
        autoRotate={true}
        autoRotateSpeed={2}
      />
      {DEBUG_MODE && <CameraDebugger />}
    </Canvas>
  );
}

useGLTF.preload('/ROBOTIS-OP3_part_asm.glb');
