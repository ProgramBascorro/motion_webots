'use client';

import dynamic from 'next/dynamic';
import { useState, useEffect } from 'react';
import { Skeleton } from '@/components/ui/skeleton';
import { Eye, EyeOff, Box, Zap } from 'lucide-react';

const STORAGE_KEY = 'bascorro-3d-enabled';

const RobotModel = dynamic(() => import('./RobotModel'), {
  ssr: false,
  loading: () => (
    <div className="w-full h-full flex items-center justify-center">
      <div className="space-y-4 w-full max-w-md p-8">
        <Skeleton className="h-8 w-3/4 mx-auto" />
        <Skeleton className="h-64 w-full rounded-xl" />
        <div className="flex gap-2 justify-center">
          <Skeleton className="h-4 w-20" />
          <Skeleton className="h-4 w-20" />
          <Skeleton className="h-4 w-20" />
        </div>
      </div>
    </div>
  ),
});

interface ModelViewerProps {
  className?: string;
}

export default function ModelViewer({ className }: ModelViewerProps) {
  const [is3DEnabled, setIs3DEnabled] = useState<boolean | null>(null);

  // Load preference from localStorage on mount
  useEffect(() => {
    const stored = localStorage.getItem(STORAGE_KEY);
    // Default to true if no preference stored
    setIs3DEnabled(stored === null ? true : stored === 'true');
  }, []);

  const toggle3D = () => {
    const newValue = !is3DEnabled;
    setIs3DEnabled(newValue);
    localStorage.setItem(STORAGE_KEY, String(newValue));
  };

  // Show loading state while checking localStorage
  if (is3DEnabled === null) {
    return (
      <div className={className}>
        <div className="w-full h-full flex items-center justify-center bg-gradient-to-br from-gray-100 to-gray-200">
          <Skeleton className="h-32 w-32 rounded-full" />
        </div>
      </div>
    );
  }

  return (
    <div className={`${className} relative`}>
      {is3DEnabled ? (
        <>
          <RobotModel />
          {/* Toggle button when 3D is enabled */}
          <button
            onClick={toggle3D}
            aria-label="Hide 3D model for better performance"
            aria-pressed="true"
            className="absolute top-4 right-4 z-10 flex items-center gap-2 px-3 py-2 bg-black/70 hover:bg-black/90 text-white text-xs font-medium rounded-lg backdrop-blur-sm transition-all group"
            title="Hide 3D model for better performance"
          >
            <EyeOff size={14} className="group-hover:scale-110 transition-transform" aria-hidden="true" />
            <span className="hidden sm:inline">Hide 3D</span>
          </button>
        </>
      ) : (
        // Placeholder when 3D is disabled
        <div className="w-full h-full flex flex-col items-center justify-center bg-gradient-to-br from-gray-100 to-gray-200 p-8">
          <div className="text-center max-w-sm">
            <div className="w-20 h-20 mx-auto mb-6 bg-white rounded-2xl shadow-sm border border-gray-200 flex items-center justify-center">
              <Box size={32} className="text-gray-400" />
            </div>
            <h4 className="font-bold text-lg text-gray-800 mb-2">3D View Disabled</h4>
            <p className="text-sm text-gray-500 mb-6">
              3D rendering is turned off for better performance. Enable it to explore the robot model interactively.
            </p>
            <button
              onClick={toggle3D}
              aria-label="Enable interactive 3D robot model view"
              aria-pressed="false"
              className="inline-flex items-center gap-2 px-5 py-2.5 bg-undip-blue hover:bg-undip-blue/90 text-white font-medium rounded-full transition-all hover:scale-105 shadow-sm"
            >
              <Eye size={16} aria-hidden="true" />
              Enable 3D View
            </button>
            <div className="mt-4 flex items-center justify-center gap-1 text-xs text-gray-400">
              <Zap size={12} />
              <span>Requires WebGL support</span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
