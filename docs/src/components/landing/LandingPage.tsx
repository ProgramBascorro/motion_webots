'use client';

import React, { useState } from 'react';
import Intro from './Intro';
import Hero from './Hero';

const LandingPage: React.FC = () => {
  const [showIntro, setShowIntro] = useState(true);

  return (
    <div className="font-sans antialiased text-gray-900 bg-[#1a1a1a] min-h-screen selection:bg-accent-yellow selection:text-black">
      {showIntro && (
        <Intro onComplete={() => setShowIntro(false)} />
      )}

      {!showIntro && (
        <Hero />
      )}
    </div>
  );
};

export default LandingPage;
