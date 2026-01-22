'use client';

import React from 'react';
import Link from 'next/link';
import { Bot, ArrowRight, BookOpen, Search } from 'lucide-react';
import { useSearchContext } from 'fumadocs-ui/contexts/search';
import { NAV_LINKS } from './constants';

const Navbar: React.FC = () => {
  const { setOpenSearch } = useSearchContext();

  return (
    <nav className="sticky top-0 z-40 bg-[#f3f4f6]/80 backdrop-blur-md flex justify-between items-center p-6 border-b border-gray-200">
      <div className="flex items-center gap-2">
        <Bot className="w-6 h-6 text-undip-blue" />
        <span className="font-display font-bold text-xl tracking-tight text-gray-900">BASCORRO</span>
      </div>

      {/* Desktop Nav - Section Scrolling */}
      <div className="hidden md:flex items-center gap-8">
        {NAV_LINKS.map((link) => (
          <a
            key={link.label}
            href={link.href}
            className="text-sm font-medium text-gray-600 hover:text-undip-blue hover:underline decoration-accent-yellow decoration-2 underline-offset-4 transition-all"
          >
            {link.label}
          </a>
        ))}
      </div>

      <div className="flex items-center gap-3">
        {/* Search Button */}
        <button
          onClick={() => setOpenSearch(true)}
          className="group flex items-center gap-2 text-xs font-medium text-gray-500 hover:text-gray-900 transition-colors bg-white px-3 py-1.5 rounded-full border border-gray-200 hover:border-gray-300"
        >
          <Search className="w-4 h-4" />
          <span className="hidden sm:inline">Search</span>
          <kbd className="hidden md:inline-flex items-center gap-0.5 px-1.5 py-0.5 text-[10px] font-mono bg-gray-100 rounded text-gray-400 border border-gray-200">
            ⌘K
          </kbd>
        </button>

        {/* Documentation Link */}
        <Link
          href="/docs"
          className="group flex items-center gap-2 text-xs font-medium text-gray-600 hover:text-black transition-colors bg-white px-3 py-1.5 rounded-full border border-gray-200"
        >
          <BookOpen className="w-4 h-4" />
          <span className="hidden sm:inline">Docs</span>
        </Link>

        <a
          href="#join"
          className="hidden sm:flex items-center gap-2 text-xs font-bold bg-undip-blue text-white px-4 py-2 rounded-full hover:bg-accent-yellow hover:text-black transition-all duration-300"
        >
          JOIN US <ArrowRight className="w-3 h-3" />
        </a>
      </div>
    </nav>
  );
};

export default Navbar;
