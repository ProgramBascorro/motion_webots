'use client';

import React from "react";
import { motion } from "framer-motion";
import {
  ArrowRight,
  Eye,
  Activity,
  Brain,
  Monitor,
  ChevronDown,
  Instagram,
  Github,
  Mail,
  Users,
  Wrench,
  Zap,
  Code,
  Search,
  Images,
  Trophy,
} from "lucide-react";
import { useSearchContext } from 'fumadocs-ui/contexts/search';
import Navbar from "./Navbar";
import {
  ROBOTS,
  COMPETITIONS,
  TECH_STACK,
  FAQ_ITEMS,
  TEAM_DIVISIONS,
} from "./constants";
import ModelViewer from "./ModelViewer";

const SectionHeader = ({
  title,
  subtitle,
}: {
  title: string;
  subtitle?: string;
}) => (
  <div className="mb-12 md:mb-20">
    <div className="flex items-center gap-4 mb-4">
      <div className="h-px w-8 bg-accent-yellow"></div>
      <span className="text-xs font-bold uppercase tracking-widest text-gray-500">
        {subtitle || "Section"}
      </span>
    </div>
    <h2 className="font-display font-bold text-4xl md:text-5xl text-gray-900 uppercase tracking-tight">
      {title}
    </h2>
  </div>
);

const Hero: React.FC = () => {
  const { setOpenSearch } = useSearchContext();

  return (
    <div className="w-full h-screen flex flex-col relative overflow-hidden bg-[#1a1a1a] ">
      {/* Main Card Container - Scrollable */}
      <motion.div
        className="flex-1 bg-[#f3f4f6]  shadow-2xl relative flex flex-col w-full h-full overflow-y-auto scroll-smooth custom-scrollbar"
        initial={{ scale: 0.95, opacity: 0 }}
        animate={{ scale: 1, opacity: 1 }}
        transition={{ duration: 0.8, delay: 0.2, ease: [0.22, 1, 0.36, 1] }}
      >
        <Navbar />

        {/* --- SECTION 1: HERO --- */}
        <header className="relative min-h-screen md:min-h-[90vh] flex flex-col">
          {/* Grid Background */}
          <div className="absolute inset-0 pointer-events-none z-0 opacity-10">
            <div className="w-full h-full grid grid-cols-6 md:grid-cols-12 gap-0">
              {[...Array(12)].map((_, i) => (
                <div key={i} className="border-r border-gray-900 h-full"></div>
              ))}
            </div>
            <div className="absolute inset-0 grid grid-rows-6 gap-0">
              {[...Array(6)].map((_, i) => (
                <div key={i} className="border-b border-gray-900 w-full"></div>
              ))}
            </div>
          </div>

          <div className="flex-1 flex flex-col md:flex-row relative z-10">
            {/* Left Text */}
            <div className="w-full md:w-3/5 p-6 sm:p-8 md:p-16 flex flex-col justify-center">
              <motion.div
                initial={{ opacity: 0, y: 30 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.8, delay: 0.4 }}
              >
                <div className="inline-block px-3 py-1 mb-6 border border-gray-300 rounded-full text-xs font-mono text-gray-500 bg-white/50 backdrop-blur-sm">
                  EST. 2024 // UNDIP ROBOTICS
                </div>
                <h1 className="font-display font-black text-5xl sm:text-6xl md:text-7xl lg:text-8xl xl:text-[9rem] leading-[0.85] tracking-tighter text-gray-900 mb-4 sm:mb-6">
                  BASCORRO
                </h1>
                <p className="font-serif text-lg sm:text-xl md:text-2xl text-gray-600 italic max-w-lg leading-relaxed mb-6 sm:mb-8 border-l-4 border-accent-yellow pl-4 sm:pl-6">
                  "Shaping the future of autonomous humanoid soccer through
                  intelligent design and engineering."
                </p>

                {/* Search Input */}
                <button
                  onClick={() => setOpenSearch(true)}
                  className="w-full max-w-md flex items-center gap-2 sm:gap-3 px-3 sm:px-4 py-2.5 sm:py-3 mb-6 sm:mb-8 bg-white border border-gray-200 rounded-xl text-left hover:border-gray-300 hover:shadow-sm transition-all group"
                >
                  <Search className="w-5 h-5 text-gray-400 group-hover:text-undip-blue transition-colors" />
                  <span className="flex-1 text-gray-400 text-sm">Search documentation...</span>
                  <kbd className="hidden sm:flex items-center gap-1 px-2 py-1 text-xs font-mono bg-gray-100 rounded border border-gray-200 text-gray-400">
                    <span className="text-xs">⌘</span>K
                  </kbd>
                </button>

                <div className="flex flex-col sm:flex-row flex-wrap gap-3 sm:gap-4">
                  <a
                    href="#robots"
                    className="px-6 sm:px-8 py-3 sm:py-4 bg-undip-blue text-white font-bold rounded-full hover:bg-gray-900 transition-colors flex items-center justify-center gap-2"
                  >
                    Meet the Robots <ArrowRight size={18} />
                  </a>
                  <a
                    href="#join"
                    className="px-6 sm:px-8 py-3 sm:py-4 border border-gray-300 text-gray-900 font-bold rounded-full hover:bg-white transition-colors text-center"
                  >
                    Join the Team
                  </a>
                  <a
                    href="/gallery"
                    className="px-6 sm:px-8 py-3 sm:py-4 border border-gray-300 text-gray-900 font-bold rounded-full hover:bg-white transition-colors flex items-center justify-center gap-2"
                  >
                    <Images size={18} />
                    Gallery
                  </a>
                  <a
                    href="/robocup"
                    className="px-6 sm:px-8 py-3 sm:py-4 border border-gray-300 text-gray-900 font-bold rounded-full hover:bg-white transition-colors flex items-center justify-center gap-2"
                  >
                    <Trophy size={18} />
                    RoboCup
                  </a>
                </div>
              </motion.div>
            </div>

            {/* Right Image */}
            <div className="w-full md:w-2/5 relative min-h-[30vh] sm:min-h-[35vh] md:min-h-auto border-t md:border-t-0 md:border-l border-gray-300 overflow-hidden bg-gray-200">
              <img
                src="https://images.unsplash.com/photo-1485827404703-89b55fcc595e?q=80&w=2070&auto=format&fit=crop"
                alt="Robotics Lab"
                className="w-full h-full object-cover grayscale mix-blend-multiply opacity-80 hover:scale-105 transition-transform duration-1000"
              />
              <div className="absolute inset-0 bg-gradient-to-t from-[#f3f4f6] via-transparent to-transparent"></div>

              <div className="absolute bottom-4 sm:bottom-8 left-4 sm:left-8 right-4 sm:right-8">
                <div className="flex justify-between items-end">
                  <div className="text-xs font-mono">
                    <div className="mb-1">SYS.STATUS: ONLINE</div>
                    <div>LOC: SEMARANG, ID</div>
                  </div>
                  <Activity className="text-accent-yellow animate-pulse" />
                </div>
              </div>
            </div>
          </div>

          <div className="absolute bottom-0 w-full hidden sm:flex justify-center pb-4 sm:pb-8 animate-bounce">
            <a
              href="#about"
              className="text-gray-400 hover:text-black transition-colors"
            >
              <ChevronDown size={32} />
            </a>
          </div>
        </header>

        {/* --- SECTION 2: ABOUT --- */}
        <section
          id="about"
          className="px-8 py-24 md:px-16 border-t border-gray-300 bg-white"
        >
          <div className="max-w-7xl mx-auto">
            <div className="flex flex-col md:flex-row gap-16">
              <div className="md:w-1/3">
                <h3 className="font-display font-bold text-4xl mb-6">
                  Who We Are
                </h3>
                <p className="text-gray-500 leading-relaxed mb-6">
                  BASCORRO is a student-driven research team from Universitas
                  Diponegoro (UNDIP). We exist at the intersection of mechanical
                  engineering, electronics, and artificial intelligence.
                </p>
                <div className="grid grid-cols-2 gap-8 mt-12">
                  <div>
                    <div className="text-4xl font-black text-undip-blue mb-2">
                      20+
                    </div>
                    <div className="text-xs font-bold uppercase tracking-widest text-gray-400">
                      Active Members
                    </div>
                  </div>
                  <div>
                    <div className="text-4xl font-black text-undip-blue mb-2">
                      2
                    </div>
                    <div className="text-xs font-bold uppercase tracking-widest text-gray-400">
                      Robots Built
                    </div>
                  </div>
                </div>
              </div>
              <div className="md:w-2/3 grid grid-cols-1 md:grid-cols-2 gap-8">
                <div className="bg-gray-50 p-8 rounded-2xl border border-gray-100">
                  <h4 className="font-bold text-xl mb-4 text-undip-blue">
                    Our Vision
                  </h4>
                  <p className="text-sm text-gray-600 leading-relaxed">
                    To become a leading force in humanoid robotics research in
                    Indonesia and represent the nation on the global RoboCup
                    stage.
                  </p>
                </div>
                <div className="bg-gray-50 p-8 rounded-2xl border border-gray-100">
                  <h4 className="font-bold text-xl mb-4 text-undip-blue">
                    Our Mission
                  </h4>
                  <p className="text-sm text-gray-600 leading-relaxed">
                    Developing autonomous systems that can perceive, decide, and
                    act in real-time dynamic environments like soccer.
                  </p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* --- SECTION 3: ROBOTS --- */}
        <section
          id="robots"
          className="px-8 py-24 md:px-16 bg-[#f3f4f6] relative"
        >
          {/* Decorative Grid */}
          <div className="absolute top-0 right-0 p-12 opacity-10">
            <div className="w-64 h-64 border border-black rounded-full border-dashed animate-spin-slow"></div>
          </div>

          <div className="max-w-7xl mx-auto relative z-10">
            <SectionHeader title="Our Machines" subtitle="Engineering" />

            {/* 3D Model Showcase */}
            <div className="mb-16 bg-white rounded-3xl border border-gray-200 shadow-sm overflow-hidden">
              <div className="flex flex-col lg:flex-row">
                {/* 3D Viewer */}
                <div className="w-full lg:w-2/3 h-[400px] md:h-[500px] bg-gradient-to-br from-gray-50 to-gray-100">
                  <ModelViewer className="w-full h-full" />
                </div>

                {/* Info Panel */}
                <div className="w-full lg:w-1/3 p-8 flex flex-col justify-center border-t lg:border-t-0 lg:border-l border-gray-200">
                  <div className="inline-block px-3 py-1 mb-4 bg-undip-blue/10 text-undip-blue text-xs font-bold uppercase tracking-widest rounded-full w-fit">
                    Interactive 3D Model
                  </div>
                  <h3 className="font-display font-bold text-2xl md:text-3xl mb-4 text-gray-900">
                    ROBOTIS OP3
                  </h3>
                  <p className="text-gray-600 text-sm leading-relaxed mb-6">
                    Explore our humanoid robot in full 3D. Drag to rotate, scroll to zoom,
                    and use two fingers to pan around the model.
                  </p>
                  <div className="flex flex-wrap gap-2">
                    <span className="px-3 py-1 bg-gray-100 rounded-full text-xs font-mono text-gray-600">
                      Rotate: Drag
                    </span>
                    <span className="px-3 py-1 bg-gray-100 rounded-full text-xs font-mono text-gray-600">
                      Zoom: Scroll
                    </span>
                    <span className="px-3 py-1 bg-gray-100 rounded-full text-xs font-mono text-gray-600">
                      Pan: Shift+Drag
                    </span>
                  </div>
                </div>
              </div>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
              {ROBOTS.map((robot, index) => (
                <div
                  key={index}
                  className="group bg-white rounded-3xl overflow-hidden shadow-sm hover:shadow-xl transition-all duration-500 border border-gray-200"
                >
                  <div className="h-64 bg-gray-200 relative overflow-hidden">
                    <div className="absolute inset-0 bg-gradient-to-br from-gray-800 to-black mix-blend-multiply opacity-60"></div>
                    {/* Placeholder for Robot Image */}
                    <div className="absolute inset-0 flex items-center justify-center text-white/20 font-display font-black text-6xl">
                      {robot.name.split(" ")[0]}
                    </div>
                    <div className="absolute top-6 right-6 bg-white/10 backdrop-blur-md px-3 py-1 rounded text-xs font-mono text-white border border-white/20">
                      STATUS: {robot.status.toUpperCase()}
                    </div>
                  </div>
                  <div className="p-8">
                    <h3 className="text-2xl font-bold font-display mb-2">
                      {robot.name}
                    </h3>
                    <p className="text-gray-600 mb-6 text-sm">{robot.desc}</p>

                    <div className="space-y-3">
                      <div className="text-xs font-bold uppercase tracking-widest text-gray-400 mb-2">
                        Specifications
                      </div>
                      <div className="grid grid-cols-2 gap-2">
                        {robot.specs.map((spec, i) => (
                          <div
                            key={i}
                            className="bg-gray-50 px-3 py-2 rounded border border-gray-100 text-xs font-mono text-gray-700"
                          >
                            {spec}
                          </div>
                        ))}
                      </div>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </section>

        {/* --- SECTION 4: TECH STACK --- */}
        <section
          id="tech"
          className="px-8 py-24 md:px-16 bg-[#1a1a1a] text-white"
        >
          <div className="max-w-7xl mx-auto">
            <div className="mb-12">
              <h2 className="font-display font-bold text-4xl md:text-5xl uppercase tracking-tight mb-4 text-white">
                Core Intelligence
              </h2>
              <p className="text-gray-400 max-w-2xl">
                Our robots don't just move; they think. Powered by ROS 2 and
                advanced computer vision.
              </p>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
              {TECH_STACK.map((tech, i) => (
                <div
                  key={i}
                  className="border border-white/10 p-6 rounded-2xl hover:bg-white/5 transition-colors group"
                >
                  <div className="w-12 h-12 bg-undip-blue/20 rounded-xl flex items-center justify-center text-accent-yellow mb-6 group-hover:scale-110 transition-transform">
                    {tech.icon === "Eye" && <Eye />}
                    {tech.icon === "Activity" && <Activity />}
                    {tech.icon === "Brain" && <Brain />}
                    {tech.icon === "Monitor" && <Monitor />}
                  </div>
                  <h4 className="font-bold text-lg mb-2">{tech.title}</h4>
                  <p className="text-sm text-gray-400 leading-relaxed">
                    {tech.desc}
                  </p>
                </div>
              ))}
            </div>
          </div>
        </section>

        {/* --- SECTION 5: COMPETITIONS --- */}
        <section className="px-8 py-24 md:px-16 bg-accent-yellow">
          <div className="max-w-7xl mx-auto">
            <h2 className="font-display font-black text-4xl md:text-6xl text-undip-blue mb-12 uppercase">
              The Arena
            </h2>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-12">
              {COMPETITIONS.map((comp, i) => (
                <div key={i} className="border-t-2 border-black/10 pt-6">
                  <div className="flex justify-between items-start mb-4">
                    <h3 className="font-bold text-2xl text-gray-900">
                      {comp.name}
                    </h3>
                    <span className="bg-black/10 px-3 py-1 rounded-full text-xs font-bold">
                      {comp.year}
                    </span>
                  </div>
                  <div className="text-sm font-bold uppercase tracking-wider text-undip-blue mb-2">
                    {comp.role}
                  </div>
                  <p className="text-gray-800 leading-relaxed">{comp.desc}</p>
                  {comp.name.includes('RoboCup') && (
                    <a
                      href="/robocup"
                      className="inline-flex items-center gap-1 mt-4 text-sm font-bold text-undip-blue hover:underline"
                    >
                      Learn More <ArrowRight size={14} />
                    </a>
                  )}
                </div>
              ))}
            </div>
          </div>
        </section>

        {/* --- SECTION 6: TEAM (NEW) --- */}
        <section id="team" className="px-8 py-24 md:px-16 bg-white">
          <div className="max-w-7xl mx-auto">
            <SectionHeader title="The Squad" subtitle="Behind the Machines" />

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
              {TEAM_DIVISIONS.map((div, i) => (
                <motion.div
                  key={i}
                  className="bg-gray-50 p-8 rounded-3xl border border-gray-100 hover:border-undip-blue/30 transition-colors group"
                  initial={{ opacity: 0, y: 20 }}
                  whileInView={{ opacity: 1, y: 0 }}
                  viewport={{ once: true, margin: "-50px" }}
                  transition={{ delay: i * 0.1, duration: 0.5 }}
                >
                  <div className="w-12 h-12 bg-white rounded-2xl border border-gray-200 flex items-center justify-center mb-6 text-gray-400 group-hover:text-undip-blue shadow-sm transition-colors">
                    {div.icon === "Users" && (
                      <motion.div
                        whileHover={{ scale: 1.15, y: -2 }}
                        transition={{
                          type: "spring",
                          stiffness: 400,
                          damping: 10,
                        }}
                      >
                        <Users size={24} />
                      </motion.div>
                    )}
                    {div.icon === "Wrench" && (
                      <motion.div
                        whileHover={{ rotate: [0, -20, 20, -10, 10, 0] }}
                        transition={{ duration: 0.6, ease: "easeInOut" }}
                      >
                        <Wrench size={24} />
                      </motion.div>
                    )}
                    {div.icon === "Zap" && (
                      <motion.div
                        whileHover={{
                          scale: [1, 1.2, 1],
                          opacity: [1, 0.7, 1],
                        }}
                        transition={{ duration: 0.4, repeat: Infinity }}
                      >
                        <Zap size={24} />
                      </motion.div>
                    )}
                    {div.icon === "Code" && (
                      <motion.div
                        whileHover={{ scale: 1.15 }}
                        transition={{ type: "spring", stiffness: 400 }}
                      >
                        <Code size={24} />
                      </motion.div>
                    )}
                  </div>
                  <h4 className="font-bold text-xl mb-1">{div.name}</h4>
                  <span className="text-xs font-bold uppercase tracking-widest text-gray-400 mb-4 block">
                    {div.role}
                  </span>
                  <p className="text-sm text-gray-600 mb-6 leading-relaxed">
                    {div.description}
                  </p>
                  <div className="border-t border-gray-200 pt-4">
                    <div className="text-xs font-bold text-gray-900 mb-3">
                      Key Roles:
                    </div>
                    <div className="flex flex-wrap gap-2">
                      {div.members.map((m, idx) => (
                        <span
                          key={idx}
                          className="text-[10px] font-medium bg-white border border-gray-200 px-2 py-1 rounded-md text-gray-500 hover:text-undip-blue hover:border-undip-blue/20 transition-colors cursor-default"
                        >
                          {m}
                        </span>
                      ))}
                    </div>
                  </div>
                </motion.div>
              ))}
            </div>
          </div>
        </section>

        {/* --- SECTION 7: JOIN --- */}
        <section id="join" className="px-8 py-24 md:px-16 bg-white relative">
          <div className="max-w-4xl mx-auto text-center">
            <h2 className="font-display font-black text-5xl md:text-7xl text-gray-900 mb-8 tracking-tighter">
              BUILD THE FUTURE
            </h2>
            <p className="text-xl text-gray-500 mb-12 max-w-2xl mx-auto">
              We are recruiting students from Universitas Diponegoro for the
              2025 season. No prior robotics experience required—just a hunger
              to learn.
            </p>

            <div className="flex flex-col md:flex-row justify-center gap-4 mb-16">
              <button className="px-8 py-4 bg-black text-white font-bold rounded-full hover:bg-gray-800 transition-all transform hover:-translate-y-1">
                Apply Now (Google Form)
              </button>
              <button className="px-8 py-4 border border-gray-300 font-bold rounded-full hover:bg-gray-50 transition-colors">
                View Open Roles
              </button>
            </div>

            {/* FAQ Preview */}
            <div className="text-left border-t border-gray-200 pt-12">
              <h3 className="font-bold text-xl mb-6">
                Frequently Asked Questions
              </h3>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
                {FAQ_ITEMS.map((item, i) => (
                  <div key={i}>
                    <h4 className="font-bold text-sm mb-2">{item.q}</h4>
                    <p className="text-xs text-gray-500 leading-relaxed">
                      {item.a}
                    </p>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </section>

        {/* --- SECTION 8: FOOTER --- */}
        <footer className="bg-[#1a1a1a] text-white pt-24 pb-12 px-8 md:px-16">
          <div className="max-w-7xl mx-auto flex flex-col md:flex-row justify-between items-start gap-12">
            <div>
              <h2 className="font-display font-black text-3xl mb-4">
                BASCORRO
              </h2>
              <p className="text-gray-500 text-sm max-w-xs mb-6">
                Humanoid Robosoccer Team
                <br />
                Universitas Diponegoro
                <br />
                Semarang, Indonesia
              </p>
              <div className="flex gap-4 text-gray-400">
                <a href="#" className="hover:text-white transition-colors">
                  <Instagram size={20} />
                </a>
                <a href="#" className="hover:text-white transition-colors">
                  <Github size={20} />
                </a>
                <a href="#" className="hover:text-white transition-colors">
                  <Mail size={20} />
                </a>
              </div>
            </div>

            <div className="grid grid-cols-2 gap-12 text-sm text-gray-400">
              <div>
                <h4 className="font-bold text-white mb-4 uppercase tracking-widest text-xs">
                  Navigation
                </h4>
                <ul className="space-y-2">
                  <li>
                    <a href="#about" className="hover:text-accent-yellow">
                      About
                    </a>
                  </li>
                  <li>
                    <a href="#robots" className="hover:text-accent-yellow">
                      Robots
                    </a>
                  </li>
                  <li>
                    <a href="#tech" className="hover:text-accent-yellow">
                      Technology
                    </a>
                  </li>
                  <li>
                    <a href="#join" className="hover:text-accent-yellow">
                      Join Us
                    </a>
                  </li>
                </ul>
              </div>
              <div>
                <h4 className="font-bold text-white mb-4 uppercase tracking-widest text-xs">
                  Connect
                </h4>
                <ul className="space-y-2">
                  <li>
                    <a href="#" className="hover:text-accent-yellow">
                      Contact Support
                    </a>
                  </li>
                  <li>
                    <a href="#" className="hover:text-accent-yellow">
                      Sponsorship
                    </a>
                  </li>
                  <li>
                    <a href="#" className="hover:text-accent-yellow">
                      UNDIP Official
                    </a>
                  </li>
                </ul>
              </div>
            </div>
          </div>

          <div className="border-t border-white/10 mt-20 pt-8 text-center text-xs text-gray-600 font-mono">
            &copy; {new Date().getFullYear()} BASCORRO TEAM. SYSTEM VERSION 4.2
          </div>
        </footer>
      </motion.div>
    </div>
  );
};

export default Hero;
