'use client';

import Link from 'next/link';
import { motion } from 'framer-motion';
import {
  ArrowLeft,
  Trophy,
  Award,
  Zap,
  Users,
  User,
  Activity,
  Eye,
  MapPin,
  Calendar,
  Target,
  ExternalLink,
  ChevronRight,
  type LucideIcon,
} from 'lucide-react';
import {
  SIZE_CLASSES,
  ROBOCUP_HISTORY,
  EVENTS,
  ACHIEVEMENTS,
  RESEARCH_AREAS,
  BASCORRO_JOURNEY,
  SOURCES,
} from '@/lib/robocup';

const SectionHeader = ({
  title,
  subtitle,
  light = false,
}: {
  title: string;
  subtitle?: string;
  light?: boolean;
}) => (
  <div className="mb-12 md:mb-16">
    <div className="flex items-center gap-4 mb-4">
      <div className="h-px w-8 bg-accent-yellow"></div>
      <span
        className={`text-xs font-bold uppercase tracking-widest ${light ? 'text-gray-400' : 'text-gray-500'}`}
      >
        {subtitle || 'Section'}
      </span>
    </div>
    <h2
      className={`font-display font-bold text-3xl md:text-4xl lg:text-5xl uppercase tracking-tight ${light ? 'text-white' : 'text-gray-900'}`}
    >
      {title}
    </h2>
  </div>
);

const IconMap: Record<string, LucideIcon> = {
  Trophy,
  Award,
  Zap,
  Users,
  User,
  Activity,
  Eye,
  MapPin,
};

export default function RoboCupPage() {
  return (
    <div className="min-h-screen bg-[#f3f4f6]">
      {/* Header */}
      <header className="sticky top-0 z-40 bg-white/80 backdrop-blur-md border-b border-gray-200">
        <div className="max-w-7xl mx-auto px-6 py-4 flex items-center justify-between">
          <div className="flex items-center gap-4">
            <Link
              href="/"
              className="flex items-center gap-2 text-gray-600 hover:text-undip-blue transition-colors"
            >
              <ArrowLeft size={20} />
              <span className="hidden sm:inline">Back to Home</span>
            </Link>
            <div className="h-6 w-px bg-gray-300 hidden sm:block" />
            <h1 className="font-display font-bold text-xl sm:text-2xl text-gray-900">
              RoboCup
            </h1>
          </div>
          <Link
            href="/docs"
            className="text-sm text-gray-500 hover:text-undip-blue transition-colors"
          >
            View Docs
          </Link>
        </div>
      </header>

      {/* Hero Section */}
      <section className="relative px-6 py-16 md:py-24 bg-gradient-to-br from-undip-blue to-gray-900 text-white overflow-hidden">
        {/* Grid Background */}
        <div className="absolute inset-0 pointer-events-none opacity-10">
          <div className="w-full h-full grid grid-cols-6 md:grid-cols-12">
            {[...Array(12)].map((_, i) => (
              <div key={i} className="border-r border-white h-full"></div>
            ))}
          </div>
        </div>

        <div className="max-w-7xl mx-auto relative z-10">
          <motion.div
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
          >
            <div className="inline-block px-3 py-1 mb-6 border border-white/30 rounded-full text-xs font-mono text-white/70 bg-white/10 backdrop-blur-sm">
              THE 2050 CHALLENGE
            </div>
            <h1 className="font-display font-black text-4xl sm:text-5xl md:text-6xl lg:text-7xl leading-tight tracking-tighter mb-6">
              RoboCup
              <br />
              <span className="text-accent-yellow">Humanoid League</span>
            </h1>
            <p className="font-serif text-lg md:text-xl text-white/80 italic max-w-2xl leading-relaxed border-l-4 border-accent-yellow pl-6">
              "By 2050, a team of fully autonomous humanoid robots shall win a
              soccer game against the human World Cup champions."
            </p>
          </motion.div>
        </div>
      </section>

      {/* About RoboCup */}
      <section className="px-6 py-16 md:py-24 bg-white">
        <div className="max-w-7xl mx-auto">
          <SectionHeader title="What is RoboCup?" subtitle="The Challenge" />
          <div className="grid grid-cols-1 md:grid-cols-2 gap-12">
            <div>
              <p className="text-gray-600 leading-relaxed mb-6">
                RoboCup is an international robotics competition founded in 1997
                with an audacious goal: develop autonomous robots capable of
                defeating the human World Cup soccer champions by 2050.
              </p>
              <p className="text-gray-600 leading-relaxed mb-6">
                This challenge represents the next frontier in artificial
                intelligence—following the historic 1997 moment when Deep Blue
                defeated chess champion Garry Kasparov. While chess required
                strategic thinking, robot soccer demands real-time perception,
                dynamic movement, and team coordination in an unpredictable
                physical environment.
              </p>
              <p className="text-gray-600 leading-relaxed">
                The Humanoid League is the competition closest to this 2050
                vision, featuring robots with human-like bodies that must rely
                solely on human-like senses—no laser rangefinders or GPS
                allowed.
              </p>
            </div>
            <div className="bg-gray-50 p-8 rounded-3xl border border-gray-100">
              <h3 className="font-bold text-xl mb-6 text-undip-blue">
                Why Humanoid Soccer?
              </h3>
              <ul className="space-y-4">
                <li className="flex items-start gap-3">
                  <div className="w-6 h-6 bg-undip-blue/10 rounded-full flex items-center justify-center flex-shrink-0 mt-0.5">
                    <ChevronRight size={14} className="text-undip-blue" />
                  </div>
                  <span className="text-sm text-gray-600">
                    Tests the full range of AI and robotics challenges in one
                    domain
                  </span>
                </li>
                <li className="flex items-start gap-3">
                  <div className="w-6 h-6 bg-undip-blue/10 rounded-full flex items-center justify-center flex-shrink-0 mt-0.5">
                    <ChevronRight size={14} className="text-undip-blue" />
                  </div>
                  <span className="text-sm text-gray-600">
                    Requires real-time decision making in dynamic environments
                  </span>
                </li>
                <li className="flex items-start gap-3">
                  <div className="w-6 h-6 bg-undip-blue/10 rounded-full flex items-center justify-center flex-shrink-0 mt-0.5">
                    <ChevronRight size={14} className="text-undip-blue" />
                  </div>
                  <span className="text-sm text-gray-600">
                    Advances technology transferable to real-world applications
                  </span>
                </li>
                <li className="flex items-start gap-3">
                  <div className="w-6 h-6 bg-undip-blue/10 rounded-full flex items-center justify-center flex-shrink-0 mt-0.5">
                    <ChevronRight size={14} className="text-undip-blue" />
                  </div>
                  <span className="text-sm text-gray-600">
                    Engages public interest and inspires future engineers
                  </span>
                </li>
              </ul>
            </div>
          </div>
        </div>
      </section>

      {/* Research Areas */}
      <section className="px-6 py-16 md:py-24 bg-[#1a1a1a] text-white">
        <div className="max-w-7xl mx-auto">
          <SectionHeader
            title="Research Focus"
            subtitle="Humanoid League"
            light
          />
          <p className="text-gray-400 max-w-2xl mb-12">
            The Humanoid League pushes the boundaries of robotics research
            across multiple disciplines. Robots must perceive, decide, and act
            autonomously using only human-like sensors.
          </p>
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
            {RESEARCH_AREAS.map((area, i) => {
              const Icon = IconMap[area.icon] || Activity;
              return (
                <motion.div
                  key={i}
                  className="border border-white/10 p-6 rounded-2xl hover:bg-white/5 transition-colors group"
                  initial={{ opacity: 0, y: 20 }}
                  whileInView={{ opacity: 1, y: 0 }}
                  viewport={{ once: true, margin: '-50px' }}
                  transition={{ delay: i * 0.1, duration: 0.5 }}
                >
                  <div className="w-12 h-12 bg-undip-blue/20 rounded-xl flex items-center justify-center text-accent-yellow mb-6 group-hover:scale-110 transition-transform">
                    <Icon size={24} />
                  </div>
                  <h4 className="font-bold text-lg mb-2">{area.title}</h4>
                  <p className="text-sm text-gray-400 leading-relaxed">
                    {area.description}
                  </p>
                </motion.div>
              );
            })}
          </div>
        </div>
      </section>

      {/* Size Classes */}
      <section className="px-6 py-16 md:py-24 bg-[#f3f4f6]">
        <div className="max-w-7xl mx-auto">
          <SectionHeader title="Size Classes" subtitle="Competition Categories" />
          <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
            {SIZE_CLASSES.map((sizeClass, i) => {
              const Icon = IconMap[sizeClass.icon] || Users;
              return (
                <motion.div
                  key={i}
                  className="bg-white rounded-3xl overflow-hidden shadow-sm hover:shadow-xl transition-all border border-gray-200"
                  initial={{ opacity: 0, y: 20 }}
                  whileInView={{ opacity: 1, y: 0 }}
                  viewport={{ once: true, margin: '-50px' }}
                  transition={{ delay: i * 0.1, duration: 0.5 }}
                >
                  <div className="h-32 bg-gradient-to-br from-undip-blue to-gray-800 flex items-center justify-center">
                    <Icon size={48} className="text-white/30" />
                  </div>
                  <div className="p-8">
                    <h3 className="text-2xl font-bold font-display mb-2">
                      {sizeClass.name}
                    </h3>
                    <p className="text-gray-600 mb-6 text-sm">
                      {sizeClass.description}
                    </p>
                    <div className="grid grid-cols-3 gap-4">
                      <div className="bg-gray-50 px-3 py-2 rounded-lg border border-gray-100">
                        <div className="text-xs text-gray-400 mb-1">Height</div>
                        <div className="text-sm font-bold text-gray-900">
                          {sizeClass.heightRange}
                        </div>
                      </div>
                      <div className="bg-gray-50 px-3 py-2 rounded-lg border border-gray-100">
                        <div className="text-xs text-gray-400 mb-1">Ball</div>
                        <div className="text-sm font-bold text-gray-900">
                          {sizeClass.ballSize}
                        </div>
                      </div>
                      <div className="bg-gray-50 px-3 py-2 rounded-lg border border-gray-100">
                        <div className="text-xs text-gray-400 mb-1">Team</div>
                        <div className="text-sm font-bold text-gray-900">
                          {sizeClass.teamSize} robots
                        </div>
                      </div>
                    </div>
                  </div>
                </motion.div>
              );
            })}
          </div>
          <p className="text-center text-sm text-gray-500 mt-8">
            Note: TeenSize (80-140cm) was historically a separate category but
            has been consolidated into the current two-class system.
          </p>
        </div>
      </section>

      {/* Achievements/Awards */}
      <section className="px-6 py-16 md:py-24 bg-accent-yellow">
        <div className="max-w-7xl mx-auto">
          <h2 className="font-display font-black text-4xl md:text-5xl text-undip-blue mb-12 uppercase">
            Awards & Recognition
          </h2>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
            {ACHIEVEMENTS.map((achievement, i) => {
              const Icon = IconMap[achievement.icon] || Trophy;
              return (
                <div
                  key={i}
                  className="bg-white/20 backdrop-blur-sm rounded-2xl p-6 border border-black/10"
                >
                  <div className="flex items-start gap-4">
                    <div className="w-12 h-12 bg-black/10 rounded-xl flex items-center justify-center">
                      <Icon size={24} className="text-undip-blue" />
                    </div>
                    <div>
                      <h3 className="font-bold text-lg text-gray-900 mb-2">
                        {achievement.name}
                      </h3>
                      <p className="text-sm text-gray-800 leading-relaxed">
                        {achievement.description}
                      </p>
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        </div>
      </section>

      {/* History Timeline */}
      <section className="px-6 py-16 md:py-24 bg-white">
        <div className="max-w-7xl mx-auto">
          <SectionHeader title="History & Milestones" subtitle="Timeline" />
          <div className="relative">
            {/* Timeline line */}
            <div className="absolute left-4 md:left-1/2 top-0 bottom-0 w-px bg-gray-200 transform md:-translate-x-1/2"></div>

            <div className="space-y-8">
              {ROBOCUP_HISTORY.map((item, i) => (
                <motion.div
                  key={i}
                  className={`relative flex flex-col md:flex-row gap-4 md:gap-8 ${i % 2 === 0 ? 'md:flex-row-reverse' : ''}`}
                  initial={{ opacity: 0, y: 20 }}
                  whileInView={{ opacity: 1, y: 0 }}
                  viewport={{ once: true, margin: '-50px' }}
                  transition={{ delay: i * 0.1, duration: 0.5 }}
                >
                  {/* Timeline dot */}
                  <div className="absolute left-4 md:left-1/2 w-3 h-3 bg-undip-blue rounded-full transform -translate-x-1/2 mt-2"></div>

                  {/* Content */}
                  <div
                    className={`ml-10 md:ml-0 md:w-1/2 ${i % 2 === 0 ? 'md:pr-12 md:text-right' : 'md:pl-12'}`}
                  >
                    <div className="bg-gray-50 p-6 rounded-2xl border border-gray-100">
                      <div className="text-sm font-bold text-undip-blue mb-1">
                        {item.year}
                      </div>
                      <h3 className="font-bold text-lg mb-2">{item.title}</h3>
                      <p className="text-sm text-gray-600">{item.description}</p>
                    </div>
                  </div>
                </motion.div>
              ))}
            </div>
          </div>
        </div>
      </section>

      {/* Events */}
      <section className="px-6 py-16 md:py-24 bg-[#f3f4f6]">
        <div className="max-w-7xl mx-auto">
          <SectionHeader title="Events" subtitle="Competition Calendar" />
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-6">
            {EVENTS.map((event, i) => (
              <motion.div
                key={i}
                className={`bg-white rounded-2xl p-6 border ${event.status === 'upcoming' ? 'border-undip-blue' : 'border-gray-200'} ${event.status === 'upcoming' ? 'ring-2 ring-undip-blue/20' : ''}`}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true, margin: '-50px' }}
                transition={{ delay: i * 0.05, duration: 0.4 }}
              >
                {event.status === 'upcoming' && (
                  <div className="inline-block px-2 py-1 bg-undip-blue text-white text-xs font-bold rounded mb-3">
                    UPCOMING
                  </div>
                )}
                <div className="flex items-center gap-2 text-gray-400 text-sm mb-2">
                  <Calendar size={14} />
                  {event.date}
                </div>
                <h3 className="font-bold text-xl mb-1">RoboCup {event.year}</h3>
                <p className="text-gray-600">{event.location}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* BASCORRO's Journey */}
      <section className="px-6 py-16 md:py-24 bg-undip-blue text-white">
        <div className="max-w-7xl mx-auto">
          <div className="flex items-center gap-4 mb-4">
            <div className="h-px w-8 bg-accent-yellow"></div>
            <span className="text-xs font-bold uppercase tracking-widest text-white/60">
              Our Path
            </span>
          </div>
          <h2 className="font-display font-bold text-3xl md:text-4xl lg:text-5xl uppercase tracking-tight text-white mb-8">
            BASCORRO's RoboCup Journey
          </h2>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-12">
            <div>
              <div className="flex items-center gap-3 mb-6">
                <Target size={24} className="text-accent-yellow" />
                <span className="text-lg font-bold">
                  {BASCORRO_JOURNEY.goal}
                </span>
              </div>
              <p className="text-white/70 leading-relaxed mb-6">
                As a student-driven team from Universitas Diponegoro, we are
                working toward competing in the RoboCup Humanoid League. Our
                focus is on the {BASCORRO_JOURNEY.currentFocus}, building
                expertise in locomotion, computer vision, and autonomous
                decision-making.
              </p>
              <div className="inline-block px-4 py-2 bg-white/10 rounded-full text-sm font-mono">
                Target Year: {BASCORRO_JOURNEY.targetYear}
              </div>
            </div>

            <div className="space-y-4">
              {BASCORRO_JOURNEY.milestones.map((milestone, i) => (
                <div
                  key={i}
                  className="bg-white/10 rounded-xl p-4 border border-white/10"
                >
                  <div className="text-sm font-bold text-accent-yellow mb-1">
                    {milestone.year}
                  </div>
                  <h4 className="font-bold mb-1">{milestone.title}</h4>
                  <p className="text-sm text-white/60">{milestone.description}</p>
                </div>
              ))}
            </div>
          </div>
        </div>
      </section>

      {/* Sources & Citations */}
      <section className="px-6 py-12 bg-white border-t border-gray-200">
        <div className="max-w-7xl mx-auto">
          <h3 className="font-bold text-lg mb-6 text-gray-900">
            Sources & Further Reading
          </h3>
          <p className="text-sm text-gray-500 mb-4">
            Content on this page has been rewritten from official RoboCup
            sources. For the most up-to-date information, please visit:
          </p>
          <div className="flex flex-wrap gap-3">
            {SOURCES.map((source, i) => (
              <a
                key={i}
                href={source.url}
                target="_blank"
                rel="noopener noreferrer"
                className="inline-flex items-center gap-2 px-4 py-2 bg-gray-100 hover:bg-gray-200 rounded-full text-sm text-gray-700 transition-colors"
              >
                <ExternalLink size={14} />
                {source.name}
              </a>
            ))}
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="bg-[#1a1a1a] text-white py-12 px-6">
        <div className="max-w-7xl mx-auto text-center">
          <Link
            href="/"
            className="font-display font-bold text-2xl hover:text-accent-yellow transition-colors"
          >
            BASCORRO
          </Link>
          <p className="text-gray-500 text-sm mt-2">
            Humanoid Robosoccer Team • Universitas Diponegoro
          </p>
          <div className="border-t border-white/10 mt-8 pt-8 text-xs text-gray-600 font-mono">
            &copy; {new Date().getFullYear()} BASCORRO TEAM
          </div>
        </div>
      </footer>
    </div>
  );
}
