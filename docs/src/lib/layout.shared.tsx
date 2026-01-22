import type { BaseLayoutProps } from 'fumadocs-ui/layouts/shared';
import { Bot, Rocket, Github, Home } from 'lucide-react';

export function baseOptions(): BaseLayoutProps {
  return {
    nav: {
      title: (
        <div className="flex items-center gap-2">
          <Bot className="w-5 h-5 text-undip-blue" />
          <span className="font-display font-bold">BASCORRO</span>
        </div>
      ),
    },
    links: [
      {
        icon: <Home />,
        text: 'Home',
        url: '/',
      },
      {
        icon: <Rocket />,
        text: 'Quick Start',
        url: '/docs/getting-started',
      },
      {
        icon: <Github />,
        text: 'GitHub',
        url: 'https://github.com/ProgramBascorro/motion_webots',
        external: true,
      },
    ],
  };
}
