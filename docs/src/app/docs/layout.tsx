import { source } from '@/lib/source';
import { DocsLayout } from 'fumadocs-ui/layouts/docs';
import type { DocsLayoutProps } from 'fumadocs-ui/layouts/docs';
import { baseOptions } from '@/lib/layout.shared';
import {
  Bot,
  Rocket,
  Cpu,
  Code,
  Trophy,
  Wrench,
  UserPlus,
} from 'lucide-react';

const docsOptions: DocsLayoutProps = {
  ...baseOptions(),
  tree: source.pageTree,
  sidebar: {
    tabs: [
      {
        title: 'Documentation',
        description: 'BASCORRO Robosoccer',
        url: '/docs',
        icon: <Bot />,
      },
      {
        title: 'Getting Started',
        description: 'Setup & Installation',
        url: '/docs/getting-started',
        icon: <Rocket />,
      },
      {
        title: 'Robot',
        description: 'Hardware & Specs',
        url: '/docs/robot',
        icon: <Cpu />,
      },
      {
        title: 'Software',
        description: 'Code & Systems',
        url: '/docs/software',
        icon: <Code />,
      },
      {
        title: 'RoboCup',
        description: 'Competition Info',
        url: '/docs/robocup',
        icon: <Trophy />,
      },
      {
        title: 'Development',
        description: 'Contributing & Tools',
        url: '/docs/development',
        icon: <Wrench />,
      },
      {
        title: 'Onboarding',
        description: 'Join the Team',
        url: '/docs/onboarding',
        icon: <UserPlus />,
      },
    ],
  },
};

export default function Layout({ children }: LayoutProps<'/docs'>) {
  return <DocsLayout {...docsOptions}>{children}</DocsLayout>;
}
