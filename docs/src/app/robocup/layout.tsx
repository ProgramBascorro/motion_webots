import type { Metadata } from 'next';
import type { ReactNode } from 'react';
import { BreadcrumbSchema } from '@/components/seo/StructuredData';

const BASE_URL =
    process.env.NEXT_PUBLIC_SITE_URL ?? 'https://bascorro.undip.ac.id';

export const metadata: Metadata = {
    title: 'RoboCup Humanoid League',
    description:
        'Learn about RoboCup, the international robotics competition with the goal of autonomous humanoid robots playing soccer. Discover the Humanoid League, size classes, research areas, and BASCORRO\'s journey toward RoboCup 2050.',
    keywords: [
        'RoboCup',
        'Humanoid League',
        'Robot Soccer',
        'Autonomous Robots',
        'BASCORRO',
        'Robotics Competition',
        'KidSize',
        'AdultSize',
        'RoboCup 2050',
    ],
    openGraph: {
        type: 'website',
        title: 'RoboCup Humanoid League | BASCORRO Robotics',
        description:
            'Discover RoboCup, the premier international robotics competition. Learn about the Humanoid League, research areas, and BASCORRO\'s journey.',
        url: `${BASE_URL}/robocup`,
        images: [
            {
                url: '/Banner.png',
                width: 1536,
                height: 1024,
                alt: 'BASCORRO RoboCup',
            },
        ],
    },
    twitter: {
        card: 'summary_large_image',
        title: 'RoboCup Humanoid League | BASCORRO Robotics',
        description:
            'Discover RoboCup, the premier international robotics competition. Learn about the Humanoid League and BASCORRO\'s journey.',
        images: ['/Banner.png'],
    },
    alternates: {
        canonical: `${BASE_URL}/robocup`,
    },
};

export default function RoboCupLayout({ children }: { children: ReactNode }) {
    return (
        <>
            <BreadcrumbSchema
                items={[
                    { name: 'Home', url: BASE_URL },
                    { name: 'RoboCup', url: `${BASE_URL}/robocup` },
                ]}
            />
            {children}
        </>
    );
}
