/**
 * Gallery Configuration
 *
 * Easy to customize - just add/edit/remove items from the arrays below.
 * Each image needs: src, alt, and optionally category and caption.
 *
 * For placeholder images, we use picsum.photos
 * Replace with real images when ready: /images/gallery/your-image.jpg
 */

export interface GalleryImage {
  id: string;
  src: string;
  alt: string;
  category: GalleryCategory;
  caption?: string;
  date?: string;
}

export type GalleryCategory =
  | 'all'
  | 'robots'
  | 'team'
  | 'competitions'
  | 'workshop'
  | 'behind-the-scenes';

export const GALLERY_CATEGORIES: { value: GalleryCategory; label: string }[] = [
  { value: 'all', label: 'All Photos' },
  { value: 'robots', label: 'Robots' },
  { value: 'team', label: 'Team' },
  { value: 'competitions', label: 'Competitions' },
  { value: 'workshop', label: 'Workshop' },
  { value: 'behind-the-scenes', label: 'Behind the Scenes' },
];

/**
 * GALLERY IMAGES
 *
 * To add new images:
 * 1. Add image file to /public/images/gallery/
 * 2. Add entry below with unique id
 *
 * Example:
 * {
 *   id: 'robot-001',
 *   src: '/images/gallery/op3-front-view.jpg',
 *   alt: 'OP3 Robot Front View',
 *   category: 'robots',
 *   caption: 'Our OP3 robot ready for competition',
 *   date: '2025-01-15',
 * }
 */
export const GALLERY_IMAGES: GalleryImage[] = [
  // === ROBOTS ===
  {
    id: 'robot-001',
    src: 'https://picsum.photos/seed/robot1/800/600',
    alt: 'Robot Front View',
    category: 'robots',
    caption: 'ROBOTIS OP3 - Front View',
    date: '2025-01-01',
  },
  {
    id: 'robot-002',
    src: 'https://picsum.photos/seed/robot2/800/600',
    alt: 'Robot Side Profile',
    category: 'robots',
    caption: 'OP3 Side Profile - Showing servo motors',
    date: '2025-01-02',
  },
  {
    id: 'robot-003',
    src: 'https://picsum.photos/seed/robot3/600/800',
    alt: 'Robot in Action',
    category: 'robots',
    caption: 'Testing walking algorithms',
    date: '2025-01-03',
  },
  {
    id: 'robot-004',
    src: 'https://picsum.photos/seed/robot4/800/600',
    alt: 'Robot Close-up',
    category: 'robots',
    caption: 'Head unit with camera system',
    date: '2025-01-04',
  },

  // === TEAM ===
  {
    id: 'team-001',
    src: 'https://picsum.photos/seed/team1/800/600',
    alt: 'Team Photo',
    category: 'team',
    caption: 'BASCORRO Team 2025',
    date: '2025-01-10',
  },
  {
    id: 'team-002',
    src: 'https://picsum.photos/seed/team2/800/600',
    alt: 'Software Division',
    category: 'team',
    caption: 'Software team working on vision system',
    date: '2025-01-11',
  },
  {
    id: 'team-003',
    src: 'https://picsum.photos/seed/team3/600/800',
    alt: 'Mechanical Team',
    category: 'team',
    caption: 'Mechanical division assembling parts',
    date: '2025-01-12',
  },

  // === COMPETITIONS ===
  {
    id: 'comp-001',
    src: 'https://picsum.photos/seed/comp1/800/600',
    alt: 'Competition Arena',
    category: 'competitions',
    caption: 'RoboCup 2024 Arena',
    date: '2024-07-15',
  },
  {
    id: 'comp-002',
    src: 'https://picsum.photos/seed/comp2/800/600',
    alt: 'Match in Progress',
    category: 'competitions',
    caption: 'Semi-final match against Team B',
    date: '2024-07-16',
  },
  {
    id: 'comp-003',
    src: 'https://picsum.photos/seed/comp3/800/600',
    alt: 'Award Ceremony',
    category: 'competitions',
    caption: 'Receiving Best Technical Challenge award',
    date: '2024-07-17',
  },
  {
    id: 'comp-004',
    src: 'https://picsum.photos/seed/comp4/600/800',
    alt: 'Team at Competition',
    category: 'competitions',
    caption: 'Team ready for KRI 2024',
    date: '2024-08-20',
  },

  // === WORKSHOP ===
  {
    id: 'workshop-001',
    src: 'https://picsum.photos/seed/work1/800/600',
    alt: 'Workshop Space',
    category: 'workshop',
    caption: 'Our main workshop at UNDIP',
    date: '2025-01-05',
  },
  {
    id: 'workshop-002',
    src: 'https://picsum.photos/seed/work2/800/600',
    alt: 'Electronics Bench',
    category: 'workshop',
    caption: 'Electronics workstation',
    date: '2025-01-06',
  },
  {
    id: 'workshop-003',
    src: 'https://picsum.photos/seed/work3/800/600',
    alt: '3D Printer',
    category: 'workshop',
    caption: 'Printing custom parts',
    date: '2025-01-07',
  },

  // === BEHIND THE SCENES ===
  {
    id: 'bts-001',
    src: 'https://picsum.photos/seed/bts1/800/600',
    alt: 'Late Night Coding',
    category: 'behind-the-scenes',
    caption: 'Debugging at 2 AM before competition',
    date: '2024-07-14',
  },
  {
    id: 'bts-002',
    src: 'https://picsum.photos/seed/bts2/600/800',
    alt: 'Team Celebration',
    category: 'behind-the-scenes',
    caption: 'Celebrating first successful walk test',
    date: '2024-06-01',
  },
  {
    id: 'bts-003',
    src: 'https://picsum.photos/seed/bts3/800/600',
    alt: 'Pizza Night',
    category: 'behind-the-scenes',
    caption: 'Mandatory pizza during build season',
    date: '2024-05-15',
  },
  {
    id: 'bts-004',
    src: 'https://picsum.photos/seed/bts4/800/600',
    alt: 'Testing Session',
    category: 'behind-the-scenes',
    caption: 'Field testing at the lab',
    date: '2024-04-20',
  },
];
