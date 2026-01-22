/**
 * RoboCup Humanoid League Data
 *
 * Content rewritten from official sources with proper citations.
 * Sources: humanoid.robocup.org, robocup.org
 */

export const SIZE_CLASSES = [
  {
    name: 'KidSize',
    heightRange: '40-100 cm',
    ballSize: 'FIFA Size 1',
    teamSize: 4,
    description:
      'The most dynamic category featuring highly agile robots. Teams of four compete in fast-paced matches that showcase advanced locomotion and ball-handling skills.',
    icon: 'Users',
  },
  {
    name: 'AdultSize',
    heightRange: '100-200 cm',
    ballSize: 'FIFA Size 5',
    teamSize: 2,
    description:
      'Full-scale humanoid robots approaching human dimensions. Teams of two robots play on larger fields, emphasizing robust bipedal walking and powerful kicks.',
    icon: 'User',
  },
];

export const ROBOCUP_HISTORY = [
  {
    year: '1997',
    title: 'RoboCup Founded',
    description:
      'The first RoboCup competition was held in Nagoya, Japan, with the ambitious 2050 goal announced.',
  },
  {
    year: '2002',
    title: 'Humanoid League Begins',
    description:
      'The Humanoid League was officially introduced, starting the journey toward human-like soccer robots.',
  },
  {
    year: '2002-2016',
    title: 'Louis Vuitton Humanoid Cup Era',
    description:
      'Louis Vuitton sponsored the Best Humanoid Award, presenting winners with a crystal globe crafted by Baccarat.',
  },
  {
    year: '2014',
    title: 'Major Size Class Reforms',
    description:
      'Radical changes introduced to size classes to accelerate progress toward the 2050 goal.',
  },
  {
    year: '2020',
    title: 'Pandemic Adaptation',
    description:
      'RoboCup moved to virtual format, demonstrating the community\'s resilience and adaptability.',
  },
  {
    year: '2025',
    title: 'Salvador, Brazil',
    description:
      'The latest edition continues pushing boundaries with improved autonomy and perception requirements.',
  },
];

export const EVENTS = [
  {
    year: '2025',
    location: 'Salvador, Brazil',
    date: 'July 15-21, 2025',
    status: 'upcoming',
  },
  {
    year: '2024',
    location: 'Eindhoven, Netherlands',
    date: 'July 15-21, 2024',
    status: 'past',
  },
  {
    year: '2023',
    location: 'Bordeaux, France',
    date: 'July 2023',
    status: 'past',
  },
  {
    year: '2022',
    location: 'Bangkok, Thailand',
    date: 'July 2022',
    status: 'past',
  },
  {
    year: '2021',
    location: 'Virtual Event',
    date: 'June 2021',
    status: 'past',
  },
];

export const ACHIEVEMENTS = [
  {
    name: 'Best Humanoid Award',
    description:
      'The highest honor in the Humanoid League, determined by team leader voting based on robustness, walking ability, ball handling, and soccer skills.',
    icon: 'Trophy',
  },
  {
    name: 'Soccer Tournament Champions',
    description:
      'Winners of the main competition brackets in KidSize and AdultSize categories.',
    icon: 'Award',
  },
  {
    name: 'Technical Challenge Winners',
    description:
      'Recognition for excellence in specific technical demonstrations like high kicks, obstacle avoidance, and push recovery.',
    icon: 'Zap',
  },
  {
    name: 'Drop-in Games',
    description:
      'Competition where robots from different teams must cooperate, testing adaptability and communication.',
    icon: 'Users',
  },
];

export const RESEARCH_AREAS = [
  {
    title: 'Bipedal Locomotion',
    description:
      'Dynamic walking, running, and maintaining balance while performing soccer actions like kicking.',
    icon: 'Activity',
  },
  {
    title: 'Visual Perception',
    description:
      'Real-time detection of the ball, other players, field lines, and goals using only human-like sensors (cameras).',
    icon: 'Eye',
  },
  {
    title: 'Self-Localization',
    description:
      'Determining the robot\'s position and orientation on the field without external positioning systems.',
    icon: 'MapPin',
  },
  {
    title: 'Team Coordination',
    description:
      'Multi-agent cooperation, role assignment, and strategic play between autonomous teammates.',
    icon: 'Users',
  },
];

export const BASCORRO_JOURNEY = {
  goal: 'Represent Indonesia at RoboCup Humanoid League',
  targetYear: '2026',
  currentFocus: 'KidSize category development',
  milestones: [
    {
      year: '2024',
      title: 'Team Formation',
      description: 'BASCORRO established at Universitas Diponegoro',
    },
    {
      year: '2025',
      title: 'National Competition',
      description: 'Competing in KRSBI-Humanoid (Kontes Robot Indonesia)',
    },
    {
      year: '2026',
      title: 'International Debut',
      description: 'Target: RoboCup qualification',
    },
  ],
};

export const SOURCES = [
  {
    name: 'RoboCup Humanoid League Official',
    url: 'https://humanoid.robocup.org/',
  },
  {
    name: 'RoboCup Federation',
    url: 'https://www.robocup.org/',
  },
  {
    name: 'Best Humanoid Award History',
    url: 'https://humanoid.robocup.org/league/best-humanoid-award/',
  },
  {
    name: 'Humanoid League History',
    url: 'https://humanoid.robocup.org/league/history/',
  },
  {
    name: 'RoboCup 2025 Rules',
    url: 'http://humanoid.robocup.org/wp-content/uploads/RC-HL-2025-Rules.pdf',
  },
];
