export const NAV_LINKS = [
  { label: 'About', href: '#about' },
  { label: 'Robots', href: '#robots' },
  { label: 'Tech', href: '#tech' },
  { label: 'Team', href: '#team' },
  { label: 'RoboCup', href: '/robocup' },
];

export const ROBOTS = [
  {
    name: "OP3 BASCORRO v1",
    desc: "Our primary striker unit built on the ROBOTIS OP3 platform, enhanced with custom kinematics.",
    specs: ["Height: 510mm", "Weight: 3.5kg", "DOF: 20", "OS: ROS 2 Humble"],
    status: "Active"
  },
  {
    name: "BASCORRO G-1",
    desc: "Next-gen goalkeeper prototype featuring advanced trajectory prediction and stability control.",
    specs: ["Height: 525mm", "Weight: 3.8kg", "DOF: 22", "Vision: Stereo Cam"],
    status: "In Development"
  }
];

export const COMPETITIONS = [
  {
    name: "RoboCup Humanoid League",
    role: "International Challenger",
    desc: "The premier international robotics competition. We aim to field a team of fully autonomous humanoid robots to play soccer against other university teams globally.",
    year: "Target 2026"
  },
  {
    name: "Kontes Robot Indonesia (KRI)",
    role: "National Finalist",
    desc: "Indonesia's national robotics contest. Participating in the KRSBI-Humanoid division to demonstrate local excellence in embedded systems and AI.",
    year: "Annual"
  }
];

export const TECH_STACK = [
  {
    title: "Computer Vision",
    desc: "Real-time ball and field line detection using YOLO and custom color segmentation algorithms.",
    icon: "Eye"
  },
  {
    title: "Locomotion",
    desc: "Stable walking gaits and omnidirectional movement generators utilizing Inverse Kinematics.",
    icon: "Activity"
  },
  {
    title: "Strategy AI",
    desc: "Multi-agent coordination and role assignment (Striker, Goalie) using Behavior Trees.",
    icon: "Brain"
  },
  {
    title: "Simulation",
    desc: "Full physics simulation in Webots to test code safely before deployment on hardware.",
    icon: "Monitor"
  }
];

export const TEAM_DIVISIONS = [
  {
    name: "Management & Official",
    role: "Core Leadership",
    description: "Orchestrating the team's vision, handling logistics, branding, and maintaining relationships with the university and sponsors.",
    members: ["Team Captain", "Vice Captain", "Treasurer", "Secretary", "Manager"],
    icon: "Users"
  },
  {
    name: "Mechanical Engineering",
    role: "Hardware Division",
    description: "Designing and fabricating the robot's physical structure, ensuring stability, durability, and optimal kinematics for soccer.",
    members: ["Lead Mechanical", "CAD Engineer", "Fabrication Specialist", "Material Analyst"],
    icon: "Wrench"
  },
  {
    name: "Electronics & Embedded",
    role: "Hardware Division",
    description: "Developing custom PCBs, managing power distribution systems, and integrating sensors (IMU, Servos) for real-time control.",
    members: ["Lead Electronics", "PCB Designer", "Embedded Engineer", "Wiring Specialist"],
    icon: "Zap"
  },
  {
    name: "Software & AI",
    role: "Intelligence Division",
    description: "The brain of the robot. Implementing computer vision, localization, walking algorithms, and game strategy using ROS 2.",
    members: ["Lead Programmer", "Computer Vision Specialist", "AI & Strategy Dev", "Simulation Eng."],
    icon: "Code"
  }
];

export const FAQ_ITEMS = [
  {
    q: "Do I need robotics experience to join?",
    a: "No! We welcome students from all backgrounds (Informatics, Engineering, Physics). Passion and a willingness to learn are the most important traits."
  },
  {
    q: "Is this only for Engineering students?",
    a: "Not at all. We need diverse skills including management, media, and documentation. However, technical roles do require a strong foundation in logic/math."
  },
  {
    q: "How can I support the team?",
    a: "We are open to sponsorships and industrial partnerships. Please use the contact section below to reach out."
  }
];
