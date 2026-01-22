/**
 * Structured Data Components for SEO
 *
 * These components add JSON-LD schema markup to improve search engine
 * understanding and enable rich snippets in search results.
 */

const BASE_URL = process.env.NEXT_PUBLIC_SITE_URL ?? 'https://bascorro.undip.ac.id';

interface StructuredDataProps {
  data: Record<string, unknown>;
}

function StructuredData({ data }: StructuredDataProps) {
  return (
    <script
      type="application/ld+json"
      dangerouslySetInnerHTML={{ __html: JSON.stringify(data) }}
    />
  );
}

/**
 * Organization Schema - Describes the BASCORRO team
 */
export function OrganizationSchema() {
  const data = {
    '@context': 'https://schema.org',
    '@type': 'Organization',
    name: 'BASCORRO',
    alternateName: 'BASCORRO Humanoid Robosoccer Team',
    url: BASE_URL,
    logo: `${BASE_URL}/favicon1.png`,
    description:
      'Student-driven humanoid robosoccer research team from Universitas Diponegoro, Indonesia. Competing in RoboCup and developing autonomous humanoid robots.',
    foundingDate: '2024',
    address: {
      '@type': 'PostalAddress',
      addressLocality: 'Semarang',
      addressRegion: 'Central Java',
      addressCountry: 'Indonesia',
    },
    parentOrganization: {
      '@type': 'EducationalOrganization',
      name: 'Universitas Diponegoro',
      alternateName: 'UNDIP',
      url: 'https://www.undip.ac.id',
    },
    sameAs: [
      'https://instagram.com/bascorro_undip',
      'https://github.com/ProgramBascorro',
    ],
    contactPoint: {
      '@type': 'ContactPoint',
      email: 'bascorro@undip.ac.id',
      contactType: 'general',
    },
    knowsAbout: [
      'Humanoid Robotics',
      'RoboCup',
      'Robot Soccer',
      'Computer Vision',
      'ROS 2',
      'Machine Learning',
      'Autonomous Systems',
    ],
  };

  return <StructuredData data={data} />;
}

/**
 * WebSite Schema - Describes the website with search action
 */
export function WebSiteSchema() {
  const data = {
    '@context': 'https://schema.org',
    '@type': 'WebSite',
    name: 'BASCORRO Documentation',
    alternateName: 'BASCORRO Docs',
    url: BASE_URL,
    description: 'Official documentation for BASCORRO Humanoid Robosoccer Team',
    publisher: {
      '@type': 'Organization',
      name: 'BASCORRO',
    },
    potentialAction: {
      '@type': 'SearchAction',
      target: {
        '@type': 'EntryPoint',
        urlTemplate: `${BASE_URL}/docs?search={search_term_string}`,
      },
      'query-input': 'required name=search_term_string',
    },
  };

  return <StructuredData data={data} />;
}

/**
 * FAQ Schema - For the landing page FAQ section
 */
export function FAQSchema() {
  const faqItems = [
    {
      question: 'Do I need prior robotics experience?',
      answer:
        'No. We welcome all UNDIP students with a passion for learning. You will be trained in your chosen division.',
    },
    {
      question: 'What is the time commitment?',
      answer:
        'Expect around 10-15 hours per week during the semester, with more intensive periods before competitions.',
    },
    {
      question: 'What will I learn?',
      answer:
        'Practical skills in mechanical design, electronics, programming (Python, C++, ROS 2), computer vision, and teamwork.',
    },
    {
      question: 'What is RoboCup?',
      answer:
        'RoboCup is an annual international robotics competition founded in 1997, with the goal of developing autonomous robots that can beat the human World Cup soccer champions by 2050.',
    },
    {
      question: 'What robot platform does BASCORRO use?',
      answer:
        'We primarily use the ROBOTIS OP3 humanoid robot platform, along with custom-built robots for specific competition categories.',
    },
  ];

  const data = {
    '@context': 'https://schema.org',
    '@type': 'FAQPage',
    mainEntity: faqItems.map((item) => ({
      '@type': 'Question',
      name: item.question,
      acceptedAnswer: {
        '@type': 'Answer',
        text: item.answer,
      },
    })),
  };

  return <StructuredData data={data} />;
}

/**
 * BreadcrumbList Schema - For documentation pages
 */
interface BreadcrumbItem {
  name: string;
  url: string;
}

export function BreadcrumbSchema({ items }: { items: BreadcrumbItem[] }) {
  const data = {
    '@context': 'https://schema.org',
    '@type': 'BreadcrumbList',
    itemListElement: items.map((item, index) => ({
      '@type': 'ListItem',
      position: index + 1,
      name: item.name,
      item: item.url,
    })),
  };

  return <StructuredData data={data} />;
}

/**
 * Article Schema - For documentation pages
 */
interface ArticleSchemaProps {
  title: string;
  description: string;
  url: string;
  dateModified?: string;
}

export function ArticleSchema({
  title,
  description,
  url,
  dateModified,
}: ArticleSchemaProps) {
  const data = {
    '@context': 'https://schema.org',
    '@type': 'TechArticle',
    headline: title,
    description: description,
    url: url,
    dateModified: dateModified ?? new Date().toISOString(),
    author: {
      '@type': 'Organization',
      name: 'BASCORRO',
    },
    publisher: {
      '@type': 'Organization',
      name: 'BASCORRO',
      logo: {
        '@type': 'ImageObject',
        url: `${BASE_URL}/favicon1.png`,
      },
    },
    mainEntityOfPage: {
      '@type': 'WebPage',
      '@id': url,
    },
  };

  return <StructuredData data={data} />;
}
