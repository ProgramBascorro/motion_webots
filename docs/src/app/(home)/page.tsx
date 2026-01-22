import LandingPage from '@/components/landing/LandingPage';
import { FAQSchema } from '@/components/seo/StructuredData';

export default function HomePage() {
  return (
    <>
      <FAQSchema />
      <LandingPage />
    </>
  );
}
